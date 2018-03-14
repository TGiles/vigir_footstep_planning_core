#include <vigir_footstep_planning_default_plugins/heuristics/path_cost_heuristic.h>
#include <vigir_footstep_planning_default_plugins/world_model/upper_body_grid_map_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_footstep_planning
{
PathCostHeuristic::PathCostHeuristic()
  : HeuristicPlugin("path_cost_heuristic")
  , ivpGrid(NULL)
  , ivGoalX(-1)
  , ivGoalY(-1)
  , ivWidth(-1)
  , ivHeight(-1)
  , gridMapName("2_upper_body_grid_map_model")
  , ivInflationRadius(0.10)
{
  ROS_INFO(" PathCostHeuristic::PathCostHeuristic - constructor!");
}

PathCostHeuristic::~PathCostHeuristic()
{
  ROS_INFO(" PathCostHeuristic::PathCostHeuristic - destructor!");
  if (ivpGrid)
    resetGrid();
}

bool PathCostHeuristic::loadParams(const vigir_generic_params::ParameterSet& params)
{
  ROS_INFO(" PathCostHeuristic::loadParams ...");
  if (!HeuristicPlugin::loadParams(params))
    return false;

  params.getParam("collision_check/cell_size", ivCellSize);
  params.getParam("collision_check/num_angle_bins", ivNumAngleBins);
  ivAngleBinSize = 2.0*M_PI / static_cast<double>(ivNumAngleBins);

  params.getParam("const_step_cost_estimator/step_cost", ivStepCost, 0.1);
  params.getParam("diff_angle_cost", ivDiffAngleCost);
  params.getParam("max_step_dist/x", ivMaxStepWidth);
  /// TODO
  ROS_ERROR(" Need to add parameters for gridmap plugin name ");
  ///gridMapName("2_upper_body_grid_map_model")
  
  return true;
}

void PathCostHeuristic::updateHeuristicValues(const State& start, const State& goal)
{
  ROS_INFO(" PathCostHeuristic::updateHeuristicValues - Updating the heuristic values ...");
  updateMap();
  calculateDistances(start, goal);


  // Debugging printout for scale
  unsigned int from_x;
  unsigned int from_y;
  m_gridMap.worldToMapNoBounds(start.getX(), start.getY(), from_x, from_y);
  double path_cost_from = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(from_x, from_y)) / 1000.0;

  unsigned int to_x;
  unsigned int to_y;
  m_gridMap.worldToMapNoBounds(goal.getX(), goal.getY(), to_x, to_y);
  double path_cost_to = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(to_x, to_y)) / 1000.0;
  double dist = path_cost_from - path_cost_to;
  if (dist < 0.0)
  {// Assume the cost_to is close to the robot center and negligible over total cost
      //ROS_INFO("   PathCostHeuristic::getHeuristicValue   %f < %f  ---> return 0.0 ", path_cost_from, path_cost_to );
      dist = 0.0;
  }
  double expected_steps = dist / ivMaxStepWidth;

  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(start.getYaw(), goal.getYaw()));

  double h = dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost;

  ROS_INFO(" PathCostHeuristic::updateHeuristicValues - Done updating the heuristic values\n                   c(start)=%f  c(goal)=%f  h(start)=%f = %f+%f+%f\n                   (MaxStepWidth=%f StepCost=%f DiffAngleCost=%f inflation radius=%f )  expected_steps=%f diff_angle=%f!",
            path_cost_from, path_cost_to, h, dist, expected_steps * ivStepCost , diff_angle * ivDiffAngleCost,
          ivMaxStepWidth, ivStepCost, ivDiffAngleCost, ivInflationRadius, expected_steps, diff_angle);
}

double PathCostHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/) const
{
  //ROS_INFO("   PathCostHeuristic::getHeuristicValue   ... " );

  if (from == to)
  {
    //ROS_INFO("   PathCostHeuristic::getHeuristicValue   from == to  ---> return 0.0 " );
    return 0.0;
  }

  assert(ivGoalX >= 0 && ivGoalY >= 0);

  unsigned int from_x;
  unsigned int from_y;
  m_gridMap.worldToMapNoBounds(from.getX(), from.getY(), from_x, from_y);
  double path_cost_from = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(from_x, from_y)) / 1000.0;

  unsigned int to_x;
  unsigned int to_y;
  m_gridMap.worldToMapNoBounds(to.getX(), to.getY(), to_x, to_y);
  double path_cost_to = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(to_x, to_y)) / 1000.0;

  double dist = path_cost_from - path_cost_to;
  if (dist < 0.0)
  {// Assume the cost_to is close to the robot center and negligible over total cost
      //ROS_INFO("   PathCostHeuristic::getHeuristicValue   %f < %f  ---> return 0.0 ", path_cost_from, path_cost_to );
      dist = 0.0;
  }
  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  if (ivDiffAngleCost > 0.0)
    diff_angle = std::abs(angles::shortest_angular_distance(to.getYaw(), from.getYaw()));

  double h = dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost;

  //ROS_INFO("   PathCostHeuristic::getHeuristicValue   %f = %f (%d, %d)  - %f (%d, %d)  ---> h=%f=%f+%f+%f ",
  //    dist, path_cost_from, from_x, from_y, path_cost_to, to_x, to_y,
  //    h, dist,expected_steps * ivStepCost, diff_angle * ivDiffAngleCost );
  return h;
}

bool PathCostHeuristic::calculateDistances(const State& from, const State& to)
{
  assert(ivpGrid);
  ROS_INFO("    PathCostHeuristic::calculateDistances ...");

  unsigned int from_x;
  unsigned int from_y;
  m_gridMap.worldToMapNoBounds(from.getX(), from.getY(), from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  m_gridMap.worldToMapNoBounds(to.getX(), to.getY(), to_x, to_y);

  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ROS_INFO("   PathCostHeuristic::calculateDistances   (%d, %d)  - (%d, %d)\n             (%f, %f)  - (%f, %f)\n             ivCellSize=%f res=%f (%d, %d)",
        from_x, from_y, to_x, to_y, from.getX(), from.getY(), to.getX(), to.getY(), ivCellSize, m_gridMap.getInfo().resolution, m_gridMap.getInfo().width, m_gridMap.getInfo().height);

    ivGoalX = to_x;
    ivGoalY = to_y;
    ivGridSearchPtr->search(ivpGrid, cvObstacleThreshold,
                            ivGoalX, ivGoalY, from_x, from_y,
                            SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }

  return true;
}

bool PathCostHeuristic::updateMap()
{

  ivGoalX = ivGoalY = -1;

  boost::shared_ptr<GridMapModel> gridMapModel;
  if (!WorldModel::mutableInstance().getPlugin<GridMapModel>(gridMapModel, gridMapName) )
  {
    ROS_ERROR("   PathCostHeuristic::updateHeuristicValues - Invalid grid map model plugin!");
    return false;
  }

  ivInflationRadius = gridMapModel->getInscribingRadius();

  gridMapModel->copyMap(m_gridMap);// get thread safe copy

  ROS_INFO("    PathCostHeuristic::updateMap - updating the map ...");
  if (ivpGrid && ( (m_gridMap.getInfo().width !=ivWidth) ||
                   (m_gridMap.getInfo().height != ivHeight) ) ) {
      ROS_INFO("    Free memory of prior planning grid on map change (%d, %d) vs. (%d, %d)",
                  m_gridMap.getInfo().width,m_gridMap.getInfo().height,
                  ivWidth,ivHeight);
      resetGrid(); // free prior memory

  }

  ivWidth  = m_gridMap.getInfo().width;
  ivHeight = m_gridMap.getInfo().height;

  ROS_INFO("    Initialize the 2D grid search using SBPL to calculate the heuristic values ...");
  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();
  ivGridSearchPtr.reset(new SBPL2DGridSearch(ivWidth, ivHeight,
                                             m_gridMap.getInfo().resolution));

  if (ivpGrid == NULL) {
      // Allocate new memory for planning grid
      ROS_INFO("    Allocate the occupancy grid used for planning ...");
      ivpGrid = new unsigned char* [ivWidth];

      for (unsigned x = 0; x < ivWidth; ++x)
        ivpGrid[x] = new unsigned char [ivHeight];
  }

  // Update the grid data in case something in the map changed
  ROS_INFO("    Update the occupancy grid used for planning ...");
  for (unsigned y = 0; y < ivHeight; ++y)
  {
    for (unsigned x = 0; x < ivWidth; ++x)
    {
      float dist = m_gridMap.distanceMapAtCell(x,y);
      if (dist < 0.0f)
        ROS_ERROR("     Distance map at %d %d out of bounds", x, y);
      else if (dist <= ivInflationRadius)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
    }
  }

  ROS_INFO("    PathCostHeuristic::updateMap - done!");
  return true;
}

void PathCostHeuristic::resetGrid()
{
  for (int x = 0; x < ivWidth; ++x)
  {
    if (ivpGrid[x])
    {
      delete[] ivpGrid[x];
      ivpGrid[x] = NULL;
    }
  }
  delete[] ivpGrid;
  ivpGrid = NULL;
  ROS_INFO("      PathCostHeuristic::resetGrid!");
}
}

PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::PathCostHeuristic, vigir_footstep_planning::HeuristicPlugin)
