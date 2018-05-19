//
// Created by tgiles on 3/6/18.
//

#include <vigir_footstep_planning_default_plugins/heuristics/test_heuristic.h>
#include <queue>


namespace vigir_footstep_planning
{
    TestHeuristic::TestHeuristic()
      : HeuristicPlugin("test_heuristic") {

    }
    bool TestHeuristic::loadParams(const vigir_generic_params::ParameterSet& params) {
      if (!HeuristicPlugin::loadParams(params)) {
        return false;
      }
      getParam("grid_map_topic", grid_map_topic_, std::string());
      params.getParam("collision_check/cell_size", cell_size_);
      int num_angle_bins;
      params.getParam("collision_check/num_angle_bins", num_angle_bins);
      angle_bin_size_ = 2.0*M_PI / static_cast<double>(num_angle_bins);
      return true;
    }
    bool TestHeuristic::initialize(const vigir_generic_params::ParameterSet& params) {
      if (!HeuristicPlugin::initialize(params)) {
        return false;
      }
      // subscribe topics
      occupancy_grid_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(grid_map_topic_, 1, &TestHeuristic::mapCallback, this);
      // Get goal pose
      // Run UCS with implicit graph structure
      // Each time a cost is found from binary map, add to lookup map
      // After environment is completely searched, return true
      cv::Mat mapper = distance_map_.binaryMap();
      int rows = mapper.rows;
      int cols = mapper.cols;
      float maxVal = 0.0; // pixel turned off is black, therefore 0.0
      lookup_map.resize(cols, std::vector<float>(rows, maxVal));

      // distance_map_.binaryMap();
      return true;
    }
    void TestHeuristic::mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map) {
      boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
      distance_map_.setMap(occupancy_grid_map);
    }
    void TestHeuristic::reset() {
      // boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
      // Figure out which data structure to reset
      // Probably not the grid map
      // boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);
      // cost_map_.reset();

    }
    double TestHeuristic::getHeuristicValue(const State& from, const State& to, const State& /*start*/, const State& /*goal*/ ) const {
      if (from == to){
        return 0.0;
      }
      double d = 0;
      {
        boost::unique_lock<boost::shared_mutex> lock(grid_map_shared_mutex_);

      }
    }
}