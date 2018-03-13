//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
// Based on http://wiki.ros.org/footstep_planner by Johannes Garimort and Armin Hornung
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_PATH_COST_HEURISTIC_H_
#define VIGIR_FOOTSTEP_PLANNING_PATH_COST_HEURISTIC_H_

#include <sbpl/headers.h>

#include <vigir_footstep_planning_plugins/plugins/heuristic_plugin.h>

#include <vigir_footstep_planning_default_plugins/world_model/grid_map_2d.h>



namespace vigir_footstep_planning
{
/**
 * @brief Determining the heuristic value by calculating a 2D path from each
 * grid cell of the map to the goal and using the path length as expected
 * distance.
 *
 * The heuristic value consists of the following factors:
 *
 *  + The expected distance retreived from the 2D path.
 *
 *  + The expected path costs.
 *
 *  + The difference between the orientation of the two states multiplied
 *    by some cost factor.
 */
class PathCostHeuristic
  : public HeuristicPlugin
{
public:
  PathCostHeuristic();

  ~PathCostHeuristic();

  bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

  /**
   * @return The estimated costs needed to reach the state 'to' from within the
   * current state.
   */
  double getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const override;

  /**
   * @brief Calculates for each grid cell of the map a 2D path to the
   * cell (to.x, to.y).
   * For forward planning 'to' is supposed to be the goal state, for backward
   * planning 'to' is supposed to be the start state.
   */
  bool calculateDistances(const State& from, const State& to);

  void updateMap(const vigir_gridmap_2d::GridMap2D& map);
  /**
   * @brief Initializes heuristic calculations based on change in planning problem
   */
  void updateHeuristicValues(const State& start, const State& goal);

protected:
  static const int cvObstacleThreshold = 200;

  unsigned char** ivpGrid;

  double ivCellSize;
  int    ivNumAngleBins;
  double ivAngleBinSize;

  double ivStepCost;
  double ivDiffAngleCost;
  double ivMaxStepWidth;
  double ivInflationRadius;

  int ivGoalX;
  int ivGoalY;

  boost::shared_ptr<SBPL2DGridSearch> ivGridSearchPtr;
  nav_msgs::MapMetaData m_mapInfo;

  std::string gridMapName;

  void resetGrid();

  void worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const
  {
    mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
    my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);
    //ROS_INFO("   PathCostHeuristic::worldToMapNoBounds   (%d, %d) (%f, %f)  - (%f, %f) res=%f (%d, %d)",
    //    mx, my, wx, wy, m_mapInfo.origin.position.x, m_mapInfo.origin.position.y, m_mapInfo.resolution, m_mapInfo.width, m_mapInfo.height);
  }


};
}

#endif
