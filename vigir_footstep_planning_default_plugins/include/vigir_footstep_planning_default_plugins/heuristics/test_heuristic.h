//
// Created by tgiles on 3/6/18.
//

#ifndef VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_HEURISTIC_H
#define VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_HEURISTIC_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_plugins/plugins/heuristic_plugin.h>
#include <vigir_footstep_planning_default_plugins/world_model/grid_map_2d.h>
#include <vigir_footstep_planning_lib/math.h>

namespace vigir_footstep_planning
{
    class TestHeuristic
      : public HeuristicPlugin
    {
    struct StateKey {
        StateKey(const State& s, double cell_size, double angle_bin_size)
          : x(state_2_cell(s.getX(), cell_size))
          , y(state_2_cell(s.getY(), cell_size))
//          , yaw(angle_state_2_cell(s.getYaw(), angle_bin_size))
        {

        }
        bool operator<(const StateKey& key) const {
            if (x < key.x) return true;
            if (x > key.x) return false;
            if (y < key.y) return true;
            if (y > key.y) return false;
//            if (yaw < key.yaw) return true;
//            if (yaw > key.yaw) return false;
            return false;
        }
        int x;
        int y;
        // int yaw;
    };
    public:
        TestHeuristic();

        bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

        bool initialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;

        double getHeuristicValue(const State& from, const State& to, const State& start, const State& goal) const override;

        void reset();

        typedef std::map<StateKey, unsigned int> CostMap;
    protected:
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid_map);

        // subscribers
        ros::Subscriber occupancy_grid_map_sub_;

        // mutex
        mutable boost::shared_mutex grid_map_shared_mutex_;
        mutable std::vector<std::vector<float>> lookup_map;
        mutable CostMap cost_map_;
        // grid map data
        std::string grid_map_topic_;
        vigir_gridmap_2d::GridMap2D distance_map_;
        double cell_size_;
        double angle_bin_size_;
    };
}

#endif //VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_HEURISTIC_H
