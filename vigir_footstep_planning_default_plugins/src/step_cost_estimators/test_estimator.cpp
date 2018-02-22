//
// Created by tgiles on 2/15/18.
//
#include <vigir_footstep_planning_default_plugins/step_cost_estimators/test_estimator.h>

namespace vigir_footstep_planning
{
    TestEstimator::TestEstimator()
            : StepCostEstimatorPlugin("test_estimator") {

    }

    bool TestEstimator::loadParams(const vigir_generic_params::ParameterSet &params) {
      if (!StepCostEstimatorPlugin::loadParams(params))
        return false;
      params.getParam("collision_check/num_angle_bins", num_angle_bins_);
      params.getParam("collision_check/cell_size", cell_size);

    }

    bool TestEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
    {
        cost = 0.0;
        cost_multiplier = 1.0;
        risk = 0.0;
        risk_multiplier = 1.0;
//        double num_angle_bins = num_angle_bins_ * 1.0;
        double angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins_);
        int num_angle_bins = num_angle_bins_;
        if (swing_foot == left_foot || swing_foot == right_foot)
            return true;

        const State& swing_foot_before = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
         int k [5] ={1, 10, 1, 10, 1};
//        double k [5] = {1 * cell_size, 10 *cell_size, 1 *cell_size, 10*cell_size, 1*cell_size};
//        double delta_x = swing_foot_before.getX() - swing_foot.getX();
//        double delta_y = swing_foot_before.getY() - swing_foot.getY();
//        double delta_theta = swing_foot_before.getYaw() - swing_foot.getYaw();
        double delta_x = swing_foot.getX() - swing_foot_before.getX();
        double delta_y = swing_foot.getY() - swing_foot_before.getY();
        double delta_theta = swing_foot.getYaw() - swing_foot_before.getYaw();
        double nominal_delta_x = cos(angle_cell_2_state(delta_theta, angle_bin_size)) * delta_x
          - sin(angle_cell_2_state(delta_theta, angle_bin_size))* delta_x;
        double nominal_delta_y = sin(angle_cell_2_state(delta_theta, angle_bin_size)) * delta_y
          + cos(angle_cell_2_state(delta_theta, angle_bin_size)) * delta_y;
//          double delta_x = disc_val(swing_foot.getX() - swing_foot_before.getX(), cell_size);
//          double delta_y = disc_val(swing_foot.getY() - swing_foot_before.getY(), cell_size);
//          double delta_theta = angle_state_2_cell(swing_foot.getYaw() - swing_foot_before.getYaw(), angle_bin_size);
      // other form_a is 1.2 - 0.4 * delta_x
        double form_a = 1.333 - 33.33 * delta_x;
//          double form_a = 1.2 - 0.4 * delta_x;
//        double form_a = 1.333 - 33.33 * nominal_delta_x;
//        ROS_WARN("-----------TEST_ESTIMATOR----------");
//        ROS_INFO("form_a val: %f", form_a);
        if (delta_x >= 0) {
            cost += k[0] * delta_x;
//            cost += k[0] * nominal_delta_x;
//          ROS_INFO("k[0] calc val: %f", cost);
        } else {
            cost += k[1] * delta_x;
//            cost += k[1] * nominal_delta_x;
//          ROS_INFO("k[1] calc val: %f", cost);
        }
        double k2 = k[2] * std::abs(delta_y);
//        double k2 = k[2] * std::abs(nominal_delta_y);
        double k3 = k[3] * std::min(1.0, std::max(0.0, (form_a))) * std::abs(delta_y);
//        double k3 = k[3] * std::min(1.0, std::max(0.0, form_a)) * std::abs(nominal_delta_y);
        double k4 = k[4] * std::abs(delta_theta);
//        ROS_INFO("k[2] calc val: %f", k2);
//        ROS_INFO("k[3] calc val: %f", k3);
//        ROS_INFO("k[4] calc val: %f", k4);
        cost += k2 + k3 + k4;
//        ROS_INFO("Delta x: %f", delta_x);
//        ROS_INFO("Delta y: %f", delta_y);
//        ROS_INFO("Delta theta: %f", delta_theta);
//        ROS_INFO("nominal delta x: %f", nominal_delta_x);
//        ROS_INFO("nominal delta y: %f", nominal_delta_y);
//        ROS_INFO("cost val: %f", cost);
//        ROS_INFO("cost val scaled by cell size: %f", cost * 20);
//        ROS_INFO("euclidean cost for comparison: %f",
//                 0.5*euclidean_distance(
//                   swing_foot_before.getX(), swing_foot_before.getY(),
//                   swing_foot.getX(), swing_foot.getY()));
        // cost = 0.5*euclidean_distance(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());
        return true;
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TestEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)