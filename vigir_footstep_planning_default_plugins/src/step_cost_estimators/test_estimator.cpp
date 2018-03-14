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

//      ros::NodeHandle nh;
//      nh.getParam("foot/separation", foot_separation);
//      ROS_INFO("foot separation in test_estimator: %f", foot_separation);
      params.getParam("collision_check/num_angle_bins", num_angle_bins_);
      params.getParam("collision_check/cell_size", cell_size);
      params.getParam("test_estimator/k0", k0);
      params.getParam("test_estimator/k1", k1);
      params.getParam("test_estimator/k2", k2);
      params.getParam("test_estimator/k3", k3);
      params.getParam("diff_angle_cost", k4);
      // get foot separation

    }

    bool TestEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
    {
        cost = 0.0;
        cost_multiplier = 1.0;
        risk = 0.0;
        risk_multiplier = 1.0;
//        double num_angle_bins = num_angle_bins_ * 1.0;
//        double angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins_);
//        int num_angle_bins = num_angle_bins_;
        if (swing_foot == left_foot || swing_foot == right_foot)
            return true;

        const State& swing_foot_before = swing_foot.getLeg() == LEFT ? right_foot : left_foot;
//         int k [5] ={1, 10, 1, 10, 1};
//        double k [5] = {1.0, 10.0, 1.0, 10.0, 1.0 / 3.0};
//          double k [5] = {1.0, 10.0, 1.0, 10.0, 0.0};
      double k [5] = {k0, k1, k2, k3, k4};
//        double delta_x = swing_foot_before.getX() - swing_foot.getX();
//        double delta_y = swing_foot_before.getY() - swing_foot.getY();
//        double delta_theta = swing_foot_before.getYaw() - swing_foot.getYaw();
        double delta_x = swing_foot.getX() - swing_foot_before.getX();
        double delta_y = swing_foot.getY() - swing_foot_before.getY();
        double delta_theta = swing_foot.getYaw() - swing_foot_before.getYaw();
        double theta_before = swing_foot_before.getYaw();
//        ROS_WARN("theta_swing %f, before: %f", swing_foot.getYaw(), theta_before);
//        theta_before = angle_state_2_cell(theta_before, angle_bin_size);
//        ROS_WARN("theta_before with angle_state_2_cell: %f", theta_before);
//        int fake = static_cast<int>(swing_foot_before.getYaw());
//        double alt_theta_before = angle_cell_2_state(fake, angle_bin_size);
//        int fake_after = static_cast<int>(swing_foot.getYaw());
//        double alt_theta_after = angle_cell_2_state(fake_after, angle_bin_size);

        // Rotate into the stance foot frame
        double nominal_delta_x =  cos(theta_before) * delta_x  + sin(theta_before)* delta_y;
        double nominal_delta_y = -sin(theta_before) * delta_x  + cos(theta_before) * delta_y;

      if(swing_foot_before.getLeg() == LEFT) {
        nominal_delta_y = nominal_delta_y * -1;
//        theta_before = theta_before * -1;
        delta_theta = delta_theta * -1;
      }
//      nominal_delta_y = nominal_delta_y - foot_separation;
      nominal_delta_y = nominal_delta_y - 0.1;
//      double nominal_delta_x = cos(alt_theta_before) * delta_x
//                               - sin(alt_theta_before)* delta_x;
//      double nominal_delta_y = sin(alt_theta_before) * delta_y
//                               + cos(alt_theta_before) * delta_y;
//          double delta_x = disc_val(swing_foot.getX() - swing_foot_before.getX(), cell_size);
//          double delta_y = disc_val(swing_foot.getY() - swing_foot_before.getY(), cell_size);
//          double delta_theta = angle_state_2_cell(swing_foot.getYaw() - swing_foot_before.getYaw(), angle_bin_size);
      // other form_a is 1.2 - 0.4 * delta_x
//        double form_a = 1.333 - 33.33 * delta_x;
//          double form_a = 1.2 - 0.4 * delta_x;
        double form_a = 1.333 - 33.33 * nominal_delta_x;
        if(form_a > 1.0) {
          form_a = 1.0;
        } else if (form_a < 0.0) {
          form_a = 0.0;
        }

//        ROS_WARN("-----------TEST_ESTIMATOR----------");
//        ROS_INFO("Curr stance leg: %s", swing_foot_before.getLeg() == LEFT ? "left" : "right");
//        ROS_INFO("Curr stance leg x: %f", swing_foot_before.getX());
//        ROS_INFO("Curr stance leg y: %f", swing_foot_before.getY());
//        ROS_INFO("Curr stance leg theta: %f", swing_foot_before.getYaw());
//        ROS_INFO("Curr swing leg: %s", swing_foot.getLeg() == LEFT ? "left" : "right");
//        ROS_INFO("Curr swing leg x: %f", swing_foot.getX());
//        ROS_INFO("Curr swing leg y: %f", swing_foot.getY());
//        ROS_INFO("Curr swing leg theta: %f", swing_foot.getYaw());
//        ROS_INFO("form_a val: %f", form_a);
        if (nominal_delta_x >= 0) {
//            cost += k[0] * delta_x;
            cost += k[0] * nominal_delta_x;
//          ROS_INFO("k[0] calc val: %f", cost);
        } else {
//            cost += k[1] * delta_x;
            cost += k[1] * std::abs(nominal_delta_x);
//          ROS_INFO("k[1] calc val: %f", cost);
        }
//        double k2 = k[2] * std::abs(delta_y);
        double k2 = k[2] * std::abs(nominal_delta_y);
//        double k3 = k[3] * std::min(1.0, std::max(0.0, (form_a))) * std::abs(delta_y);
        double k3 = k[3] * std::min(1.0, std::max(0.0, form_a)) * (0.01 + std::abs(nominal_delta_y));
        double k4 = k[4] * std::abs(delta_theta)* std::abs(delta_theta);
//        ROS_INFO("k[2] calc val: %f", k2);
//        ROS_INFO("k[3] calc val: %f", k3);
//        ROS_INFO("k[4] calc val: %f", k4);
//          cost += 0.01;
        cost += k2 + k3 + k4;
//        ROS_INFO("Delta x: %f", delta_x);
//        ROS_INFO("Delta y: %f", delta_y);
//        ROS_INFO("Delta theta: %f", delta_theta);
//        ROS_INFO("Theta before: %f", theta_before);
//        ROS_INFO("nominal delta x: %f", nominal_delta_x);
//        ROS_INFO("nominal delta y: %f", nominal_delta_y);
//        ROS_INFO("cost val: %f", cost);
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
