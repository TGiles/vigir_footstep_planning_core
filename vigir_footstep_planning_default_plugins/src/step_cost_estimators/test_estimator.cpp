//
// Created by tgiles on 2/15/18.
//
#include <vigir_footstep_planning_default_plugins/step_cost_estimators/test_estimator.h>

namespace vigir_footstep_planning
{
    TestEstimator::TestEstimator()
            : StepCostEstimatorPlugin("test_estimator") {

    }
    bool TestEstimator::getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const
    {
        cost = 0.0;
        cost_multiplier = 1.0;
        risk = 0.0;
        risk_multiplier = 1.0;

        if (swing_foot == left_foot || swing_foot == right_foot)
            return true;

        const State& swing_foot_before = swing_foot.getLeg() == LEFT ? left_foot : right_foot;
        // int k [5] ={1, 10, 1, 10, 1};
        int k [5] = {1, 10, 1, 10, 1};
        double delta_x = swing_foot_before.getX() - swing_foot.getX();
        double delta_y = swing_foot_before.getY() - swing_foot.getY();
        double delta_theta = swing_foot_before.getYaw() - swing_foot.getYaw();
        double form_a = 1.333 - (-33.33 * delta_x);
//        ROS_WARN("-----------------------------");
//        ROS_INFO("Delta x: %f", delta_x);
//        ROS_INFO("Delta y: %f", delta_y);
//        ROS_INFO("Delta yaw: %f", delta_theta);
//        ROS_INFO("Formula A: %f", form_a);
        if (delta_x >= 0) {
            cost += k[0] * std::max(0.0, delta_x);
        } else {
            cost += k[1] * std::max(0.0, delta_x);
        }

        cost += k[2] * std::abs(delta_y)
                + k[3] * std::min(1.0, std::max(0.0, (form_a))) * std::abs(delta_y)
                + k[4] * std::abs(delta_theta);
        // cost = 0.5*euclidean_distance(swing_foot_before.getX(), swing_foot_before.getY(), swing_foot.getX(), swing_foot.getY());
        return true;
    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::TestEstimator, vigir_footstep_planning::StepCostEstimatorPlugin)