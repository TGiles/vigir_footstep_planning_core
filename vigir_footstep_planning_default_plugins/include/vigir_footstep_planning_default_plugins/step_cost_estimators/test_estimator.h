//
// Created by tgiles on 2/15/18.
//

#ifndef VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_ESTIMATOR_H
#define VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_ESTIMATOR_H

#include <vigir_footstep_planning_plugins/plugins/step_cost_estimator_plugin.h>

namespace vigir_footstep_planning
{
    class TestEstimator
            : public StepCostEstimatorPlugin {
    public: TestEstimator();
        bool loadParams(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override;
        bool getCost(const State& left_foot, const State& right_foot, const State& swing_foot, double& cost, double& cost_multiplier, double& risk, double& risk_multiplier) const override;
    protected:
        int num_angle_bins_;
        double cell_size;
    };
}


#endif //VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_TEST_ESTIMATOR_H
