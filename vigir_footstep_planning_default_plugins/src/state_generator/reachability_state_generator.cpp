#include <vigir_footstep_planning_default_plugins/state_generator/reachability_state_generator.h>

#include <vigir_footstep_planning_lib/math.h>

#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>



namespace vigir_footstep_planning
{
    ReachabilityStateGenerator::ReachabilityStateGenerator()
            : StateGeneratorPlugin("reachability_state_generator")
    {
    }

    bool ReachabilityStateGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
    {
      if (!StateGeneratorPlugin::loadParams(params))
        return false;

      bool result = true;
      ros::NodeHandle nh;
      int threads;
      unsigned int jobs_per_thread;
      result &= params.getParam("threads", threads);
      result &= params.getParam("jobs_per_thread", jobs_per_thread);

      int hash_table_size;
      double cell_size;
      int num_angle_bins;
      result &= params.getParam("max_hash_size", hash_table_size);
      result &= params.getParam("collision_check/cell_size", cell_size);
      result &= params.getParam("collision_check/num_angle_bins", num_angle_bins);
      ROS_ERROR("START OF REACHABILITY_STATE_GENERATOR.CPP");
      ROS_INFO("cell_size: %f", cell_size);
      ROS_INFO("num_angle_bins: %i", num_angle_bins);
      double angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);
      ROS_INFO("angle_bin_size: %f", angle_bin_size);

      int ivMaxStepRangeX, ivMaxStepRangeY, ivMaxStepRangeTheta;
      int ivMaxInvStepRangeX, ivMaxInvStepRangeY, ivMaxInvStepRangeTheta;
      double foot_separation;

      /// Defines the area of performable (discrete) steps.
      std::vector<std::pair<int, int> > step_range;

      // step range
      XmlRpc::XmlRpcValue step_range_x;
      XmlRpc::XmlRpcValue step_range_y;
      if (params.getParam("step_range/x", step_range_x) && params.getParam("step_range/y", step_range_y))
      {
        // create step range
        step_range.clear();
        step_range.reserve(step_range_x.size());
        double max_x = (double)step_range_x[0];
        double max_y = (double)step_range_y[0];
        double max_inv_x = (double)step_range_x[0];
        double max_inv_y = (double)step_range_y[0];
        for (int i=0; i < step_range_x.size(); ++i)
        {
          double x = (double)step_range_x[i];
          double y = (double)step_range_y[i];

          max_x = std::max(max_x, x);
          max_y = std::max(max_y, y);
          max_inv_x = std::min(max_inv_x, x);
          max_inv_y = std::min(max_inv_y, y);

          step_range.push_back(std::pair<int, int>(disc_val(x, cell_size), disc_val(y, cell_size)));
        }

        double max_step_range_theta;
        double max_inverse_step_range_theta;
        nh.getParam("foot/separation", foot_separation);
        result &= params.getParam("foot/max/step/theta", max_step_range_theta);
        result &= params.getParam("foot/max/inverse/step/theta", max_inverse_step_range_theta);
        ROS_INFO("max step theta param: %f", max_step_range_theta);
        ROS_INFO("max step theta inverse param: %f", max_inverse_step_range_theta);
        ivMaxStepRangeX = disc_val(max_x, cell_size);
        ivMaxStepRangeY = disc_val(max_y, cell_size);
        ivMaxStepRangeTheta = angle_state_2_cell(max_step_range_theta, angle_bin_size);
        ivMaxInvStepRangeX = disc_val(max_inv_x, cell_size);
        ivMaxInvStepRangeY = disc_val(max_inv_y, cell_size);
        ivMaxInvStepRangeTheta = angle_state_2_cell(max_inverse_step_range_theta, angle_bin_size);
        ROS_INFO("ivMaxStepRangeX: %i", ivMaxStepRangeX);
        ROS_INFO("ivMaxStepRangeY: %i", ivMaxStepRangeY);
        ROS_INFO("ivMaxStepRangeTheta: %i", ivMaxStepRangeTheta);
        ROS_INFO("ivMaxInvStepRangeX: %i", ivMaxInvStepRangeX);
        ROS_INFO("ivMaxInvStepRangeY: %i", ivMaxInvStepRangeY);
        ROS_INFO("ivMaxInvStepRangeTheta: %i", ivMaxInvStepRangeTheta);
      }


      // we can't proceed if parameters are missing
      if (!result)
        return false;

      // setup state expansion manager
      expand_states_manager.reset(new threading::ThreadingManager<threading::ExpandStateJob>(threads, jobs_per_thread));

      // determine whether a (x,y) translation can be performed by the robot by
      // checking if it is within a certain area of performable steps
      for (int y = ivMaxInvStepRangeY; y <= ivMaxStepRangeY; y++)
      {
        for (int x = ivMaxInvStepRangeX; x <= ivMaxStepRangeX; x++)
        {
          bool in_step_range = pointWithinPolygon(x, y, step_range);

          if (!in_step_range)
            continue;
          int k [5] = {1, 10, 1, 10 ,1};
//          ROS_ERROR("In step range");
//          ROS_INFO("Current Y iteration: %i", y);
//          ROS_INFO("Current X iteration: %i", x);
          // generate area of samplings for gpr/map-based planning
          for (int theta = ivMaxInvStepRangeTheta; theta <= ivMaxStepRangeTheta; theta++)
          {
            ROS_ERROR("Footstep sample theta: %i", theta);
            double cost = 0.0;
            // TODO: Fix delta values
//          double delta_x = 0.0 - cont_val(x, cell_size);
//          double delta_y = foot_separation - cont_val(y, cell_size);
//          double delta_theta = 0.0 - angle_cell_2_state(theta, angle_bin_size);
            double delta_x = 0.0 + x;
            double delta_y = y - foot_separation;
            double delta_theta = 0.0 + theta;
            double form_a = 1.333 - 33.33 * delta_x;
            ROS_INFO("form_a val: %f", form_a);
            if (delta_x > 0) {
              cost += k[0] * delta_x;
              ROS_INFO("k[0] calc val: %f", cost);
            } else {
              cost += k[1] * std::abs(delta_x);
              ROS_INFO("k[1] calc val: %f", cost);
            }
            double k2 = k[2] * std::abs(delta_y);
            double k3 = k[3] * std::min(1.0, std::max(0.0, form_a)) * std::abs(delta_y);
            double k4 = k[4] * std::abs(delta_theta);
            ROS_INFO("k[2] calc val: %f", k2);
            ROS_INFO("k[3] calc val: %f", k3);
            ROS_INFO("k[4] calc val: %f", k4);
            cost += k2 + k3 + k4;
//            Footstep f(cont_val(x, cell_size), cont_val(y, cell_size), angle_cell_2_state(theta, angle_bin_size), 0.0, cell_size, num_angle_bins, hash_table_size);
            Footstep f(cont_val(x, cell_size), cont_val(y, cell_size), angle_cell_2_state(theta, angle_bin_size), cost, cell_size, num_angle_bins, hash_table_size);
            ivContFootstepSet.push_back(f);

            ROS_INFO("Sample continuous (x, cell_size): %f", cont_val(x, cell_size));
            ROS_INFO("Sample continuous (y, cell_size): %f", cont_val(y, cell_size));
            ROS_INFO("Sample continuous (theta, angle_bin_size): %f", angle_cell_2_state(theta, angle_bin_size));
            ROS_INFO("Delta x: %f", delta_x);
            ROS_INFO("Delta y: %f", delta_y);
            ROS_INFO("Delta theta: %f", delta_theta);
            ROS_WARN("Cost of current footstep: %f", cost);
            // dump to screen
          }
        }
      }
      ROS_ERROR("END OF REACHABILITY_STATE_GENERATOR.CPP");
      return result;
    }

    std::list<PlanningState::Ptr> ReachabilityStateGenerator::generatePredecessor(const PlanningState& state) const
    {
      std::list<PlanningState::Ptr> result;

      ROS_ERROR("[ReachabilityStateGenerator] generatePredecessor not implemented yet!");

      return result;
    }

    std::list<PlanningState::Ptr> ReachabilityStateGenerator::generateSuccessor(const PlanningState& state) const
    {
      std::list<PlanningState::Ptr> result;

      // explorate all state
      std::list<threading::ExpandStateJob::Ptr> jobs;
      for (const Footstep& footstep : ivContFootstepSet)
        jobs.push_back(threading::ExpandStateJob::Ptr(new threading::ExpandStateJob(footstep, state)));

      expand_states_manager->addJobs(jobs);
      expand_states_manager->waitUntilJobsFinished();

      for (std::list<threading::ExpandStateJob::Ptr>::iterator itr = jobs.begin(); itr != jobs.end(); itr++)
      {
        threading::ExpandStateJob::Ptr& job = *itr;
        if (job->successful)
          result.push_back(job->next);
      }

      return result;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_footstep_planning::ReachabilityStateGenerator, vigir_footstep_planning::StateGeneratorPlugin)
