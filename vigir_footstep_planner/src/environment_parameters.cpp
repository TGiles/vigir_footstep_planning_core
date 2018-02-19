#include <vigir_footstep_planner/environment_parameters.h>

namespace vigir_footstep_planning
{
EnvironmentParameters::EnvironmentParameters(const vigir_generic_params::ParameterSet& params)
{
  // get robot specific parameters not included in the parameter set
  ros::NodeHandle nh;

  // get foot dimensions
  nh.getParam("foot/size/x", foot_size.x);
  nh.getParam("foot/size/y", foot_size.y);
  nh.getParam("foot/size/z", foot_size.z);
  nh.getParam("foot/separation", foot_seperation);
  ROS_ERROR("START OF ENVIRONMENT_PARAMETERS.CPP");
  ROS_INFO("foot/separation: %f", foot_seperation);
  ROS_INFO("foot/size/x: %f", foot_size.x);
  ROS_INFO("foot/size/y: %f", foot_size.y);
  ROS_INFO("foot/size/z: %f", foot_size.z);

  params.getParam("max_risk", max_risk, 1.0);

  // load remaining parameters from parameter set
  params.getParam("heuristic_scale", heuristic_scale);
  params.getParam("max_hash_size", hash_table_size);

  params.getParam("planner_type", ivPlannerType);
  params.getParam("search_until_first_solution", ivSearchUntilFirstSolution);
  params.getParam("forward_search", forward_search);

  params.getParam("max_planning_time", max_planning_time);
  params.getParam("initial_epsilon", initial_eps);
  params.getParam("decrease_epsilon", decrease_eps);
  //params.getParam("num_random_nodes", num_random_nodes);
  //params.getParam("random_node_dist", random_node_distance);

  //params.getParam("xxx", ivEnvironmentmap_step_cost_filename);

  // set collision mode
  params.getParam("collision_check/cell_size", cell_size);
  params.getParam("collision_check/num_angle_bins", num_angle_bins);
  angle_bin_size = 2.0*M_PI / static_cast<double>(num_angle_bins);

  double max_step_range_x, max_step_range_y, max_step_range_theta;
  double max_inverse_step_range_x, max_inverse_step_range_y, max_inverse_step_range_theta;

  // step range
  XmlRpc::XmlRpcValue step_range_x;
  XmlRpc::XmlRpcValue step_range_y;
  if (params.getParam("step_range/x", step_range_x) && params.getParam("step_range/y", step_range_y))
  {
    max_step_dist = 0;
    double max_x = (double)step_range_x[0];
    double max_y = (double)step_range_y[0];
    double max_inv_x = (double)step_range_x[0];
    double max_inv_y = (double)step_range_y[0];
    for (int i=0; i < step_range_x.size(); ++i)
    {
      double x = (double)step_range_x[i];
      double y = (double)step_range_y[i];
      ROS_INFO("Current x in iteration: %f %i", x, i);
      ROS_INFO("Current y in iteration: %f %i", y, i);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
      max_inv_x = std::min(max_inv_x, x);
      max_inv_y = std::min(max_inv_y, y);

      double cur_step_width = sqrt(x*x + y*y);
      ROS_INFO("Current step width in iteration: %f %i", cur_step_width, i);
      if (cur_step_width > max_step_dist)
        max_step_dist = cur_step_width;
      ROS_INFO("Current max_step_dist in iteration: %f %i", max_step_dist, i);
    }

    max_step_range_x = max_x;
    max_step_range_y = max_y;
    params.getParam("foot/max/step/theta", max_step_range_theta);
    max_inverse_step_range_x = max_inv_x;
    max_inverse_step_range_y = max_inv_y;
    params.getParam("foot/max/inverse/step/theta", max_inverse_step_range_theta);

    max_x = std::max(std::abs(max_x), std::abs(max_inv_x));
    max_y = std::max(std::abs(max_y), std::abs(max_inv_y));
    max_step_range_width = sqrt(max_x*max_x + max_y*max_y);
  }

  // misc parameters
  params.getParam("feedback_rate", feedback_rate);
  params.getParam("threads", threads);
  params.getParam("jobs_per_thread", jobs_per_thread);

  // upload online generated parameters
  /// TODO: Find smarter way
  vigir_generic_params::ParameterSet& params_(const_cast<vigir_generic_params::ParameterSet&>(params));

  params_.setParam("max_step_range/x", max_step_range_x);
  params_.setParam("max_step_range/y", max_step_range_y);
  params_.setParam("max_step_range/yaw", max_step_range_theta);

  params_.setParam("max_step_range/inv_x", max_inverse_step_range_x);
  params_.setParam("max_step_range/inv_y", max_inverse_step_range_y);
  params_.setParam("max_step_range/inv_yaw", max_inverse_step_range_theta);

  params_.setParam("max_step_range/width", max_step_range_width);

  params_.setParam("max_step_dist/x",      std::max(std::abs(max_step_range_x), std::abs(max_inverse_step_range_x)));
  params_.setParam("max_step_dist/y", 0.5*(std::max(std::abs(max_step_range_y), std::abs(max_inverse_step_range_y)) - foot_seperation));

  ROS_INFO("max_step_range/x: %f", max_step_range_x);
  ROS_INFO("max_step_range/y: %f", max_step_range_y);
  ROS_INFO("max_step_range/theta: %f", max_step_range_theta);
  ROS_INFO("max_step_range/inv_x: %f", max_inverse_step_range_x);
  ROS_INFO("max_step_range/inv_y: %f", max_inverse_step_range_y);
  ROS_INFO("max_step_range/inv_theta: %f", max_inverse_step_range_theta);
  ROS_INFO("max_step_range/width: %f", max_step_range_width);
  ROS_INFO("max_step_dist/x: %f",      std::max(std::abs(max_step_range_x), std::abs(max_inverse_step_range_x)));
  ROS_INFO("max_step_dist/y: %f",      0.5*std::max(std::abs(max_step_range_y), std::abs(max_inverse_step_range_y) - foot_seperation));

  ROS_ERROR("END OF ENVIRONMENT_PARAMETERS.CPP");
}

EnvironmentParameters::~EnvironmentParameters()
{
}
}
