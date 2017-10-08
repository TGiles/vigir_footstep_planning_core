#include <vigir_pattern_generator/joystick_handler.h>

namespace vigir_footstep_planning
{
JoystickHandler::JoystickHandler(ros::NodeHandle& nh)
  : enable_generator(false)
{
  // get threshold values for joystick control
  nh.param("joystick/thresh_x", thresh_lin.x, 0.0);
  nh.param("joystick/thresh_y", thresh_lin.y, 0.0);
  nh.param("joystick/thresh_yaw", thresh_rot.z, 0.0);

  // get acceleration factors for joystick control
  nh.param("joystick/sensivity_x", sensivity_lin.x, 0.0);
  nh.param("joystick/sensivity_y", sensivity_lin.y, 0.0);
  nh.param("joystick/sensivity_yaw", sensivity_rot.z, 0.0);

  // get velocity limits for joystick control
  nh.param("joystick/min_vel_x", limits_min_lin_vel.x, 0.0);
  nh.param("joystick/max_vel_x", limits_max_lin_vel.x, 0.0);
  nh.param("joystick/min_vel_y", limits_min_lin_vel.y, 0.0);
  nh.param("joystick/max_vel_y", limits_max_lin_vel.y, 0.0);
  nh.param("joystick/min_vel_yaw", limits_min_rot_vel.z, 0.0); limits_min_rot_vel.z *= (M_PI / 180.0);
  nh.param("joystick/max_vel_yaw", limits_max_rot_vel.z, 0.0); limits_max_rot_vel.z *= (M_PI / 180.0);

  // get acceleration limits for joystick control
  nh.param("joystick/min_acc_x", limits_min_lin_acc.x, 0.0);
  nh.param("joystick/max_acc_x", limits_max_lin_acc.x, 0.0);
  nh.param("joystick/min_acc_y", limits_min_lin_acc.y, 0.0);
  nh.param("joystick/max_acc_y", limits_max_lin_acc.y, 0.0);
  nh.param("joystick/min_acc_yaw", limits_min_rot_acc.z, 0.0); limits_min_rot_acc.z *= (M_PI / 180.0);
  nh.param("joystick/max_acc_yaw", limits_max_rot_acc.z, 0.0); limits_max_rot_acc.z *= (M_PI / 180.0);

  joy_sub = nh.subscribe("/joy", 1,& JoystickHandler::joyCallback, this);
}

JoystickHandler::~JoystickHandler()
{
}

void JoystickHandler::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  last_joy_msg = joy;

  if (last_joy_msg->buttons.size() < 8)
  {
    ROS_ERROR_THROTTLE(1.0, "Couldn't read the correct buttons. Maybe wrong controller connected?");
    return;
  }

  // check for (de-)activation
  if (last_joy_msg->buttons[7])
    enable_generator = true;
  else if (last_joy_msg->buttons[6])
    enable_generator = false;
}

void JoystickHandler::getJoystickCommand(double elapsed_time_sec, bool& enable, geometry_msgs::Twist& twist) const
{
  if (!last_joy_msg)
  {
    ROS_ERROR("Didn't get joystick input sor far. Check if joystick is connected and joy node running!");
    return;
  }

  if (last_joy_msg->axes.size() < 5)
  {
    ROS_ERROR("Couldn't read the correct axes. Maybe wrong controller connected?");
    return;
  }

  enable = enable_generator;

  // check x axis
  double joy_x = last_joy_msg->axes[4];
  joy_x = std::abs(joy_x) > thresh_lin.x ? joy_x : 0.0;
  updateJoystickCommand(elapsed_time_sec, joy_x, d_x, limits_min_lin_vel.x, limits_max_lin_vel.x, limits_min_lin_acc.x, limits_max_lin_acc.x, sensivity_lin.x);

  // check y axis
  double joy_y = last_joy_msg->axes[0];
  joy_y =  std::abs(joy_y) > thresh_lin.y ? joy_y : 0.0;
  updateJoystickCommand(elapsed_time_sec, joy_y, d_y, limits_min_lin_vel.y, limits_max_lin_vel.y, limits_min_lin_acc.y, limits_max_lin_acc.y, sensivity_lin.y);

  // check yaw axis
  double joy_yaw = last_joy_msg->axes[3];
  joy_yaw = std::abs(joy_yaw) > thresh_rot.z ? joy_yaw : 0.0;
  if (joy_x < 0.0)
    joy_yaw = -joy_yaw;
  updateJoystickCommand(elapsed_time_sec, joy_yaw, d_yaw, limits_min_rot_vel.z, limits_max_rot_vel.z, limits_min_rot_acc.z, limits_max_rot_acc.z, sensivity_rot.z);

  //ROS_INFO("%f (%f) / %f (%f) / %f (%f)", d_x, last_joy_msg->axes[4], d_y, last_joy_msg->axes[0], d_yaw, last_joy_msg->axes[3]);
}


void JoystickHandler::updateJoystickCommand(double elapsed_time_sec, double joy_val, double& val, double min_vel, double max_vel, double min_acc, double max_acc, double sensivity) const
{
  // determine acceleration
  double acc = convertJoyAxisToAcc(elapsed_time_sec, joy_val, val, min_vel, max_vel) * sensivity;
  acc = std::min(max_acc, std::max(min_acc, acc));

  // update value
  val = std::min(max_vel, std::max(min_vel, val+acc));
}

double JoystickHandler::convertJoyAxisToAcc(double elapsed_time_sec, double joy_val, double val, double min_vel, double max_vel) const
{
  // using current val to determine joy_val scale
  if (val >= 0.0)
    return (joy_val*std::abs(max_vel) - val)*elapsed_time_sec;
  else
    return (joy_val*std::abs(min_vel) - val)*elapsed_time_sec;
}
}
