#include <roscco/ros_to_oscc.h>

RosToOscc::RosToOscc(ros::NodeHandle* public_nh, ros::NodeHandle* private_nh, double& steering_address)
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Temporary block of OSCC SIGIO while initializing ROS publication to prevent
  // signal conflicts
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0)
  {
    ROS_ERROR("Failed to block SIGIO");
  }

  //subscribers
  topic_brake_command_ = public_nh->subscribe<roscco::BrakeCommand>("brake_command", 10, &RosToOscc::brakeCommandCallback, this);
  topic_steering_command_ = public_nh->subscribe<roscco::SteeringCommand>("steering_torque_command", 10, &RosToOscc::steeringCommandCallback, this);
  topic_steering_angle_command_ = public_nh->subscribe<roscco::SteeringAngleCommand>("steering_angle_command", 10, &RosToOscc::steeringAngleCommandCallback, this);

  topic_throttle_command_ = public_nh->subscribe<roscco::ThrottleCommand>("throttle_command", 10, &RosToOscc::throttleCommandCallback, this);

  topic_enable_disable_command_ = public_nh->subscribe<roscco::EnableDisable>("enable_disable", 10, &RosToOscc::enableDisableCallback, this);

  steering_angle_report_ptr = &steering_address;

  createPIDState( 0, steer_state);
  //dynamic_reconfigure these + define macro
  params->max = 1;
  params->min = -1;
  params->p_term = 1.5;
  params->i_term = 0.1;
  params->d_term = 0.06;
  params->i_max = 400;

  if (sigprocmask(SIG_SETMASK, &orig_mask, NULL) < 0)
  {
    ROS_ERROR("Failed to unblock SIGIO");
  }
};

void RosToOscc::brakeCommandCallback(const roscco::BrakeCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_brake_position(msg->brake_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the brake position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the brake position.");
  }
};

void RosToOscc::steeringCommandCallback(const roscco::SteeringCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_steering_torque(msg->steering_torque);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the steering torque.");
  }
};

void RosToOscc::steeringAngleCommandCallback(const roscco::SteeringAngleCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;
  static double steering_angle_command;
  steering_angle_command = msg->steering_angle;
  new_pid_state = abs(steering_angle_command - prev_steering_angle_command) > STEERING_STATE_TOLERANCE;
  //DO PID HERE
  if(new_pid_state){
            createPIDState( steering_angle_command, steer_state);
        }
    else{
          steer_state->setpoint = steering_angle_command;
  }
  prev_steering_angle_command = steering_angle_command;
  ret = oscc_publish_steering_torque(pidController( params, steer_state, *steering_angle_report_ptr ));

  // ret = oscc_publish_steering_torque(msg->steering_angle);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the steering torque.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the steering torque.");
  }
};

void RosToOscc::throttleCommandCallback(const roscco::ThrottleCommand::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = oscc_publish_throttle_position(msg->throttle_position);

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying send the throttle position.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying send the throttle position.");
  }
};

void RosToOscc::enableDisableCallback(const roscco::EnableDisable::ConstPtr& msg)
{
  oscc_result_t ret = OSCC_ERROR;

  ret = msg->enable_control ? oscc_enable() : oscc_disable();

  if (ret == OSCC_ERROR)
  {
    ROS_ERROR("OSCC_ERROR occured while trying to enable or disable control.");
  }
  else if (ret == OSCC_WARNING)
  {
    ROS_WARN("OSCC_WARNING occured while trying to enable or disable control.");
  }
}
