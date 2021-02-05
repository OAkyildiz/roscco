#include <geometry_msgs/Accel.h>
#include <ros/ros.h>
#include <roscco/BrakeCommand.h>
#include <roscco/EnableDisable.h>
#include <roscco/SteeringCommand.h>
#include <roscco/SteeringAngleCommand.h>

#include <roscco/ThrottleCommand.h>
#include <sensor_msgs/Joy.h>

double calc_exponential_average(double AVERAGE, double SETPOINT, double FACTOR);
double linear_tranformation(double VALUE, double HIGH_1, double LOW_1, double HIGH_2, double LOW_2);
double map_and_clip(double VALUE, double HIGH_1, double LOW_1, double HIGH_2, double LOW_2, double CUTOFF);


// TODO: seriously refactor. Valuea acquisition in callbacks, enable/disable sub in callback
// TODO: * publish, spin, initiate in LOOP?
// TODO: *ROS_DEBUG for published values
// TODO: CHECK STEERING DIRECTION
class RosccoTeleop
{
public:
  RosccoTeleop();

private:
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher throttle_pub_;
  ros::Publisher brake_pub_;
  ros::Publisher steering_pub_;
  ros::Publisher enable_disable_pub_;
  ros::Subscriber joy_sub_;

  int previous_start_state_ = 0;
  int previous_back_state_ = 0;

  // Number of messages to retain when the message queue is full
  const int QUEUE_SIZE_ = 10;

  // Timed callback frequency set to OSCC recommended publishing rate of 20 Hz (50 ms == 0.05 s)
  const float CALLBACK_FREQ_ = 0.05;  // Units in Seconds

  // OSCC input range
  const double BRAKE_MAX_ = 1;
  const double BRAKE_MIN_ = 0;
  const double THROTTLE_MAX_ = 1;
  const double THROTTLE_MIN_ = 0;
  const double STEERING_MAX_ = 1;
  const double STEERING_MIN_ = -1;
  const int BRAKE_AXIS_DIRECTION = -1;

  // Store last known value for timed callback
  double brake_ = 0.0;
  double throttle_ = 0.0;
  double steering_ = 0.0;
  bool enabled_ = false;

  // Smooth the steering to remove twitchy joystick movements
  const double DATA_SMOOTHING_FACTOR_ = 0.1;
  double steering_average_ = 0.0;

  // Variable to ensure joystick triggers have been initialized
  bool throttle_check_ = false;
  bool brake_check_ = false;

  bool initialized_ = false;

  // The threshold for considering the controller triggers to be parked in the correct position
  const double PARKED_THRESHOLD_ = 0.99;

//TODO: ros param <-> yaml
  const int BRAKE_AXES_ = 3;
  const int THROTTLE_AXES_ = 3;
  const int STEERING_AXES_ = 0;
  const int START_BUTTON_ = 9;
  const int BACK_BUTTON_ = 8;

  const double TRIGGER_MIN_ = 1;
  const double TRIGGER_MAX_ = -1;
  const double JOYSTICK_MIN_ = -1;
  const double JOYSTICK_MAX_ = 1;
};

/**
 * @brief ROSCCOTeleop class initializer
 *
 * This function constructs a class which subscribes to ROS Joystick messages, converts the inputs to ROSCCO relevant
 * values and publishes ROSCCO messages on a 20 Hz cadence.
 */
RosccoTeleop::RosccoTeleop()
{
  brake_pub_ = nh_.advertise<roscco::BrakeCommand>("brake_command", QUEUE_SIZE_);
  throttle_pub_ = nh_.advertise<roscco::ThrottleCommand>("throttle_command", QUEUE_SIZE_);
  steering_pub_ = nh_.advertise<roscco::SteeringAngleCommand>("steering_angle_command", QUEUE_SIZE_);
  enable_disable_pub_ = nh_.advertise<roscco::EnableDisable>("enable_disable", QUEUE_SIZE_);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", QUEUE_SIZE_, &RosccoTeleop::joystickCallback, this);
}

/**
 * @brief Callback function consume a joystick message and map values to ROSCCO ranges
 *
 * This function consumes a joystick message and maps the joystick inputs to ROSCCO ranges which are stored to private
 * class variables. Since the Joystick messages are published before all buttons are initialized this function also
 * validates that the button ranges are valid before consuming the date.
 *
 * @param joy The ROS Joystick message to be consumed.
 */
void RosccoTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{          brake_check_ = true;

  // gamepad triggers default 0 prior to using them which is 50% for the logitech and xbox controller the initilization
  // is to ensure the triggers have been pulled prior to enabling OSCC command
  if (initialized_)
  {
    // Map the trigger values [1, -1] to oscc values [0, 1]
    brake_ = map_and_clip(joy->axes[BRAKE_AXES_], JOYSTICK_MIN_, JOYSTICK_MAX_, BRAKE_MAX_, JOYSTICK_MIN_,BRAKE_MIN_);
    throttle_ = map_and_clip(joy->axes[THROTTLE_AXES_], JOYSTICK_MAX_, JOYSTICK_MIN_, THROTTLE_MAX_, JOYSTICK_MIN_,THROTTLE_MIN_);
    
    // Map the joystick to steering [1, -1] to oscc values [-1, 1]
    steering_ =
        linear_tranformation(joy->axes[STEERING_AXES_], JOYSTICK_MAX_, JOYSTICK_MIN_, STEERING_MIN_, STEERING_MAX_);
    
    ROS_DEBUG("Brake: %f    Throttle: %f    Steering: %f",brake_, throttle_, steering_);
    roscco::EnableDisable enable_msg;
    enable_msg.header.stamp = ros::Time::now();

    if ((previous_back_state_ == 0) && joy->buttons[BACK_BUTTON_])
    {
      enable_msg.enable_control = false;
      enable_disable_pub_.publish(enable_msg);
      if (enabled_){ROS_DEBUG("Disabled");}
      enabled_ = false;
      

    }
    else if ((previous_start_state_ == 0) && joy->buttons[START_BUTTON_])
    {
      enable_msg.enable_control = true;
      enable_disable_pub_.publish(enable_msg);
      if (!enabled_){ROS_DEBUG("Enabled");}
      enabled_ = true;

    }

    previous_back_state_ = joy->buttons[BACK_BUTTON_];
    previous_start_state_ = joy->buttons[START_BUTTON_];

    if (enabled_)
    {
      //TODO: everything into  class members here
      roscco::BrakeCommand brake_msg;
      brake_msg.header.stamp = ros::Time::now();
      brake_msg.brake_position = brake_;
      brake_pub_.publish(brake_msg);

      roscco::ThrottleCommand throttle_msg;
      throttle_msg.header.stamp = ros::Time::now();
      throttle_msg.throttle_position = throttle_;
      throttle_pub_.publish(throttle_msg);

      // Utilize exponential average similar to OSCC's joystick commander for smoothing of joystick twitchy output
      steering_average_ = calc_exponential_average(steering_average_, steering_, DATA_SMOOTHING_FACTOR_);

      roscco::SteeringAngleCommand steering_msg;
      steering_msg.header.stamp = ros::Time::now();
      steering_msg.steering_angle = steering_average_;
      steering_pub_.publish(steering_msg);
    }
  }
  else
  {
    // Ensure the trigger values have been initialized
    if (brake_check_ && throttle_check_) {
      initialized_ = true;
    }

    // throttle must be cleared beofre brake
    if ( throttle_check_ ) {
	if (((BRAKE_AXIS_DIRECTION * joy->axes[BRAKE_AXES_]) <= PARKED_THRESHOLD_) &&  (!(brake_check_)) ) {
        ROS_INFO("Pull the brake stick to initialize.");
      }
    	else{
          brake_check_ = true;
	  ROS_INFO("Brake OK");
      }
    } 
    else{
      if ((joy->axes[THROTTLE_AXES_] <= PARKED_THRESHOLD_) && (  !throttle_check_ ) ){
        ROS_INFO("Floor the throttle stick to initilize.");
        }
      else{
          throttle_check_ = true;
	  ROS_INFO("Throttle OK");

      }
    }
  }
}


/**
 * @brief Calculate the exponential average
 *
 * Calculates and returns a new exponential moving average (EMA) based on the new values.
 *
 * @param  AVERAGE  The current average value.
 * @param  SETPOINT The new datapoint to be included in the average.
 * @param  FACTOR   The coeffecient for smoothing rate, higher number yields faster discount of older values.
 * @return          The new exponential average value the includes the new setpoint.
 */
double calc_exponential_average(const double AVERAGE, const double SETPOINT, const double FACTOR)
{
  double exponential_average = (SETPOINT * FACTOR) + ((1.0 - FACTOR) * AVERAGE);

  return (exponential_average);
}

/**
 * @brief Remaps values from one linear range to another.
 *
 * Remap the value in an existing linear range to an new linear range example 0 in [-1, 1] to [0, 1] results in 0.5
 *
 * @param  VALUE  Data value to be remapped.
 * @param  HIGH_1 High value of the old range
 * @param  LOW_1  Low value of the old range
 * @param  HIGH_2 High value of the new range
 * @param  LOW_2  Low value of the new range
 * @return        Data value mapped to the new range
 */

double linear_tranformation(const double VALUE, const double HIGH_1, const double LOW_1, const double HIGH_2,
                            const double LOW_2)
{
  return LOW_2 + (VALUE - LOW_1) * (HIGH_2 - LOW_2) / (HIGH_1 - LOW_1);
}
/**
 * @brief Remaps values from one linear range to another, thenpulls the value velow threshold to 0.
 *
 * Remap the value in an existing linear range to an new linear range example 0 in [-1, 1] to [0, 1] results in 0.5
 *
 * @param  VALUE  Data value to be remapped.
 * @param  HIGH_1 High value of the old range
 * @param  LOW_1  Low value of the old range
 * @param  HIGH_2 High value of the new range
 * @param  LOW_2  Low value of the new range
 * @param  CUTOFF  cutoff threshold 
 * @return        Data value mapped to the new range
 */

double map_and_clip(const double VALUE, const double HIGH_1, const double LOW_1, const double HIGH_2,
                            const double LOW_2, const double CUTOFF)
{
  double val =linear_tranformation(VALUE, HIGH_1, LOW_1, HIGH_2, LOW_2);
  return val > CUTOFF ? val : CUTOFF; 
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "roscco_teleop");
  RosccoTeleop roscco_teleop;

  ros::spin();
}
