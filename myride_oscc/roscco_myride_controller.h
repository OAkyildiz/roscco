#include <ros/ros.h>

//to_OSCC
#include <roscco/BrakeCommand.h>
#include <roscco/SteeringCommand.h>
#include <roscco/SteeringAngleCommand.h>
#include <roscco/ThrottleCommand.h>
#include <roscco/EnableDisable.h>


// FROM OSCC
#include <roscco/CanFrame.h>

//#include <modules/control/proto/control_cmd.pb.h>


#include <roscco/pid_control.h>

//Send OBDII data to ROS (also target speed & steering)
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//From ROS
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <string>
#include <math.h>
#include <vehicles.h>

//rosparam these
<<<<<<< HEAD

//#define THROTTLE_RATIO 0.393
//#define STEERING_RATIO 0.018
#define THROTTLE_RATIO 0.00393 //  ~1/255
#define STEERING_RATIO 0.00019 //~1/5200
=======
>>>>>>> 6763d7ee1f21133ea460e4c1e1fa016a336364e3

//#define THROTTLE_RATIO 0.393
//#define STEERING_RATIO 0.018
#define THROTTLE_RATIO 0.00393 //  ~1/255
#define THROTTLE_LIMIT 0.4
<<<<<<< HEAD
#define BRAKE_LIMIT 0.35
#define BRAKE_LIMIT_SOFT 0.02

#define THROTTLE_INCREMENT THROTTLE_RATIO *2

=======
#define THROTTLE_INCREMENT THROTTLE_RATIO *2

#define BRAKE_LIMIT 0.35
#define BRAKE_LIMIT_SOFT 0.02

#define STEERING_RATIO 0.00019 //~1/5200
#define STEERING_STATE_TOLERANCE 0.15
>>>>>>> 6763d7ee1f21133ea460e4c1e1fa016a336364e3

#define SPEED_TOLERANCE 0.5
#define SPEED_PANIC 3.5 //if the speed error is bigger than this, soft brake

#define KM_TO_M 0.621371

#if defined( KIA_SOUL_EV )
    #define BRAKE_RATIO 0.00115
    #define SPEED_RATIO 0.5
#elif defined( KIA_NIRO )
    #define BRAKE_RATIO 0.033
    #define SPEED_RATIO 0.3
#endif

#define BRAKE_INCREMENT BRAKE_RATIO*20

class MyRideOSCC
{
public:

    /**
     * @brief MyRideOSCC class initializer
     *
     * This function construct a class which subscribes to Apollo messages and publishes ROSCCO messages
     */
    MyRideOSCC();
    bool loop_once();


private:

    bool controlLoop();
   // void commandCallback( const std_msgs::String&::ConstPtr input ); 
   // void ackermannCallback( const ackermann_msgs::AckermannDriveStamped&::ConstPtr input ); 
    void speedCmdCallback( const std_msgs::Float32::ConstPtr& input ); 

    
    /**
     * @brief Callback function to pipe Apollo SteeringCommand to ROSCCO
     *
     * @param apollo control command message to be consumed
     */
    void steeringCmdCallback( const std_msgs::Float32::ConstPtr& input ); 
    
    /**
     * @brief Callback function to pipe Apollo BrakeCommand to ROSCCO
     *
     * @param apollo control command message to be consumed
     */
    void brakeCmdCallback( const std_msgs::Int16::ConstPtr& input );
    
    /**
     * @brief Callback function to pipe Apollo ThrottleCommand to ROSCCO
     *
     * @param apollo control command message to be consumed
     */
    void throttleCmdCallback( const std_msgs::Int16::ConstPtr& input );
    
    /**
     * @brief Soul EV Callback function to save and publish chassis status
     *
     * @param roscco can frame message to be consumed
     */
    void canFrameCallback( const roscco::CanFrame& input );

    bool pubEnableMsg();
    bool pubDisableMsg();
    
    void OSCCStateCallback(const std_msgs::Bool::ConstPtr& state);

    ros::NodeHandle nh;

    ros::Publisher throttle_pub;
    ros::Publisher brake_pub;
    ros::Publisher steering_torque_pub;
    ros::Publisher steering_angle_pub;

    //ros::Publisher chassis_pub;
    //publishers for data from processed can_frames
    ros::Publisher  obd2_speed_pub;
    ros::Publisher  obd2_throttle_pub;
    ros::Publisher  obd2_brake_pub;
    ros::Publisher  obd2_steer_angle_pub;
    ros::Publisher  obd2_steer_torque_pub;
    ros::Publisher  enable_disable_pub;
   
   // Incoming from state controllers
   // ros::Subscriber ackermann_sub;
    ros::Subscriber cmd_speed_sub;
    ros::Subscriber cmd_steering_sub;
    ros::Subscriber cmd_throttle_sub;
    ros::Subscriber cmd_brake_sub;
    
    ros::Subscriber can_frame_sub;
    ros::Subscriber oscc_state_sub;
    ros::Subscriber localization_sub;

    double steering_angle_report = 0;
    double steering_angle_raw_prev = 0;
    double steering_torque_report = 0;

    double throttle_report = 0;
    double brake_report = 0;
    double speed_report = 0;
    
    double target_speed = 0;
    double target_steering = 0;
    double prev_target_steering=0;

    double v_d=0;
<<<<<<< HEAD
    double error=0;
=======
    double speed_error=0;
>>>>>>> 6763d7ee1f21133ea460e4c1e1fa016a336364e3
 
    bool enabled_=false;
    bool state_; // placeholder flag, future enum
    bool processed_last_=true;
<<<<<<< HEAD
=======

    pid_terms* params;
    pid_state* state;

>>>>>>> 6763d7ee1f21133ea460e4c1e1fa016a336364e3
};


/**
 * @brief setup a pid to control the steering angle
 * 
 * @param setpoint/target
 * @param command
 * @param steering angle position
 */
//  void closedLoopControl( double setpoint,
//                         roscco::SteeringCommand& output,
//                         double steering_angle_report );
