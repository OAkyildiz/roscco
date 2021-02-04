// #include <geometry_msgs/Accel.h>
// #include <ros/ros.h>
// #include <roscco/BrakeCommand.h>
// #include <roscco/EnableDisable.h>
// #include <roscco/SteeringCommand.h>
// #include <roscco/ThrottleCommand.h>
// #include <roscco/CanFrame.h>

#include "roscco_myride_controller.h"
#include "ros/console.h"
MyRideOSCC::MyRideOSCC()
{
    enabled_ = false;
    
    //initialize PID state (call this at new distinct command as well)
    createPIDState( 0, steer_state);
    //dynamic_reconfigure these
    params->max = 1;
    params->min = -1;
    params->p_term = 1.5;
    params->i_term = 0.1;
    params->d_term = 0.06;
    params->i_max = 400;

    //TO OSCC NODE
    brake_pub = nh.advertise<roscco::BrakeCommand>( "brake_command", 1 );
    throttle_pub = nh.advertise<roscco::ThrottleCommand>( "throttle_command", 1 );
    steering_torque_pub = nh.advertise<roscco::SteeringCommand>( "steering_torque_command", 1 );
    steering_angle_pub = nh.advertise<roscco::SteeringAngleCommand>( "steering_angle_command", 1 );

    enable_disable_pub = nh.advertise<roscco::EnableDisable>("enable_disable", 1 );

   //FROM OBDII TO CONTROLLER
    //chassis_pub = nh.advertise<apollo::canbus::Chassis>( "/apollo/canbus/chassis", 1 );
    obd2_speed_pub = nh.advertise<std_msgs::Float32>("oscc/speed", 1);
    obd2_throttle_pub = nh.advertise<std_msgs::Int16>("oscc/throttle", 1);
    obd2_brake_pub = nh.advertise<std_msgs::Int16>("oscc/brake", 1);
    obd2_steer_angle_pub = nh.advertise<std_msgs::Int16>("oscc/steering", 1);
    obd2_steer_torque_pub = nh.advertise<std_msgs::Int16>("oscc/steering_torque", 1);

    //receive from move_base -> ackermann (?) . do we get anything from amcl?
    //command_sub = nh.subscribe("/command",&MyRideOSCC::commandCallback, this );  
    //ackermann_sub =nh.subscribe("/ackermann_command",&MyRideOSCC::ackermannCallback, this );  

    //FROM CONTROLLER: DESIRED STATES
    cmd_speed_sub = nh.subscribe( "/navigation/target/speed", 1, &MyRideOSCC::speedCmdCallback, this );
    cmd_steering_sub = nh.subscribe( "/navigation/target/steering", 1, &MyRideOSCC::steeringCmdCallback, this );  
    cmd_steering_pid_sub = nh.subscribe( "/navigation/target/steering_pid", 1, &MyRideOSCC::steeringCmdPIDCallback, this );  

    //This is direct control primarily we are concerned with the desired speed

    // but accel, brake could be utilized for more advanced motion profiles

    cmd_brake_sub = nh.subscribe( "/navigation/target/brake", 1, &MyRideOSCC::brakeCmdCallback, this );
    cmd_throttle_sub = nh.subscribe( "/navigation/target/throttle", 1, &MyRideOSCC::throttleCmdCallback, this );

    //FROM OSCC: REPORT FRAMES, ERRORS, OBDII (to parse)
    can_frame_sub = nh.subscribe( "/can_frame", 1, &MyRideOSCC::canFrameCallback, this );
    oscc_state_sub = nh.subscribe( "/oscc_state", 1, &MyRideOSCC::OSCCStateCallback, this );

}
// This is control loop where speed and steering commands are received once

//deprecated
bool MyRideOSCC::controlLoop(){
    // Copy speedCmdCallback here with v_d replaced with target_speed
    // append steering control loop
    return false;
}
// void commandCallback( const std_msgs::String::ConstPtr& input ){

// //received "start" set speed to 15.0
// //received "stop" desired speed=0, throttle off
// //received "park" for now: do stop.

//     string cmd = msg.data;
//     ROS_INFO("Received %s", cmd.c_str());

// }

// void ackermannCallback( const ackermann_msgs::AckermannDriveStamped& input ){

// }

void MyRideOSCC::speedCmdCallback( const std_msgs::Float32::ConstPtr& input ){

    //double v_d=input->data; //desired_speed
    //double speed_error= v_d - speed_report;
    v_d=input->data;//TODO: refactor
}
//void myRideOSCC::yawCallback()
//void myRideOSCC::trunCallback()
void MyRideOSCC::steeringCmdPIDCallback( const std_msgs::Float32::ConstPtr& input ) 
{

    bool new_state= abs(target_steering - prev_target_steering) > STEERING_STATE_TOLERANCE;
    target_steering = input->data;
    if(enabled_){

        if(new_state){
            createPIDState( target_steering, steer_state);
        }
        else{
            steer_state->setpoint = target_steering;
        }
        roscco::SteeringCommand output;
        output.header.stamp = ros::Time::now();
        //closedLoopControl( input.steering_target(), output, steering_angle_report );
        output.steering_torque=pidController( params, steer_state, steering_angle_report );
        //P( target_steering, output, steering_angle_report );
        ROS_INFO("      [CMD] Steering: %f", output.steering_torque);
        steering_angle_pub.publish( output );
        

    }

    prev_target_steering = target_steering;
}
void MyRideOSCC::steeringCmdCallback( const std_msgs::Float32::ConstPtr& input ) 
{
    //TODO: consider hnadling command here ( setpoint, counter, conversion etc. and publishing in main loop)
    
    bool new_state= abs(target_steering - prev_target_steering) > STEERING_STATE_TOLERANCE;
    target_steering = input->data;

 
    roscco::SteeringAngleCommand output;

    output.header.stamp = ros::Time::now();

    output.steering_angle = input->data; // raw data or angle2int or angle2[-1.0, 1.0]
    steering_angle_pub.publish( output );

    

    prev_target_steering = target_steering;
}

//Following two directly dictate throttle and brake values. Not used in this example

void MyRideOSCC::brakeCmdCallback( const std_msgs::Int16::ConstPtr& input ) 
{
    roscco::BrakeCommand output;

    output.header.stamp = ros::Time::now();
   // output.brake_position = input.input.data () / 100;
    output.brake_position = input->data / 100;
    brake_pub.publish( output );
}


void MyRideOSCC::throttleCmdCallback( const std_msgs::Int16::ConstPtr& input ) 
{
    roscco::ThrottleCommand output;

    output.header.stamp = ros::Time::now();
    output.throttle_position = input->data / 100;

    throttle_pub.publish( output );
}
 


void MyRideOSCC::canFrameCallback( const roscco::CanFrame& input ) 
{
    static bool first = true;
    static double throttle_raw = 0;
    static double brake_raw = 0;
    static double steering_angle_raw = 0;
    static double steering_torque_raw = 0;
    static std_msgs::Int16 int_msg;
    static std_msgs::Float32 float_msg;

    //TODO: try this
    // static bool first = true;
    // double throttle_raw = 0;
    // double brake_raw = 0;
    // double steering_angle_raw = 0;
    // double steering_torque_raw = 0;
    // std_msgs::Int16 int_msg;
    // std_msgs::Float32 float_msg;
    switch( input.frame.can_id )
    {
        //TODO: * check rate
        //      * enable flag
        // make it a mutex lock
        case KIA_SOUL_OBD_THROTTLE_PRESSURE_CAN_ID: 
        {
            #if defined( KIA_SOUL_EV )
                throttle_raw = input.frame.data[4];
            #elif defined( KIA_NIRO )
                throttle_raw = input.frame.data[7];
            #endif
            //if (processed_last_)
            throttle_report = throttle_raw * THROTTLE_RATIO;
            // throttle_report = throttle_report ;
            break;
        }
        case KIA_SOUL_OBD_BRAKE_PRESSURE_CAN_ID: 
        {
            #if defined( KIA_SOUL_EV )
                brake_raw = input.frame.data[4] + input.frame.data[5] * 256;
            #elif defined( KIA_NIRO )
                brake_raw = input.frame.data[3] + input.frame.data[4] * 256;
            #endif
            //if (processed_last_)
            brake_report = brake_raw * BRAKE_RATIO;
            // brake_report = brake_report;
            break;
        }
        case KIA_SOUL_OBD_STEERING_WHEEL_ANGLE_CAN_ID: 
        {
            steering_angle_raw = input.frame.data[0] + input.frame.data[1] * 256;
            steering_torque_raw = input.frame.data[2] ;
            if( steering_angle_raw > 27768 ) 
                steering_angle_raw -= 65535;

            if (steering_angle_raw_prev>steering_angle_raw)
                steering_torque_raw=-steering_torque_raw;
            //if (processed_last_)
            steering_angle_report = steering_angle_raw / STEERING_MAX;
            // steering_angle_report = steering_angle_report;
            break;
        }
        case KIA_SOUL_OBD_SPEED_CAN_ID:
        {
            #if defined( KIA_SOUL_EV )
                speed_report = input.frame.data[1];

                speed_report = speed_report * SPEED_RATIO;

            #elif defined( KIA_NIRO )
                speed_report = input.frame.data[0];

                speed_report = speed_report * SPEED_RATIO;
            
            #endif
            //where to update speed
            // speed_error = v_d-speed_report;

            break;
        }
        default :
        {
            /// Empty default to leave no untested cases
            break;
        }
    }
    // instead our own individual pubs. consider making a chassis-like message
    //apollo::canbus::Chassis output;

    /// THIS CAN GO INTO THE MAIN_LOOP/ 2nd thread
    
    /*
    // output.set_steering_percentage( steering_angle_report );
    // output.set_throttle_percentage( throttle_report );
    // output.set_brake_percentage( brake_report );
    // output.set_speed_mps( speed_report );
    */


    float_msg.data=speed_report*1;
    obd2_speed_pub.publish(float_msg);
    
    //int_msg.data = int(100*throttle_report); // left this here for reference, do percenteage conv later
   
    int_msg.data = int(throttle_raw);
    obd2_throttle_pub.publish(int_msg);

    int_msg.data = int(brake_raw);
    obd2_brake_pub.publish(int_msg);

    int_msg.data = int(steering_angle_raw);
    obd2_steer_angle_pub.publish(int_msg);

    int_msg.data = int(steering_torque_raw);
    obd2_steer_torque_pub.publish(int_msg);
    //ROS_INFO("[REP] Throttle: %f  Brake: %f Steering: %f ", throttle_report, brake_report, steering_angle_report);

   // chassis_pub.publish( output );
}
//TODO Class this: AND/OR Make params struct a member

void MyRideOSCC::OSCCStateCallback(const std_msgs::Bool::ConstPtr& state){
    
    enabled_ = state->data;
    ROS_INFO("OSCC: %s", enabled_ ? "Enabled" : " Disabled");
}
 

bool MyRideOSCC::pubEnableMsg(){
    roscco::EnableDisable enable_msg;
    enable_msg.header.stamp = ros::Time::now();
    enable_msg.enable_control = true;
    enable_disable_pub.publish(enable_msg);
    ROS_DEBUG("Enable message sent");
    return true;
    }

bool MyRideOSCC::pubDisableMsg(){
    roscco::EnableDisable disable_msg;
    disable_msg.header.stamp = ros::Time::now();
    disable_msg.enable_control = false;
    enable_disable_pub.publish(disable_msg);
    ROS_DEBUG("Disable message sent");
    return false;
    }

bool MyRideOSCC::loop_once(){
    static roscco::ThrottleCommand throttle_msg;
    static roscco::BrakeCommand brake_msg;   
    // TODO: make switch case
    static bool first;  
    if (!enabled_ && first){
        first=false;
        ROS_INFO("Initial brake");
        //roscco::BrakeCommand brake_msg;   
        enabled_ = pubEnableMsg();
        brake_msg.brake_position = BRAKE_LIMIT;
        
        brake_msg.header.stamp = ros::Time::now(); ;
        brake_pub.publish(brake_msg);
    }
    // convert to int math
    if (enabled_ ){
        speed_error = v_d - speed_report;

        if(v_d < 0){
        ROS_INFO("CB Park");
        throttle_msg.throttle_position = 0;
        brake_msg.brake_position = BRAKE_LIMIT;
        enabled_ = pubDisableMsg();
        }
        
        else if (v_d == 0){ 
            //Temporary!
            ROS_INFO("Brake %f", brake_report);

            throttle_msg.throttle_position = 0;
            
            brake_msg.brake_position = brake_report + BRAKE_INCREMENT;
            if(brake_report < 0.05)
                brake_msg.brake_position = 0.1 + BRAKE_INCREMENT;
            if(brake_report> BRAKE_LIMIT)
                brake_msg.brake_position = BRAKE_LIMIT;
            }
            
            
        else if (speed_error < -1 *SPEED_PANIC){
            //TODO: implement throttle memory
            throttle_msg.throttle_position = 0;
            brake_msg.brake_position = BRAKE_INCREMENT + brake_report;//*(1+ BRAKE_INCREMENT);
            if(brake_report> BRAKE_LIMIT_SOFT)
                brake_msg.brake_position = BRAKE_LIMIT_SOFT;
            ROS_INFO("Soft Brake");
        }
            
         else if ( abs(speed_error) > SPEED_TOLERANCE ){ //if (speed_error)
          
            throttle_msg.throttle_position = throttle_report + speed_error*THROTTLE_INCREMENT;
            
           // if(throttle_report < 0.5)
           //     throttle_msg.throttle_position = 0.1 + THROTTLE_INCREMENT;
            if(throttle_report> THROTTLE_LIMIT)
                throttle_msg.throttle_position = THROTTLE_LIMIT;
            //throttle_msg.throttle_position = throttle_report > THROTTLE_LIMIT ? throttle_report + 0.05 *speed_error : THROTTLE_LIMIT; 
            ROS_INFO("Throttle: adjust %f", throttle_report);
            brake_msg.brake_position = 0;
        }

    else{
            ROS_INFO("Throttle: maintain");
            throttle_msg.throttle_position = throttle_report;
            brake_msg.brake_position = 0;
    }

   
    
    
    throttle_msg.header.stamp = ros::Time::now();
    brake_msg.header.stamp = throttle_msg.header.stamp ;

    brake_pub.publish(brake_msg);
    throttle_pub.publish(throttle_msg);
    ROS_INFO("      [CMD]Throttle: %f  Brake: %f E_sp: %f",throttle_msg.throttle_position, brake_msg.brake_position, speed_error);
    //for now steering is outside
    }
        
    else if (!enabled_ && (v_d == 0)){
        ROS_INFO("Enable");
        throttle_msg.throttle_position = 0;
        enabled_ = pubEnableMsg();
        brake_msg.brake_position = BRAKE_LIMIT;
        }

    return true;
}

// void closedLoopControl( double setpoint, 
//                         roscco::SteeringCommand& output,
//                         double steering_angle_report ) //Position must be [-1,1]
// {
//     output.steering_torque = pidController( &params, &state, steering_angle_report );
// }


int main( int argc, char** argv )
{
    ros::init( argc, argv, "roscco_myride" );
    MyRideOSCC *roscco_myride = new MyRideOSCC();
    ros::Rate r(40); // 10 hz

    //#if defined( M_THREAD )

    //ros::MultiThreadedSpinner spinner(2);
    //ros::AsyncSpinner spinner(2);

    while (ros::ok()){
        
            roscco_myride->loop_once();
            ros::spinOnce();
            //r.spinOnce()
            r.sleep();
        }
   // ROS_INFO("Exiting")
   // roscco_myride.pubDisableMsg();


}
