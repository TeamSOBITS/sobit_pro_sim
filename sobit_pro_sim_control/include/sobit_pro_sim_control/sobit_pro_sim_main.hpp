#ifndef SOBIT_PRO_MAIN_H_
#define SOBIT_PRO_MAIN_H_

#include <iostream>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>      // [SIM]
#include <trajectory_msgs/JointTrajectory.h> // [SIM]
#include <sensor_msgs/JointState.h>          // [Real robot]


class SobitProMain{
    private:
        int motion;
        
        void setPosJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ); // [SIM]
        void addPosJointTrajectory( const std::string& joint_name, const double rad, const double sec, trajectory_msgs::JointTrajectory* jt ); // [SIM]
        void checkPublishersConnection( const ros::Publisher& pub ); // [SIM]
        
        void callback(const geometry_msgs::Twist vel_twist);
        void joint_callback(const sensor_msgs::JointState joint_info); // [SIM]

        double getJointPos( const std::string& joint_name ); // [SIM]
        double getJointVel( const std::string& joint_name ); // [SIM]

        ros::NodeHandle nh;

        ros::Subscriber sub_vel          = nh.subscribe("mobile_base/commands/velocity", 1, &SobitProMain::callback, this);
        ros::Subscriber sub_joint_info   = nh.subscribe("joint_states", 1, &SobitProMain::joint_callback, this); // [SIM]
        ros::Publisher  pub_odometry     = nh.advertise<nav_msgs::Odometry>("odom", 1);
        // ros::Publisher  pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1); // [Real Robot]
        ros::Publisher  pub_steer_joint_ = nh.advertise<trajectory_msgs::JointTrajectory>("steer_trajectory_controller/command", 5); // [SIM]
        ros::Publisher  pub_wheel_joint_ = nh.advertise<std_msgs::Float64MultiArray>("wheel_trajectory_controller/command", 5);      // [SIM]
        ros::Publisher  pub_wheels_error = nh.advertise<std_msgs::Bool>("wheels_error", 1);
        // CHECK topic name
        // ros::Publisher  pub_wheel_joint_state_ = nh.advertise<sensor_msgs::JointState>("/joint_command", 1); // [ISAAC SIM] CHECK! - topic name

        int32_t wheel_fl_init_pos;
        int32_t wheel_fr_init_pos;
        int32_t wheel_bl_init_pos;
        int32_t wheel_br_init_pos;

        int32_t wheel_fl_curt_pos;
        int32_t wheel_fr_curt_pos;
        int32_t wheel_bl_curt_pos;
        int32_t wheel_br_curt_pos;

        int32_t steer_fl_curt_pos;
        int32_t steer_fr_curt_pos;
        int32_t steer_bl_curt_pos;
        int32_t steer_br_curt_pos;

        int32_t *set_steer_pos;
        int32_t *set_wheel_vel;

        int32_t prev_motion = -1; // When init avoid 0~3 motions

        // sensor_msgs::JointState joint_state; // [Real Robot]

        nav_msgs::Odometry result_odom;
        nav_msgs::Odometry prev_odom;

        std_msgs::Bool wheels_error;

    public:
        bool start_up_sound();
        bool shut_down_sound();
        void control_wheel();

        std::map<std::string, double> joints_pos; // [SIM]
        std::map<std::string, double> joints_vel; // [SIM]

};

// [SIM]
inline double SobitProMain::getJointPos( const std::string& joint_name ) {
    return joints_pos[joint_name];
}

// [SIM]
inline double SobitProMain::getJointVel( const std::string& joint_name ) {
    return joints_vel[joint_name];
}

// [SIM]
inline void SobitProMain::setPosJointTrajectory( const std::string& joint_name,
                                                 const double       rad,
                                                 const double       sec,
                                                 trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory      joint_trajectory;
    trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;

    joint_trajectory.joint_names.push_back( joint_name );
    joint_trajectory_point.positions.push_back( rad );
    // joint_trajectory_point.velocities.push_back( 0.0 );
    // joint_trajectory_point.accelerations.push_back( 0.0 );
    // joint_trajectory_point.effort.push_back( 0.0 );
    joint_trajectory_point.time_from_start = ros::Duration( sec );
    joint_trajectory.points.push_back( joint_trajectory_point );

    *jt = joint_trajectory;

    return;
}

// [SIM]
inline void SobitProMain::addPosJointTrajectory( const std::string& joint_name,
                                                 const double       rad,
                                                 const double       sec,
                                                 trajectory_msgs::JointTrajectory* jt ) {
    trajectory_msgs::JointTrajectory joint_trajectory = *jt;

    joint_trajectory.joint_names.push_back( joint_name );
    joint_trajectory.points[0].positions.push_back( rad );
    // joint_trajectory.points[0].velocities.push_back( 0.0 );
    // joint_trajectory.points[0].accelerations.push_back( 0.0 );
    // joint_trajectory.points[0].effort.push_back( 0.0 );
    joint_trajectory.points[0].time_from_start = ros::Duration( sec );

    *jt = joint_trajectory;

    return;
}

// [SIM]
inline void SobitProMain::checkPublishersConnection( const ros::Publisher& pub ) {
    ros::Rate loop_rate( 10 );

    while( pub.getNumSubscribers( ) == 0 && ros::ok( ) ){
        try{ loop_rate.sleep(); }
        catch( const std::exception& ex ){ break; }
    }

    return;
}

#endif // SOBIT_PRO_MAIN_H_
