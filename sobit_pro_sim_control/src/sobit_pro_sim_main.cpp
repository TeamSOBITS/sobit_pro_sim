#include "sobit_pro_sim_control/sobit_pro_sim_main.hpp"
#include "sobit_pro_sim_control/sobit_pro_sim_control.hpp"
#include "sobit_pro_sim_control/sobit_pro_sim_motor_driver.hpp" // [Real Robot]
#include "sobit_pro_sim_control/sobit_pro_sim_odometry.hpp"

// Create the instance
SobitProControl     sobit_pro_control;
SobitProSimMotorDriver sobit_pro_motor_driver; // [Real Robot]
SobitProOdometry    sobit_pro_odometry;

// Twist callback
void SobitProMain::callback(const geometry_msgs::Twist vel_twist){
    // Translational
    if( ((fabsf(vel_twist.linear.x) > 0.000) || (fabsf(vel_twist.linear.y) > 0.000)) && (fabsf(vel_twist.angular.z) <= 0.001) ){
        // ROS_INFO("Translational motion.");
        motion = SobitProControl::TRANSLATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Rotational
    else if( ((fabsf(vel_twist.linear.x) <= 0.001) && (fabsf(vel_twist.linear.y) <= 0.001)) && (fabsf(vel_twist.angular.z) > 0.000) ){
        // ROS_INFO("Rotational motion.");
        motion = SobitProControl::ROTATIONAL_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Swivel
    else if( ((fabsf(vel_twist.linear.x) > 0.000) || (fabsf(vel_twist.linear.y) > 0.000)) && (fabsf(vel_twist.angular.z) > 0.000) ){
        if( fabsf(2.0*(sqrtf(powf(vel_twist.linear.x,2.) + powf(vel_twist.linear.y,2.)))) > fabsf(vel_twist.angular.z*SobitProControl::TRACK) ){
            // ROS_INFO("Swivel motion.");
            motion = SobitProControl::SWIVEL_MOTION;

        }else{
            // ROS_INFO("Swivel motion to Rotational motion.");
            motion = SobitProControl::ROTATIONAL_MOTION;
        }
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }
    // Stop motion
    else{
        // ROS_INFO("Stop motion.");
        motion = SobitProControl::STOP_MOTION;
        wheels_error.data = false;
        pub_wheels_error.publish(wheels_error);
    }

    sobit_pro_control.getMotion(motion);
    sobit_pro_odometry.getMotion(motion);
    sobit_pro_control.setParams(vel_twist);

    return;
}

// [SIM] Obtain the JointState of each joint
void SobitProMain::joint_callback(const sensor_msgs::JointState joint_info){
    // Update the map of joint positions
    for (size_t i = 0; i < joint_info.name.size(); ++i) {
        joints_pos[joint_info.name[i]] = joint_info.position[i];
        joints_vel[joint_info.name[i]] = joint_info.velocity[i];
    }
}

// Start up sound
bool SobitProMain::start_up_sound(){
    bool is_sound  = false;

    // Generate a random number
    std::random_device rnd;
    std::mt19937 gen(rnd());
    std::uniform_int_distribution<int> distribution(1, 100);
    int rand_sound = distribution(gen);
    int sound_param;
    nh.param("/sound_param", sound_param, 75);
    
    std::string sound      = rand_sound <= sound_param ? "start_up" : "soka_univ_gakuseika";
    std::string pack_path  = ros::package::getPath("sobit_pro_sim_control");
    std::string sound_path = pack_path + "/mp3/" + sound + ".mp3";

    std::cout << std::endl;
    std::cout << "rand_sound: " << rand_sound << std::endl;
    std::cout << "sound_param: " << sound_param << std::endl;
    std::cout << "Start Up: " << sound << ".mp3" << std::endl;
    std::cout << std::endl;

    is_sound = std::system(("mpg321 --quiet "+ sound_path).c_str());
    ros::Duration(2.).sleep();

    if( is_sound ) ROS_ERROR("There was an error reproducing the start up sound.");


    return is_sound;
}

// Shut down sound
bool SobitProMain::shut_down_sound(){
    bool is_sound  = false;
    std::string package_path = ros::package::getPath("sobit_pro_sim_control");
    std::string sound_path   = package_path + "/mp3/" + "shut_down.mp3";

    std::cout << std::endl;
    std::cout << "Shutdown Sound" << std::endl;

    is_sound = std::system(("mpg321 --quiet "+ sound_path).c_str());
    ros::Duration(2.).sleep();

    if( is_sound ) ROS_ERROR("There was an error reproducing the shutdown sound.");
    

    return is_sound;
}

// Control wheel
void SobitProMain::control_wheel(){
    // [SIM] Wait for the joint_states to be published
    do{
        ros::spinOnce();
    } while(!joints_pos.size() and !joints_vel.size());

    // [Real Robot] Set the initial position of the wheel
    // wheel_fl_init_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_F_L);
    // wheel_fr_init_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_F_R);
    // wheel_bl_init_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_B_L);
    // wheel_br_init_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_B_R);

    // [SIM] Set the initial position of the wheel
    wheel_fl_init_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint") * 1024. / (M_PI/2.) + 2048.;
    wheel_fr_init_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint") * 1024. / (M_PI/2.) + 2048.;
    wheel_bl_init_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint") * 1024. / (M_PI/2.) + 2048.;
    wheel_br_init_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint") * 1024. / (M_PI/2.) + 2048.;


    // Get the robot namespace
    std::string robot_name = (ros::this_node::getNamespace() != "/")
                            ? ros::this_node::getNamespace().substr(1) + "/"
                            : "";
    // Initilize Odometry
    prev_odom.header.stamp            = ros::Time::now();
    prev_odom.header.frame_id         = robot_name + "odom";
    prev_odom.child_frame_id          = robot_name + "base_footprint";
    prev_odom.pose.pose.position.x    = 0.0;
    prev_odom.pose.pose.position.y    = 0.0;
    prev_odom.pose.pose.position.z    = 0.0;
    prev_odom.pose.pose.orientation.x = 0.0;
    prev_odom.pose.pose.orientation.y = 0.0;
    prev_odom.pose.pose.orientation.z = 0.0;
    prev_odom.pose.pose.orientation.w = 1.0;
    prev_odom.twist.twist.linear.x    = 0.0;
    prev_odom.twist.twist.linear.y    = 0.0;
    prev_odom.twist.twist.linear.z    = 0.0;
    prev_odom.twist.twist.angular.x   = 0.0;
    prev_odom.twist.twist.angular.y   = 0.0;
    prev_odom.twist.twist.angular.z   = 0.0;

    result_odom.header.stamp            = ros::Time::now();
    result_odom.header.frame_id         = robot_name + "odom";
    result_odom.child_frame_id          = robot_name + "base_footprint";
    result_odom.pose.pose.position.x    = 0.0;
    result_odom.pose.pose.position.y    = 0.0;
    result_odom.pose.pose.position.z    = 0.0;
    result_odom.pose.pose.orientation.x = 0.0;
    result_odom.pose.pose.orientation.y = 0.0;
    result_odom.pose.pose.orientation.z = 0.0;
    result_odom.pose.pose.orientation.w = 1.0;
    result_odom.twist.twist.linear.x    = 0.0;
    result_odom.twist.twist.linear.y    = 0.0;
    result_odom.twist.twist.linear.z    = 0.0;
    result_odom.twist.twist.angular.x   = 0.0;
    result_odom.twist.twist.angular.y   = 0.0;
    result_odom.twist.twist.angular.z   = 0.0;

    ros::Rate rate(50);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    trajectory_msgs::JointTrajectory steer_joint_trajectory; // [SIM]
    std_msgs::Float64MultiArray      wheel_joint_vel;        // [SIM]

    while( ros::ok() ){
        // Set goal steer position value based on initial position (2048) 
        set_steer_pos = sobit_pro_control.setSteerPos();

        // [Real Robot] Get current steer position value
        // steer_fl_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_F_L);
        // steer_fr_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_F_R);
        // steer_bl_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_B_L);
        // steer_br_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_B_R);

        // [SIM] Get current steer position value
        steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint") * 1024. / (M_PI/2.) + 2048.;
        steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint") * 1024. / (M_PI/2.) + 2048.;
        steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint") * 1024. / (M_PI/2.) + 2048.;
        steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint") * 1024. / (M_PI/2.) + 2048.;

        // When steer changing angle is large(>90[deg]), stop the wheels to avoid hardware damage
        if( (1024 <= fabsf(set_steer_pos[0] - steer_fl_curt_pos))
         || (1024 <= fabsf(set_steer_pos[1] - steer_fr_curt_pos))
         || (1024 <= fabsf(set_steer_pos[2] - steer_bl_curt_pos))
         || (1024 <= fabsf(set_steer_pos[3] - steer_br_curt_pos)) ){
            ROS_INFO("Changing the direction of the wheel");
            set_wheel_vel[0] = set_wheel_vel[1] = set_wheel_vel[2] = set_wheel_vel[3] = 0.;
            // [Real Robot] Write goal wheel velocity value [Dynamixel]
            // sobit_pro_motor_driver.controlWheelsVel(set_wheel_vel);

            // [SIM] Publish Float64MultiArray [rad/s]
            wheel_joint_vel.data.clear();
            wheel_joint_vel.data.push_back(set_wheel_vel[0] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[1] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[2] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            wheel_joint_vel.data.push_back(set_wheel_vel[3] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
            checkPublishersConnection( pub_wheel_joint_ );
            pub_wheel_joint_.publish(wheel_joint_vel);

            ros::Duration(0.5).sleep();
        }

        // [Real Robot] Write goal steer position value [Dynamixel]
        // sobit_pro_motor_driver.controlSteersPos(set_steer_pos);

        // [SIM] Publish JointTrajectory
        steer_joint_trajectory.joint_names.clear();
        steer_joint_trajectory.points.clear();
        setPosJointTrajectory( "wheel_f_l_steer_joint", (set_steer_pos[0] - 2048.) * (M_PI/2. / 1024.), 0.5, &steer_joint_trajectory );
        addPosJointTrajectory( "wheel_f_r_steer_joint", (set_steer_pos[1] - 2048.) * (M_PI/2. / 1024.), 0.5, &steer_joint_trajectory );
        addPosJointTrajectory( "wheel_b_l_steer_joint", (set_steer_pos[2] - 2048.) * (M_PI/2. / 1024.), 0.5, &steer_joint_trajectory );
        addPosJointTrajectory( "wheel_b_r_steer_joint", (set_steer_pos[3] - 2048.) * (M_PI/2. / 1024.), 0.5, &steer_joint_trajectory );
        checkPublishersConnection( pub_steer_joint_ );
        pub_steer_joint_.publish(steer_joint_trajectory);

        // Update current steer position and wait until steer goal is reached
        do{
            // [Real Robot] Get current steer position value
            // steer_fl_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_F_L);
            // steer_fr_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_F_R);
            // steer_bl_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_B_L);
            // steer_br_curt_pos = sobit_pro_motor_driver.feedbackSteerPos(SobitProSimMotorDriver::STEER_B_R);

            // [SIM] Get current steer position value [rad]
            steer_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_steer_joint") * 1024. / (M_PI/2.) + 2048.;
            steer_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_steer_joint") * 1024. / (M_PI/2.) + 2048.;
            steer_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_steer_joint") * 1024. / (M_PI/2.) + 2048.;
            steer_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_steer_joint") * 1024. / (M_PI/2.) + 2048.;
        }while( (SobitProSimMotorDriver::DXL_MOVING_STATUS_THRESHOLD < fabsf(set_steer_pos[0] - steer_fl_curt_pos)) 
             && (SobitProSimMotorDriver::DXL_MOVING_STATUS_THRESHOLD < fabsf(set_steer_pos[1] - steer_fr_curt_pos))
             && (SobitProSimMotorDriver::DXL_MOVING_STATUS_THRESHOLD < fabsf(set_steer_pos[2] - steer_bl_curt_pos))
             && (SobitProSimMotorDriver::DXL_MOVING_STATUS_THRESHOLD < fabsf(set_steer_pos[3] - steer_br_curt_pos)) );

        // Set goal wheel velocity value
        set_wheel_vel = sobit_pro_control.setWheelVel();

        // [Real Robot] Write goal wheel velocity value
        // sobit_pro_motor_driver.controlWheelsVel(set_wheel_vel);


        // [Real Robot] Publish JointState
        // joint_state.header.stamp = ros::Time::now();

        // joint_state.name.resize(8);
        // joint_state.name[0] = "wheel_f_l_steer_joint";
        // joint_state.name[1] = "wheel_f_r_steer_joint";
        // joint_state.name[2] = "wheel_b_l_steer_joint";
        // joint_state.name[3] = "wheel_b_r_steer_joint";

        // joint_state.name[4] = "wheel_f_l_drive_joint";
        // joint_state.name[5] = "wheel_f_r_drive_joint";
        // joint_state.name[6] = "wheel_b_l_drive_joint";
        // joint_state.name[7] = "wheel_b_r_drive_joint";

        // joint_state.position.resize(8);
        // joint_state.position[0] = (set_steer_pos[0] - 2048.) * (M_PI/2. / 1024.); // Convert 2048. to 0.[rad]
        // joint_state.position[1] = (set_steer_pos[1] - 2048.) * (M_PI/2. / 1024.); // Convert 2048. to 0.[rad]
        // joint_state.position[2] = (set_steer_pos[2] - 2048.) * (M_PI/2. / 1024.); // Convert 2048. to 0.[rad]
        // joint_state.position[3] = (set_steer_pos[3] - 2048.) * (M_PI/2. / 1024.); // Convert 2048. to 0.[rad]

        // joint_state.position[4] = set_wheel_vel[0] * SobitProControl::VEL_UNIT * (M_PI*SobitProControl::WHEEL_DIAMETER) / 60.; // Convert rpm to m/s
        // joint_state.position[5] = set_wheel_vel[1] * SobitProControl::VEL_UNIT * (M_PI*SobitProControl::WHEEL_DIAMETER) / 60.; // Convert rpm to m/s
        // joint_state.position[6] = set_wheel_vel[2] * SobitProControl::VEL_UNIT * (M_PI*SobitProControl::WHEEL_DIAMETER) / 60.; // Convert rpm to m/s
        // joint_state.position[7] = set_wheel_vel[3] * SobitProControl::VEL_UNIT * (M_PI*SobitProControl::WHEEL_DIAMETER) / 60.; // Convert rpm to m/s

        // [Real Robot] Publish JointState
        // pub_joint_states.publish(joint_state);

        // [SIM] Publish Float64MultiArray [rad/s]
        wheel_joint_vel.data.clear();
        wheel_joint_vel.data.push_back(set_wheel_vel[0] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[1] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[2] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        wheel_joint_vel.data.push_back(set_wheel_vel[3] * SobitProControl::VEL_UNIT * (2.*M_PI/60.));
        checkPublishersConnection( pub_wheel_joint_ );
        pub_wheel_joint_.publish(wheel_joint_vel);


        // [Real Robot] Update the current wheel position
        // wheel_fl_curt_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_F_L);
        // wheel_fr_curt_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_F_R);
        // wheel_bl_curt_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_B_L);
        // wheel_br_curt_pos = sobit_pro_motor_driver.feedbackWheelPos(SobitProSimMotorDriver::WHEEL_B_R);

        // [SIM] Update the current wheel position
        wheel_fl_curt_pos = SobitProMain::getJointPos("wheel_f_l_drive_joint") * 1024. / (M_PI/2.) + 2048.;
        wheel_fr_curt_pos = SobitProMain::getJointPos("wheel_f_r_drive_joint") * 1024. / (M_PI/2.) + 2048.;
        wheel_bl_curt_pos = SobitProMain::getJointPos("wheel_b_l_drive_joint") * 1024. / (M_PI/2.) + 2048.;
        wheel_br_curt_pos = SobitProMain::getJointPos("wheel_b_r_drive_joint") * 1024. / (M_PI/2.) + 2048.;


        // Calculate Odometry based on motion mode (check!)
        sobit_pro_odometry.odom(steer_fl_curt_pos, steer_fr_curt_pos,
                                steer_bl_curt_pos, steer_br_curt_pos,
                                wheel_fl_curt_pos, wheel_fr_curt_pos,
                                wheel_bl_curt_pos, wheel_br_curt_pos,
                                wheel_fl_init_pos, wheel_fr_init_pos,
                                wheel_bl_init_pos, wheel_br_init_pos,
                                prev_motion,
                                prev_odom, &result_odom,
                                prev_odom.header.stamp);

        // Update the initial wheel position value for next loop calculation
        wheel_fl_init_pos = wheel_fl_curt_pos;
        wheel_fr_init_pos = wheel_fr_curt_pos;
        wheel_bl_init_pos = wheel_bl_curt_pos;
        wheel_br_init_pos = wheel_br_curt_pos;

        // Update odom for next loop calculation
        prev_odom.header.stamp            = result_odom.header.stamp;
        prev_odom.pose.pose.position.x    = result_odom.pose.pose.position.x;
        prev_odom.pose.pose.position.y    = result_odom.pose.pose.position.y;
        prev_odom.pose.pose.position.z    = result_odom.pose.pose.position.z;
        prev_odom.pose.pose.orientation.x = result_odom.pose.pose.orientation.x;
        prev_odom.pose.pose.orientation.y = result_odom.pose.pose.orientation.y;
        prev_odom.pose.pose.orientation.z = result_odom.pose.pose.orientation.z;
        prev_odom.pose.pose.orientation.w = result_odom.pose.pose.orientation.w;
        prev_odom.twist.twist.linear.x    = result_odom.twist.twist.linear.x;
        prev_odom.twist.twist.linear.y    = result_odom.twist.twist.linear.y;
        prev_odom.twist.twist.linear.z    = result_odom.twist.twist.linear.z;
        prev_odom.twist.twist.angular.x   = result_odom.twist.twist.angular.x;
        prev_odom.twist.twist.angular.y   = result_odom.twist.twist.angular.y;
        prev_odom.twist.twist.angular.z   = result_odom.twist.twist.angular.z;

        result_odom.header.stamp = ros::Time::now();

        // Publish Odometry
        // std::cout << "\n[ Odometry ]\n" << result_odom << std::endl;
        // std::cout << "\n[ Odometry position ]\n" << result_odom.pose.pose.position << std::endl;
        // std::cout << "\n[ Odometry orientation ]\n" << result_odom.pose.pose.orientation << std::endl;
        sobit_pro_odometry.pose_broadcaster(result_odom); // [NOT for Isaac Sim!]
        pub_odometry.publish(result_odom);                // [NOT for Isaac Sim!]

        rate.sleep();
    }

    spinner.stop();
}

// Bring Up SOBIT PRO main function
int main(int argc, char **argv){
    ros::init(argc, argv, "sobit_pro_control");

    // Initialize SobitProMain class
    SobitProMain sobit_pro_main;

    // [Real Robot] Start up motor
    // sobit_pro_motor_driver.init();
    // sobit_pro_motor_driver.addPresentParam();

    // Start up sound
    sobit_pro_main.start_up_sound();

    // Control wheel (main loop)
    sobit_pro_main.control_wheel();

    // Shut down sound
    sobit_pro_main.shut_down_sound();

    // [Real Robot] Shut down motor
    // sobit_pro_motor_driver.closeDynamixel();

    return 0;
}
