#include "sobit_pro_sim_library/sobit_pro_sim_library.h"
#include "sobit_pro_sim_library/sobit_pro_sim_joint_controller.h"
#include "sobit_pro_sim_library/sobit_pro_sim_wheel_controller.hpp"

using namespace sobit_pro;

PYBIND11_MODULE( sobit_pro_sim_module, m ) {
    pybind11::enum_<Joint>( m, "Joint" )
        .value( "ARM_SHOULDER_1_TILT_JOINT",    Joint::ARM_SHOULDER_1_TILT_JOINT )
        // .value( "ARM_SHOULDER_2_TILT_JOINT",    Joint::ARM_SHOULDER_2_TILT_JOINT ) // [Real Robot]
        .value( "ARM_ELBOW_UPPER_1_TILT_JOINT", Joint::ARM_ELBOW_UPPER_1_TILT_JOINT )
        // .value( "ARM_ELBOW_UPPER_2_TILT_JOINT", Joint::ARM_ELBOW_UPPER_2_TILT_JOINT ) // [Real Robot]
        .value( "ARM_ELBOW_LOWER_TILT_JOINT",   Joint::ARM_ELBOW_LOWER_TILT_JOINT )
        .value( "ARM_ELBOW_LOWER_PAN_JOINT",    Joint::ARM_ELBOW_LOWER_PAN_JOINT )
        .value( "ARM_WRIST_TILT_JOINT",         Joint::ARM_WRIST_TILT_JOINT )
        .value( "HAND_JOINT",                   Joint::HAND_JOINT )
        .value( "HEAD_PAN_JOINT",               Joint::HEAD_PAN_JOINT )
        .value( "HEAD_TILT_JOINT",              Joint::HEAD_TILT_JOINT )
        .value( "JOINT_NUM",                    Joint::JOINT_NUM )
        .export_values( );

    pybind11::class_<SobitProSimJointController>( m, "SobitProSimJointController" )
        .def( pybind11::init<const std::string&>( ) )

        .def( "moveToPose", &SobitProSimJointController::moveToPose, "move Pose",
            pybind11::arg( "pose_name" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true )
        .def( "moveAllJoint", &SobitProSimJointController::moveAllJoint, "move all Joint",
            pybind11::arg( "arm_shoulder_tilt_joint" ),
            pybind11::arg( "arm_elbow_upper_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_pan_joint" ),
            pybind11::arg( "arm_wrist_tilt_joint" ),
            pybind11::arg( "hand_joint" ),
            pybind11::arg( "head_pan_joint" ),
            pybind11::arg( "head_tilt_joint" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveJoint", &SobitProSimJointController::moveJoint, "move Joint",
            pybind11::arg( "joint_num" ),
            pybind11::arg( "rad" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveArm", &SobitProSimJointController::moveArm, "move Arm",
            pybind11::arg( "arm_shoulder_tilt_joint" ),
            pybind11::arg( "arm_elbow_upper_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_tilt_joint" ),
            pybind11::arg( "arm_elbow_lower_pan_joint" ),
            pybind11::arg( "arm_wrist" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveHeadPanTilt", &SobitProSimJointController::moveHeadPanTilt, "move Head PanTilt",
            pybind11::arg( "head_pan_joint" ),
            pybind11::arg( "head_tilt_joint" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveHandToTargetCoord", &SobitProSimJointController::moveHandToTargetCoord, "move Gripper To Target Coordinate",
            pybind11::arg( "target_pos_x" ), pybind11::arg( "target_pos_y" ), pybind11::arg( "target_pos_z" ),
            pybind11::arg( "shift_x" )     , pybind11::arg( "shift_y" )     , pybind11::arg( "shift_z" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveHandToTargetTF", &SobitProSimJointController::moveHandToTargetTF, "move Gripper To Target TF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "shift_x" ), pybind11::arg( "shift_y" ), pybind11::arg( "shift_z" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveHandToPlaceCoord", &SobitProSimJointController::moveHandToPlaceCoord, "move Gripper To Placeable Position Coordinate",
            pybind11::arg( "target_pos_x" ), pybind11::arg( "target_pos_y" ), pybind11::arg( "target_pos_z" ),
            pybind11::arg( "shift_x" )     , pybind11::arg( "shift_y" )     , pybind11::arg( "shift_z" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "moveHandToPlaceTF", &SobitProSimJointController::moveHandToPlaceTF, "move Gripper To Placeable Position TF",
            pybind11::arg( "target_name" ),
            pybind11::arg( "shift_x" ), pybind11::arg( "shift_y" ), pybind11::arg( "shift_z" ),
            pybind11::arg( "sec" ) = 5.0, pybind11::arg( "is_sleep" ) = true
            )
        .def( "graspDecision", &SobitProSimJointController::graspDecision, "grasp Decision",
            pybind11::arg( "min_curr" ) = 300, pybind11::arg( "max_curr" ) = 1000
            )
        .def( "placeDecision", &SobitProSimJointController::placeDecision, "place Decision",
            pybind11::arg( "min_curr" ) = 500, pybind11::arg( "max_curr" ) = 1000
            );

    pybind11::class_<SobitProSimWheelController>( m, "SobitProSimWheelController" )
        .def( pybind11::init<const std::string&>( ) )

        .def( "controlWheelLinear", &SobitProSimWheelController::controlWheelLinear, "control Wheel Linear", 
            pybind11::arg( "distance_x" ),
            pybind11::arg( "distance_y" )
            )
        .def( "controlWheelRotateRad", &SobitProSimWheelController::controlWheelRotateRad, "control Wheel Rotate Rad", 
            pybind11::arg( "angle_rad" )
            )
        .def( "controlWheelRotateDeg", &SobitProSimWheelController::controlWheelRotateDeg,"control Wheel Rotate Deg",
            pybind11::arg( "angle_deg" )
            );
}