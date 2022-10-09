// a pick ang place demo, initialized in 211124
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <iostream>

#include "mission_mode/mode_switch.h"
#include "general_arm_planning.h"

Manipulator::Manipulator()
{
    visual_last_seq_ = -1;
    visual_last_stamp_ = 0.0;
    arm_real_position_.setZero();

    obj_visual_yaw_ = 0.0;

    maxtries = 100;   // max attempts
    jump_threshold = 0.0;
    eef_step = 0.002;

    controller_flag_ = 0; // set as planning mode for pereferenece

    static_arm_base_translation << 0.365, 0.0, 0.28;
    static_arm_base_orientation =  Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) *
                                   Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());

    // sub_obj_visual_pose_ = n_.subscribe("/aruco_single/pose", 10, &Manipulator::obj_visual_callback, this);

    sub_base_odom_ = n_.subscribe("/base_pose_ground_truth", 1, &Manipulator::base_odom_callback, this, ros::TransportHints().tcpNoDelay());

    sub_global_command_ = n_.subscribe("/move_base/TebLocalPlannerROS/ee_next_global_pose", 1, &Manipulator::global_command_callback, this, ros::TransportHints().tcpNoDelay());

    sub_base_pitch_command_ = n_.subscribe("/move_base/TebLocalPlannerROS/next_base_pitch", 1, &Manipulator::base_pitch_callback, this, ros::TransportHints().tcpNoDelay());

    // sub_arm_state_ = n_.subscribe("/sim_cartesian_velocity_controller/ee_state", 1,
    //                              &Manipulator::state_arm_callback, this,
    //                              ros::TransportHints().reliable().tcpNoDelay());

    equilibrium_info_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/now_equilibrium", 1);

    plate_info_pub_ = n_.advertise<std_msgs::Float64>("/joint_position_controller/command", 1);

    // controller switch, by Wang ju, his contribution shall be 
    controller_switch_ = n_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
}

void Manipulator::open_gripper()
{

}

void Manipulator::close_gripper()
{

}

void Manipulator::preset_plan(moveit::planning_interface::MoveGroupInterface& arm_group)
{
    plan_mode(controller_switch_);
    arm_group.setNamedTarget("init");
    arm_group.move();
    sleep(1);

}

void Manipulator::tracking()
{
    custom_mode(controller_switch_);
    memory_servo();
}

void Manipulator::stablize()
{   
    custom_mode(controller_switch_);
    preset_servo();
    // memory_servo();
}

bool Manipulator::plan_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;
    switcher.request.stop_controllers.push_back("sim_cartesian_velocity_controller_sim");
    switcher.request.start_controllers.push_back("arm_joint_controller");
    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;
    controller_switch_.call(switcher);

    return switcher.response.ok;
}

bool Manipulator::custom_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;
    switcher.request.stop_controllers.push_back("arm_joint_controller");
    switcher.request.start_controllers.push_back("sim_cartesian_velocity_controller_sim");
    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;
    controller_switch_.call(switcher);

    return switcher.response.ok;
}


void Manipulator::visual_servo()
{
    while(n_.ok() && controller_flag_ == 1)
    {
        Eigen::Vector3f obj_base_translation;
        Eigen::Vector3f obj_base_last_translation;
        Eigen::Quaternionf obj_base_orientation;
        Eigen::Quaternionf obj_base_last_orientation;

        if(visual_current_seq_ != 0)
        {
            if(visual_last_seq_ == visual_current_seq_)
            {
                obj_base_translation = obj_base_last_translation;
                obj_base_orientation = obj_base_last_orientation;
            }
            else
            {
                obj_base_translation = arm_real_orientation_ * obj_visual_translation_ + arm_real_position_;

                float obj_visual_roll = atan(obj_visual_translation_(1)/obj_visual_translation_(2));
                float obj_visual_pitch = atan(obj_visual_translation_(0)/obj_visual_translation_(2));
                obj_visual_direct_orientation_ = Eigen::AngleAxisf(obj_visual_yaw_, Eigen::Vector3f::UnitZ()) *
                                                 Eigen::AngleAxisf(obj_visual_pitch, Eigen::Vector3f::UnitY()) *
                                                 Eigen::AngleAxisf(obj_visual_roll, Eigen::Vector3f::UnitX());
                obj_base_orientation = obj_visual_direct_orientation_.inverse() * arm_real_orientation_;

                obj_base_last_orientation = obj_base_orientation;
                obj_base_last_translation = obj_base_translation;
                visual_last_seq_ = visual_current_seq_;
            }
        }
        // std::cout << "the desired ee pose: "<< desired_ee_arm_position.transpose() << std::endl;
        ee_pose_msg.pose.position.x = obj_base_translation(0);
        ee_pose_msg.pose.position.y = obj_base_translation(1);
        ee_pose_msg.pose.position.z = obj_base_translation(2);
        ee_pose_msg.pose.orientation.x = obj_base_orientation.x();
        ee_pose_msg.pose.orientation.y = obj_base_orientation.y();
        ee_pose_msg.pose.orientation.z = obj_base_orientation.z();
        ee_pose_msg.pose.orientation.w = obj_base_orientation.w();
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);
    }
}

void Manipulator::memory_servo()
{
    Eigen::Vector3f arm_map_translation;
    Eigen::Quaternionf arm_map_orientation;

    while(n_.ok())
    {
        arm_map_orientation = base_map_orientation_ * static_arm_base_orientation;
        arm_map_translation = base_map_orientation_.toRotationMatrix() * static_arm_base_translation + base_map_translation_;
        Eigen::Quaternionf desired_ee_arm_orientation = arm_map_orientation.inverse() * desired_ee_map_orientation_;
        Eigen::Vector3f desired_ee_arm_position = arm_map_orientation.toRotationMatrix().inverse() * (desired_ee_map_position_ - arm_map_translation);
        // ROS_INFO_STREAM("p: " << desired_ee_map_position_.transpose());
        ee_pose_msg.header.stamp = ros::Time::now();
        ee_pose_msg.pose.position.x = desired_ee_arm_position(0);
        ee_pose_msg.pose.position.y = desired_ee_arm_position(1);
        ee_pose_msg.pose.position.z = desired_ee_arm_position(2);
        ee_pose_msg.pose.orientation.x = desired_ee_arm_orientation.x();
        ee_pose_msg.pose.orientation.y = desired_ee_arm_orientation.y();
        ee_pose_msg.pose.orientation.z = desired_ee_arm_orientation.z();
        ee_pose_msg.pose.orientation.w = desired_ee_arm_orientation.w();
        // ee_pose_msg.pose.position.x = 0.60;
        // ee_pose_msg.pose.position.y = 0.0;
        // ee_pose_msg.pose.position.z = 0.35;
        // ee_pose_msg.pose.orientation.x = 0.0;
        // ee_pose_msg.pose.orientation.y = 0.0;
        // ee_pose_msg.pose.orientation.z = 0.0;
        // ee_pose_msg.pose.orientation.w = 1.0;
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);

        // double base_map_pitch;
        // double sinp = 2 * (base_map_orientation_.w() * base_map_orientation_.y() - base_map_orientation_.z() * base_map_orientation_.x());
        // if (std::abs(sinp) >= 1)
        //     base_map_pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        // else
        //     base_map_pitch = std::asin(sinp);
        // ROS_INFO_STREAM("base_pitch: " <<  base_map_pitch << "next_base_pitch_: " << desired_base_pitch_);

        // plate_angle_msg.data = base_map_pitch - desired_base_pitch_;
        // plate_info_pub_.publish(plate_angle_msg);
        ros::Duration(0.001).sleep();
    }
}

void Manipulator::preset_servo()
{
    Eigen::Vector3f arm_map_translation;
    Eigen::Quaternionf arm_map_orientation;

    while(n_.ok() && controller_flag_ == 0)
    {
        ee_pose_msg.header.stamp = ros::Time::now();
        ee_pose_msg.pose.position.x = 0.50;
        ee_pose_msg.pose.position.y = 0.0;
        ee_pose_msg.pose.position.z = 0.55;
        ee_pose_msg.pose.orientation.x = 0.0;
        ee_pose_msg.pose.orientation.y = 0.0;
        ee_pose_msg.pose.orientation.z = 0.0;
        ee_pose_msg.pose.orientation.w = 1.0;
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);

        plate_angle_msg.data = 1.0;
        plate_info_pub_.publish(plate_angle_msg);
        ros::Duration(0.001).sleep();
    }
}

void Manipulator::obj_visual_callback(const geometry_msgs::PoseStamped& obj_visual_pose)
{
    visual_last_stamp_ = ros::Time::now().toSec();
    visual_current_seq_ = obj_visual_pose.header.seq;
    obj_visual_translation_ << obj_visual_pose.pose.position.x,
                               obj_visual_pose.pose.position.y,
                               obj_visual_pose.pose.position.z;
    Eigen::Quaternionf mid_term_1(obj_visual_pose.pose.orientation.w, 
                                  obj_visual_pose.pose.orientation.x,
                                  obj_visual_pose.pose.orientation.y,
                                  obj_visual_pose.pose.orientation.z);
    Eigen::Vector3f obj_visual_euler = mid_term_1.matrix().eulerAngles(2, 1, 0);
    obj_visual_yaw_ = obj_visual_euler(0);
}

void Manipulator::base_odom_callback(const nav_msgs::Odometry& base_odom)
{
    base_map_translation_ << base_odom.pose.pose.position.x,
                             base_odom.pose.pose.position.y,
                             base_odom.pose.pose.position.z;
    base_map_orientation_.coeffs() <<  base_odom.pose.pose.orientation.x,
                                       base_odom.pose.pose.orientation.y,
                                       base_odom.pose.pose.orientation.z,
                                       base_odom.pose.pose.orientation.w;
}

void Manipulator::state_arm_callback(const nav_msgs::Odometry& msg) {
    arm_real_position_ << msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z;
    arm_real_orientation_.coeffs() << msg.pose.pose.orientation.x,
                                      msg.pose.pose.orientation.y,
                                      msg.pose.pose.orientation.z,
                                      msg.pose.pose.orientation.w;
}

void Manipulator::global_command_callback(const geometry_msgs::PoseStamped& desired_ee_global_pose)
{
    desired_ee_map_position_ << desired_ee_global_pose.pose.position.x,
                                desired_ee_global_pose.pose.position.y,
                                desired_ee_global_pose.pose.position.z;
    desired_ee_map_orientation_.coeffs() << desired_ee_global_pose.pose.orientation.x,
                                            desired_ee_global_pose.pose.orientation.y,
                                            desired_ee_global_pose.pose.orientation.z,
                                            desired_ee_global_pose.pose.orientation.w;

    if(desired_ee_global_pose.header.frame_id == "pid") controller_flag_ = 1;
    if(desired_ee_global_pose.header.frame_id == "adm") controller_flag_ = 2;
}

void Manipulator::base_pitch_callback(const std_msgs::Float64& desired_base_pitch)
{
    desired_base_pitch_ = desired_base_pitch.data;
}