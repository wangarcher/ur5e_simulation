// a simple planning demo, initialized in 210813
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
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mission_mode/mode_switch.h>



class GeneralPickPlace
{
    public:
        Eigen::Vector3d obj_cam_translation_;
        Eigen::Quaterniond obj_cam_orientation_;
        bool first_reco_flag; // outwards flag, when the ee reached the desired pre-picking place
        bool second_reco_flag; // outwards flag, when the ee reached the desired pre-placeing place
        bool pick_flag = false; // inwards flag, when the flag set as true, the robot is ready for the pick motion sequence.
        bool place_flag = false; // inwards flag, whent the flag set as true, the robot is ready for the place motion sequence.
        
    public:
        void obj_visual_callback(const geometry_msgs::PoseStamped & odom);
};

void GeneralPickPlace::obj_visual_callback(const geometry_msgs::PoseStamped & odom)
{
    obj_cam_translation_ << odom.pose.position.x,
                            odom.pose.position.y,
                            odom.pose.position.z;
    obj_cam_orientation_.coeffs() <<  odom.pose.orientation.x,
                                      odom.pose.orientation.y,
                                      odom.pose.orientation.z,
                                      odom.pose.orientation.w;
    if (odom.header.frame_id == "1st")
    {
        pick_flag = true;
        place_flag = false;
    }
    if (odom.header.frame_id == "2nd")
    {
        place_flag = true;
        pick_flag = false;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_plan");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
	spinner.start();

    // some flag were set here, so as to make sure the sequence flow running in a proper way.
    bool pre_pick_flag = true;
    bool pick_flag = false;
    bool pre_place_flag = false;
    bool place_flag = false;
    bool sequence_flag = true;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); // set position torlerance
    arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);
    
    moveit::planning_interface::MoveGroupInterface ee("endeffector");
    ee.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    ee.setGoalJointTolerance(0.001);
    ee.setGoalPositionTolerance(0.001); // set position torlerance
    ee.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    ee.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    ee.setMaxVelocityScalingFactor(0.2);
    std::string end_effector_link = arm.getEndEffectorLink(); //get the end_effector link

    GeneralPickPlace general_pick_place;

    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/object_visual_odometry", 1000, &GeneralPickPlace::obj_visual_callback, &general_pick_place);

    ros::Publisher reco_trigger_pub = n.advertise<std_msgs::Int8>("/reco_trigger", 1);

    // the messages may needed in the process
    std_msgs::Int8 reco_trigger_msg;


    // geometry info
    Eigen::Quaterniond tcp_obj_orientation(1, 0, 0, 0);
    Eigen::Vector3d tcp_obj_translation(0, 0, -0.22);
    Eigen::Quaterniond cam_tool_orientatoin(1, 0, 0, 0);
    Eigen::Vector3d cam_tool_translation(-0.035, -0.09, 0.035);

    Eigen::Quaterniond tool_base_orientation;
    Eigen::Vector3d tool_base_translation;  
    Eigen::Quaterniond cam_base_orientation;
    Eigen::Vector3d cam_base_translation;
    Eigen::Quaterniond obj_base_orientation;
    Eigen::Vector3d obj_base_translation;
    Eigen::Quaterniond tcp_base_orientation;
    Eigen::Vector3d tcp_base_translation;
    
    while(n.ok() && sequence_flag)
    {
        if(pre_pick_flag)
        {
            std::cout<<"Caution: moving to the pre-pick pose!"<<std::endl;
            arm.setNamedTarget("1st_reco_start"); // first reco state, for the best top-looking view
            arm.move();
            ee.setNamedTarget("open"); // gripper open 
            ee.move();
            reco_trigger_msg.data = 1;
            reco_trigger_pub.publish(reco_trigger_msg);
            sleep(1);
            pre_pick_flag = false;
            pick_flag = true;
        }

        if(pick_flag && general_pick_place.pick_flag)
        {   
            std::cout<<"Caution: object pose received, now picking!"<<std::endl;
            std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
            std::string reference_frame = "base_link"; // set the reference frame
            arm.setPoseReferenceFrame(reference_frame);

            geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
            std::cout<<"now Robot position: [x,y,z]: ["
                    <<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"
                    <<std::endl;
            std::cout<<"now Robot orientation: [x,y,z,w]: ["
                    <<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z<<","<<now_pose.orientation.w<<"]"
                    <<std::endl;

            tool_base_translation << now_pose.position.x, now_pose.position.y, now_pose.position.z;
            tool_base_orientation.coeffs() << now_pose.orientation.x,
                                            now_pose.orientation.y,
                                            now_pose.orientation.z,
                                            now_pose.orientation.w;

            cam_base_orientation = tool_base_orientation * cam_tool_orientatoin;
            cam_base_translation = tool_base_orientation.toRotationMatrix() * cam_tool_translation + tool_base_translation;

            obj_base_orientation = cam_base_orientation * general_pick_place.obj_cam_orientation_;
            obj_base_translation = cam_base_orientation.toRotationMatrix() * general_pick_place.obj_cam_translation_ + cam_base_translation;

            tcp_base_orientation = obj_base_orientation * tcp_obj_orientation;
            tcp_base_translation = obj_base_orientation.toRotationMatrix() * tcp_obj_translation + obj_base_translation; 

            std::cout<<"now obj position: "<< tcp_base_translation.transpose() <<std::endl;
            std::cout<<"now obj orientation: [w, x, y, z]: ["
                    << tcp_base_orientation.w() <<", "<< tcp_base_orientation.x() <<", "<<tcp_base_orientation.y()<<", "<<tcp_base_orientation.z() <<std::endl;

            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose pose1;
            pose1.position.x = tcp_base_translation(0);
            pose1.position.y = tcp_base_translation(1);	
            pose1.position.z = tool_base_translation(2);	
            pose1.orientation.x = obj_base_orientation.x();
            pose1.orientation.y = obj_base_orientation.y();
            pose1.orientation.z = obj_base_orientation.z();
            pose1.orientation.w = obj_base_orientation.w();
            waypoints.push_back(pose1);

            geometry_msgs::Pose pose2;
            pose2.position.x = tcp_base_translation(0);
            pose2.position.y = tcp_base_translation(1);	
            pose2.position.z = tcp_base_translation(2);	
            pose2.orientation.x = obj_base_orientation.x();
            pose2.orientation.y = obj_base_orientation.y();
            pose2.orientation.z = obj_base_orientation.z();
            pose2.orientation.w = obj_base_orientation.w();
            waypoints.push_back(pose2);

            // planning
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.002;
            double fraction = 0.0;
            int maxtries = 100;   // max attempts
            int attempts = 0;     // attempts done

            while(fraction < 1.0 && attempts < maxtries)
            {
                fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                attempts++;
                if(attempts % 10 == 0)
                    ROS_INFO("Still trying after %d attempts...", attempts);
            }
            
            if(fraction == 1)
            {   
                ROS_INFO("Path computed successfully. Moving the arm.");
                // plan data
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
                sleep(1);
            }
            else
            {
                ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
            }
            ee.setNamedTarget("close"); //first reco state, for the best top-looking view
            ee.move();
            sleep(1); 
            pick_flag = false;
            pre_place_flag = true;  
        }
        
        if(pre_place_flag)
        {
            std::cout<<"Caution: moving to the pre-place pose!"<<std::endl;
            arm.setNamedTarget("2nd_reco_start"); //second reco state, for the best top-looking view
            arm.move();
            reco_trigger_msg.data = 2;
            reco_trigger_pub.publish(reco_trigger_msg);
            sleep(1);
            pre_place_flag = false;
            place_flag = true;
        }

        if(place_flag && general_pick_place.place_flag)
        {
            std::cout<<"Caution: workspace pose received, now placing!"<<std::endl;
            geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
            std::cout<<"now Robot position: [x,y,z]: ["
                    <<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"
                    <<std::endl;
            std::cout<<"now Robot orientation: [x,y,z,w]: ["
                    <<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z<<","<<now_pose.orientation.w<<"]"
                    <<std::endl;

            tool_base_translation << now_pose.position.x, now_pose.position.y, now_pose.position.z;
            tool_base_orientation.coeffs() << now_pose.orientation.x,
                                            now_pose.orientation.y,
                                            now_pose.orientation.z,
                                            now_pose.orientation.w;

            cam_base_orientation = tool_base_orientation * cam_tool_orientatoin;
            cam_base_translation = tool_base_orientation.toRotationMatrix() * cam_tool_translation + tool_base_translation;

            obj_base_orientation = cam_base_orientation * general_pick_place.obj_cam_orientation_;
            obj_base_translation = cam_base_orientation.toRotationMatrix() * general_pick_place.obj_cam_translation_ + cam_base_translation;

            tcp_base_orientation = obj_base_orientation * tcp_obj_orientation;
            tcp_base_translation = obj_base_orientation.toRotationMatrix() * tcp_obj_translation + obj_base_translation; 

            std::cout<<"now obj position: "<< tcp_base_translation.transpose() <<std::endl;
            std::cout<<"now obj orientation: [w, x, y, z]: ["
                    << tcp_base_orientation.w() <<", "<< tcp_base_orientation.x() <<", "<<tcp_base_orientation.y()<<", "<<tcp_base_orientation.z() <<std::endl;

            std::vector<geometry_msgs::Pose> waypoints;
            geometry_msgs::Pose pose1;
            pose1.position.x = tcp_base_translation(0);
            pose1.position.y = tcp_base_translation(1);	
            pose1.position.z = tool_base_translation(2);	
            pose1.orientation.x = obj_base_orientation.x();
            pose1.orientation.y = obj_base_orientation.y();
            pose1.orientation.z = obj_base_orientation.z();
            pose1.orientation.w = obj_base_orientation.w();
            waypoints.push_back(pose1);

            geometry_msgs::Pose pose2;
            pose2.position.x = tcp_base_translation(0);
            pose2.position.y = tcp_base_translation(1);	
            pose2.position.z = tcp_base_translation(2);	
            pose2.orientation.x = obj_base_orientation.x();
            pose2.orientation.y = obj_base_orientation.y();
            pose2.orientation.z = obj_base_orientation.z();
            pose2.orientation.w = obj_base_orientation.w();
            waypoints.push_back(pose2);

            // planning
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.002;
            double fraction = 0.0;
            int maxtries = 100;   // max attempts
            int attempts = 0;     // attempts done

            while(fraction < 1.0 && attempts < maxtries)
            {
                fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                attempts++;
                if(attempts % 10 == 0)
                    ROS_INFO("Still trying after %d attempts...", attempts);
            }
            
            if(fraction == 1)
            {   
                ROS_INFO("Path computed successfully. Moving the arm.");
                // plan data
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
                sleep(1);
            }
            else
            {
                ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
            }
            ee.setNamedTarget("open");  // gripper open 
            ee.move();
            sleep(1);
            place_flag = false;
            sequence_flag = false;
        }
        loop_rate.sleep();
    }
    std::cout<<"Info: mission accomplished!!!"<<std::endl;
    ros::waitForShutdown();

    return 0;
}