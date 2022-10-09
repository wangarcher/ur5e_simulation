#include "PIDController.h"

PIDController::PIDController(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_command,
    std::string topic_arm_state,
    std::vector<double> K_p,
    std::vector<double> K_i,
    std::vector<double> K_d,
    std::vector<double> workspace_limits,
    double arm_max_vel,
    double arm_acc_upper_limit,
    double arm_acc_lower_limit) :
  nh_(n), loop_rate_(frequency),
  K_p_(K_p.data()),K_i_(K_i.data()), K_d_(K_d.data()),
  workspace_limits_(workspace_limits.data()),
  arm_max_vel_(arm_max_vel),
  arm_acc_upper_limit_(arm_acc_upper_limit),
  arm_acc_lower_limit_(arm_acc_lower_limit)
{
  ///// Subscribers
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 1,
                                 &PIDController::state_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_now_equilibrium_ = nh_.subscribe("/now_equilibrium", 1,
                                       &PIDController::now_equilibrium_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());

  ////// Publishers
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 1);

  // Init integrator
  arm_desired_twist_final_.setZero();
  last_arm_desired_twist_final_.setZero();
  // last_arm_real_twist_.setZero();
  last_error_.setZero();
  // total_error_.setZero();

  transformation = false;
  pid_ready_flag_ = false;
  eps_ = 0.001;
  // between zero and one that modulated the input force as to control the stiffness(刚度)

  wait_for_transformations();

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  Matrix3d rotation_from_to,rotation_repair;
  bool flag = true;
  rotation_repair << -1.,0.,0.,0.,-1.,0.,0.,0.,1.;
  
  while (flag)
  {
    ros::spinOnce();
    // ROS_INFO("ur5e_mode is: %d", ur5e_mode_);
    try 
    {
      tf_listener.lookupTransform("base_link", "ee_link",ros::Time(0), transform);
      equilibrium_position_ << transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ();
      tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
      equilibrium_orientation_ = Quaterniond(rotation_repair*rotation_from_to);
      equilibrium_orientation_.coeffs() << equilibrium_orientation_.coeffs() /equilibrium_orientation_.coeffs().norm();
      flag = false;
    }
    catch (tf::TransformException ex) 
    {
      flag = true;
      ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << "base_link" << " to: " << "ee_link" );
      sleep(1);
    }
  }
  arm_real_orientation_ = equilibrium_orientation_;
  arm_real_position_ = equilibrium_position_ ;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::state_arm_callback(const nav_msgs::OdometryConstPtr msg) 
{
    arm_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

    arm_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z,
                                      msg->pose.pose.orientation.w;

    // arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
    //                    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}



void PIDController::now_equilibrium_callback(const geometry_msgs::PoseStampedConstPtr msg)
{

  equilibrium_position_(0) = msg->pose.position.x;
  equilibrium_position_(1) = msg->pose.position.y;
  equilibrium_position_(2) = msg->pose.position.z;

  equilibrium_orientation_.coeffs() << msg->pose.orientation.x,
                                       msg->pose.orientation.y,
                                       msg->pose.orientation.z,
                                       msg->pose.orientation.w;

  if(msg->header.frame_id == "pid")
  {
    pid_ready_flag_ = true;
  }
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::run() 
{
  ROS_INFO("Running the pid control loop .................");
  while (nh_.ok()) 
  {
    if(pid_ready_flag_)
    {
      // lowpass_filter();
      // Admittance Dynamics computation
      compute_pid();

      // sum the vel from admittance to DS in this function
      // limit the the movement of the arm to the permitted workspace
      limit_to_workspace();

      // Copy commands to messages
      send_commands_to_robot();

      // publishing visualization/debugging info
      //    publish_debuggings_signals();
    }
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

///////////////////////////////////////////////////////////////
///////////////////// PID Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void PIDController::compute_pid() 
{
  Vector6d arm_desired_accelaration;
  Vector6d calculated_arm_accelaration;
  Vector6d error;
  Vector6d calculated_arm_twist;

  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) 
  {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err(arm_real_orientation_* equilibrium_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) 
  {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }

  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() *err_arm_des_orient.angle();
  error.topRows(3) = arm_real_position_ - equilibrium_position_;

  calculated_arm_twist = error - last_error_;

  arm_desired_twist_final_ = - K_p_ * error - K_d_ * calculated_arm_twist;
  ros::Duration duration = loop_rate_.expectedCycleTime();
  double time_diff = duration.toSec();
  
  last_error_ =  error;
  last_arm_desired_twist_final_ = arm_desired_twist_final_;
}

///////////////////////////////////////////////////////////////
//////////////////// Limit to workspace////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::limit_to_workspace() 
{
    if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1)) {
        ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  x = "
                << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                << workspace_limits_(1) << "]");
    }

    if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3)) {
        ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  y = "
                << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                << workspace_limits_(3) << "]");
    }

    if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5)) {
        ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  z = "
                << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                << workspace_limits_(5) << "]");
    }

    if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
        arm_desired_twist_final_(0) = 0;
    }

    if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
        arm_desired_twist_final_(0) = 0;
    }

    if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
        arm_desired_twist_final_(1) = 0;
    }

    if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
        arm_desired_twist_final_(1) = 0;
    }

    if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
        arm_desired_twist_final_(2) = 0;
    }

    if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
        arm_desired_twist_final_(2) = 0;
    }

    // velocity of the arm along x, y, and z axis
    double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

    if (norm_vel_des > arm_max_vel_) {
        ROS_WARN_STREAM_THROTTLE(1, "pid generate fast arm movements! velocity norm: " << norm_vel_des);
        arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
    }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void PIDController::send_commands_to_robot() 
{
  geometry_msgs::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x  = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);
  pub_arm_cmd_.publish(arm_twist_cmd);
}


//////////////////////
/// INITIALIZATION ///
//////////////////////
void PIDController::wait_for_transformations() 
{
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // while (!get_rotation_matrix(rot_matrix, listener, "world", "base_link")) {
  //     sleep(1);
  // }

  while (!get_rotation_matrix(rotation_base_, listener, "base_link", "base_link")) {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener, "base_link", "ee_link")) {
    sleep(1);
  }
  transformation = true;

  ROS_INFO("The Force/Torque sensor is ready to use.");
}

////////////
/// UTIL ///
////////////

bool PIDController::get_rotation_matrix(Matrix6d & rotation_matrix, 
                                        tf::TransformListener & listener, 
                                        std::string from_frame,
                                        std::string to_frame)
{
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try 
  {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) 
  {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}