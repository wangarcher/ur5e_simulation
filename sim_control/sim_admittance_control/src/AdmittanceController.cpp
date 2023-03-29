#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
                                           double frequency,
                                           std::string topic_arm_command,
                                           std::string topic_arm_state,
                                           std::string topic_external_wrench,
                                           std::vector<double> M_a,
                                           std::vector<double> D_a,
                                           std::vector<double> K_a,
                                           std::vector<double> d_e,
                                           std::vector<double> workspace_limits,
                                           double arm_max_vel,
                                           double arm_max_acc,
                                           double wrench_filter_factor,
                                           double force_dead_zone_thres,
                                           double torque_dead_zone_thres) : nh_(n), loop_rate_(frequency),
                                                                            wrench_filter_factor_(wrench_filter_factor),
                                                                            force_dead_zone_thres_(force_dead_zone_thres),
                                                                            torque_dead_zone_thres_(torque_dead_zone_thres),
                                                                            M_a_(M_a.data()), D_a_(D_a.data()), K_a_(K_a.data()),
                                                                            workspace_limits_(workspace_limits.data()),
                                                                            arm_max_vel_(arm_max_vel),
                                                                            arm_max_acc_(arm_max_acc)
{
  ///// Subscribers
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 10,
                                 &AdmittanceController::state_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_wrench_external_ = nh_.subscribe(topic_external_wrench, 5,
                                       &AdmittanceController::wrench_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());

  sub_now_equilibrium_ = nh_.subscribe("/now_equilibrium", 1,
                                       &AdmittanceController::now_equilibrium_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());

  sub_accel_ = nh_.subscribe("/imu2", 1,
                             &AdmittanceController::accel_callback, this,
                             ros::TransportHints().reliable().tcpNoDelay());

  ////// Publishers
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 1);

  ready_flag = false;
  zero_ftsensor_flag = true;
  init_count_ = 0;

  /// initializing the class variables
  wrench_external_.setZero();
  wrench_control_.setZero();
  linear_accel_external_.setZero();
  angular_vel_external_.setZero();
  // Init integrator
  arm_desired_twist_final_.setZero();
  transformation = false;

  // between zero and one that modulated the input force as to control the stiffness
  admittance_ratio_ = 0.77;

  wait_for_transformations();

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  Matrix3d rotation_from_to, rotation_repair;
  bool flag = true;

  rotation_repair << -1., 0., 0., 0., -1., 0., 0., 0., 1.;
  force_dead_zone_thres_ = 10.0;
  torque_dead_zone_thres_ = 10.0;
  while (flag)
  {
    ros::spinOnce();

    try
    {
      tf_listener.lookupTransform("base_link", "ee_link", ros::Time(0), transform);

      equilibrium_position_ << transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ();
      tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
      equilibrium_orientation_ = Quaterniond(rotation_repair * rotation_from_to);
      equilibrium_orientation_.coeffs() << equilibrium_orientation_.coeffs() / equilibrium_orientation_.coeffs().norm();
      flag = false;
    }
    catch (tf::TransformException ex)
    {
      flag = true;
      ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
                                      << "base_link"
                                      << " to: "
                                      << "ee_link");
      sleep(1);
    }
  }

  arm_real_orientation_ = equilibrium_orientation_;
  arm_real_position_ = equilibrium_position_;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::state_arm_callback(const nav_msgs::OdometryConstPtr msg)
{
  arm_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  arm_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w;

  arm_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

// void AdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg) {
//     Vector6d wrench_ft_frame;
//     Matrix6d rotation_ft_base;
//     Matrix6d rotation_sensor_ft;
//     rotation_sensor_ft = Matrix6d::Zero();
//     rotation_sensor_ft.block(0,0,3,3) << -1., 0., 0., 0., -1., 0., 0., 0., 1.; //z
//     rotation_sensor_ft.block(3,3,3,3) << -1., 0., 0., 0., -1., 0., 0., 0., 1.; //z
//     if (transformation)
//     {
//         // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
//         wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
//                            msg->wrench.force.z, msg->wrench.torque.x,
//                            msg->wrench.torque.y, msg->wrench.torque.z;
//         if(ready_flag && zero_ftsensor_flag)
//         {
//           last_wrench_ft_frame_ = wrench_ft_frame;
//           zero_ftsensor_flag = false;
//         }
//         wrench_ft_frame = wrench_ft_frame - last_wrench_ft_frame_;
//         for (int i = 0; i < 3; i++) {
//             if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_) {
//                 wrench_ft_frame(i) = 0;
//             }
//             if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_) {
//                 wrench_ft_frame(i + 3) = 0;
//             }
//         }
//         // Get transform from arm base link to platform base link
//         get_rotation_matrix(rotation_ft_base, listener_ft_, "base_link", "tool0");
//         // Filter and update
//         wrench_external_ << (1 - wrench_filter_factor_) * wrench_external_ +
//                             wrench_filter_factor_ * rotation_sensor_ft * rotation_ft_base * wrench_ft_frame;
//     }
// }

void AdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr msg)
{
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;

  Matrix6d rotation_sensor_ft;
  rotation_sensor_ft = Matrix6d::Zero();
  rotation_sensor_ft.block(0, 0, 3, 3) << -1., 0., 0., 0., -1., 0., 0., 0., 1.; // z
  rotation_sensor_ft.block(3, 3, 3, 3) << -1., 0., 0., 0., -1., 0., 0., 0., 1.; // z
  if (transformation)
  {

    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y,
        msg->wrench.force.z, msg->wrench.torque.x,
        msg->wrench.torque.y, msg->wrench.torque.z;

    for (int i = 0; i < 3; i++)
    {
      if (abs(wrench_ft_frame(i)) < force_dead_zone_thres_)
      {
        wrench_ft_frame(i) = 0;
      }
      if (abs(wrench_ft_frame(i + 3)) < torque_dead_zone_thres_)
      {
        wrench_ft_frame(i + 3) = 0;
      }
    }
    // Get transform from arm base link to platform base link
    get_rotation_matrix(rotation_ft_base, listener_ft_, "base_link", "ee_link");

    // Filter and update
    wrench_external_ << (1 - wrench_filter_factor_) * wrench_external_ +
                            wrench_filter_factor_ * rotation_sensor_ft * rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::accel_callback(const sensor_msgs::ImuConstPtr msg)
{
  linear_accel_external_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  angular_vel_external_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  time_ = ros::Time::now().toSec();
  double time_diff = time_ - last_time_;
  angular_accel_external_ = (angular_vel_external_ - last_angular_vel_external_) / time_diff;
  last_angular_vel_external_ = angular_vel_external_;
  last_time_ = time_;

  accel_external_.topRows(3) = linear_accel_external_;
  accel_external_.bottomRows(3) = angular_accel_external_;
  // accel_external_ << 0, 0, -10, 0, 0, 0;
  std::cout << "transformed accel" << accel_external_.transpose() << std::endl;
}

void AdmittanceController::now_equilibrium_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
  equilibrium_position_(0) = msg->pose.position.x;
  equilibrium_position_(1) = msg->pose.position.y;
  equilibrium_position_(2) = msg->pose.position.z;

  equilibrium_orientation_.coeffs() << msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w;
  if (msg->header.frame_id == "adm")
  {
    ready_flag = true;
  }
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::run()
{

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok())
  {
    if (ready_flag)
    {
      // Admittance Dynamics computation
      compute_admittance();

      // sum the vel from admittance to DS in this function
      // limit the the movement of the arm to the permitted workspace
      limit_to_workspace();

      // Copy commands to messages
      send_commands_to_robot();

      // publishing visualization/debugging info
      // publish_debuggings_signals();
    }
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

///////////////////////////////////////////////////////////////
///////////////////// Admittance Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::compute_admittance()
{

  Vector6d arm_desired_accelaration;
  Vector6d error;

  // Orientation error w.r.t. desired equilibriums
  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0)
  {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err(arm_real_orientation_ * equilibrium_orientation_.inverse());

  if (quat_rot_err.coeffs().norm() > 1e-3)
  {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
  }

  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();
  error.topRows(3) = arm_real_position_ - equilibrium_position_;

  arm_desired_accelaration = M_a_.inverse() * (-K_a_ * error - D_a_ * arm_desired_twist_final_ + admittance_ratio_ * accel_external_);

  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();
  if (a_acc_norm > arm_max_acc_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                                    << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_final_ += arm_desired_accelaration * duration.toSec();
}

///////////////////////////////////////////////////////////////
//////////////////// Limit to workspace////////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::limit_to_workspace()
{

  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  x = "
                                    << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                                    << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  y = "
                                    << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                                    << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  z = "
                                    << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                                    << workspace_limits_(5) << "]");
  }

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0))
  {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1))
  {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2))
  {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3))
  {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4))
  {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5))
  {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);
    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
  }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void AdmittanceController::send_commands_to_robot()
{
  geometry_msgs::Twist arm_twist_cmd;

  arm_twist_cmd.linear.x = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);

  pub_arm_cmd_.publish(arm_twist_cmd);
}

//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::wait_for_transformations()
{
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // while (!get_rotation_matrix(rot_matrix, listener, "world", "base_link")) {
  //     sleep(1);
  // }

  while (!get_rotation_matrix(rotation_base_, listener, "base_link", "base_link"))
  {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener, "base_link", "ee_link"))
  {
    sleep(1);
  }
  transformation = true;

  ROS_INFO("The Force/Torque sensor is ready to use.");
}

////////////
/// UTIL ///
////////////

bool AdmittanceController::get_rotation_matrix(Matrix6d &rotation_matrix,
                                               tf::TransformListener &listener,
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
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame);
    return false;
  }

  return true;
}