#ifndef QUADROTOR_SIMULATOR_BASE_HPP_
#define QUADROTOR_SIMULATOR_BASE_HPP_

#include <ros/ros.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3Stamped.h>
#include <quadrotor_msgs/OutputData.h>
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "flightlib/objects/static_object.hpp"
#include <sensor_msgs/CameraInfo.h>

namespace QuadrotorSimulator
{
template <typename T, typename U>
class QuadrotorSimulatorBase
{
 public:
  QuadrotorSimulatorBase(ros::NodeHandle &n);
  void run(void);
  void extern_force_callback(
      const geometry_msgs::Vector3Stamped::ConstPtr &f_ext);
  void extern_moment_callback(
      const geometry_msgs::Vector3Stamped::ConstPtr &m_ext);
  void addTag(int id, Eigen::Vector3f pose, flightlib::Quaternion quat);

 protected:

  typedef struct _ControlInput
  {
    double rpm[4];
  } ControlInput;

  /*
   * The callback called when ROS message arrives and needs to fill in the
   * command_ field that would be passed to the getControl function.
   */
  virtual void cmd_callback(const typename T::ConstPtr &cmd) = 0;

  /*
   * Called by the simulator to get the current motor speeds. This is the
   * controller that would be running on the robot.
   * @param[in] quad Quadrotor instance which is being simulated
   * @param[in] cmd The command input which is filled by cmd_callback
   */
  virtual ControlInput getControl(const Quadrotor &quad,
                                  const U &cmd) const = 0;

  Quadrotor quad_;
  U command_;
  
  //Unity 
  std::shared_ptr<flightlib::Quadrotor> quad_ptr_; // = std::make_shared<flightlib::Quadrotor>();
  std::shared_ptr<flightlib::Quadrotor> quad2_ptr_; // = std::make_shared<flightlib::Quadrotor>();
  flightlib::QuadState quad_state_;
  std::shared_ptr<flightlib::UnityBridge> unity_bridge_ptr_;
  std::shared_ptr<flightlib::RGBCamera>  rgb_camera_;
  std::shared_ptr<flightlib::RGBCamera>  rgb2_camera_;

 private:
  void stateToOdomMsg(const Quadrotor::State &state,
                      nav_msgs::Odometry &odom) const;
  void quadToImuMsg(const Quadrotor &quad, sensor_msgs::Imu &imu) const;
  void tfBroadcast(const nav_msgs::Odometry &odom_msg);
  //bool setUnity();
  //bool connectUnity();
  
  ros::Publisher pub_odom_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_output_data_;
  ros::Publisher pub_cam_info_;
  image_transport::Publisher pub_cam_;
  image_transport::Publisher pub_cam_long_view_;
  ros::Subscriber sub_cmd_;
  ros::Subscriber sub_extern_force_;
  ros::Subscriber sub_extern_moment_;
  double simulation_rate_;
  double odom_rate_;
  std::string quad_name_;
  std::string world_frame_id_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::CameraInfo cam_info;
  bool unity_render_;
};

template <typename T, typename U>
QuadrotorSimulatorBase<T, U>::QuadrotorSimulatorBase(ros::NodeHandle &n):
  unity_render_(false)
{
  pub_odom_ = n.advertise<nav_msgs::Odometry>("odom", 100);
  pub_imu_ = n.advertise<sensor_msgs::Imu>("imu", 100);
  pub_cam_info_ = n.advertise<sensor_msgs::CameraInfo>("camera_info", 100);
  image_transport::ImageTransport it(n);
  image_transport::ImageTransport it2(n);
  pub_cam_ = it.advertise("unity_drone_cam", 1);
  pub_cam_long_view_ = it2.advertise("long_view_cam",1);
  pub_output_data_ = n.advertise<quadrotor_msgs::OutputData>("output_data", 100);
  sub_cmd_ = n.subscribe<T>("cmd", 100, &QuadrotorSimulatorBase::cmd_callback,
                            this, ros::TransportHints().tcpNoDelay());
  sub_extern_force_ = n.subscribe<geometry_msgs::Vector3Stamped>(
      "extern_force", 10, &QuadrotorSimulatorBase::extern_force_callback, this,
      ros::TransportHints().tcpNoDelay());
  sub_extern_moment_ = n.subscribe<geometry_msgs::Vector3Stamped>(
      "extern_moment", 10, &QuadrotorSimulatorBase::extern_moment_callback,
      this, ros::TransportHints().tcpNoDelay());
  n.setParam("enableUnity", true);

  n.param("rate/simulation", simulation_rate_, 1000.0);
  ROS_ASSERT(simulation_rate_ > 0);

  n.param("rate/odom", odom_rate_, 500.0);
	odom_rate_ =500.0;
  n.param("world_frame_id", world_frame_id_, std::string("simulator"));
  n.param("quadrotor_name", quad_name_, std::string("quadrotor"));

  auto get_param = [&n](const std::string &param_name) {
    double param;
    if(!n.hasParam(param_name))
    {
      ROS_WARN("Simulator sleeping to wait for param %s", param_name.c_str());
      ros::Duration(0.5).sleep();
    }
    if(!n.getParam(param_name, param))
    {
      const std::string error_msg = param_name + " not set";
      ROS_FATAL_STREAM(error_msg);
      throw std::logic_error(error_msg);
    }
    return param;
  };

  quad_.setMass(get_param("mass"));
  quad_.setInertia(
      Eigen::Vector3d(get_param("Ixx"), get_param("Iyy"), get_param("Izz"))
          .asDiagonal());
  quad_.setGravity(get_param("gravity"));
  quad_.setPropRadius(get_param("prop_radius"));
  quad_.setPropellerThrustCoefficient(get_param("thrust_coefficient"));
  quad_.setArmLength(get_param("arm_length"));
  quad_.setMotorTimeConstant(get_param("motor_time_constant"));
  quad_.setMinRPM(get_param("min_rpm"));
  quad_.setMaxRPM(get_param("max_rpm"));
  quad_.setDragCoefficient(get_param("drag_coefficient"));
  
  Eigen::Vector3d initial_pos;
  n.param("initial_position/x", initial_pos(0), 0.0);
  n.param("initial_position/y", initial_pos(1), 0.0);
  n.param("initial_position/z", initial_pos(2), 0.0);


  Eigen::Quaterniond initial_q;
  n.param("initial_orientation/w", initial_q.w(), 1.0);
  n.param("initial_orientation/x", initial_q.x(), 0.0);
  n.param("initial_orientation/y", initial_q.y(), 0.0);
  n.param("initial_orientation/z", initial_q.z(), 0.0);
  initial_q.normalize();

  Quadrotor::State state = quad_.getState();
  state.x(0) = initial_pos(0);
  state.x(1) = initial_pos(1);
  state.x(2) = initial_pos(2);
  state.R = initial_q.matrix();

  quad_.setState(state);
  
  // Add quadrotor unity
  ros::Duration(5.0).sleep();
  quad_ptr_ = std::make_shared<flightlib::Quadrotor>();
  ROS_INFO("Quadrotor created.");  
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);
  rgb_camera_ = std::make_shared<flightlib::RGBCamera>();
  //ROS_INFO("Camera created.");  
  quad_ptr_->addRGBCamera(rgb_camera_);

  //make RGB CAMERa


  //bring camera
  flightlib::Vector<3> B_r_BC(1.0, 0.0, 0.0);
  flightlib::Matrix<3, 3> R_BC;
  //Add Camera -> This transforamtion is dumb.
  // which means that Idenitity is a camera facing forward
  R_BC << 0.0,1.0,0.0,  -1.0,0.0,0.0,  0.0,0.0,1.0;
  rgb_camera_->setFOV(70);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(  std::vector<bool>{false, false, false});  // depth, segmentation, optical flow
  cam_info.height = rgb_camera_->getHeight();
  cam_info.width = rgb_camera_->getWidth();
  
  //cam_info.distortion_model = "plumb_bob";
  double focal = 0.5*cam_info.height/tan(0.5*3.141* static_cast< double >(rgb_camera_->getFOV())/180);
  //double K[9] 
  cam_info.K = {focal, 0, rgb_camera_->getWidth()*0.5, 0, focal, rgb_camera_->getHeight()*0.5, 0,0,1};
  //double  P[12] = {focal, 0, rgb_camera_->getWidth()*0.5, 0, 0,focal, rgb_camera_->getHeight()*0.5, 0,0,0,1,0};
  cam_info.P = {focal, 0, rgb_camera_->getWidth()*0.5, 0, 0,focal, rgb_camera_->getHeight()*0.5, 0,0,0,1,0};

    ROS_INFO("Camera parameter made.");  
  // Initialize Unity bridge
  //addTag(0,Eigen::Vector3f(0, 4, 1),flightlib::Quaternion(0.8660254 ,0.5, 0, 0.0  ));
  //addTag(1,Eigen::Vector3f(1.5, 3, 0.7),flightlib::Quaternion(0.9659258,  0, -0.258819, 0  ));
  //addTag(2,Eigen::Vector3f(0, -1, 0.6),flightlib::Quaternion(0.8660254 ,-0.5, 0, 0.0  ));
  //addTag(3,Eigen::Vector3f(3.5, -2, 1),flightlib::Quaternion(0.9659258,  0,0.-0.258819, 0   ));
  //addTag(4,Eigen::Vector3f(-3, 1, 1),flightlib::Quaternion(1.0,  0,0.0, 0   ));
  // wait until the gazebo and unity are loaded

  // connect unity
  ROS_INFO("Before Unity Bridge is created.");  

  unity_bridge_ptr_ = flightlib::UnityBridge::getInstance();
  ROS_INFO("Unity Bridge is create.");  
  ROS_INFO("Before Unity Bridge Connection Wait.");  
  ros::Duration(7.0).sleep();
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  unity_render_ = unity_bridge_ptr_->connectUnity(flightlib::UnityScene::WAREHOUSE);
  ROS_INFO("connect Unity Bridge is created.");  

}



// ID 0 is the main target
template <typename T, typename U>
void QuadrotorSimulatorBase<T,U>::addTag(int id, Eigen::Vector3f pose, flightlib::Quaternion quat){
    const std::string stub = "AprilTag";
    const std::string box_wooden = "cube_box";
    std::string box_name = box_wooden+std::to_string(id);
    std::string apriltag_name = stub+std::to_string(id);
    std::shared_ptr<flightlib::StaticObject> gate =  std::make_shared<flightlib::StaticObject>(apriltag_name, apriltag_name);
    gate->setPosition(pose);
    const float scale = 0.075; //Make the tags bigger
    gate->setSize(Eigen::Vector3f(scale , scale*1.25, scale));
    gate->setQuaternion(quat);
    unity_bridge_ptr_->addStaticObject(gate);
    std::shared_ptr<flightlib::StaticObject> box_gate =  std::make_shared<flightlib::StaticObject>(box_name, box_wooden);
    Eigen::Vector3f box_pose = pose;
    box_pose(2) = pose(2)*0.5-scale*10;
    box_gate->setPosition(box_pose);
    box_gate->setSize(Eigen::Vector3f(scale*10 , scale*10, pose(2)));
    box_gate->setQuaternion(quat.Identity());
    unity_bridge_ptr_->addStaticObject(box_gate);
    std::shared_ptr<flightlib::StaticObject> box_gate2 =  std::make_shared<flightlib::StaticObject>(box_name+"next_box", box_wooden);
    Eigen::Vector3f box_pose2 = pose;
    box_pose2(2) = pose(2)-0.2;
    box_gate2->setPosition(box_pose2);
    box_gate2->setSize(Eigen::Vector3f(0.2 , 0.2, 0.1));
    box_gate2->setQuaternion(quat.Identity());
    //unity_bridge_ptr_->addStaticObject(box_gate2);
    //ADD THE RPG GATE AS A CROWN OBJECT TO OBSERVE
    if(id==0){
      std::shared_ptr<flightlib::StaticObject> gate2 =  std::make_shared<flightlib::StaticObject>("rpg_gate", "rpg_gate");
      gate2->setPosition(pose);
      gate2->setSize(Eigen::Vector3f(scale*6 , scale*6, scale*6));
      Eigen::Matrix3f rot = quat.toRotationMatrix();
      Eigen::Matrix3f rot2 = Eigen::Matrix3f::Zero();
      rot2(0,0) = 1;
      rot2(1,2) = -1;
      rot2(2,1) = 1;
      rot = rot*rot2;
      Eigen::Quaternionf quat_fixed(rot);
      gate2->setQuaternion(flightlib::Quaternion(quat_fixed));
      unity_bridge_ptr_->addStaticObject(gate2);
    }
}

Eigen::Vector4f mul(Eigen::Vector4f q1,Eigen::Vector4f q2) {
    Eigen::Vector4f output;
    output[1] =  q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1];
    output[2] = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2];
    output[3] =  q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3];
    output[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
    return output;
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::run(void)
{
  std::cout << " Start run stuff " <<std::endl;
  static int downsample_unity = 0;
  // Call once with empty command to initialize values
  cmd_callback(boost::make_shared<T>());
   std::cout << " command callback " <<std::endl;
  static bool last_pose = true;
  QuadrotorSimulatorBase::ControlInput control;
  int frame_id = 0;
  nav_msgs::Odometry odom_msg;
  sensor_msgs::Imu imu_msg;
  quadrotor_msgs::OutputData output_data_msg;
  odom_msg.header.frame_id = world_frame_id_;
  odom_msg.child_frame_id = quad_name_;
  imu_msg.header.frame_id = quad_name_;
  output_data_msg.header.frame_id = quad_name_;

  const double simulation_dt = 1 / simulation_rate_;
  ros::Rate r(simulation_rate_);

  const ros::Duration odom_pub_duration(1 / odom_rate_);
  ros::Time next_odom_pub_time = ros::Time::now();
  while(ros::ok())
  {

    ros::spinOnce();

    control = getControl(quad_, command_);
    quad_.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                   control.rpm[3]);
    quad_.step(simulation_dt);

    ros::Time tnow = ros::Time::now();
    bool mirrorUnity = true;
    ros::NodeHandle n("~");
    n.getParam("enableUnity", mirrorUnity);

    if((tnow >= next_odom_pub_time) )
    {

      next_odom_pub_time += odom_pub_duration;
      const Quadrotor::State &state = quad_.getState();
      stateToOdomMsg(state, odom_msg);
      odom_msg.header.stamp = tnow;
      pub_odom_.publish(odom_msg);
      tfBroadcast(odom_msg);
      Eigen::Vector4f robot_odom;
      // update the robot state only if we are mirro r unite
      if(true){
        //Static quaternion
        Eigen::Vector4f quat_staic;
        quat_staic[1]=0.0;
        quat_staic[2]=0.0;
        quat_staic[3]=0.707;
        quat_staic[0]=0.707;
        quat_staic.normalize();
        robot_odom[0] = odom_msg.pose.pose.orientation.w;
        robot_odom[1] = odom_msg.pose.pose.orientation.x;
        robot_odom[2] = odom_msg.pose.pose.orientation.y;
        robot_odom[3] = odom_msg.pose.pose.orientation.z;
        robot_odom = mul(quat_staic , robot_odom);
        quad_state_.x[flightlib::QS::POSX] = -1*odom_msg.pose.pose.position.y;
        quad_state_.x[flightlib::QS::POSY] = odom_msg.pose.pose.position.x;
        quad_state_.x[flightlib::QS::POSZ] = odom_msg.pose.pose.position.z;
        quad_state_.x[flightlib::QS::ATTW] =  robot_odom[0];
        quad_state_.x[flightlib::QS::ATTX] =  robot_odom[1];
        quad_state_.x[flightlib::QS::ATTY] =  robot_odom[2];
        quad_state_.x[flightlib::QS::ATTZ] =  robot_odom[3];

      }
      else{
        if(last_pose){
          last_pose = false;
          std::cout << quad_state_.x[flightlib::QS::POSX] <<std::endl;
          std::cout << quad_state_.x[flightlib::QS::POSY] <<std::endl;
          std::cout << quad_state_.x[flightlib::QS::POSZ] <<std::endl;
        }
      }
      // Set new state
      quad_ptr_->setState(quad_state_);
      downsample_unity+=1;
      // Render next frame
      if(downsample_unity==5){
        cv::Mat img;

        unity_bridge_ptr_->getRender(0);
        unity_bridge_ptr_->handleOutput();
        downsample_unity = 0;
        if(rgb_camera_->getRGBImage(img)){
          sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
          rgb_msg->header.stamp = tnow;
          pub_cam_.publish(rgb_msg);   
          cam_info.header.stamp = tnow;
          pub_cam_info_.publish(cam_info);
        }
        /*if(rgb2_camera_->getRGBImage(img)){
          sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
          rgb_msg->header.stamp = tnow;
          pub_cam_long_view_.publish(rgb_msg);   
        }*/
      }      
      quadToImuMsg(quad_, imu_msg);
      imu_msg.header.stamp = tnow;
      pub_imu_.publish(imu_msg);
      // Also publish an OutputData msg
      output_data_msg.header.stamp = tnow;
      output_data_msg.orientation = imu_msg.orientation;
      output_data_msg.angular_velocity = imu_msg.angular_velocity;
      output_data_msg.linear_acceleration = imu_msg.linear_acceleration;
      output_data_msg.motor_rpm[0] = state.motor_rpm(0);
      output_data_msg.motor_rpm[1] = state.motor_rpm(1);
      output_data_msg.motor_rpm[2] = state.motor_rpm(2);
      output_data_msg.motor_rpm[3] = state.motor_rpm(3);
      pub_output_data_.publish(output_data_msg);

    }
    r.sleep();
  }
}


template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::extern_force_callback(
    const geometry_msgs::Vector3Stamped::ConstPtr &f_ext)
{
  quad_.setExternalForce(
      Eigen::Vector3d(f_ext->vector.x, f_ext->vector.y, f_ext->vector.z));
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::extern_moment_callback(
    const geometry_msgs::Vector3Stamped::ConstPtr &m_ext)
{
  quad_.setExternalMoment(
      Eigen::Vector3d(m_ext->vector.x, m_ext->vector.y, m_ext->vector.z));
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::stateToOdomMsg(
    const Quadrotor::State &state, nav_msgs::Odometry &odom) const
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::quadToImuMsg(const Quadrotor &quad,
                                                sensor_msgs::Imu &imu) const
{
  const Quadrotor::State state = quad.getState();
  Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  const double kf = quad.getPropellerThrustCoefficient();
  const double m = quad.getMass();
  const Eigen::Vector3d &external_force = quad.getExternalForce();
  const double g = quad.getGravity();
  const double thrust = kf * state.motor_rpm.square().sum();
  Eigen::Vector3d acc;
  if(state.x(2) < 1e-4)
  {
    acc = state.R.transpose() * (external_force / m + Eigen::Vector3d(0, 0, g));
  }
  else
  {
    acc = thrust / m * Eigen::Vector3d(0, 0, 1) +
          state.R.transpose() * external_force / m;
    if(quad.getDragCoefficient() != 0)
    {
      const double drag_coefficient = quad.getDragCoefficient();
      const double mass = quad.getMass();
      Eigen::Matrix3d P;
      P << 1, 0, 0,
           0, 1, 0,
           0, 0, 0;
      acc -= drag_coefficient / mass * P * state.R.transpose() * state.v;
    }
  }

  imu.linear_acceleration.x = acc(0);
  imu.linear_acceleration.y = acc(1);
  imu.linear_acceleration.z = acc(2);
}

template <typename T, typename U>
void QuadrotorSimulatorBase<T, U>::tfBroadcast(
    const nav_msgs::Odometry &odom_msg)
{
  geometry_msgs::TransformStamped ts;

  ts.header.stamp = odom_msg.header.stamp;
  ts.header.frame_id = odom_msg.header.frame_id;
  ts.child_frame_id = odom_msg.child_frame_id;

  ts.transform.translation.x = odom_msg.pose.pose.position.x;
  ts.transform.translation.y = odom_msg.pose.pose.position.y;
  ts.transform.translation.z = odom_msg.pose.pose.position.z;

  ts.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_.sendTransform(ts);
}
}

#endif
