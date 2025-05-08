#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"  // IMU 메시지 타입 헤더
#include <ros_gz_interfaces/msg/entity_wrench.hpp>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "sedas_rot.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "ButterworthFilter.hpp"
#include "FilteredVector.hpp"
#include <iostream>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ign_pubsub : public rclcpp::Node
{
  public:
    ign_pubsub()
      : Node("ign_pubsub"), 
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), 
      count_(0),
      state_filter(9, 1.0, 0.005), // FilteredVector 초기화
      torque_measured_filter(3, 1, 0.005), // FilteredVector 초기화
      drone_tau_measured_filter(3, 2, 0.005), // FilteredVector 초기화
      drone_force_measured_filter(3, 2, 0.005) // FilteredVector 초기화
    {
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      cmd_joint1_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_1/command", qos_settings);
      cmd_joint2_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_2/command", qos_settings);      
      cmd_joint3_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/joint_3/command", qos_settings);            
      wrench_publisher_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/link_drone/wrench", qos_settings);
      velocity_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/velocity_marker", qos_settings);
      state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_vector", qos_settings);            
      state_dot_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_dot_vector", qos_settings);            
      commanded_publsiher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/commanded_input_U", qos_settings);
      state_U_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_U", qos_settings);
      kroc_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/kroc_data", qos_settings);


      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/manipulator/joint_states", qos_settings,
          std::bind(&ign_pubsub::joint_state_subsciber_callback, this, std::placeholders::_1));
      link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/manipulator/imu", qos_settings,
          std::bind(&ign_pubsub::imu_subscriber_callback, this, std::placeholders::_1));

      position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/manipulator/pose_info", qos_settings,
          std::bind(&ign_pubsub::global_pose_callback, this, std::placeholders::_1));            
	

      // Joint 1 Subscriber
      joint_1_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint1", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint1_torque_Callback, this, std::placeholders::_1));

      // Joint 2 Subscriber
      joint_2_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint2", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint2_torque_Callback, this, std::placeholders::_1));

      // Joint 3 Subscriber
      joint_3_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_joint3", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::joint3_torque_Callback, this, std::placeholders::_1));

      // Joint EE Subscriber
      joint_EE_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_EE", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::jointEE_torque_Callback, this, std::placeholders::_1));

      // Pinocchio Gravity Subscriber
      pinocchio_gravity_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pinnochio/gravity", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::pinocchio_gravity_Callback, this, std::placeholders::_1));

      // FK Subscriber
      FK_meas_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/manipulator/FK", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::FK_Callback, this, std::placeholders::_1));


      drone_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/manipulator/EE_cmd", 10,  // Topic name and QoS depth
          std::bind(&ign_pubsub::EE_cmd_Callback, this, std::placeholders::_1));




      timer_ = this->create_wall_timer(
      5ms, std::bind(&ign_pubsub::timer_callback, this));


    body_xyz_P.diagonal() << 100, 100, 500;
    body_xyz_I.diagonal() << 0., 0., 0.;
    body_xyz_D.diagonal() << 10, 10, 50;
    body_rpy_P.diagonal() << 150, 150, 30;
    body_rpy_D.diagonal() << 15, 15, 4;
      wrench_msg.entity.name = "link_drone"; // 링크 이름
      wrench_msg.entity.type = ros_gz_interfaces::msg::Entity::LINK; // 엔티티 유형: LINK


    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      set_state_and_dot();
      set_traj();
      PID_controller();		
      data_publish();     	    

    }



  double saturation(double max, double min, double value){
  if (value > max) value = max;
  else if (value < min) value = min;
  return value;
  }



void PID_controller()
{

  global_xyz_error = global_xyz_cmd - global_xyz_meas;
  body_xyz_error = Rot_G2D(global_xyz_error, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  body_xyz_error_integral += body_xyz_error * delta_time;
  body_xyz_error_d = (body_xyz_error - prev_body_xyz_error) / delta_time;
  prev_body_xyz_error = body_xyz_error;


  body_force_cmd = body_xyz_P * body_xyz_error + body_xyz_I * body_xyz_error_integral + body_xyz_D * body_xyz_error_d;
  global_force_cmd = Rot_D2G(body_force_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);





  // body_rpy_cmd = Rot_G2D(global_rpy_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

  body_rpy_error = body_rpy_cmd - body_rpy_meas;
  // Wrapping to [-π, π] range
  body_rpy_error[2] = std::atan2(std::sin(body_rpy_error[2]), std::cos(body_rpy_error[2]));
  body_rpy_error_integral += body_rpy_error * delta_time;
  body_rpy_error_d = - body_rpy_vel_meas;

  body_torque_cmd = body_rpy_P * body_rpy_error + body_rpy_D * body_rpy_error_d;


  global_torque_cmd = Rot_D2G(body_torque_cmd, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);



}


void set_traj()
{
  const Eigen::Vector3d rod_offset(0.4, 0.0, 0.2);
  
  //TODO: IK Solve
  //Known: global_EE_xyz_cmd[0, 1, 2]
  //       body_EE_rpy_cmd[0, 1, 2]

  //Need:  global_xyz_cmd[0, 1, 2]
  //       body_rpy_cmd[0, 1, 2]

      // **Step 2: RPY에서 회전 행렬 생성 (R = Rz * Ry * Rx)**
    double roll = body_EE_rpy_cmd[0];
    double pitch = body_EE_rpy_cmd[1];
    double yaw = body_EE_rpy_cmd[2];

    Eigen::Matrix3d R_EE;
    R_EE = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // **Step 3: Rod의 오프셋을 고려하여 드론의 목표 위치 역계산**
    global_xyz_cmd = global_EE_xyz_cmd - (R_EE * rod_offset);



    // **Step 4: 드론의 목표 자세 = End-Effector의 목표 자세**
    body_rpy_cmd = body_EE_rpy_cmd;

    body_rpy_cmd[0] = body_rpy_cmd.x();
    body_rpy_cmd[1] = body_rpy_cmd.y();
    body_rpy_cmd[2] = body_rpy_cmd.z();


}




void data_publish()
{	// publish!!
  joint_1_cmd_msg.data = joint_angle_cmd[0];
  joint_2_cmd_msg.data = joint_angle_cmd[1];
  joint_3_cmd_msg.data = joint_angle_cmd[2];	      
  wrench_msg.wrench.force.x = global_force_cmd[0];
  wrench_msg.wrench.force.y = global_force_cmd[1];
  wrench_msg.wrench.force.z = global_force_cmd[2];  // 500 N 힘 적용
  wrench_msg.wrench.torque.x = global_torque_cmd[0];
  wrench_msg.wrench.torque.y = global_torque_cmd[1];
  wrench_msg.wrench.torque.z = global_torque_cmd[2];

  cmd_joint1_publisher_->publish(joint_1_cmd_msg);
  cmd_joint2_publisher_->publish(joint_2_cmd_msg);       
  cmd_joint3_publisher_->publish(joint_3_cmd_msg);       	      
  wrench_publisher_->publish(wrench_msg);






  std_msgs::msg::Float64MultiArray state_msg;
  state_msg.data.push_back(global_xyz_meas[0]);
  state_msg.data.push_back(global_xyz_meas[1]);
  state_msg.data.push_back(global_xyz_meas[2]);
  state_msg.data.push_back(quat_meas[0]);
  state_msg.data.push_back(quat_meas[1]);
  state_msg.data.push_back(quat_meas[2]);
  state_msg.data.push_back(quat_meas[3]);
  state_msg.data.push_back(joint_angle_meas[0]);
  state_msg.data.push_back(joint_angle_meas[1]);
  state_msg.data.push_back(joint_angle_meas[2]);
  state_publisher_->publish(state_msg);




  std_msgs::msg::Float64MultiArray state_dot_msg;
  state_dot_msg.data.push_back(filtered_state_dot[0]);
  state_dot_msg.data.push_back(filtered_state_dot[1]);
  state_dot_msg.data.push_back(filtered_state_dot[2]);
  state_dot_msg.data.push_back(filtered_state_dot[3]);
  state_dot_msg.data.push_back(filtered_state_dot[4]);
  state_dot_msg.data.push_back(filtered_state_dot[5]);
  state_dot_msg.data.push_back(filtered_state_dot[6]);
  state_dot_msg.data.push_back(filtered_state_dot[7]);
  state_dot_msg.data.push_back(filtered_state_dot[8]);
  state_dot_publisher_->publish(state_dot_msg);


  global_force_cmd = drone_force_measured_filter.apply(global_force_cmd);
  global_torque_cmd = drone_tau_measured_filter.apply(global_torque_cmd);
  joint_effort_meas = torque_measured_filter.apply(joint_effort_meas);

  std_msgs::msg::Float64MultiArray state_U_msg;
  state_U_msg.data.push_back(global_force_cmd[0]);
  state_U_msg.data.push_back(global_force_cmd[1]);
  state_U_msg.data.push_back(global_force_cmd[2]);
  state_U_msg.data.push_back(global_torque_cmd[0]);
  state_U_msg.data.push_back(global_torque_cmd[1]);
  state_U_msg.data.push_back(global_torque_cmd[2]);
  state_U_msg.data.push_back(joint_effort_meas[0]);
  state_U_msg.data.push_back(joint_effort_meas[1]);
  state_U_msg.data.push_back(joint_effort_meas[2]);
  state_U_publisher_->publish(state_U_msg);



  std_msgs::msg::Float64MultiArray commanded_input;
  commanded_input.data.push_back(global_force_cmd[0]);
  commanded_input.data.push_back(global_force_cmd[1]);
  commanded_input.data.push_back(global_force_cmd[2]);
  commanded_input.data.push_back(global_torque_cmd[0]);
  commanded_input.data.push_back(global_torque_cmd[1]);
  commanded_input.data.push_back(global_torque_cmd[2]);
  commanded_input.data.push_back(joint_angle_cmd[0]);
  commanded_input.data.push_back(joint_angle_cmd[1]);
  commanded_input.data.push_back(joint_angle_cmd[2]);
  commanded_publsiher_->publish(commanded_input);


  if(External_force_sensor_meas.norm() > 0.01) contact_flag = true;

  if(contact_flag){
    std_msgs::msg::Float64MultiArray kroc_data;
    kroc_data.data.push_back(-100);
    kroc_data.data.push_back(FK_meas[0]);
    kroc_data.data.push_back(FK_meas[1]);
    kroc_data.data.push_back(FK_meas[2]);
    kroc_data.data.push_back(body_rpy_cmd[2]);
    kroc_data.data.push_back(body_rpy_meas[2]);
    kroc_data.data.push_back(100);
    kroc_publisher_->publish(kroc_data);
    }
}
 
void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_angle_dot_meas[0] = msg->velocity[0];
  joint_angle_dot_meas[1] = msg->velocity[1];
  joint_angle_dot_meas[2] = msg->velocity[2];
  joint_angle_meas[0] = msg->position[0]; // D-H Parameter!!
  joint_angle_meas[1] = msg->position[1]; 
  joint_angle_meas[2] = msg->position[2];

}
 
void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{

    // 쿼터니언 값 가져오기
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;

  quat_meas[0] = qx;
  quat_meas[1] = qy;
  quat_meas[2] = qz;
  quat_meas[3] = qw;

    // Roll, Pitch, Yaw 계산
    body_rpy_meas[0] = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    body_rpy_meas[1] = std::asin(2.0 * (qw * qy - qz * qx));
    body_rpy_meas[2] = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    body_rpy_vel_meas[0] = msg->angular_velocity.x;
    body_rpy_vel_meas[1] = msg->angular_velocity.y;
    body_rpy_vel_meas[2] = msg->angular_velocity.z;

    body_xyz_ddot_meas[0] = msg->linear_acceleration.x;
    body_xyz_ddot_meas[1] = msg->linear_acceleration.y;
    body_xyz_ddot_meas[2] = msg->linear_acceleration.z;


  global_rpy_meas = Rot_D2G(body_rpy_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  global_rpy_vel_meas = Rot_D2G(body_rpy_vel_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


}

	    
	    
void global_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{

    // link_yaw의 id는 7로 고정
    const int link_drone = 1;

    if (link_drone < msg->poses.size())
    {
        const auto &pose = msg->poses[link_drone];
      global_xyz_meas[0] = pose.position.x;
      global_xyz_meas[1] = pose.position.y;
      global_xyz_meas[2] = pose.position.z;                        
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "link_yaw id (1) is out of bounds in PoseArray.");
    }




}


void joint1_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[0] = msg->wrench.torque.z;
}

void joint2_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[1] = msg->wrench.torque.z;
}

void joint3_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    joint_effort_meas[2] = msg->wrench.torque.z;
}

void jointEE_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    External_force_sensor_meas[0] = msg->wrench.force.x;
    External_force_sensor_meas[1] = msg->wrench.force.y;
    External_force_sensor_meas[2] = msg->wrench.force.z;
    // RCLCPP_INFO(this->get_logger(), "EE_FORCE [%lf] [%lf] [%lf]", External_force_sensor_meas[0], External_force_sensor_meas[1], External_force_sensor_meas[2]);    
}

void pinocchio_gravity_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{

  for (int i = 0; i < 9; i++)
  {
    pinocchio_gravity[i] = msg->data[i];
  }

}

void FK_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (int i = 0; i<6; i++)
  {
  FK_meas[i] = msg->data[i];
  }
}

void EE_cmd_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{ // 이제부턴 드론 아니고 엔드이펙터임.
  for (int i = 0; i<3; i++)
  {
  global_EE_xyz_cmd[i] = msg->data[i];
  }
  body_EE_rpy_cmd[0] = msg->data[3];
  body_EE_rpy_cmd[1] = msg->data[4];
  body_EE_rpy_cmd[2] = msg->data[5];
}


void set_state_and_dot()
{

  global_xyz_vel = (global_xyz_meas - global_xyz_meas_prev) / delta_time;
  global_xyz_meas_prev = global_xyz_meas;
  body_xyz_vel = Rot_G2D(global_xyz_vel, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);


//  joint_angle_vel = (joint_angle_meas - joint_angle_meas_prev) / delta_time;
//  joint_angle_meas_prev = joint_angle_meas;


  raw_State_dot << body_xyz_vel[0], body_xyz_vel[1], body_xyz_vel[2], 
    body_rpy_vel_meas[0], body_rpy_vel_meas[1], body_rpy_vel_meas[2], 
    joint_angle_dot_meas[0], joint_angle_dot_meas[1], joint_angle_dot_meas[2];
 

    filtered_state_dot = state_filter.apply(raw_State_dot);

}




  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint1_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint2_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_joint3_publisher_;    
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_axis_publisher_;        
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_axis_publisher_;    
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_axis_publisher_;    
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_axis_publisher_;     
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_axis_publisher_;           
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_axis_publisher_;    
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_dot_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_U_publisher_;
  rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_publisher_;    
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_; 
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_1_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_2_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_3_torque_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_EE_torque_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pinocchio_gravity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commanded_publsiher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr FK_meas_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drone_cmd_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kroc_publisher_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    

  size_t count_;
  std_msgs::msg::Float64 joint_1_cmd_msg;
  std_msgs::msg::Float64 joint_2_cmd_msg;
  std_msgs::msg::Float64 joint_3_cmd_msg;    
  //TODO:: 아래 세 줄 정의 제대로 하기
  ros_gz_interfaces::msg::EntityWrench wrench_msg;
  // msg.entity.name = "link_drone";
  // msg.entity.type = ros_gz_interfaces::msg::Entity::LINK;


  Eigen::Vector3d global_xyz_meas;
  Eigen::Vector3d global_xyz_meas_prev;
  Eigen::Vector3d global_xyz_vel;
  Eigen::Vector3d body_xyz_vel;





  Eigen::Vector3d global_xyz_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_EE_rpy_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_EE_xyz_cmd = Eigen::Vector3d::Zero();
  
  Eigen::Vector3d global_xyz_error;
  Eigen::Vector3d global_xyz_error_integral;
  Eigen::Vector3d global_xyz_error_d;
  Eigen::Vector3d global_xyz_ddot_meas;
  Eigen::Vector3d body_xyz_ddot_meas;
  Eigen::Vector3d body_xyz_error;
  Eigen::Vector3d body_xyz_error_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_xyz_error_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_body_xyz_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_force_cmd;
  Eigen::Vector3d global_force_cmd;  
  Eigen::Matrix3d body_xyz_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_xyz_I = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_xyz_D = Eigen::Matrix3d::Zero();



  Eigen::Vector3d global_rpy_cmd;
  Eigen::Vector3d global_rpy_meas;
  Eigen::Vector3d body_rpy_meas;
  Eigen::Vector3d body_rpy_cmd;
  Eigen::Vector3d body_rpy_error;
  Eigen::Vector3d body_rpy_error_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_rpy_vel_meas;
  Eigen::Vector3d global_rpy_vel_meas;
  Eigen::Vector3d body_rpy_error_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_torque_cmd;  
  Eigen::Vector3d global_torque_cmd;  
  Eigen::Matrix3d body_rpy_P = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_rpy_I = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d body_rpy_D = Eigen::Matrix3d::Zero();
  Eigen::VectorXd quat_meas = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd FK_meas = Eigen::VectorXd::Zero(6);


  


  Eigen::Vector3d joint_angle_cmd;
  Eigen::Vector3d joint_angle_meas;
  Eigen::Vector3d joint_angle_meas_prev;
  Eigen::Vector3d joint_angle_vel;
  Eigen::Vector3d joint_angle_dot_meas;  
  Eigen::VectorXd joint_effort_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd External_force_sensor_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd State = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_prev = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd raw_State_dot = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_dot = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_quat = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd State_quat_prev = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd filtered_state_dot = Eigen::VectorXd::Zero(9);


  Eigen::VectorXd FK_EE_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw1_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw2_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Tw3_Pos = Eigen::VectorXd::Zero(6);
  Eigen::Matrix4d T_w1, T_w2, T_w3, T_w0;
  Eigen::Matrix4d T_01 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_23 = Eigen::Matrix4d::Identity();

  Eigen::VectorXd T01_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd T12_Pos = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd T23_Pos = Eigen::VectorXd::Zero(6);

  Eigen::VectorXd pinocchio_gravity = Eigen::VectorXd::Zero(10);

    double time;
    double time_cnt;
    double sine;

    double delta_time = 0.005;

    double l1 = 0.1;
    double l2 = 0.2;
    double l3 = 0.2;

  double contact_flag = false;

  FilteredVector state_filter;
  FilteredVector torque_measured_filter;
  FilteredVector drone_tau_measured_filter;
  FilteredVector drone_force_measured_filter;
};

int main(int argc, char * argv[])
{
//ros2 topic pub /joint_1/command std_msgs/msg/Float64 "{data: 1.0}"
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ign_pubsub>());
  rclcpp::shutdown();
  return 0;
}