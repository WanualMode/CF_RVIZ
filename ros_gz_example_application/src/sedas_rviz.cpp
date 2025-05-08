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
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시
#include <tf2/LinearMath/Quaternion.h>
#include <random>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class sedas_rviz : public rclcpp::Node
{
  public:
    sedas_rviz()
      : Node("sedas_rviz"), 
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)), 
      count_(0)
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/manipulator/joint_states", qos_settings,
          std::bind(&sedas_rviz::joint_state_subsciber_callback, this, std::placeholders::_1));
      link_yaw_imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/manipulator/imu", qos_settings,
          std::bind(&sedas_rviz::imu_subscriber_callback, this, std::placeholders::_1));
      position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "/manipulator/pose_info", qos_settings,
          std::bind(&sedas_rviz::global_pose_callback, this, std::placeholders::_1));            
      EE_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pinnochio/EE_vel", qos_settings,
          std::bind(&sedas_rviz::EE_vel_callback, this, std::placeholders::_1)); 
      EE_pos_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pinnochio/EE_pos", qos_settings,
          std::bind(&sedas_rviz::EE_pos_callback, this, std::placeholders::_1)); 

      FK_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/manipulator/FK", 10);



      // Joint EE Subscriber
      joint_EE_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
          "/force_torque_EE", 10,  // Topic name and QoS depth
          std::bind(&sedas_rviz::jointEE_torque_Callback, this, std::placeholders::_1));

      EE_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/manipulator/EE_cmd", 10,  // Topic name and QoS depth
          std::bind(&sedas_rviz::EE_cmd_Callback, this, std::placeholders::_1));





      EE_Vel_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/EE_Vel_marker", 10);

      EE_Force_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/EE_Force_marker", 10);    

      Normal_Vector_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/Normal_Vector_marker", 10);        

      Normal_rpy_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/Normal_Vector_rpy_angle", 10);

      obstacle_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/obstacle_marker", 10);


      Normal_X_Axis_Yaw_Rotation_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/Normal_Vector", 10);


      timer_ = this->create_wall_timer(
      10ms, std::bind(&sedas_rviz::timer_callback, this));




    }

  private:
	    void timer_callback()
    {	//main loop, 100Hz
    // 현재 시간 계산
      contact_checker();
      Calc_FK();
      Robot_State_Publisher();
      End_Effector_Pos_Vel_Publisher();
      End_Effector_Force_Publisher();
      if(contact_Flag) Normal_vector_estimation_and_visual_Publisher();
      if(contact_Flag) Define_Normal_Frame();
      else Remove_Normal_Frame();
      EE_cmd_publisher();
      obstacle_visualizer();
      Calculate_X_Axis_Yaw_Rotation();
      data_publisher();
    }

    void data_publisher()
    {
      std_msgs::msg::Float64MultiArray FK_meas;
    FK_meas.data.push_back(Tw3_Pos[0]);
    FK_meas.data.push_back(Tw3_Pos[1]);
    FK_meas.data.push_back(Tw3_Pos[2]);
    FK_meas.data.push_back(Tw3_Pos[3]);
    FK_meas.data.push_back(Tw3_Pos[4]);
    FK_meas.data.push_back(Tw3_Pos[5]);

    FK_publisher_->publish(FK_meas);
    }


void EE_cmd_publisher()
{
    static std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = 
        std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // **Step 1: TF 메시지 생성**
    geometry_msgs::msg::TransformStamped transform_drone;
    transform_drone.header.stamp = this->get_clock()->now();
    transform_drone.header.frame_id = "world";      // 기준 좌표계
    transform_drone.child_frame_id = "EE_cmd";  // 드론 좌표계

    // **Step 2: 위치 설정 (Translation)**
    transform_drone.transform.translation.x = EE_global_xyz_cmd[0];
    transform_drone.transform.translation.y = EE_global_xyz_cmd[1];
    transform_drone.transform.translation.z = EE_global_xyz_cmd[2];

    // **Step 3: RPY -> Quaternion 변환**
    tf2::Quaternion q;
    q.setRPY(EE_body_rpy_cmd[0], EE_body_rpy_cmd[1], EE_body_rpy_cmd[2]); // Roll, Pitch, Yaw

    transform_drone.transform.rotation.x = q.x();
    transform_drone.transform.rotation.y = q.y();
    transform_drone.transform.rotation.z = q.z();
    transform_drone.transform.rotation.w = q.w();

    // **Step 4: TF Broadcast**
    tf_broadcaster_->sendTransform(transform_drone);
}


    void Calc_FK()
    { 
    // 기본적으로 드론의 Global Frame 기준 위치 및 자세를 기반으로 변환 행렬 T_w0 계산
    Eigen::Matrix3d R_B = get_rotation_matrix(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
    T_w0.block<3, 3>(0, 0) = R_B;
    T_w0.block<3, 1>(0, 3) = global_xyz_meas;

    // DH 파라미터 기반의 변환 행렬 정의
    T_01 << std::cos(joint_angle_meas[0]), 0, std::sin(joint_angle_meas[0]), 0,
            std::sin(joint_angle_meas[0]), 0, -std::cos(joint_angle_meas[0]), 0,
            0, 1, 0, l1,
            0, 0, 0, 1;

    T_12 << std::cos(joint_angle_meas[1]), -std::sin(joint_angle_meas[1]), 0, l2 * std::cos(joint_angle_meas[1]),
            std::sin(joint_angle_meas[1]), std::cos(joint_angle_meas[1]), 0, l2 * std::sin(joint_angle_meas[1]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    T_23 << std::cos(joint_angle_meas[2]), -std::sin(joint_angle_meas[2]), 0, l3 * std::cos(joint_angle_meas[2]),
            std::sin(joint_angle_meas[2]), std::cos(joint_angle_meas[2]), 0, l3 * std::sin(joint_angle_meas[2]),
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Forward Kinematics 계산
    Eigen::Matrix4d T_w1 = T_w0 * T_01;
    Eigen::Matrix4d T_w2 = T_w1 * T_12;
    Eigen::Matrix4d T_w3 = T_w2 * T_23;

    // 엔드 이펙터의 위치 및 자세 추출
    p_E = T_w3.block<3, 1>(0, 3); // 엔드 이펙터 위치
    R_E = T_w3.block<3, 3>(0, 0); // 엔드 이펙터 자세

    FK_EE_Pos[0] = p_E[0];
    FK_EE_Pos[1] = p_E[1];
    FK_EE_Pos[2] = p_E[2];


    // Global 기준 r, p, y angle 추출
    FK_EE_Pos[3] = std::atan2(R_E(2, 1), R_E(2, 2));
    FK_EE_Pos[4] = std::asin(-R_E(2, 0));
    FK_EE_Pos[5] = std::atan2(R_E(1, 0), R_E(0, 0));

    // T_w1 위치 및 자세 추출
    Eigen::Vector3d p_w1 = T_w1.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w1 = T_w1.block<3, 3>(0, 0);
    Tw1_Pos[0] = p_w1[0];
    Tw1_Pos[1] = p_w1[1];
    Tw1_Pos[2] = p_w1[2];
    Tw1_Pos[3] = std::atan2(R_w1(2, 1), R_w1(2, 2));
    Tw1_Pos[4] = std::asin(-R_w1(2, 0));
    Tw1_Pos[5] = std::atan2(R_w1(1, 0), R_w1(0, 0));

    // T_w2 위치 및 자세 추출
    Eigen::Vector3d p_w2 = T_w2.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w2 = T_w2.block<3, 3>(0, 0);
    Tw2_Pos[0] = p_w2[0];
    Tw2_Pos[1] = p_w2[1];
    Tw2_Pos[2] = p_w2[2];
    Tw2_Pos[3] = std::atan2(R_w2(2, 1), R_w2(2, 2));
    Tw2_Pos[4] = std::asin(-R_w2(2, 0));
    Tw2_Pos[5] = std::atan2(R_w2(1, 0), R_w2(0, 0));

    // T_w3 위치 및 자세 추출
    Eigen::Vector3d p_w3 = T_w3.block<3, 1>(0, 3);
    Eigen::Matrix3d R_w3 = T_w3.block<3, 3>(0, 0);
    Tw3_Pos[0] = p_w3[0];
    Tw3_Pos[1] = p_w3[1];
    Tw3_Pos[2] = p_w3[2];
    Tw3_Pos[3] = std::atan2(R_w3(2, 1), R_w3(2, 2));
    Tw3_Pos[4] = std::asin(-R_w3(2, 0));
    Tw3_Pos[5] = std::atan2(R_w3(1, 0), R_w3(0, 0));
    // State 벡터 출력 (디버깅용)
    // std::cout << "State: \n" << State_dot << std::endl;


    // T_01의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_01 = T_01.block<3, 1>(0, 3);
    Eigen::Matrix3d R_01 = T_01.block<3, 3>(0, 0);
    T01_Pos[0] = p_01[0];
    T01_Pos[1] = p_01[1];
    T01_Pos[2] = p_01[2];

    T01_Pos[3] = std::atan2(R_01(2, 1), R_01(2, 2));
    T01_Pos[4] = std::asin(-R_01(2, 0));
    T01_Pos[5] = std::atan2(R_01(1, 0), R_01(0, 0));

    // T_12의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_12 = T_12.block<3, 1>(0, 3);
    Eigen::Matrix3d R_12 = T_12.block<3, 3>(0, 0);
    T12_Pos[0] = p_12[0];
    T12_Pos[1] = p_12[1];
    T12_Pos[2] = p_12[2];    
    
    T12_Pos[3] = std::atan2(R_12(2, 1), R_12(2, 2));
    T12_Pos[4] = std::asin(-R_12(2, 0));
    T12_Pos[5] = std::atan2(R_12(1, 0), R_12(0, 0));

    // T_23의 위치 벡터 및 자세 추출
    Eigen::Vector3d p_23 = T_23.block<3, 1>(0, 3);
    Eigen::Matrix3d R_23 = T_23.block<3, 3>(0, 0);
    T23_Pos[0] = p_23[0];
    T23_Pos[1] = p_23[1];
    T23_Pos[2] = p_23[2];    

    T23_Pos[3] = std::atan2(R_23(2, 1), R_23(2, 2));
    T23_Pos[4] = std::asin(-R_23(2, 0));
    T23_Pos[5] = std::atan2(R_23(1, 0), R_23(0, 0));
    }


    void Robot_State_Publisher()
    {
        geometry_msgs::msg::TransformStamped transform_EE;
        transform_EE.header.stamp = this->get_clock()->now();
        transform_EE.header.frame_id = "world"; // Parent frame
        transform_EE.child_frame_id = "tf/EE_FK";  // Child frame

        transform_EE.transform.translation.x = FK_EE_Pos[0];
        transform_EE.transform.translation.y = FK_EE_Pos[1];
        transform_EE.transform.translation.z = FK_EE_Pos[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_EE;
        q_EE.setRPY(FK_EE_Pos[3], FK_EE_Pos[4], FK_EE_Pos[5]);
        transform_EE.transform.rotation.x = q_EE.x();
        transform_EE.transform.rotation.y = q_EE.y();
        transform_EE.transform.rotation.z = q_EE.z();
        transform_EE.transform.rotation.w = q_EE.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_EE);


  //TODO 2: Drone TF Publish
        geometry_msgs::msg::TransformStamped transform_drone;
        transform_drone.header.stamp = this->get_clock()->now();
        transform_drone.header.frame_id = "world"; // Parent frame
        transform_drone.child_frame_id = "link_drone";  // Child frame

        transform_drone.transform.translation.x = global_xyz_meas[0];
        transform_drone.transform.translation.y = global_xyz_meas[1];
        transform_drone.transform.translation.z = global_xyz_meas[2];

        // Convert roll, pitch, yaw to quaternion
        tf2::Quaternion q_drone;
        q_drone.setRPY(global_rpy_meas[0], global_rpy_meas[1], global_rpy_meas[2]);
        transform_drone.transform.rotation.x = q_drone.x();
        transform_drone.transform.rotation.y = q_drone.y();
        transform_drone.transform.rotation.z = q_drone.z();
        transform_drone.transform.rotation.w = q_drone.w();

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_drone);



    }


  void End_Effector_Pos_Vel_Publisher()
  {
    // Rviz에서 시각화할 Marker 메시지 생성
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "ee_velocity";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 화살표의 시작점과 끝점 설정
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = FK_EE_Pos[0]; // 현재 위치
    start_point.y = FK_EE_Pos[1];
    start_point.z = FK_EE_Pos[2];

    end_point.x = start_point.x + EE_lin_vel[0]; // 속도 벡터 방향
    end_point.y = start_point.y + EE_lin_vel[1];
    end_point.z = start_point.z + EE_lin_vel[2];

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 화살표의 색상 및 크기 설정
    marker.scale.x = 0.02; // 화살표의 줄기 두께
    marker.scale.y = 0.05; // 화살표의 머리 크기
    marker.scale.z = 0.05;

    marker.color.a = 1.0; // 불투명도
    marker.color.r = 0.0; // 빨간색 (속도)
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // 퍼블리시
    EE_Vel_publisher_->publish(marker);

  }


  void End_Effector_Force_Publisher()
  {
    // Rviz에서 시각화할 Marker 메시지 생성
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "EE_Force";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 화살표의 시작점과 끝점 설정
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = FK_EE_Pos[0]; // 현재 위치
    start_point.y = FK_EE_Pos[1];
    start_point.z = FK_EE_Pos[2];

    end_point.x = start_point.x + External_force_sensor_meas_global[0]; // 속도 벡터 방향
    end_point.y = start_point.y + External_force_sensor_meas_global[1];
    end_point.z = start_point.z + External_force_sensor_meas_global[2];

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 화살표의 색상 및 크기 설정
    marker.scale.x = 0.02; // 화살표의 줄기 두께
    marker.scale.y = 0.05; // 화살표의 머리 크기
    marker.scale.z = 0.05;

    marker.color.a = 1.0; // 불투명도
    marker.color.r = 1.0; // 빨간색 (속도)
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // 퍼블리시
    EE_Force_publisher_->publish(marker);
  }

  void Normal_vector_estimation_and_visual_Publisher()
  {
    // external force sensor data: External_force_sensor_meas_global
    // end effector velocity data: EE_lin_vel
    // Estimated_normal_Vector: Estimated_normal_Vector
    double alpha = (External_force_sensor_meas_global.dot(EE_lin_vel)) / (EE_lin_vel.dot(EE_lin_vel));
    Estimated_normal_Vector = External_force_sensor_meas_global -  alpha * EE_lin_vel;


    // Estimated_normal_Vector = Estimated_normal_Vector.normalized();
    Estimated_normal_Vector = External_force_sensor_meas_global;





    Estimated_normal_Vector[2] = 0;              // IF wanna set 3dof, delete
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "Normal_Vector";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 화살표의 시작점과 끝점 설정
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = FK_EE_Pos[0]; // 현재 위치
    start_point.y = FK_EE_Pos[1];
    start_point.z = FK_EE_Pos[2];

    end_point.x = start_point.x + Estimated_normal_Vector[0]; // 속도 벡터 방향
    end_point.y = start_point.y + Estimated_normal_Vector[1];
    end_point.z = start_point.z + Estimated_normal_Vector[2];

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    // 화살표의 색상 및 크기 설정
    marker.scale.x = 0.02; // 화살표의 줄기 두께
    marker.scale.y = 0.05; // 화살표의 머리 크기
    marker.scale.z = 0.05;

    marker.color.a = 1.0; // 불투명도
    marker.color.r = 0.0; // 빨간색 (속도)
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    // 퍼블리시
    Normal_Vector_publisher_->publish(marker);
  }

void Define_Normal_Frame()
{
    // 중력 벡터 (World 기준)
    Eigen::Vector3d g(0, 0, -9.81); // 일반적인 중력 방향

    // **Step 1: Normal Frame 정의**
    Eigen::Vector3d C_x = Estimated_normal_Vector;
    Eigen::Vector3d C_y = C_x.cross(g);
    if (C_y.norm() < 1e-6) {
        // RCLCPP_WARN(this->get_logger(), "Gravity vector and normal vector are parallel!");
        return;
    }
    C_x.normalize();
    C_y.normalize();
    Eigen::Vector3d C_z = C_x.cross(C_y);
    C_z.normalize();

    // **Step 2: TF 변환 메시지 생성**
    geometry_msgs::msg::TransformStamped transform_normal;
    transform_normal.header.stamp = this->get_clock()->now();
    transform_normal.header.frame_id = "world";  // World 좌표계를 기준으로 정의
    transform_normal.child_frame_id = "normal_frame";  // Normal 좌표계 ID

    transform_normal.transform.translation.x = FK_EE_Pos[0];
    transform_normal.transform.translation.y = FK_EE_Pos[1];
    transform_normal.transform.translation.z = FK_EE_Pos[2];

    // **행렬을 Quaternion으로 변환**
    Eigen::Matrix3d R_C;
    R_C.col(0) = C_x;
    R_C.col(1) = C_y;
    R_C.col(2) = C_z;
    Eigen::Quaterniond q(R_C);

    transform_normal.transform.rotation.x = q.x();
    transform_normal.transform.rotation.y = q.y();
    transform_normal.transform.rotation.z = q.z();
    transform_normal.transform.rotation.w = q.w();

    // **TF Broadcast**
    tf_broadcaster_->sendTransform(transform_normal);

    // **Yaw, Pitch, Roll 추출**
    Eigen::Vector3d normal_rpy;
    normal_rpy(0) = std::atan2(R_C(2,1), R_C(2,2));  // Roll
    normal_rpy(1) = std::asin(-R_C(2,0));           // Pitch
    normal_rpy(2) = std::atan2(R_C(1,0), R_C(0,0)); // Yaw



  std_msgs::msg::Float64MultiArray Normal_rpy;
  Normal_rpy.data.push_back(normal_rpy(0));
  Normal_rpy.data.push_back(normal_rpy(1));
  Normal_rpy.data.push_back(normal_rpy(2));

  Normal_rpy_angle_publisher_->publish(Normal_rpy);

}

void Remove_Normal_Frame()
{
    geometry_msgs::msg::TransformStamped transform_normal;
    transform_normal.header.stamp = this->get_clock()->now();
    transform_normal.header.frame_id = "world";
    transform_normal.child_frame_id = "normal_frame"; // 같은 ID를 유지
    
    // 좌표 원점으로 이동
    transform_normal.transform.translation.x = 0;
    transform_normal.transform.translation.y = 0;
    transform_normal.transform.translation.z = 0;
    
    // 단위 Quaternion으로 설정 (즉, 회전 없음)
    transform_normal.transform.rotation.x = 0;
    transform_normal.transform.rotation.y = 0;
    transform_normal.transform.rotation.z = 0;
    transform_normal.transform.rotation.w = 1;

    tf_broadcaster_->sendTransform(transform_normal);
}


  void contact_checker()
  {
    if(External_force_sensor_meas_global.norm() > 0.01 && EE_lin_vel.norm() > 0.01)
      contact_Flag = true;
    else
      contact_Flag = false;

  }

  void obstacle_visualizer()
  {
    // TODO: rviz에 속이 빈 원기둥(파이프) 시각화하기

    visualization_msgs::msg::Marker outer_cylinder;

    // 공통 설정
    outer_cylinder.header.frame_id = "world";
    outer_cylinder.header.stamp = this->now();
    outer_cylinder.ns = "obstacle";
    outer_cylinder.action = visualization_msgs::msg::Marker::ADD;
    outer_cylinder.type = visualization_msgs::msg::Marker::CYLINDER;

    // 위치 설정 (두 개의 원기둥이 동일한 위치에 있어야 함)
    outer_cylinder.pose.position.x = 3.0;
    outer_cylinder.pose.position.y = 1.0;
    outer_cylinder.pose.position.z = 0.0;
    outer_cylinder.pose.orientation.x = 0.0;
    outer_cylinder.pose.orientation.y = 0.0;
    outer_cylinder.pose.orientation.z = 0.0;
    outer_cylinder.pose.orientation.w = 1.0;

    // 크기 설정
    double outer_radius = 1.0;  // 외부 원기둥 반지름
    double height = 1.0;        // 원기둥 높이

    outer_cylinder.scale.x = outer_radius * 2;
    outer_cylinder.scale.y = outer_radius * 2;
    outer_cylinder.scale.z = height;


    // 색상 설정
    outer_cylinder.color.r = 0.3;
    outer_cylinder.color.g = 0.3;
    outer_cylinder.color.b = 1.0;
    outer_cylinder.color.a = 0.8;  // 외부 원기둥은 불투명하게

    // ID 설정
    outer_cylinder.id = 0;

    // 메시지 퍼블리시
    obstacle_publisher_->publish(outer_cylinder);
  }



  void Calculate_X_Axis_Yaw_Rotation()
  {
      // Normal Frame의 X축 벡터 (Estimated_normal_Vector)
      Eigen::Vector3d C_x = Estimated_normal_Vector.normalized();
      
      // X축 회전량 (Yaw) 계산
      double x_axis_yaw_rotation = std::atan2(C_x[1], C_x[0]);
      
      // External Force Sensor 방향 벡터 (Global 기준)
      Eigen::Vector3d force_dir = External_force_sensor_meas_global.normalized();
      
      // Force Vector의 yaw 회전량 (global x축과의 차이)
      double force_yaw_rotation = std::atan2(force_dir[1], force_dir[0]);
      
      // ROS 메시지로 Float64MultiArray로 퍼블리시
      std_msgs::msg::Float64MultiArray yaw_msg;
      yaw_msg.data.push_back(x_axis_yaw_rotation);
      yaw_msg.data.push_back(force_yaw_rotation);
      Normal_X_Axis_Yaw_Rotation_publisher_->publish(yaw_msg);
  }





  void data_publish()
  {	// publish!!




  }
 
void joint_state_subsciber_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Manipulator의 상~ㅌ태!!
  
  joint_angle_dot_meas[0] = msg->velocity[0];
  joint_angle_dot_meas[1] = msg->velocity[1];
  joint_angle_dot_meas[2] = msg->velocity[2];
  joint_angle_meas[0] = msg->position[0]; // D-H Parameter!!
  joint_angle_meas[1] = msg->position[1]; 
  joint_angle_meas[2] = msg->position[2];

}
 
void imu_subscriber_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Drone의 상.태~
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

    global_xyz_ddot_meas[0] = msg->linear_acceleration.x;
    global_xyz_ddot_meas[1] = msg->linear_acceleration.y;
    global_xyz_ddot_meas[2] = msg->linear_acceleration.z;


  global_rpy_meas = Rot_D2G(body_rpy_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  global_rpy_vel_meas = Rot_D2G(body_rpy_vel_meas, body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
}


	    
void global_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{

    // link_yaw의 id는 7로 고정
    const int link_yaw_id = 1;

    if (link_yaw_id < msg->poses.size())
    {
        const auto &pose = msg->poses[link_yaw_id];
      global_xyz_meas[0] = pose.position.x;
      global_xyz_meas[1] = pose.position.y;
      global_xyz_meas[2] = pose.position.z;                        
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "link_yaw id (17) is out of bounds in PoseArray.");
    }


}

void EE_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{  

    EE_lin_vel[0] = msg->data[0];
    EE_lin_vel[1] = msg->data[1];
    EE_lin_vel[2] = msg->data[2];
    EE_ang_vel[0] = msg->data[3];
    EE_ang_vel[1] = msg->data[4];
    EE_ang_vel[2] = msg->data[5];
}

void EE_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  EE_lin_pos[0] = msg->data[0];
  EE_lin_pos[1] = msg->data[1];
  EE_lin_pos[2] = msg->data[2];
  EE_ang_pos[0] = msg->data[3];
  EE_ang_pos[1] = msg->data[4];
  EE_ang_pos[2] = msg->data[5];
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

    External_force_sensor_meas_global = Rot_D2G(External_force_sensor_meas, Tw3_Pos[3], Tw3_Pos[4], Tw3_Pos[5]);
    External_force_sensor_meas_global.normalized();
}

void EE_cmd_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  for (int i = 0; i<3; i++)
  {
  EE_global_xyz_cmd[i] = msg->data[i];
  }
  EE_body_rpy_cmd[0] = msg->data[3];
  EE_body_rpy_cmd[1] = msg->data[4];
  EE_body_rpy_cmd[2] = msg->data[5];
}

// 가우시안 노이즈 생성 함수
double Generate_Gaussian_Noise(double mean, double stddev)
{
    static std::random_device rd;
    static std::mt19937 generator(rd());
    static std::normal_distribution<double> distribution(mean, stddev);
    
    return distribution(generator);
}


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_; 
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr link_yaw_imu_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_pos_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_EE_torque_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr FK_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Normal_rpy_angle_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr EE_Vel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr EE_Force_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr Normal_Vector_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Normal_X_Axis_Yaw_Rotation_publisher_;



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
  Eigen::Vector3d global_xyz_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d global_xyz_error;
  Eigen::Vector3d global_xyz_error_integral;
  Eigen::Vector3d global_xyz_error_d;
  Eigen::Vector3d global_xyz_ddot_meas;
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


  Eigen::VectorXd External_force_sensor_meas = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd External_force_sensor_meas_global = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd Estimated_normal_Vector = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd Estimated_normal_Vector_norm = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd External_normal_force_meas = Eigen::VectorXd::Zero(3);



  Eigen::Vector3d joint_angle_cmd;
  Eigen::Vector3d joint_angle_meas;
  Eigen::Vector3d joint_angle_dot_meas;  
  Eigen::Vector3d joint_effort_meas;
  Eigen::VectorXd State = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_prev = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_dot = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd State_quat = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd State_quat_prev = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd filtered_state_dot = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd FK_meas = Eigen::VectorXd::Zero(6);


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

  Eigen::Vector3d EE_lin_vel;
  Eigen::Vector3d EE_ang_vel;
  Eigen::Vector3d EE_lin_pos;
  Eigen::Vector3d EE_ang_pos;
  Eigen::Vector3d p_E;
  Eigen::Matrix3d R_E;

  Eigen::Vector3d EE_global_xyz_cmd;
  Eigen::Vector3d EE_body_rpy_cmd;
  




  Eigen::Vector3d EE_lin_vel_global;



    double time;
    double time_cnt;
    double sine;

    double delta_time = 0.005;

    double l1 = 0.02;
    double l2 = 0.04;
    double l3 = 0.035;

    bool contact_Flag = false;


};

int main(int argc, char * argv[])
{
//ros2 topic pub /joint_1/command std_msgs/msg/Float64 "{data: 1.0}"
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sedas_rviz>());
  rclcpp::shutdown();
  return 0;
}