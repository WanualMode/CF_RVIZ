#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시
#include <std_msgs/msg/string.hpp>  // 다중 float64 배열 퍼블리시
#include <string> // std::string 헤더 추가
#include "std_msgs/msg/float64.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ncurses.h> // ncurses 헤더
#include "sedas_rot.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "ButterworthFilter.hpp"
#include "FilteredVector.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class sedas_traj : public rclcpp::Node
{
  public:
    sedas_traj()
      : Node("sedas_traj"), 
      count_(0),
      rpy_cmd_filter(3, 1., 0.01), // FilteredVector 초기화
      xyz_cmd_filter(3, 1, 0.01) // FilteredVector 초기화
    {      
      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
          "keyboard_input", qos_settings,
          std::bind(&sedas_traj::keyboard_subsciber_callback, this, std::placeholders::_1));

      Normal_rpy_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/Normal_Vector_rpy_angle", qos_settings,
          std::bind(&sedas_traj::Normal_rpy_subscriber_callback, this, std::placeholders::_1));



    drone_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/manipulator/EE_cmd", 10);

    // Joint EE Subscriber
    joint_EE_torque_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/force_torque_EE", 10,  // Topic name and QoS depth
        std::bind(&sedas_traj::jointEE_torque_Callback, this, std::placeholders::_1));

    EE_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pinnochio/EE_vel", qos_settings,
        std::bind(&sedas_traj::EE_vel_callback, this, std::placeholders::_1)); 


      timer_ = this->create_wall_timer(
      10ms, std::bind(&sedas_traj::timer_callback, this));


  // EE_xyz_position_cmd[0] = 0.5;
  // EE_xyz_position_cmd[2] = 1;

    }

  private:
    void timer_callback()
    {   


        traj_gen();
        data_publisher();
    }



    void data_publisher()
    {
    // EE_xyz_position_cmd_filtered = xyz_cmd_filter.apply(EE_xyz_position_cmd);
      std_msgs::msg::Float64MultiArray drone_cmd;
      drone_cmd.data.push_back(EE_xyz_position_cmd[0]);
      drone_cmd.data.push_back(EE_xyz_position_cmd[1]);
      drone_cmd.data.push_back(EE_xyz_position_cmd[2] +1);
      drone_cmd.data.push_back(EE_rpy_position_cmd[0]);
      drone_cmd.data.push_back(EE_rpy_position_cmd[1]);
      drone_cmd.data.push_back(EE_rpy_position_cmd[2]);


      drone_cmd_publisher_->publish(drone_cmd);
    }





void traj_gen()
{

        EE_xyz_position_cmd += EE_xyz_vel_cmd * 0.01;
        EE_rpy_position_cmd += EE_rpy_vel_cmd * 0.01;


}







  void Normal_rpy_subscriber_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    Normal_rpy_angle[0] = msg->data[0];
    Normal_rpy_angle[1] = msg->data[1];
    Normal_rpy_angle[2] = msg->data[2];
  }


  void keyboard_subsciber_callback(const std_msgs::msg::String::SharedPtr msg)
  {
      // 입력된 키를 문자열로 가져옴
      std::string input = msg->data;

        if (!input.empty()) // 입력 값이 비어있지 않을 경우
        {
            char input_char = input[0]; // 문자열의 첫 번째 문자만 사용

            if (input_char == 'w')
            {
                EE_xyz_vel_cmd[0] += 0.1;
            }
            else if (input_char == 's')
            {
                EE_xyz_vel_cmd[0] -= 0.1;
            }
            else if (input_char == 'a')
            {
                EE_xyz_vel_cmd[1] += 0.1;
            }
            else if (input_char == 'd')
            {
                EE_xyz_vel_cmd[1] -= 0.1;
            }
            else if (input_char == 'e')
            {
                EE_xyz_vel_cmd[2] += 0.1;
            }
            else if (input_char == 'q')
            {
                EE_xyz_vel_cmd[2] -= 0.1;
            }
            else if (input_char == 'z')
            {
                EE_rpy_vel_cmd[2] += 0.1;
            }
            else if (input_char == 'c')
            {
                EE_rpy_vel_cmd[2] -= 0.1;
            }
            else if (input_char == 'n' || input_char == 'x')
            {
                EE_xyz_vel_cmd[0] = 0;
                EE_xyz_vel_cmd[1] = 0;
                EE_xyz_vel_cmd[2] = 0;

                EE_rpy_vel_cmd[0] = 0;
                EE_rpy_vel_cmd[1] = 0;
                EE_rpy_vel_cmd[2] = 0;
            }
        }
          else
          {
              RCLCPP_WARN(this->get_logger(), "입력된 키가 없습니다!");
          }


    }

  void jointEE_torque_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
      External_force_sensor_meas[0] = msg->wrench.force.x;
      External_force_sensor_meas[1] = msg->wrench.force.y;
      External_force_sensor_meas[2] = msg->wrench.force.z;
      // RCLCPP_INFO(this->get_logger(), "EE_FORCE [%lf] [%lf] [%lf]", External_force_sensor_meas[0], External_force_sensor_meas[1], External_force_sensor_meas[2]);    

  }

  void EE_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {  

      EE_lin_vel[0] = msg->data[0];
      EE_lin_vel[1] = msg->data[1];
      EE_lin_vel[2] = msg->data[2];
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_visual;



  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_; 
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Normal_rpy_angle_subscriber_; 
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drone_cmd_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr joint_EE_torque_subscriber_; 
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EE_vel_subscriber_; 



  size_t count_;
  std_msgs::msg::Float64 joint_1_cmd_msg;
  std_msgs::msg::Float64 joint_2_cmd_msg;
  std_msgs::msg::Float64 joint_3_cmd_msg;    
  //TODO:: 아래 세 줄 정의 제대로 하기


  Eigen::Vector3d drone_xyz_position_cmd = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d drone_xyz_vel_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d drone_xyz_normal_vel_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d External_force_sensor_meas = Eigen::Vector3d::Zero();
  Eigen::Vector3d External_force_sensor_meas_global = Eigen::Vector3d::Zero();

  Eigen::Vector3d EE_xyz_position_cmd_filtered = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d drone_rpy_position_cmd = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d drone_rpy_vel_cmd = Eigen::Vector3d::Zero();
  Eigen::Vector3d drone_rpy_normal_vel_cmd = Eigen::Vector3d::Zero();

  Eigen::Vector3d EE_rpy_position_cmd = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d EE_xyz_position_cmd = Eigen::Vector3d::Zero();

  Eigen::Vector3d EE_xyz_vel_cmd = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d EE_rpy_vel_cmd = Eigen::Vector3d::Zero();


  Eigen::Vector3d EE_lin_vel;
  Eigen::Vector3d Normal_rpy_angle;



  Eigen::Matrix3d R_B2G;


    double time;
    double time_cnt;
    double sine;

    double delta_time = 0.005;

    double l1 = 0.1;
    double l2 = 0.2;
    double l3 = 0.2;
    double external_force_norm = 0;



  FilteredVector rpy_cmd_filter;
  FilteredVector xyz_cmd_filter;

};

int main(int argc, char * argv[])
{
//ros2 topic pub /joint_1/command std_msgs/msg/Float64 "{data: 1.0}"
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sedas_traj>());
  rclcpp::shutdown();
  return 0;
}