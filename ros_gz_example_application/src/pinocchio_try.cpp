#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>         // CRBA(관성 행렬 계산)
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pinocchio/algorithm/jacobian.hpp>    // Jacobian 계산
#include <pinocchio/algorithm/frames.hpp>   // Frames 업데이트


class PinocchioHandler : public rclcpp::Node {
public:
    PinocchioHandler()
        : Node("pinocchio_handler"), model(), data(model) {
        // URDF 파일 경로

      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        const std::string urdf_filename = "/home/mrl-seuk/ros2_ws/src/ros_gz_project_template/ros_gz_example_description/models/manipulator/model.urdf";
        // Pinocchio 모델 초기화
        try {
            pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model);
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully with nq=%d, nv=%d", model.nq, model.nv);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading URDF: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        data = pinocchio::Data(model);

        // 상태 벡터 및 입력 벡터 초기화
        state = Eigen::VectorXd::Zero(model.nq);
        input_data = Eigen::VectorXd::Zero(model.nv);

        // 타이머 설정 (디버깅용, 주기적으로 RNEA 계산)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 20Hz = 50ms
            std::bind(&PinocchioHandler::timerCallback, this));

      gravity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pinnochio/gravity", qos_settings);
      EE_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pinnochio/EE_vel", qos_settings);
      EE_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pinnochio/EE_pos", qos_settings);

        // ROS2 구독자 생성
        state_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/state_vector", 10,
            std::bind(&PinocchioHandler::stateCallback, this, std::placeholders::_1));

        state_dot_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/state_dot_vector", 10,
            std::bind(&PinocchioHandler::stateDotCallback, this, std::placeholders::_1));


        input_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/input_vector", 10,
            std::bind(&PinocchioHandler::inputCallback, this, std::placeholders::_1));
    }

private:
    pinocchio::Model model;
    pinocchio::Data data;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_dot_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr EE_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr EE_pos_publisher_;





void timerCallback() {
    if (state.size() != model.nq || input_data.size() != model.nv) {
        RCLCPP_ERROR(this->get_logger(), "State or input vector size mismatch!");
        return;
    }

    try {

        CalcJacobian();
        CalcMCGDynamics();
        CalcEndEffectorPose(); // 추가
        data_publish();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during RNEA computation: %s", e.what());
    }
}


    void data_publish()
    {


    }

    void CalcJacobian()
    {
        // Jacobian 계산
        pinocchio::computeJointJacobians(model, data, state);
        pinocchio::updateFramePlacements(model, data); // 모든 프레임 업데이트

        // 엔드 이펙터 프레임 ID 가져오기
        pinocchio::FrameIndex ee_frame_id = model.getFrameId("link_EE");

        // Jacobian 행렬 초기화
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // 크기 검증
        if (state_dot.size() != model.nv) {
            RCLCPP_ERROR(this->get_logger(), "Mismatch: state_dot size (%ld) != model.nv (%d)", 
                        state_dot.size(), model.nv);
            return;
        }

        if (J.cols() != state_dot.size()) {
            RCLCPP_ERROR(this->get_logger(), "Mismatch: Jacobian cols (%d) != state_dot size (%ld)",
                        J.cols(), state_dot.size());
            return;
        }

        // 엔드 이펙터 속도 계산
        Eigen::VectorXd ee_velocity = J * state_dot;

        // 선형 및 각속도 분리
        Eigen::Vector3d linear_velocity = ee_velocity.head<3>();
        Eigen::Vector3d angular_velocity = ee_velocity.tail<3>();

        // 로그 출력
        // RCLCPP_INFO(this->get_logger(), "End effector linear velocity: [%f, %f, %f]",
        //             linear_velocity[0], linear_velocity[1], linear_velocity[2]);
        // RCLCPP_INFO(this->get_logger(), "End effector angular velocity: [%f, %f, %f]",
        //             angular_velocity[0], angular_velocity[1], angular_velocity[2]);



        std_msgs::msg::Float64MultiArray EE_vel_msg;
        EE_vel_msg.data.push_back(linear_velocity[0]);
        EE_vel_msg.data.push_back(linear_velocity[1]);
        EE_vel_msg.data.push_back(linear_velocity[2]);
        EE_vel_msg.data.push_back(angular_velocity[0]);
        EE_vel_msg.data.push_back(angular_velocity[1]);
        EE_vel_msg.data.push_back(angular_velocity[2]);
        EE_vel_publisher_->publish(EE_vel_msg);
    }


    void CalcMCGDynamics()
    {
        // RNEA 호출
        Eigen::VectorXd G = pinocchio::rnea(model, data, state, state_dot, state_ddot);

        // 관성 행렬(M) 계산
        pinocchio::crba(model, data, state);
        Eigen::MatrixXd M = data.M; // 관성 행렬


        // 결과 출력
        std::stringstream ss_g, ss_m, ss_state;
        ss_g << G;
        ss_state << "\n" << state;
        ss_m << "\n" << M;
        // RCLCPP_INFO(this->get_logger(), "State vector: %s \n", ss_state.str().c_str());
        // RCLCPP_INFO(this->get_logger(), "Gravity vector:\n %s \n", ss_g.str().c_str());
        // RCLCPP_INFO(this->get_logger(), "Mass matrix M: %s \n", ss_m.str().c_str());


        std_msgs::msg::Float64MultiArray gravity_msg;
        for (int i =0; i<9; i++){
        gravity_msg.data.push_back(G[i]);
        }
        gravity_publisher_->publish(gravity_msg);
    }

    void CalcEndEffectorPose() {
        pinocchio::framesForwardKinematics(model, data, state);

        pinocchio::FrameIndex ee_frame_id = model.getFrameId("link_EE");
        const auto &ee_pose = data.oMf[ee_frame_id]; // 엔드 이펙터의 변환 행렬

        Eigen::Vector3d position = ee_pose.translation(); // 위치
        Eigen::Matrix3d orientation = ee_pose.rotation(); // 자세 (회전 행렬)

        // RCLCPP_INFO(this->get_logger(), "End effector position: [%f, %f, %f]",
        //             position[0], position[1], position[2]);
        // RCLCPP_INFO(this->get_logger(), "End effector orientation:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
        //             orientation(0, 0), orientation(0, 1), orientation(0, 2),
        //             orientation(1, 0), orientation(1, 1), orientation(1, 2),
        //             orientation(2, 0), orientation(2, 1), orientation(2, 2));

    //Publisher 만들어서 교차검증하자
        std_msgs::msg::Float64MultiArray position_msg;
    position_msg.data.push_back(position[0]);
    position_msg.data.push_back(position[1]);
    position_msg.data.push_back(position[2]);
    EE_pos_publisher_->publish(position_msg);
    
    }


    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != model.nq) {
            RCLCPP_WARN(this->get_logger(), "Received state vector size mismatch: %ld (expected %ld)",
                        msg->data.size(), model.nq);
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            state[i] = msg->data[i];
        }
    }

    void stateDotCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != model.nv) {
            RCLCPP_WARN(this->get_logger(), "Received state_DOT vector size mismatch: %ld (expected %ld)",
                        msg->data.size(), model.nv);
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            state_dot[i] = msg->data[i];
        }


        // RCLCPP_INFO(this->get_logger(), "State_DOT: [%f, %f, %f] [%f, %f, %f]",
        //             state_dot[0], state_dot[1], state_dot[2],
        //             state_dot[3], state_dot[4], state_dot[5]
        //             );

    }

    void inputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != model.nv) {
            RCLCPP_WARN(this->get_logger(), "Received input vector size mismatch: %ld (expected %ld)",
                        msg->data.size(), model.nv);
            return;
        }

        for (size_t i = 0; i < msg->data.size(); ++i) {
            input_data[i] = msg->data[i];
        }
    }



    Eigen::VectorXd state = Eigen::VectorXd::Zero(10);    
    Eigen::VectorXd input_data;
    Eigen::VectorXd state_dot = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd state_ddot = Eigen::VectorXd::Zero(9);



};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinocchioHandler>());
    rclcpp::shutdown();
    return 0;
}
