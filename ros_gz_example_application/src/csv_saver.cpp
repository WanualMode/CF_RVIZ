#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <filesystem>
#include <csignal>

namespace fs = std::filesystem;

class FilteredCSVLogger : public rclcpp::Node
{
public:
    FilteredCSVLogger(const std::string &output_csv)
        : Node("filtered_csv_logger"), output_csv_(output_csv)
    {
        fs::path dir_path = fs::path(output_csv_).parent_path();
        if (!fs::exists(dir_path))
        {
            fs::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", dir_path.c_str());
        }

        csv_file_.open(output_csv_, std::ios::out | std::ios::app);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", output_csv_.c_str());
            return;
        }

        if (fs::file_size(output_csv_) == 0)
        {
            write_header();
        }

        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/kroc_data", 10,
            std::bind(&FilteredCSVLogger::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Logging to CSV: %s", output_csv_.c_str());
    }

    ~FilteredCSVLogger()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed: %s", output_csv_.c_str());
        }
    }

private:
    std::string output_csv_;
    std::ofstream csv_file_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

    void write_header()
    {
        csv_file_ << "Timestamp,Data_0,Data_1,Data_2,Data_3,Data_4,Data_5,Data_6" << std::endl;
    }

    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 7)
        {
            RCLCPP_WARN(this->get_logger(), "Received invalid data (size < 7), ignoring.");
            return;
        }

        auto timestamp = this->now().nanoseconds();
        csv_file_ << timestamp;
        for (const auto &value : msg->data)
        {
            csv_file_ << "," << value;
        }
        csv_file_ << std::endl;
        csv_file_.flush();
    }
};

void signal_handler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("filtered_csv_logger"), "Caught signal %d, shutting down.", signum);
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string default_csv_path = "/home/mrlseuk/kroc_data/kroc_filtered_data.csv";
    std::string csv_path = (argc >= 2) ? argv[1] : default_csv_path;

    std::signal(SIGINT, signal_handler);
    auto node = std::make_shared<FilteredCSVLogger>(csv_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
