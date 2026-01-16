#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sandbox_interfaces/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class TalkerPublisher : public rclcpp::Node
{
    public:
        TalkerPublisher()
        : Node("talker_publisher"),
        count_(0)
        {
            // Makes a topic with name 'sensor_topic' with the structure of the SensorData from sandbox_interfaces.
            // 10 means how many items can be stored in the buffer
            publisher_ = this->create_publisher<sandbox_interfaces::msg::SensorData>("sensor_topic", 10);
            
            // timer_= this->create_wall_timer(500ms, std::bind(&TalkerPublisher::timer_callback, this));
            timer_ = this->create_wall_timer(500ms [](){});
        }
    private:
        void timer_callback()
        {
            auto message = sandbox_interfaces::msg::SensorData();

            message.sensor_id = count_
            message.status = "Operating Normally!";
            message.temperature = "72";      // Saying in fahrenheit
            message.active = true;

            RCLCPP_INFO(this->get_logger(), "%s", std::format("Publishing ID: %d | Temp: %.2f", message.sensor_id, message.temperature).c_str())
            
            publisher_->publish(message);
        }
        
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sandbox_interfaces::msg::SensorData>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}