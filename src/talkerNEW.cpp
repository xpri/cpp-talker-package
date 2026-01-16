#include <chrono>
// #include <functional>        // No need for this since lambda function is used instead of std::bind
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
            
            // NO NEED FOR THIS IF USING LAMBDA
            // timer_= this->create_wall_timer(500ms, std::bind(&TalkerPublisher::timer_callback, this));

            timer_ = this->create_wall_timer
            (
                500ms,
                [this]()
                {
                    auto message = sandbox_interfaces::msg::SensorData();
                    
                    message.sensor_id = this->count_;
                    message.status = "Operating Normally!";
                    message.temperature = 72;      // Saying in fahrenheit
                    message.active = true;

                    RCLCPP_INFO(this->get_logger(), "Publishing ID: #%d | Temperature = %d", message.sensor_id, message.temperature);
                                        
                    this->count_++;

                    this->publisher_->publish(message);
                }
            );        
        }
    private:
        rclcpp::Publisher<sandbox_interfaces::msg::SensorData>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerPublisher>());
  rclcpp::shutdown();
  return 0;
}