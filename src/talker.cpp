#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Include the ROS 2 C++ client library
#include "rclcpp/rclcpp.hpp"
// Include the standard string message type library
#include "std_msgs/msg/string.hpp"

// To use s, ms, etc
using namespace std::chrono_literals;

class TalkerPublisher : public rclcpp::Node
{
public:
    /*
        Following line explination for myself:
        TalkerPublisher() initializes a node and the publishing mechanism is set up.
        
        Node("talker_publisher"), count_(0)    // Calls the parent class constructor rclcpp::Node
        and assigns the name "talker_publisher" to this node in the ROS2 network. It also initializes a counter variable (count_) to 0.
    */
    TalkerPublisher()
        : Node("talker_publisher"),
        count_(0)     //initializes node with the name "talker_publisher"
    {
        /*
            Activates the publisher_ variable as an object with the create_publisher function from the ros2 node library. 
            the std_msgs::msg::String is just a parameter for create_publisher to create a ros2 object that only accepts that specific data type.
            ("topic", 10) is another parameter. It publishes data to the channel called "topic". The 10 is a buffer, aka it can only store up to 10
            messages. If another message wants to send and there are already 10 messages in the buffer, the message fails.
            Difference between the <> and (), the <> are for things that are static and never change. And the () are for dynamic things that could change later on.
        */
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // Btw 'this->' is just refering to the pointer of the current instance of the talker_publisher object that it is referring to.
    }
private:

}