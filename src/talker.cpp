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
    TalkerPublisher() : Node("talker_publisher"), count_(0)     //initializes node with the name "talker_publisher"
    {
        
    }
private:

}