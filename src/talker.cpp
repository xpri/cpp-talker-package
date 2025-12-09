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
        
        /*
            Activates the timer_ variable as an object with the create_wall_timer function from the ros2 node library.
            The create_wall_timer function monitors the real world time aka the "wall clock"
            It makes a "tick" every 500ms or twice a second for every second in actual real world. So every 500ms it calls something...
            std::bind(&TalkerPublisher::timer_callback, this) is what it calls every 500ms. or every time the "alarm" goes off (500ms thing)
            std::bind squishes everything inside into one thing that the alarm can go off for.
            &TalkerPublisher::timer_callback is specifying where to look for the code. The & gets the memory address of the function and
            the TalkerPublisher::timer_callback specifies that the timer_callback is inside of the TalkerPublisher blueprint.
            this is the single and most important piece of code. It says that when you run that specific thing you run them on ME. aka the 
            specific TalkerPublisher object, the one we are currently inside
        */
        timer_= this->create_wall_timer(500ms, std::bind(&TalkerPublisher::timer_callback, this));
    }
private:

    void timer_callback()
    {
    /*
        Smarter way to use c++ smart pointers. It calles make_unique which is a robust way to delete the pointer after it is done using it (blah blah blah)
        Then the <std_msgs::msg::String>() means it creates an actual message object of the String data type.
    */ 
    auto message = std::make_unique<std_msgs::msg::String>();
    }

    /* 
        -> means to go to what the pointer message holds (in this case a message object of the String data type) and access it.
        (goes to what message points to and access the "data"). In this case it assignes a value to the data variable. And stores
        the hello world plus blah. The blah is a to string of the current count (and increments it after that current line of code is called).
    */
    message->data = "Hello World: " + std::to_string(count_++);

}