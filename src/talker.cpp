#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Include the ROS 2 C++ client library
#include "rclcpp/rclcpp.hpp"
// Include the standard string message type library
#include "std_msgs/msg/string.hpp"
// Include the new custom message header
#include "sandbox_interfaces/msg/sensor_data.hpp"

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
            messages. If another message wants to send and there are already 10 messages in the buffer, the oldest message gets dropped.
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
        // The resulting object is now called a "function object" or "callable" in c++.
    }
private:

    void timer_callback()
    {
        /*
            Smarter way to use c++ smart pointers. It calles make_unique which is a robust way to delete the pointer after it is done using it (blah blah blah)
            Then the <std_msgs::msg::String>() means it creates an actual message object of the String data type.
        */ 
        auto message = std::make_unique<std_msgs::msg::String>();
        /* 
            -> means to go to what the pointer message holds (in this case a message object of the String data type) and access it.
            (goes to what message points to and access the "data"). In this case it assignes a value to the data variable. And stores
            the hello world plus blah. The blah is a to string of the current count (and increments it after that current line of code is called).
        */
        message->data = "Hello World: " + std::to_string(count_++);
        
        /*
            This is a logging macro provided by the ros2 library. It outputs the message content to the console/log file with the INFO severity level.
            this->get_logger() gets the logger associated with this node. All nodes are automatically initialized with their own log.
            And the macro needs to know which node is sending what log. It would input something similar to [INFO] [minimal_publisher]:
            The data.c_str() is that it converts it to a c-style string which is required for the logging in the macro.
        */
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->data.c_str());
        
        /*
            Enters and accesses the publisher_'s publish function and moves the message object of String data type from the macro to be used
            for the publish function. Since message is a smart pointer then only one "owner" can access or "own" the pointer at a time.
            Since the timer_callback is using the pointer currently then its ownership will be transferred to the publish function member inside of publisher_
        */
        publisher_->publish(std::move(message));
    }
    // Private member variables

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    // Initializes the communication layer of the ros2 node with the inputs from the command line with argc, and argv.
    rclcpp::init(argc, argv);

    // Make an object of the MinimalPublisher class
    rclcpp::spin(std::make_shared<TalkerPublisher>());
    /*
        This takes the object and puts it into an infinite processing loop. This also blocks the rest of the code until "spin" exits. While it is spinning ROS2
        constantly monitors the node for events. (like incoming data/data requests from other nodes or the timer function I also made).
        The loop only exits when ROS2 is terminated (aka like ctrl + c).
    */
    // Once spin exits shutdown the communication layer
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}