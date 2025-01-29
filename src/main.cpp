#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <jetsonGPIO.h>

class GpioControlNode {
public:
    GpioControlNode() {
        // Get the GPIO pin from the parameter server (default is pin 18)
        ros::param::param<int>("~gpio_pin", gpio_pin_, 18);
        
        // Set up GPIO
        if (jetsonGPIOInit() != 0) {
            ROS_ERROR("Failed to initialize Jetson GPIO!");
            ros::shutdown();
            return;
        }

        // Set the specified GPIO pin as an output
        if (GPIOpinMode(gpio_pin_, outputPin) != 0) {
            ROS_ERROR("Failed to set GPIO pin mode!");
            ros::shutdown();
            return;
        }

        // Subscribe to the /gpio_control topic to receive Boolean messages
        subscriber_ = nh_.subscribe("/gpio_control", 10, &GpioControlNode::gpioCallback, this);
        
        ROS_INFO("Using GPIO pin %d", gpio_pin_);
    }

    ~GpioControlNode() {
        // Clean up GPIO settings when node is shut down
        GPIOpinMode(gpio_pin_, inputPin);
        jetsonGPIOExit();
    }

    void gpioCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            GPIOpinWrite(gpio_pin_, high);  // Set GPIO pin HIGH
            ROS_INFO("GPIO Pin %d set HIGH", gpio_pin_);
        } else {
            GPIOpinWrite(gpio_pin_, low);   // Set GPIO pin LOW
            ROS_INFO("GPIO Pin %d set LOW", gpio_pin_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    int gpio_pin_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_control_node");

    GpioControlNode gpio_control_node;
    
    ros::spin();

    return 0;
}
