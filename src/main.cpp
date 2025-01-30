#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gpiod.h>

#define GPIO_CHIP "/dev/gpiochip0" // GPIO chip device

class GPIOController {
public:
    GPIOController(ros::NodeHandle& nh, int gpio_pin) : gpio_pin_(gpio_pin) {
        // Initialize GPIO
        chip = gpiod_chip_open(GPIO_CHIP);
        if (!chip) {
            ROS_ERROR("Failed to open GPIO chip.");
            ros::shutdown();
        }

        line = gpiod_chip_get_line(chip, gpio_pin_);
        if (!line) {
            ROS_ERROR("Failed to get GPIO line %d.", gpio_pin_);
            gpiod_chip_close(chip);
            ros::shutdown();
        }

        if (gpiod_line_request_output(line, "gpio_control", 0) < 0) {
            ROS_ERROR("Failed to request GPIO line %d as output.", gpio_pin_);
            gpiod_chip_close(chip);
            ros::shutdown();
        }

        // Subscribe to a topic that controls the GPIO state
        gpio_sub = nh.subscribe("/gpio_control", 10, &GPIOController::gpioCallback, this);
        ROS_INFO("GPIO Controller Node Started. Controlling GPIO %d. Listening on /gpio_control", gpio_pin_);
    }

    ~GPIOController() {
        gpiod_line_set_value(line, 0);  // Turn off before exiting
        gpiod_line_release(line);
        gpiod_chip_close(chip);
    }

private:
    gpiod_chip* chip;
    gpiod_line* line;
    int gpio_pin_;
    ros::Subscriber gpio_sub;

    void gpioCallback(const std_msgs::Bool::ConstPtr& msg) {
        int value = msg->data ? 1 : 0;
        gpiod_line_set_value(line, value);
        ROS_INFO("GPIO %d set to %d", gpio_pin_, value);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpio_control_node");
    ros::NodeHandle nh("~"); // Use private namespace for parameters

    int gpio_pin;
    if (!nh.getParam("gpio_pin", gpio_pin)) {
        ROS_ERROR("No GPIO pin provided! Use _gpio_pin:=XX when running.");
        return 1;
    }

    GPIOController gpioController(nh, gpio_pin);
    ros::spin();

    return 0;
}