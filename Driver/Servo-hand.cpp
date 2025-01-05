#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pigpio.h> // Include the pigpio library for PWM control

class ServoMotorDriver : public rclcpp::Node
{
public:
    ServoMotorDriver() : Node("servo_motor_driver")
    {
        // Declare and get parameters
        this->declare_parameter<int>("pwm_pin", 18); // GPIO pin
        this->declare_parameter<int>("pwm_frequency", 50); // PWM frequency
        this->declare_parameter<int>("min_pulse_width", 500); // Min pulse width in microseconds
        this->declare_parameter<int>("max_pulse_width", 2500); // Max pulse width in microseconds
        this->declare_parameter<std::string>("servo_topic", "servo_angle");

        pwm_pin_ = this->get_parameter("pwm_pin").as_int();
        pwm_frequency_ = this->get_parameter("pwm_frequency").as_int();
        min_pulse_width_ = this->get_parameter("min_pulse_width").as_int();
        max_pulse_width_ = this->get_parameter("max_pulse_width").as_int();
        servo_topic_ = this->get_parameter("servo_topic").as_string();

        // Initialize pigpio library
        if (gpioInitialise() < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize pigpio library. Ensure it is installed and running.");
            throw std::runtime_error("Failed to initialize pigpio");
        }

        // Configure the PWM pin
        gpioSetMode(pwm_pin_, PI_OUTPUT);

        // Create subscription for servo control
        servo_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            servo_topic_, 10,
            std::bind(&ServoMotorDriver::servoCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servo Motor Driver Node started.");
        RCLCPP_INFO(this->get_logger(), "Listening on topic: %s", servo_topic_.c_str());
    }

    ~ServoMotorDriver()
    {
        // Turn off PWM signal and terminate pigpio library
        gpioPWM(pwm_pin_, 0);
        gpioTerminate();
        RCLCPP_INFO(this->get_logger(), "Servo Motor Driver Node stopped. PWM disabled.");
    }

private:
    void servoCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double angle = msg->data;

        // Validate the input angle
        if (angle < 0.0 || angle > 180.0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid angle received: %f. Expected range: 0-180.", angle);
            return;
        }

        // Map the angle to a pulse width
        int pulse_width = static_cast<int>(min_pulse_width_ + (angle / 180.0) * (max_pulse_width_ - min_pulse_width_));

        // Set the PWM signal for the servo
        if (gpioServo(pwm_pin_, pulse_width) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set PWM signal for angle: %f, pulse width: %d.", angle, pulse_width);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Servo set to angle: %f°, Pulse width: %d μs", angle, pulse_width);
    }

    // Node parameters
    int pwm_pin_;
    int pwm_frequency_;
    int min_pulse_width_;
    int max_pulse_width_;
    std::string servo_topic_;

    // ROS2 subscription
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoMotorDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
