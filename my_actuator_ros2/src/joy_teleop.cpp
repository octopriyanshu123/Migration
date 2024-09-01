


//#include "my_actuator_ros2/msg/anglinarr.hpp"
#include "my_actuator_ros2/my_actuator_control.hpp"




JoyTeleop::JoyTeleop() : Node("teleop_joy")
{
    // Declare parameters
    this->declare_parameter("axis_linear", 3);
    this->declare_parameter("axis_angular", 0);

    // Get parameters
    this->get_parameter("axis_linear", linear_);
    this->get_parameter("axis_angular", angular_);

    // Publishers and Subscribers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("skid_steer/cmd_vel", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyTeleop::joyCallback, this, std::placeholders::_1));
    toggle_joy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/toggle_joy", 10, std::bind(&JoyTeleop::toggleJoyCallback, this, std::placeholders::_1));
    vel_gui_pub_ = this->create_publisher<interfaces::msg::Anglinarr>("/vel_status", 10);

    lin_inc = 5;
    ang_inc = 5;

    l_inc_ = 3; a_inc_ = 0; l_dec_ = 1; a_dec_ = 2;
    toggle_joy_flag_ = -1;

    RCLCPP_WARN(this->get_logger(), "teleop_joy NODE STARTED!!!");
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (toggle_joy_flag_ == -1)
    {
        RCLCPP_INFO(this->get_logger(), "inside: %d crawler", toggle_joy_flag_);

        auto twist = geometry_msgs::msg::Twist();
        lin_inc += joy->buttons[l_inc_] ? 1 : 0;
        ang_inc += joy->buttons[a_inc_] ? 2 : 0;

        lin_inc -= joy->buttons[l_dec_] ? 1 : 0;
        ang_inc -= joy->buttons[a_dec_] ? 2 : 0;

        lin_inc = boost::algorithm::clamp(lin_inc, 0, 28);
        ang_inc = boost::algorithm::clamp(ang_inc, 0, 15);

        if (joy->buttons[l_inc_] || joy->buttons[a_inc_] || joy->buttons[l_dec_] || joy->buttons[a_dec_])
        {
            RCLCPP_INFO(this->get_logger(), "linear: %d , angular: %d", lin_inc, ang_inc);
        }

        twist.angular.z = ang_inc * joy->axes[angular_];
        twist.linear.x = lin_inc * joy->axes[linear_];

        auto gui_vel = interfaces::msg::Anglinarr();
        gui_vel.name = "velocity_info";
        gui_vel.coordinates.x = twist.linear.x;
        gui_vel.coordinates.y = twist.angular.z;
        gui_vel.coordinates.z = 0.0;

        vel_gui_pub_->publish(gui_vel);
        vel_pub_->publish(twist);
    }
}

void JoyTeleop::toggleJoyCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    toggle_joy_flag_ = msg->data ? 1 : -1;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyTeleop>());
    rclcpp::shutdown();
    return 0;
}

