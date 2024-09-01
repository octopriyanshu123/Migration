#ifndef MY_ACTUATOR_ROS2__MY_ACTUATOR_CONTROL_HPP_
#define MY_ACTUATOR_ROS2__MY_ACTUATOR_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int8.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"


#include <cstdlib>
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include "/home/octobotics/ros2_ws/src/my_actuator_ros2/RmdApiSerial/rmd-sdk/include/x10_api.hpp"
#include "interfaces/msg/plot.hpp"
#include "interfaces/msg/vitals.hpp"
#include "interfaces/srv/relay_control.hpp"
#include "interfaces/msg/anglinarr.hpp"
#include "interfaces/msg/tempmsgs.hpp"

using namespace std;

enum JoyStick
{
    // axes
    FORWARD_BACKWARD = 3,
    LEFT_RIGHT = 0,

    // buttons
    ACT1_2_ON_OFF = 4, // l1
    ACT3_4_ON_OFF = 5, // r1
};

struct vit
{
    float voltage;
    int error;
    int temperature;
    int16_t current;
};



class MyActuatorControl : public rclcpp::Node
{
public:
    MyActuatorControl(const rclcpp::NodeOptions &options);

    ~MyActuatorControl(); // Destructor

    void get_params();

    void init_subs_pubs_srvs();

    void init_timer();

    bool init_teleop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool stop_teleop_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool stop_motors_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool reset_motors_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool reset_tripmeter_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool set_joint_angle_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool increaseRasterSpeedCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

    bool decreaseRasterSpeedCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);
    
    bool cycleAllMotorCallback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);


    //void get_speed();

    void get_tripmeter();

    void get_odometer();

    void init_actuators();

    void stop_actuators(int id);

    void run_actuator(int id, double vel);

    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void get_joy(const sensor_msgs::msg::Joy::SharedPtr joy);

    void actuator_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void toggle_joy_callback_(const std_msgs::msg::Bool::SharedPtr msg);

    void get_vitals(int id_);

    void motor_status();

    //void get_voltage();

    void linearMotorCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void angleValueCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void strokeLengthCallback(const std_msgs::msg::Int16::SharedPtr msg);

    //void publishRotationTime();

    void clockwise();

    void anticlockwise();

    void stop();

    

    //void scanSpeed();
    

private:
    // Node-specific parameters
    std::string port_;
    X10ApiSerial *act_api_;

    // Publishers
    rclcpp::Publisher<interfaces::msg::Vitals>::SharedPtr vitals_pub_;
    rclcpp::Publisher<interfaces::msg::Vitals>::SharedPtr vitals_pub_2_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vitals_pub_x;
    rclcpp::Publisher<interfaces::msg::Plot>::SharedPtr plot_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr odometer_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr scan_speed_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr actuator_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_real_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr linear_motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr angle_value_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr stroke_length_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_teleop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_teleop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motors_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_motors_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tripmeter_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_joint_angle_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr increase_raster_speed_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr decrease_raster_speed_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cycle_all_motor_srv_;

    // Clients
    std::string toggle_relay_client_name;
    rclcpp::Client<interfaces::srv::RelayControl>::SharedPtr toggle_relay_client_;

    vector<int> motor_ids;
    vector<vit> health;

    interfaces::msg::Vitals send_vitals;
    std_msgs::msg::Float64MultiArray send_vitals_2;


    interfaces::srv::RelayControl control_relay_;
    interfaces::msg::Tempmsgs temmsgs ; 

    map<int, string> error_;

    int toggle_joy_flag_;

    int velocity_scale_;

    int right_front_wheel_id_;
    int left_front_wheel_id_;
    int right_rear_wheel_id_;
    int left_rear_wheel_id_;

    int right_front_wheel_dir_;
    int left_front_wheel_dir_;
    int right_rear_wheel_dir_;
    int left_rear_wheel_dir_;
    int velocity_scake;
    int angle_value_;
    int raster_speed_;
    int16_t rotation_value_;
    int16_t stroke_length_;

    // Actuator enable/disable flags
    bool act1_sw_;
    bool act2_sw_;
    bool act3_sw_;
    bool act4_sw_;
    bool act1_enabled_;
    bool act2_enabled_;
    bool act3_enabled_;
    bool act4_enabled_;
    bool init_teleop_;
    bool reset_tripmeter_flag_;
    float velocity_left_cmd_;
    float velocity_right_cmd_;
    double WHEEL_BASE_;
    double WHEEL_RADIUS_;
    std::string toggle_joy_topic_;
    geometry_msgs::msg::Twist vel;

    

    // Timers
    rclcpp::TimerBase::SharedPtr joy_reader_timer_;
    rclcpp::TimerBase::SharedPtr actuator_status_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_2_;
    rclcpp::TimerBase::SharedPtr distance_timer_;
    rclcpp::TimerBase::SharedPtr odometer_timer_;
    rclcpp::TimerBase::SharedPtr speed_timer_;
    rclcpp::TimerBase::SharedPtr voltage_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    void steer_calc_callback_();
    void actuator_status_callback_(const rclcpp::TimerBase::SharedPtr timer);
    void health_callback_();
    void health_callback_2_();
    void get_speed();
    void get_tripmeter(const rclcpp::TimerBase::SharedPtr timer);
    void get_odometer_callback();
    void get_voltage();
    void publishRotationTime();
    void scanSpeed();
};

class JoyTeleop : public rclcpp::Node
{
public:
    JoyTeleop();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void toggleJoyCallback(const std_msgs::msg::Bool::SharedPtr msg);

    int lin_inc, ang_inc;
    int linear_, angular_;
    int l_inc_, a_inc_, l_dec_, a_dec_;
    int toggle_joy_flag_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_joy_sub_;
    rclcpp::Publisher<interfaces::msg::Anglinarr>::SharedPtr vel_gui_pub_;
};

#endif // MY_ACTUATOR_ROS2__MY_ACTUATOR_CONTROL_HPP_
