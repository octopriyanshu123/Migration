#include "my_actuator_ros2/my_actuator_control.hpp"
#include <unistd.h>

using namespace std::chrono_literals;

MyActuatorControl::MyActuatorControl(const rclcpp::NodeOptions &options)
    : Node("my_actuator_control")
{
  // auto node = std::make_shared<MyActuatorControl>(rclcpp::NodeOptions());
  init_teleop_ = false;
  reset_tripmeter_flag_ = false;

  act1_enabled_ = false;
  act2_enabled_ = false;
  act3_enabled_ = false;
  act4_enabled_ = false;
  act1_sw_ = false;
  act2_sw_ = false;
  act3_sw_ = false;
  act4_sw_ = false;
  velocity_scale_ = 60;
  raster_speed_ = 150;
  motor_ids.reserve(4);
  error_[0] = "NO Error [Radhe Radhe]";
  error_[2] = "Motor Stalled [Critical]";
  error_[4] = "Low Pressure [Donno Dude]";
  error_[8] = " Over Voltage [Moderate]";
  error_[16] = "Over Current [Critical]";
  error_[64] = "Power Overrun [Moderate]";
  error_[256] = "Speeding [Moderate]";
  error_[512] = "Empty Error 1";
  error_[1024] = "Empty Error 2";
  error_[2048] = "Empty Error 3";
  error_[4096] = "Very High Temperature [Moderate]";
  error_[8192] = "Encoder Calibration Error [Moderate]";
  // -----------------------------------------------------------------------------------------Declare parameters with default values

  this->declare_parameter<int>("/my_actuator/right_front_wheel/id", 2);
  this->declare_parameter<int>("/my_actuator/left_front_wheel/id", 1);
  this->declare_parameter<int>("/my_actuator/right_rear_wheel/id", 3);
  this->declare_parameter<int>("/my_actuator/left_rear_wheel/id", 4);
  this->declare_parameter<int>("/my_actuator/right_front_wheel/dir", 1);
  this->declare_parameter<int>("/my_actuator/left_front_wheel/dir", 1);
  this->declare_parameter<int>("/my_actuator/right_rear_wheel/dir", 1);
  this->declare_parameter<int>("/my_actuator/left_rear_wheel/dir", 1);
  this->declare_parameter<double>("/my_actuator/wheel_base", 0.2913);
  this->declare_parameter<double>("/my_actuator/wheel_radius", 0.0525);

  auto logger = this->get_logger();
  toggle_joy_flag_ = -1;
  port_ = "/dev/ttyUSB0";
  act_api_ = new X10ApiSerial();
  act_api_->get_port_address(port_);

  act_api_->rmdX10_shut_down();
  geometry_msgs::msg::Twist vel;
  get_params();
  init_actuators();
  init_subs_pubs_srvs();
  init_timer();

  send_vitals_2.layout.dim.resize(2);
  send_vitals_2.layout.dim[0].label = "rows";
  send_vitals_2.layout.dim[0].size = 4;   // Number of rows
  send_vitals_2.layout.dim[0].stride = 4; // The number of elements in a row

  send_vitals_2.layout.dim[1].label = "cols";
  send_vitals_2.layout.dim[1].size = 4;   // Number of columns
  send_vitals_2.layout.dim[1].stride = 1; // The number of elements in a column

  send_vitals_2.data.resize(4 * 4); // 3 rows and 4 columns/
}

MyActuatorControl::~MyActuatorControl()
{
  act_api_->rmdX10_shut_down();
  RCLCPP_INFO(this->get_logger(), "Sting dhutown");
}
void MyActuatorControl::get_params()
{

  // Use get_parameter_or to set default values if parameters are not set
  this->get_parameter_or("/my_actuator/right_front_wheel/id", right_front_wheel_id_, 2);
  this->get_parameter_or("/my_actuator/left_front_wheel/id", left_front_wheel_id_, 1);
  this->get_parameter_or("/my_actuator/right_rear_wheel/id", right_rear_wheel_id_, 3);
  this->get_parameter_or("/my_actuator/left_rear_wheel/id", left_rear_wheel_id_, 4);
  this->get_parameter_or("/my_actuator/right_front_wheel/dir", right_front_wheel_dir_, 1);
  this->get_parameter_or("/my_actuator/left_front_wheel/dir", left_front_wheel_dir_, 1);
  this->get_parameter_or("/my_actuator/right_rear_wheel/dir", right_rear_wheel_dir_, 1);
  this->get_parameter_or("/my_actuator/left_rear_wheel/dir", left_rear_wheel_dir_, 1);
  this->get_parameter_or("/my_actuator/wheel_base", WHEEL_BASE_, 0.2913);
  this->get_parameter_or("/my_actuator/wheel_radius", WHEEL_RADIUS_, 0.0525);

  // Print warnings if default values are used
  if (!this->has_parameter("/my_actuator/right_front_wheel/id"))
  {
    RCLCPP_WARN(this->get_logger(), "Right Front wheel motor id param not found, using the default value: %d", right_front_wheel_id_);
  }
  if (!this->has_parameter("/my_actuator/left_front_wheel/id"))
  {
    RCLCPP_WARN(this->get_logger(), "Left Front wheel motor id param not found, using the default value: %d", left_front_wheel_id_);
  }
  if (!this->has_parameter("/my_actuator/right_rear_wheel/id"))
  {
    RCLCPP_WARN(this->get_logger(), "Right Rear wheel motor id param not found, using the default value: %d", right_rear_wheel_id_);
  }
  if (!this->has_parameter("/my_actuator/left_rear_wheel/id"))
  {
    RCLCPP_WARN(this->get_logger(), "Left Rear wheel motor id param not found, using the default value: %d", left_rear_wheel_id_);
  }

  if (!this->has_parameter("/my_actuator/right_front_wheel/dir"))
  {
    RCLCPP_WARN(this->get_logger(), "Right Front wheel motor dir param not found, using the default value: %d", right_front_wheel_dir_);
  }
  if (!this->has_parameter("/my_actuator/left_front_wheel/dir"))
  {
    RCLCPP_WARN(this->get_logger(), "Left Front wheel motor dir param not found, using the default value: %d", left_front_wheel_dir_);
  }
  if (!this->has_parameter("/my_actuator/right_rear_wheel/dir"))
  {
    RCLCPP_WARN(this->get_logger(), "Right Rear wheel motor dir param not found, using the default value: %d", right_rear_wheel_dir_);
  }
  if (!this->has_parameter("/my_actuator/left_rear_wheel/dir"))
  {
    RCLCPP_WARN(this->get_logger(), "Left Rear wheel motor dir param not found, using the default value: %d", left_rear_wheel_dir_);
  }

  if (!this->has_parameter("/my_actuator/wheel_base"))
  {
    RCLCPP_WARN(this->get_logger(), "Wheel Base param not found, using the default value: %f", WHEEL_BASE_);
  }
  if (!this->has_parameter("/my_actuator/wheel_radius"))
  {
    RCLCPP_WARN(this->get_logger(), "Wheel Radius param not found, using the default value: %f", WHEEL_RADIUS_);
  }

  // Populate motor_ids and health
  motor_ids.push_back(left_front_wheel_id_);
  motor_ids.push_back(right_front_wheel_id_);
  motor_ids.push_back(right_rear_wheel_id_);
  motor_ids.push_back(left_rear_wheel_id_);
  std::sort(motor_ids.begin(), motor_ids.end());
  // std::cout<<left_rear_wheel_id_;

  for (int8_t i = 0; i < 4; i++)
  {
    health.push_back(vit());
  }

  RCLCPP_INFO(this->get_logger(), "Health temperature: %d", health[0].temperature);
}

void MyActuatorControl::init_actuators()
{
  int setVEL, res;
  act_api_->rmdX10_init();
}

void MyActuatorControl::init_subs_pubs_srvs()
{
  // -----------------------------------------------------------------------------------------Publishers
  // plot_pub_ = this->create_publisher<interfaces::msg::Plot>("/plot", 10);
  vitals_pub_ = this->create_publisher<interfaces::msg::Vitals>("/crawler_vitals", 10);
  vitals_pub_2_ = this->create_publisher<interfaces::msg::Vitals>("/c_vitals", 10);
  // vitals_pub_x = this->create_publisher<interfaces::msg::Tempmsgs>("/temp", 10);
  speed_pub_ = this->create_publisher<std_msgs::msg::Int16>("/motor_speed", 10);
  distance_pub_ = this->create_publisher<std_msgs::msg::Int32>("/tripmeter", 10);
  odometer_pub_ = this->create_publisher<std_msgs::msg::Int32>("/odometer", 10);
  voltage_pub_ = this->create_publisher<std_msgs::msg::Int16>("/voltage", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rotation_time", 10);
  scan_speed_pub_ = this->create_publisher<std_msgs::msg::Int32>("/scan_speed", 10);

  //-----------------------------------------------------------------------------------------Subscribers

  joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/skid_steer/cmd_vel", 10,
      std::bind(&MyActuatorControl::joy_callback, this, std::placeholders::_1));

  // joy_sub_real_ = this->create_subscription<sensor_msgs::msg::Joy>(
  //     "/joy", 10,
  //     std::bind(&MyActuatorControl::get_joy, this, std::placeholders::_1));

  toggle_joy_topic_ = "/toggle_joy";
  toggle_joy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      toggle_joy_topic_, 1,
      std::bind(&MyActuatorControl::toggle_joy_callback_, this, std::placeholders::_1));

  linear_motor_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/linear_motor", 10,
      std::bind(&MyActuatorControl::linearMotorCallback, this, std::placeholders::_1));

  angle_value_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/set_joint_angle_value", 10,
      std::bind(&MyActuatorControl::angleValueCallback, this, std::placeholders::_1));

  stroke_length_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/stroke_length", 10,
      std::bind(&MyActuatorControl::strokeLengthCallback, this, std::placeholders::_1));

  // -------------------------------------------------------------------------------------------Services

  init_teleop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/crawler_control_node/init_teleop",
      std::bind(&MyActuatorControl::init_teleop_callback, this, std::placeholders::_1, std::placeholders::_2));

  stop_teleop_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/crawler_control_node/stop_teleop",
      std::bind(&MyActuatorControl::stop_teleop_callback, this, std::placeholders::_1, std::placeholders::_2));

  stop_motors_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/crawler_control_node/stop_motors",
      std::bind(&MyActuatorControl::stop_motors_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset_motors_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/crawler_control_node/reset_motors",
      std::bind(&MyActuatorControl::reset_motors_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset_tripmeter_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/crawler_control_node/reset_tripmeter",
      std::bind(&MyActuatorControl::reset_tripmeter_callback, this, std::placeholders::_1, std::placeholders::_2));

  set_joint_angle_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/set_joint_angle",
      std::bind(&MyActuatorControl::set_joint_angle_callback, this, std::placeholders::_1, std::placeholders::_2));

  increase_raster_speed_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/increase_raster_speed",
      std::bind(&MyActuatorControl::increaseRasterSpeedCallback, this, std::placeholders::_1, std::placeholders::_2));

  decrease_raster_speed_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/decrease_raster_speed",
      std::bind(&MyActuatorControl::decreaseRasterSpeedCallback, this, std::placeholders::_1, std::placeholders::_2));

  cycle_all_motor_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/cycle_all_motor",
      std::bind(&MyActuatorControl::cycleAllMotorCallback, this, std::placeholders::_1, std::placeholders::_2));

  // // Client
  toggle_relay_client_ = this->create_client<interfaces::srv::RelayControl>("/relay_toggle_channel");

  // Request initialization
  // control_relay_ = std::make_shared<interfaces::srv::RelayControl::Request>();
  // control_relay_.request.data = 9;
}

void MyActuatorControl::init_timer()
{
  // ----------------------------------------------------------------------------- Timer
  joy_reader_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::steer_calc_callback_, this));

  odometer_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::get_odometer_callback, this));

  health_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(1.0, 0),
      std::bind(&MyActuatorControl::health_callback_, this));

  health_timer_2_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.1, 0),
      std::bind(&MyActuatorControl::health_callback_2_, this));

  speed_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::get_speed, this));

  voltage_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::get_voltage, this));

  publish_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::publishRotationTime, this));

  scan_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.01, 0),
      std::bind(&MyActuatorControl::scanSpeed, this));
}

// -----------------------------------------------------------------------------------------Callback function of the Subscribers

void MyActuatorControl::joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  vel.linear.x = msg->linear.x;
  vel.angular.z = msg->angular.z;
}

// void MyActuatorControl::get_joy(const sensor_msgs::msg::Joy::SharedPtr joy)
// {

//   if (init_teleop_ && toggle_joy_flag_ == -1)
//   {

//     if (joy->buttons[JoyStick::ACT1_2_ON_OFF])
//     {
//       act1_sw_ = !act1_sw_;
//       act2_sw_ = !act2_sw_;
//     }
//     else if (joy->buttons[JoyStick::ACT3_4_ON_OFF])
//     {
//       act3_sw_ = !act3_sw_;
//       act4_sw_ = !act4_sw_;
//     }
//   }
// }

void MyActuatorControl::toggle_joy_callback_(const std_msgs::msg::Bool::SharedPtr msg)
{
  toggle_joy_flag_ = msg->data;
}

void MyActuatorControl::linearMotorCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  switch (msg->data)
  {
  case 1:
    MyActuatorControl::clockwise();
    break;
  case 2:
    MyActuatorControl::anticlockwise();
    break;
  case 3:
    MyActuatorControl::stop();
    break;
  default:
    RCLCPP_WARN(this->get_logger(), "Received unknown command: %d", msg->data);

    break;
  }
}

void MyActuatorControl::angleValueCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  angle_value_ = msg->data;
}

void MyActuatorControl::strokeLengthCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  stroke_length_ = msg->data;
  rotation_value_ = static_cast<int>(msg->data * 2.98 * 100);
}

// -----------------------------------------------------------------------------------------Logic function of the Services

bool MyActuatorControl::init_teleop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!init_teleop_)
  {
    // init_actuators();
    init_teleop_ = true;
    res->success = true;
    res->message = "teleop started";
  }
  else
  {
    res->message = "init is already done.";
  }

  return true;
}

bool MyActuatorControl::decreaseRasterSpeedCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (raster_speed_ - 75 >= 75)
  {
    raster_speed_ -= 75;
    res->success = true;
    res->message = "Raster speed decreased to " + std::to_string(raster_speed_);
    // publishRotationTime();
  }
  else
  {
    res->success = false;
    res->message = "Raster speed is already at or below minimum value (75).";
  }
  return true;
}

bool MyActuatorControl::cycleAllMotorCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  act_api_->increment_control(0x01, 100, 18000);

  std::this_thread::sleep_for(std::chrono::seconds(4));
  act_api_->increment_control(0x02, 100, 18000);

  std::this_thread::sleep_for(std::chrono::seconds(6));
  act_api_->increment_control(0x03, 100, 18000);

  std::this_thread::sleep_for(std::chrono::seconds(6));
  act_api_->increment_control(0x04, 100, 18000);

  res->success = true;
  // res->message = std::to_string(res_array[0]) + std::to_string(res_array[1]) + std::to_string(res_array[2]) + std::to_string(res_array[3]);
  return true;
}

bool MyActuatorControl::increaseRasterSpeedCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (raster_speed_ + 75 <= 900)
  {
    raster_speed_ += 75;
    res->success = true;
    res->message = "Raster speed increased to " + std::to_string(raster_speed_);
    // publishRotationTime();
  }
  else
  {
    res->success = false;
    res->message = "Raster speed is already at or above maximum value (900).";
  }
  return true;
}

bool MyActuatorControl::set_joint_angle_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  uint8_t motor_ids[] = {0x01, 0x02, 0x03, 0x04};

  for (uint8_t motor_id : motor_ids)
  {
    int steps = (motor_id == 0x02) ? -angle_value_ * 1.0909091 * 100 : angle_value_ * 1.0909091 * 100;
    act_api_->increment_control(motor_id, 100, steps);
  }

  res->success = true;
  // res->message = "joint angle set";
  return true;
}

bool MyActuatorControl::reset_tripmeter_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (!reset_tripmeter_flag_)
  {
    // init_actuators();
    reset_tripmeter_flag_ = true;
    res->success = true;
    res->message = "tripmeter reset";
  }

  return true;
}
bool MyActuatorControl::reset_motors_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Step 0");

  rclcpp::sleep_for(std::chrono::seconds(3));

  auto request = std::make_shared<interfaces::srv::RelayControl::Request>();
  request->data = 1;

  auto result = toggle_relay_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service toggle_relay_client_");
    res->success = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "crawler off");
  rclcpp::sleep_for(std::chrono::seconds(5));

  result = toggle_relay_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service toggle_relay_client_");
    res->success = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "crawler on");

  rclcpp::sleep_for(std::chrono::seconds(3));

  init_teleop_ = false;
  delete act_api_;

  res->success = true;
  res->message = "Motors reset successfully";
  return true;
}

bool MyActuatorControl::stop_motors_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  // RCLCPP_INFO(this->get_logger(), "%sStopping motors", log_header_.c_str());

  for (int8_t i = 0; i < 4; i++)
  {
    act_api_->Motor_stop(i + 1);
  }

  init_teleop_ = false;
  res->success = true;
  res->message = "teleop stopped";
  return true;
}

bool MyActuatorControl::stop_teleop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{

  init_teleop_ = false;
  // RCLCPP_INFO(this->get_logger(), "%s Stopping teleop", log_header_.c_str());
  for (int8_t i = 0; i < 4; i++)
  {
    act_api_->Motor_stop(i + 1);
  }
  res->success = true;
  res->message = "teleop stopped";
  return true;
}

// -----------------------------Timer function callback

void MyActuatorControl::steer_calc_callback_()
{
  if (init_teleop_)
  {

    if (vel.linear.x == 0 && vel.angular.z == 0)
    {

      velocity_left_cmd_ = 0;
      velocity_right_cmd_ = 0;
    }
    else
    {
      velocity_left_cmd_ = ((vel.linear.x - vel.angular.z * WHEEL_BASE_ / 2.0) / WHEEL_RADIUS_) * velocity_scale_;
      //      ROS_INFO("Speed_Left_Commmand%d", velocity_left_cmd_);

      velocity_right_cmd_ = ((vel.linear.x + vel.angular.z * WHEEL_BASE_ / 2.0) / WHEEL_RADIUS_) * velocity_scale_;
      // ROS_INFO("Speed_Right_Command%d", velocity_right_cmd_);
    }

    if ((act1_sw_ == true && act2_sw_ == true) && (act3_sw_ == true && act4_sw_ == true))
    {
      act_api_->Motor_shut_down(left_front_wheel_id_);
      act_api_->Motor_shut_down(right_front_wheel_id_);
      act_api_->Motor_shut_down(left_rear_wheel_id_);
      act_api_->Motor_shut_down(right_rear_wheel_id_);
      // ROS_WARN("ALL shutdown.");
    }
    else if (act1_sw_ == true && act2_sw_ == true)
    {
      act_api_->Motor_shut_down(left_front_wheel_id_);
      act_api_->Motor_shut_down(right_front_wheel_id_);
      run_actuator(right_rear_wheel_id_, velocity_right_cmd_ * right_rear_wheel_dir_);
      run_actuator(left_rear_wheel_id_, velocity_left_cmd_ * left_rear_wheel_dir_);

      // ROS_INFO("In act1");
      // ROS_INFO("In act2");
    }
    else if (act3_sw_ == true && act4_sw_ == true)
    {
      act_api_->Motor_shut_down(left_rear_wheel_id_);
      act_api_->Motor_shut_down(right_rear_wheel_id_);
      run_actuator(left_front_wheel_id_, velocity_left_cmd_ * left_front_wheel_dir_);
      run_actuator(right_front_wheel_id_, velocity_right_cmd_ * right_front_wheel_dir_);
      // ROS_INFO("In act3");
      // ROS_INFO("In act4");
    }
    else
    {
      run_actuator(left_front_wheel_id_, velocity_left_cmd_ * left_front_wheel_dir_);
      run_actuator(right_front_wheel_id_, velocity_right_cmd_ * right_front_wheel_dir_);
      run_actuator(right_rear_wheel_id_, velocity_right_cmd_ * right_rear_wheel_dir_);
      run_actuator(left_rear_wheel_id_, velocity_left_cmd_ * left_rear_wheel_dir_);
      // ROS_WARN("NORMAL drive");
    }
  }
}

void MyActuatorControl::get_odometer_callback()
{
  static double last_save_time = this->now().seconds();
  static double cumulative_distance = 0.0;
  static double accumulated_distance = 0.0;
  static std::fstream file("/home/octobotics/ros2_ws/src/my_actuator_ros2/last_distance.txt", std::ios::in | std::ios::out | std::ios::trunc);

  if (!file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file");
    return;
  }

  file.clear();
  file.seekg(0);
  file >> cumulative_distance;

  std_msgs::msg::Int32 cumulative_distance_msg;
  cumulative_distance_msg.data = static_cast<int32_t>(cumulative_distance);
  odometer_pub_->publish(cumulative_distance_msg);

  int16_t motor_stat[4] = {0};
  act_api_->Motor_state2(2, motor_stat);

  int32_t velocity = std::abs(motor_stat[2]);
  int32_t distance = velocity * 2 * 3.14 * 52.5 * 0.1667 / 60;

  cumulative_distance += static_cast<int32_t>(distance);
  accumulated_distance += static_cast<int32_t>(distance);

  if (reset_tripmeter_flag_)
  {
    accumulated_distance = 0;
    reset_tripmeter_flag_ = false;
  }

  std_msgs::msg::Int32 distance_msg;
  distance_msg.data = static_cast<int32_t>(accumulated_distance);
  distance_pub_->publish(distance_msg);

  cumulative_distance_msg.data = static_cast<int32_t>(cumulative_distance);
  odometer_pub_->publish(cumulative_distance_msg);

  if (this->now().seconds() - last_save_time >= 2.0)
  {
    file.clear();
    file.seekp(0);
    file << cumulative_distance << std::endl;
    if (file.fail())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to file");
    }
    file.flush();
    last_save_time = this->now().seconds();
  }
}
void MyActuatorControl::health_callback_2_()
{
  uint8_t mode_var;
  int16_t error_ret_[4];

  for (int id = 1; id <= 4; ++id)
  {

    int ret = act_api_->Motor_state1(id, error_ret_);
    if (ret)
    {

      // RCLCPP_INFO("Motor %d State1: Temp=%d, Voltage=%d, Error=%d",
      //  id, error_ret_[0], error_ret_[2], error_ret_[3]);

      health[id - 1].temperature = (int)error_ret_[0];
      health[id - 1].voltage = (float)error_ret_[2] / 10.0;
      health[id - 1].error = (int)error_ret_[3];
    }

    ret = act_api_->Motor_state2(id, error_ret_);
    if (ret)
    {

      // RCLCPP_INFO(this->get_logger(), "Motor %d State2: Current=%d", id, error_ret_[1]);

      health[id - 1].current = error_ret_[1];
    }

    mode_var = -1;
    ret = act_api_->Motor_mode(id, mode_var);
    if (ret)
    {

      // RCLCPP_INFO(this->get_logger(), "Motor %d Mode: %d", id, mode_var);

      send_vitals.mode[id - 1] = mode_var;
    }

    send_vitals.temp[id - 1] = health[id - 1].temperature;
    send_vitals.error[id - 1] = health[id - 1].error;
    send_vitals.current[id - 1] = health[id - 1].current * 0.01;
  }

  send_vitals.voltage = health[0].voltage;
  send_vitals.header.frame_id = "crawler";
  vitals_pub_2_->publish(send_vitals);
}
void MyActuatorControl::health_callback_()
{
  // uint8_t mode_var;
  // for (auto &it : motor_ids)
  // {
  //   mode_var = -1;
  //   get_vitals(it);

  //   send_vitals_2.data[it - 1 + 0] = health[it - 1].temperature;
  //   send_vitals_2.data[it - 1 + 1] = health[it - 1].error;
  //   send_vitals_2.data[it - 1 + 2] = (health[it - 1].current) * 0.01;

  //   send_vitals.temp[it - 1] = health[it - 1].temperature;
  //   send_vitals.error[it - 1] = health[it - 1].error;
  //   send_vitals.current[it - 1] = (health[it - 1].current) * 0.01;

  //   int ret = act_api_->Motor_mode(it, mode_var);
  //   // ROS_WARN("Motor %d: %u", it, mode_var);
  //   if (ret)
  //     send_vitals.mode[it - 1] = mode_var;
  // }

  // send_vitals.voltage = health[0].voltage;
  // send_vitals.header.frame_id = "crawler";
  // vitals_pub_->publish(send_vitals);
  // vitals_pub_x->publish(send_vitals_2);

  // int ret = 0;
  //     int16_t error_ret_[4];
  //     // 0: temp | 2: voltage | 3: error
  //     ret = act_api_->Motor_state1(id_, error_ret_);
  //     if (ret)
  //     {
  //         health[id_ - 1].temperature = (int)error_ret_[0];
  //         health[id_ - 1].voltage = (float)error_ret_[2] / 10.0;
  //         health[id_ - 1].error = (int)error_ret_[3];
  //     }

  //     // 1: current
  //     ret = act_api_->Motor_state2(id_, error_ret_);
  //     if (ret)
  //     {
  //         health[id_ - 1].current = error_ret_[1];
  //     }
  // int index = 0;

  // for (int it =1 ; it<5 ; it++)
  // {
  // int ret = 0;
  // int16_t error_ret_[4];
  // uint8_t mode_var;
  // // 0: temp | 2: voltage | 3: error
  // ret = act_api_->Motor_state1(it, error_ret_);
  // if (ret)
  // {
  //   send_vitals_2.data[index++]  = (int)error_ret_[0];
  //   send_vitals_2.data[index++]  = (float)error_ret_[2] / 10.0;
  //   send_vitals_2.data[index++ ] = (int)error_ret_[3];
  // }
  //   mode_var = -1;
  //   ret = act_api_->Motor_mode(it, mode_var);
  //   if (ret)
  //   {
  //   send_vitals_2.data[index++ ] = mode_var;
  //   }
  // }

  // v//itals_pub_x->publish(temmsgs);

  int ret = 0;
  int16_t error_ret_[4];
  // int16_t  temp;

  
  ret = act_api_->Motor_state1(0x01, error_ret_);

  temmsgs.temp[0] = (int)error_ret_[0];

  ret = act_api_->Motor_state1(0x02, error_ret_);

  temmsgs.temp[1] = (int)error_ret_[0];

  ret = act_api_->Motor_state1(0x03, error_ret_);

  temmsgs.temp[2] = (int)error_ret_[0];

  ret = act_api_->Motor_state1(0x04, error_ret_);

  temmsgs.temp[3] = (int)error_ret_[0];
  
  RCLCPP_INFO(this->get_logger(), "Action 3 performed. %d , %d , %d , %d", temmsgs.temp[0]  ,  temmsgs.temp[1]   , temmsgs.temp[2]  ,  temmsgs.temp[3]    );
}

void MyActuatorControl::get_speed()
{
  int16_t motor_stat2[4] = {0};

  if (init_teleop_)
  {
    act_api_->Motor_state2(1, motor_stat2);

    // ROS_INFO("Debug: motor_stat[2] = %d", motor_stat2[2]);
    std_msgs::msg::Int16 speed_msg;
    int16_t velocity = std::abs(motor_stat2[2]) * 0.16667 * 3.14 * 105 / 60;
    speed_msg.data = velocity;
    speed_pub_->publish(speed_msg);
  }
}

void MyActuatorControl::get_voltage()
{
  int16_t motor_stat1[4] = {0};

  if (init_teleop_)
  {
    act_api_->Motor_state1(3, motor_stat1);

    // ROS_INFO("Debug: motor_stat[2] = %d", motor_stat2[2]);
    std_msgs::msg::Int16 voltage_msg;
    int16_t voltage = motor_stat1[2] / 10.0;
    voltage_msg.data = voltage;
    voltage_pub_->publish(voltage_msg);
  }
}

void MyActuatorControl::publishRotationTime()
{

  std_msgs::msg::Float32 time_msg;

  if (raster_speed_ != 0)
  {
    time_msg.data = (static_cast<float>(stroke_length_) / (raster_speed_ / 3.0f)) + 0.5f;
  }
  else
  {
    time_msg.data = 0.0f;
    // ROS_WARN("Raster speed is zero, unable to calculate rotation time.");
  }

  time_pub_->publish(time_msg);
  // ROS_INFO("Published rotation time: %.2f seconds for raster_speed_: %d", time_msg.data, raster_speed_);
}

void MyActuatorControl::scanSpeed()
{

  std_msgs::msg::Float32 time_msg;

  if (raster_speed_ != 0)
  {
    time_msg.data = (static_cast<float>(stroke_length_) / (raster_speed_ / 3.0f)) + 0.5f;
  }
  else
  {
    time_msg.data = 0.0f;
    // ROS_WARN("Raster speed is zero, unable to calculate rotation time.");
  }

  time_pub_->publish(time_msg);
  // ROS_INFO("Published rotation time: %.2f seconds for raster_speed_: %d", time_msg.data, raster_speed_);
}
// ----------------------------Helper function

void MyActuatorControl::clockwise()
{
  act_api_->increment_control(0x05, raster_speed_, rotation_value_);
}

void MyActuatorControl::anticlockwise()
{
  act_api_->increment_control(0x05, raster_speed_, -rotation_value_);
}

void MyActuatorControl::stop()
{
  RCLCPP_INFO(this->get_logger(), "Action 3 performed.");
}

void MyActuatorControl::run_actuator(int id, double curr_vel)
{
  if (init_teleop_)
  {
    act_api_->speedControl(id, (int)curr_vel);
  }
}

// void MyActuatorControl::get_vitals(int id_)
// {
//   int ret = 0;
//   int16_t error_ret_[4];
//   // 0: temp | 2: voltage | 3: error
//   ret = act_api_->Motor_state1(id_, error_ret_);
//   if (ret)
//   {
//     health[id_ - 1].temperature = (int)error_ret_[0];
//     health[id_ - 1].voltage = (float)error_ret_[2] / 10.0;
//     health[id_ - 1].error = (int)error_ret_[3];
//   }

//   // 1: current
//   ret = act_api_->Motor_state2(id_, error_ret_);
//   if (ret)
//   {
//     health[id_ - 1].current = error_ret_[1];
//   }
// }
// ----------------------------Main Function -----------------------------------------------------

int main(int argc, char **argv)
{
  // std::cout<<"hi";

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyActuatorControl>(rclcpp::NodeOptions());
  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception thrown: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
