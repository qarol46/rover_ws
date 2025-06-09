// Standart C++ libs
#include <chrono> // Timer lib
#include <functional> // Calculations
#include <memory> // Dynamic memory
#include <cmath>
// ROS2 libs
#include "rclcpp/rclcpp.hpp" // ROS2 lib
#include "sensor_msgs/msg/imu.hpp"// Msg lib

using std::placeholders::_1;
using namespace std::chrono_literals;

class XsensUserCalib : public rclcpp::Node // Node class
{
  public: 
    /*Initialization*/
    XsensUserCalib() : Node("xsens_user_calib"), count_(0)
    {
      pub = this->create_publisher<sensor_msgs::msg::Imu>("MTiG710/imu", 1);
      sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 1, std::bind(&XsensUserCalib::topic_callback, this, _1));
      timer_ = this->create_wall_timer(0.1ms, std::bind(&XsensUserCalib::timer_callback, this));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Imu & msg);
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    geometry_msgs::msg::Vector3 gyro;
    geometry_msgs::msg::Vector3 accel;
    // Gains
    float a_ssf[3] = {1462, 1469, 1443}; // LSB/g      min step - 1-3
    float g_ssf[3] = {50.30, 49.49, 51.44}; // LSB/(deg/s)
    float a_off[3] = {-32819, -32704, -32699}; // LSB       min step - 1-3
    float g_off[3] = {-32828, -33075, -32254}; // LSB
    int round2sign_a = 10000;
    int round2sign_g = 100;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XsensUserCalib>());
  rclcpp::shutdown();
  return 0;
}

/*Read data in cycle while ros is ok*/
void XsensUserCalib::topic_callback(const sensor_msgs::msg::Imu & msg)
{
    /*** Read sensors measured data in digitized voltages ***/

    geometry_msgs::msg::Vector3 g_lsb = msg.angular_velocity;
    geometry_msgs::msg::Vector3 a_lsb = msg.linear_acceleration;

    /*** Get raw data through SensitivityScaleFactor (choose axis) ***/

    // For accelerometer [g]
    accel.x = - round(((a_lsb.x + a_off[0]) * (1 / a_ssf[0])) * round2sign_a) / round2sign_a;
    accel.y = round(((a_lsb.y + a_off[1]) * (1 / a_ssf[1])) * round2sign_a) / round2sign_a;
    accel.z = round(((a_lsb.z + a_off[2]) * (1 / a_ssf[2])) * round2sign_a) / round2sign_a;
    // For gyroscope [deg/s]
    gyro.x = round(((g_lsb.x + g_off[0]) * (1 / g_ssf[0])) * round2sign_g) / round2sign_g;
    gyro.y = - round(((g_lsb.y + g_off[1]) * (1 / g_ssf[1])) * round2sign_g) / round2sign_g;
    gyro.z = - round(((g_lsb.z + g_off[2]) * (1 / g_ssf[2])) * round2sign_g) / round2sign_g;

}

/*Send data in cycle while ros is ok*/
void XsensUserCalib::timer_callback()
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "imu";

  msg.angular_velocity = gyro;
  msg.linear_acceleration = accel;

  pub->publish(msg);
}
