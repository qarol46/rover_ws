#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <deque>

class OdomFusion : public rclcpp::Node {
public:
  OdomFusion() : Node("odometry_fus"), current_direction_(1.0) {
    // Параметры
    declare_parameter("odom_topic", "/diff_cont/odom");
    declare_parameter("imu_topic", "/imu");
    declare_parameter("output_topic", "/odom");
    declare_parameter("child_frame", "root_link");
    declare_parameter("world_frame", "odom");
    declare_parameter("publish_tf", true);
    declare_parameter("min_speed", 0.001);
    declare_parameter("direction_threshold", 0.5);
    declare_parameter("min_angular_speed", 0.005); // минимальная угловая скорость для обновления
    declare_parameter("angle_change_threshold", 0.0005); // порог изменения угла для фильтрации
    declare_parameter("max_angle_difference", 0.05);
    declare_parameter("moving_average_window", 10); // размер окна для скользящего среднего
    
    // Инициализация
    last_position_.x = 0.0;
    last_position_.y = 0.0;
    last_position_.z = 0.0;
    current_yaw_ = 0.0;
    last_imu_yaw_ = 0.0;
    last_valid_imu_yaw_ = 0.0;
    angular_speed_zero_time_ = this->now();
    imu_yaw_offset_ = 0.0;
    is_first_imu_ = true;
    moving_average_window_ = get_parameter("moving_average_window").as_int();

    // Подписки
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&OdomFusion::odom_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), 10,
      std::bind(&OdomFusion::imu_callback, this, std::placeholders::_1));

    // Публикаторы
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("output_topic").as_string(), 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    if (is_first_imu_) {
      last_imu_yaw_ = yaw;
      last_valid_imu_yaw_ = yaw;
      imu_yaw_offset_ = yaw;
      is_first_imu_ = false;
      
      // Инициализация фильтра скользящего среднего
      yaw_history_.clear();
      for (int i = 0; i < moving_average_window_; ++i) {
        yaw_history_.push_back(yaw);
      }
      return;
    }
    
    // Добавляем новое значение в историю
    yaw_history_.push_back(yaw);
    // Удаляем самое старое значение, если окно переполнено
    if (yaw_history_.size() > moving_average_window_) {
      yaw_history_.pop_front();
    }
    
    // Вычисляем скользящее среднее
    double sum = 0.0;
    for (double val : yaw_history_) {
      sum += val;
    }
    double filtered_yaw = sum / yaw_history_.size();
    
    // Фильтрация мелких изменений
    double yaw_diff = filtered_yaw - last_imu_yaw_;
    if (fabs(yaw_diff) > get_parameter("angle_change_threshold").as_double()) {
      last_valid_imu_yaw_ = filtered_yaw;
    }
    last_imu_yaw_ = filtered_yaw;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    static rclcpp::Time last_time = msg->header.stamp;
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    if (dt <= 0) return;

    double angular_speed = msg->twist.twist.angular.z;

    if (fabs(angular_speed) > get_parameter("min_angular_speed").as_double()) {
      double yaw_diff = last_valid_imu_yaw_ - imu_yaw_offset_;
      if(fabs(yaw_diff) < get_parameter("max_angle_difference").as_double()) current_yaw_ += yaw_diff; // Используем угол от IMU
      imu_yaw_offset_ = last_valid_imu_yaw_;
    }
    
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double speed_module = std::hypot(vx, vy);

    current_direction_ = (vx >= 0) ? 1.0 : -1.0;

    if (speed_module < get_parameter("min_speed").as_double()) {
      speed_module = 0.0;
    }

    double corrected_vx = current_direction_ * speed_module * std::cos(current_yaw_);
    double corrected_vy = current_direction_ * speed_module * std::sin(current_yaw_);

    last_position_.x += corrected_vx * dt;
    last_position_.y += corrected_vy * dt;

    auto new_odom = nav_msgs::msg::Odometry(*msg);
    new_odom.header.frame_id = get_parameter("world_frame").as_string();
    new_odom.child_frame_id = get_parameter("child_frame").as_string();
    
    new_odom.pose.pose.position = last_position_;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_);
    new_odom.pose.pose.orientation.x = q.x();
    new_odom.pose.pose.orientation.y = q.y();
    new_odom.pose.pose.orientation.z = q.z();
    new_odom.pose.pose.orientation.w = q.w();

    new_odom.twist.twist.linear.x = corrected_vx;
    new_odom.twist.twist.linear.y = corrected_vy;

    odom_pub_->publish(new_odom);

    // 7. Публикация TF
    if (get_parameter("publish_tf").as_bool()) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = new_odom.header;
      transform.child_frame_id = new_odom.child_frame_id;
      transform.transform.translation.x = last_position_.x;
      transform.transform.translation.y = last_position_.y;
      transform.transform.translation.z = last_position_.z;
      transform.transform.rotation = new_odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  geometry_msgs::msg::Point last_position_;
  double current_yaw_;
  double current_direction_;
  
  double last_imu_yaw_;
  double last_valid_imu_yaw_;
  double imu_yaw_offset_;
  rclcpp::Time angular_speed_zero_time_;
  bool is_first_imu_;
  
  // Для фильтра скользящего среднего
  std::deque<double> yaw_history_;
  int moving_average_window_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusion>());
  rclcpp::shutdown();
  return 0;
}