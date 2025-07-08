#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <deque>

class OdomFusion : public rclcpp::Node {
public:
  OdomFusion() : Node("odometry_fus"), current_direction_(1.0) {
    declare_parameter("odom_topic", "/diff_cont/odom");
    declare_parameter("imu_topic", "/imu/data");
    declare_parameter("output_topic", "/odom");
    declare_parameter("child_frame", "root_link");
    declare_parameter("world_frame", "odom");
    declare_parameter("publish_tf", true);
    declare_parameter("min_speed", 0.001);
    declare_parameter("direction_threshold", 0.5);
    declare_parameter("min_angular_speed", 0.03);
    declare_parameter("angle_change_threshold", 0.0005);
    declare_parameter("max_angle_difference", 0.05);
    declare_parameter("moving_average_window", 10);
    declare_parameter("imu_drift_rate", 0.0035);
    
    last_position_.x = 0.0;
    last_position_.y = 0.0;
    last_position_.z = 0.0;
    current_yaw_ = 0.0;
    last_imu_yaw_ = 0.0;
    last_valid_imu_yaw_ = 0.0;
    last_straight_yaw_ = 0.0;
    imu_yaw_offset_ = 0.0;
    is_first_imu_ = true;
    is_moving_straight_ = true;
    is_turning_ = false;
    first_turn_ = true;
    moving_average_window_ = get_parameter("moving_average_window").as_int();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&OdomFusion::odom_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), 10,
      std::bind(&OdomFusion::imu_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("output_topic").as_string(), 10);

    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/odom_fusion_debug", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    last_odom_time_ = this->now();
    last_imu_time_ = this->now();

    log_parameters();
  }

private:
  void publish_debug_info(double yaw_diff) {
    auto debug_msg = std_msgs::msg::Float64MultiArray();
    
    debug_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    debug_msg.layout.dim[0].label = "state,is_turning,is_moving_straight,current_yaw,last_straight_yaw,last_valid_imu_yaw,imu_yaw_offset,yaw_diff,min_angular_speed,max_angle_diff,angle_change_threshold";
    debug_msg.layout.dim[0].size = 11;
    debug_msg.layout.dim[0].stride = 11;
    
    debug_msg.data.push_back(is_turning_ ? 2.0 : (is_moving_straight_ ? 1.0 : 0.0));
    debug_msg.data.push_back(is_turning_ ? 1.0 : 0.0);
    debug_msg.data.push_back(is_moving_straight_ ? 1.0 : 0.0);
    debug_msg.data.push_back(current_yaw_);
    debug_msg.data.push_back(last_straight_yaw_);
    debug_msg.data.push_back(last_valid_imu_yaw_);
    debug_msg.data.push_back(imu_yaw_offset_);
    debug_msg.data.push_back(yaw_diff);
    debug_msg.data.push_back(get_parameter("min_angular_speed").as_double());
    debug_msg.data.push_back(get_parameter("max_angle_difference").as_double());
    debug_msg.data.push_back(get_parameter("angle_change_threshold").as_double());
    
    debug_pub_->publish(debug_msg);
  }

  void log_parameters() {
    RCLCPP_INFO(this->get_logger(), "Odometry Fusion Parameters:");
    RCLCPP_INFO(this->get_logger(), "  min_speed: %.4f", get_parameter("min_speed").as_double());
    RCLCPP_INFO(this->get_logger(), "  min_angular_speed: %.4f", get_parameter("min_angular_speed").as_double());
    RCLCPP_INFO(this->get_logger(), "  angle_change_threshold: %.6f", get_parameter("angle_change_threshold").as_double());
    RCLCPP_INFO(this->get_logger(), "  max_angle_difference: %.4f", get_parameter("max_angle_difference").as_double());
    RCLCPP_INFO(this->get_logger(), "  imu_drift_rate: %.6f", get_parameter("imu_drift_rate").as_double());
    RCLCPP_INFO(this->get_logger(), "  moving_average_window: %ld", get_parameter("moving_average_window").as_int());
  }

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
      last_straight_yaw_ = yaw;
      is_first_imu_ = false;
      
      yaw_history_.clear();
      for (int i = 0; i < moving_average_window_; ++i) {
        yaw_history_.push_back(yaw);
      }
      return;
    }
    
    rclcpp::Time current_time;
    try {
      current_time = msg->header.stamp;
      (void)(current_time - last_imu_time_);
    } catch (const std::runtime_error& e) {
      current_time = this->now();
    }
    
    double dt = (current_time - last_imu_time_).seconds();
    //if (dt > 0) {
    //  yaw -= get_parameter("imu_drift_rate").as_double() * dt;
    //}
    last_imu_time_ = current_time;
    
    yaw_history_.push_back(yaw);
    if (yaw_history_.size() > moving_average_window_) {
      yaw_history_.pop_front();
    }
    
    double sum = 0.0;
    for (double val : yaw_history_) {
      sum += val;
    }
    double filtered_yaw = sum / yaw_history_.size();
    
    double yaw_diff = filtered_yaw - last_imu_yaw_;
    if (fabs(yaw_diff) > get_parameter("angle_change_threshold").as_double()) {
      last_valid_imu_yaw_ = filtered_yaw;
      RCLCPP_INFO(this->get_logger(), "Significant yaw change detected: %.6f", yaw_diff);
    }
    last_imu_yaw_ = filtered_yaw;

    publish_debug_info(yaw_diff);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    rclcpp::Time current_time;
    try {
      current_time = msg->header.stamp;
      (void)(current_time - last_odom_time_);
    } catch (const std::runtime_error& e) {
      current_time = this->now();
    }
    
    double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;

    if (dt <= 0) return;

    double angular_speed = msg->twist.twist.angular.z;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double speed_module = std::hypot(vx, vy);

    bool was_turning = is_turning_;
    bool was_moving_straight = is_moving_straight_;
    
    is_turning_ = fabs(angular_speed) > get_parameter("min_angular_speed").as_double();
    is_moving_straight_ = !is_turning_ && speed_module > get_parameter("min_speed").as_double();

    if (was_turning != is_turning_) {
      RCLCPP_INFO(this->get_logger(), "State changed to: %s", is_turning_ ? "TURNING" : "NOT_TURNING");
    }
    if (was_moving_straight != is_moving_straight_) {
      RCLCPP_INFO(this->get_logger(), "State changed to: %s", is_moving_straight_ ? "MOVING_STRAIGHT" : "NOT_MOVING_STRAIGHT");
    }

    if (is_moving_straight_) {
      last_straight_yaw_ = current_yaw_;
      first_turn_ = true;
    }

    if (is_turning_) {
      double yaw_diff = last_valid_imu_yaw_ - imu_yaw_offset_;
      
      if (first_turn_) {
        current_yaw_ = last_straight_yaw_;
        first_turn_ = false;
        RCLCPP_INFO(this->get_logger(), "First turn detected, setting yaw to last straight yaw: %.4f", current_yaw_);
      } else if(fabs(yaw_diff) < get_parameter("max_angle_difference").as_double()) {
        current_yaw_ += yaw_diff;
        RCLCPP_DEBUG(this->get_logger(), "Updating yaw by %.6f to %.4f", yaw_diff, current_yaw_);
      }
      imu_yaw_offset_ = last_valid_imu_yaw_;
    }
    
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
    new_odom.header.stamp = current_time;
    
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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  geometry_msgs::msg::Point last_position_;
  double current_yaw_;
  double current_direction_;
  double last_straight_yaw_;
  
  double last_imu_yaw_;
  double last_valid_imu_yaw_;
  double imu_yaw_offset_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_imu_time_;
  bool is_first_imu_;
  bool is_moving_straight_;
  bool is_turning_;
  bool first_turn_;
  
  std::deque<double> yaw_history_;
  int moving_average_window_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusion>());
  rclcpp::shutdown();
  return 0;
}