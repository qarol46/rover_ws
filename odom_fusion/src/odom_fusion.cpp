#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

class OdomFusion : public rclcpp::Node {
public:
  OdomFusion() : Node("odom_fusion"), first_message_(true) {
    // Параметры
    declare_parameter("odom_topic", "/diff_cont/odom");
    declare_parameter("imu_topic", "/imu");
    declare_parameter("output_topic", "/odom");
    declare_parameter("child_frame", "root_link");
    declare_parameter("world_frame", "odom");
    declare_parameter("publish_tf", true);
    declare_parameter("min_movement", 0.001);
    declare_parameter("min_rotation", 0.017);
    declare_parameter("max_speed", 2.0);

    // Инициализация
    last_position_.x = 0.0;
    last_position_.y = 0.0;
    last_position_.z = 0.0;
    current_yaw_ = 0.0;
    last_yaw_rate_ = 0.0;

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
    // Получаем yaw и угловую скорость
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    current_yaw_ = yaw;
    last_yaw_rate_ = msg->angular_velocity.z;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (first_message_) {
      last_position_ = msg->pose.pose.position;
      first_message_ = false;
      return;
    }

    // Получаем параметры
    double min_movement = get_parameter("min_movement").as_double();
    double min_rotation = get_parameter("min_rotation").as_double();
    double max_speed = get_parameter("max_speed").as_double();

    // 1. Проверяем угловую скорость
    bool is_rotating = (fabs(last_yaw_rate_) > min_rotation);

    // 2. Вычисляем модуль перемещения
    double dx = msg->pose.pose.position.x - last_position_.x;
    double dy = msg->pose.pose.position.y - last_position_.y;
    double ds = std::hypot(dx, dy);

    // 3. Фильтрация перемещения
    if (ds < min_movement) {
      ds = 0.0; // Игнорируем микро-перемещения
    }

    // 4. Вычисляем модуль скорости
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double speed_module = std::min(std::hypot(vx, vy), max_speed);

    // 5. Обновляем позицию только если есть значимое движение или вращение
    if (ds > 0 || is_rotating) {
      last_position_.x += ds * std::cos(current_yaw_);
      last_position_.y += ds * std::sin(current_yaw_);
    }

    // 6. Корректируем скорость
    double corrected_vx = speed_module * std::cos(current_yaw_);
    double corrected_vy = speed_module * std::sin(current_yaw_);

    // 7. Создаем сообщение
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

    // 8. Публикация TF
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

  // Члены класса
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  geometry_msgs::msg::Point last_position_;
  double current_yaw_;
  double last_yaw_rate_;
  bool first_message_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusion>());
  rclcpp::shutdown();
  return 0;
}