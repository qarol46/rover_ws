#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
class GroundFilter : public rclcpp::Node {
public:
  GroundFilter() : Node("ground_filter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // Объявление параметров с значениями по умолчанию
    max_distance_ = this->declare_parameter("max_distance", 0.15);
    angular_threshold_ = this->declare_parameter("angular_threshold", 10.0);
    min_ground_points_ = this->declare_parameter("min_ground_points", 500);
    use_imu_ = this->declare_parameter("use_imu", true);
    target_frame_ = this->declare_parameter("target_frame", "base_link");
    subscribe_topic_ = this->declare_parameter("subscribe_topic", "/velodyne_points");
    publish_topic_ = this->declare_parameter("publish_topic", "/velodyne_points_filtered");
    point_stack_ = this->declare_parameter("number_of_points", 50);
    dist_treshold_ = this->declare_parameter("distance_threshold", 1.0);
    min_height_ = this->declare_parameter("min_height", -0.3); 
    
    // Подписка и публикация с параметризованными топиками
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      subscribe_topic_, 10,
      std::bind(&GroundFilter::cloudCallback, this, std::placeholders::_1));
      
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "Ground filter initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "  Subscribe topic: %s", subscribe_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publish topic: %s", publish_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Max distance: %.2f m", max_distance_);
    RCLCPP_INFO(this->get_logger(), "  Angular threshold: %.1f°", angular_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Min ground points: %d", min_ground_points_);
    RCLCPP_INFO(this->get_logger(), "  Use IMU: %s", use_imu_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Number of near points: %d", point_stack_);
    RCLCPP_INFO(this->get_logger(), "  Distanse treshhold: %.1f", dist_treshold_);
    RCLCPP_INFO(this->get_logger(), "  Min height: %.2f m", min_height_);
  }
private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Конвертация в PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }
    // Получение трансформации для преобразования в целевую систему координат
    geometry_msgs::msg::TransformStamped transform;
    bool transform_available = false;
    if (use_imu_ || !target_frame_.empty()) {
      try {
        transform = tf_buffer_.lookupTransform(
            target_frame_, 
            msg->header.frame_id, 
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1));
        transform_available = true;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
      }
    }
    // Преобразование облака в целевую систему координат
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3d transform_eigen = Eigen::Affine3d::Identity();
    if (transform_available) {
      transform_eigen = tf2::transformToEigen(transform.transform);
      pcl::transformPointCloud(*cloud, *transformed_cloud, transform_eigen);
    } else {
      *transformed_cloud = *cloud;
    }
    // Определение оси для сегментации (по умолчанию вертикаль Z)
    Eigen::Vector3f axis(0, 0, 1);
    if (use_imu_ && transform_available) {
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
        
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      // Коррекция оси с учетом pitch робота
      axis = Eigen::Vector3f(0, -sin(pitch), cos(pitch));
      RCLCPP_DEBUG(this->get_logger(), "Adjusted axis to: [%.3f, %.3f, %.3f] (pitch: %.1f°)", 
                   axis.x(), axis.y(), axis.z(), pitch * 180/M_PI);
    }
    // Сегментация плоскости земли
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(max_distance_);
    seg.setAxis(axis);
    seg.setEpsAngle(angular_threshold_ * M_PI / 180.0);
    seg.setInputCloud(transformed_cloud);
    seg.segment(*inliers, *coefficients);
    // Извлечение не-плоскостных точек (препятствия)
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (inliers->indices.size() > static_cast<size_t>(min_ground_points_)) {
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(transformed_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*filtered_cloud);
      RCLCPP_DEBUG(this->get_logger(), "Filtered %zu ground points, leaving %zu obstacles", 
                  inliers->indices.size(), filtered_cloud->size());
    } else {
      *filtered_cloud = *transformed_cloud;
      RCLCPP_WARN(this->get_logger(), "Not enough ground points: %zu, using original cloud", inliers->indices.size());
    }
    // Фильтрация выбросов
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(point_stack_);
    sor.setStddevMulThresh(dist_treshold_);
    sor.filter(*filtered_cloud);
    // Фильтрация по высоте: удаляем точки ниже min_height_ в целевой системе координат
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_height_, std::numeric_limits<float>::max());
    pass.filter(*height_filtered_cloud);
    // Публикация результата в target_frame_
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*height_filtered_cloud, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = target_frame_; // публикуем в целевом фрейме
    pub_->publish(output);
  }
  // Члены класса
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  double max_distance_;
  double angular_threshold_;
  int min_ground_points_;
  bool use_imu_;
  std::string target_frame_;
  std::string subscribe_topic_;
  std::string publish_topic_;
  int point_stack_;
  double dist_treshold_;
  double min_height_; // новый параметр
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}