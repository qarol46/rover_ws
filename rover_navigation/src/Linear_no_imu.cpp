#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

class GroundFilter : public rclcpp::Node {
public:
    GroundFilter() : Node("ground_filter"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Parameter declarations with defaults
        declare_parameters();
        
        // Subscription and publication
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            subscribe_topic_, 10,
            std::bind(&GroundFilter::cloud_callback, this, std::placeholders::_1));
            
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 10);
        
        log_parameters();
    }

private:
    void declare_parameters() {
        max_distance_ = declare_parameter("max_distance", 0.15);
        angular_threshold_ = declare_parameter("angular_threshold", 10.0);
        min_ground_points_ = declare_parameter("min_ground_points", 500);
        use_imu_ = declare_parameter("use_imu", true);
        target_frame_ = declare_parameter("target_frame", "base_link");
        subscribe_topic_ = declare_parameter("subscribe_topic", "/velodyne_points");
        publish_topic_ = declare_parameter("publish_topic", "/velodyne_points_filtered");
        point_stack_ = declare_parameter("number_of_points", 50);
        dist_threshold_ = declare_parameter("distance_threshold", 1.0);
        line_min_length_ = declare_parameter("line_min_length", 1.5);  
        line_point_density_ = declare_parameter("line_point_density", 50);
        buffer_size_ = declare_parameter("buffer_size", 3);
        min_point_age_ = declare_parameter("min_point_age", 2);
    }

    void log_parameters() {
        RCLCPP_INFO(get_logger(), "Ground filter initialized with parameters:");
        RCLCPP_INFO(get_logger(), "  Subscribe topic: %s", subscribe_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Publish topic: %s", publish_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Max distance: %.2f m", max_distance_);
        RCLCPP_INFO(get_logger(), "  Angular threshold: %.1f°", angular_threshold_);
        RCLCPP_INFO(get_logger(), "  Min ground points: %d", min_ground_points_);
        RCLCPP_INFO(get_logger(), "  Use IMU: %s", use_imu_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(get_logger(), "  Number of near points: %d", point_stack_);
        RCLCPP_INFO(get_logger(), "  Distance threshold: %.1f", dist_threshold_);
        RCLCPP_INFO(get_logger(), "  Line min length: %.1f m", line_min_length_);
        RCLCPP_INFO(get_logger(), "  Line point density: %d pts/m", line_point_density_);
        RCLCPP_INFO(get_logger(), "  Buffer size: %d frames", buffer_size_);
        RCLCPP_INFO(get_logger(), "  Min point age: %d appearances", min_point_age_);
    }


    bool is_line_like(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                     const pcl::PointIndices& cluster) {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, cluster.indices, min_pt, max_pt);
        
        float dx = max_pt[0] - min_pt[0];
        float dy = max_pt[1] - min_pt[1];
        float dz = max_pt[2] - min_pt[2];
        
        float length = std::max({dx, dy, dz});
        float density = static_cast<float>(cluster.indices.size()) / (length + 1e-5f);
        
        return (length > line_min_length_) && (density > line_point_density_);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            RCLCPP_WARN(get_logger(), "Received empty point cloud");
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Processing cloud with %zu points", cloud->size());

        // Ground plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(max_distance_);
        seg.setMaxIterations(1000);
        
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Extract ground and obstacles
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (inliers->indices.size() > static_cast<size_t>(min_ground_points_)) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            
            // Extract ground
            extract.setNegative(false);
            extract.filter(*ground_cloud);
            
            // Extract obstacles
            extract.setNegative(true);
            extract.filter(*obstacle_cloud);
        } else {
            RCLCPP_WARN(get_logger(), "Insufficient ground points (%zu)", inliers->indices.size());
            *obstacle_cloud = *cloud;
        }

        // Temporal filtering
        point_history_.push_back(*obstacle_cloud);
        if (point_history_.size() > static_cast<size_t>(buffer_size_)) {
            point_history_.pop_front();
        }

        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*obstacle_cloud, output);
        output.header = msg->header;
        pub_->publish(output);

        RCLCPP_DEBUG(get_logger(), "Published filtered cloud with %zu points", obstacle_cloud->size());
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double max_distance_;
    double angular_threshold_;
    int min_ground_points_;
    bool use_imu_;
    std::string target_frame_;
    std::string subscribe_topic_;
    std::string publish_topic_;
    int point_stack_;
    double dist_threshold_;
    double line_min_length_;
    int line_point_density_;
    int buffer_size_;
    int min_point_age_;
    
    std::deque<pcl::PointCloud<pcl::PointXYZ>> point_history_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundFilter>());
    rclcpp::shutdown();
    return 0;
}