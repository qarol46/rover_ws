#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

class GroundFilter : public rclcpp::Node
{
public:
    GroundFilter() : Node("ground_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters
        declare_parameter("input_topic", "/velodyne_points");
        declare_parameter("output_topic", "/velodyne_points_filtered");
        declare_parameter("ground_threshold", 0.15);
        declare_parameter("min_ground_points", 500);
        declare_parameter("min_height", 0.1);
        declare_parameter("outlier_stddev", 1.0);
        declare_parameter("outlier_mean_k", 50);
        declare_parameter("target_frame", "root_link");
        declare_parameter("use_tf", false);

        // Get parameters
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        ground_threshold_ = get_parameter("ground_threshold").as_double();
        min_ground_points_ = get_parameter("min_ground_points").as_int();
        min_height_ = get_parameter("min_height").as_double();
        outlier_stddev_ = get_parameter("outlier_stddev").as_double();
        outlier_mean_k_ = get_parameter("outlier_mean_k").as_int();
        target_frame_ = get_parameter("target_frame").as_string();
        use_tf_ = get_parameter("use_tf").as_bool();

        // Initialize PCL objects
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setDistanceThreshold(ground_threshold_);
        seg_.setMaxIterations(1000);

        // Publishers/Subscribers
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->cloudCallback(msg);
            });

        filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        log_parameters();
    }

private:
    void log_parameters() {
        RCLCPP_INFO(get_logger(), "Ground filter initialized with parameters:");
        RCLCPP_INFO(get_logger(), "  Subscribe topic: %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Publish topic: %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Ground threshold: %.2f", ground_threshold_);
        RCLCPP_INFO(get_logger(), "  Min height: %.2f", min_height_);
        RCLCPP_INFO(get_logger(), "  Min ground points: %d", min_ground_points_);
        RCLCPP_INFO(get_logger(), "  Outlier mean k: %d", outlier_mean_k_);
        RCLCPP_INFO(get_logger(), "  Use TF: %s", use_tf_ ? "true" : "false");
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(get_logger(), "Received empty point cloud");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles = processCloud(cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*obstacles, output);
        output.header = msg->header;
        filtered_pub_->publish(output);
    }

       pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg_.setInputCloud(cloud);
        seg_.segment(*inliers, *coefficients);  // segment() is void - doesn't return bool

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Check if we found enough inliers
        if (inliers->indices.size() >= static_cast<size_t>(min_ground_points_)) {
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*obstacles);
            
            RCLCPP_DEBUG(get_logger(), "Found %zu ground points, %zu obstacles remaining",
                        inliers->indices.size(), obstacles->size());
        } else {
            RCLCPP_WARN(get_logger(), "Insufficient ground points (%zu), using full cloud",
                       inliers->indices.size());
            *obstacles = *cloud;
        }

        // Height filtering
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(obstacles);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, std::numeric_limits<float>::max());
        pass.filter(*obstacles);

        // Outlier removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(obstacles);
        sor.setMeanK(outlier_mean_k_);
        sor.setStddevMulThresh(outlier_stddev_);
        sor.filter(*obstacles);

        return obstacles;
    }

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // PCL
    pcl::SACSegmentation<pcl::PointXYZ> seg_;

    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double ground_threshold_;
    double min_height_;
    double outlier_stddev_;
    int min_ground_points_;
    int outlier_mean_k_;
    bool use_tf_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GroundFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}