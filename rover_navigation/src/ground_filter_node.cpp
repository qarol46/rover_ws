#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>  // Критически важный заголовок
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

class GroundFilter : public rclcpp::Node
{
public:
    GroundFilter() : Node("ground_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        // Объявление параметров
        declare_parameter("input_topic", "/velodyne_points");
        declare_parameter("output_topic", "/velodyne_points_filtered");
        declare_parameter("ground_threshold", 0.15);
        declare_parameter("min_ground_points", 500);
        declare_parameter("min_height", 0.1);
        declare_parameter("use_tf", false);
        declare_parameter("target_frame", "root_link");
        
        // Получение параметров
        input_topic_ = get_parameter("input_topic").as_string();
        output_topic_ = get_parameter("output_topic").as_string();
        ground_threshold_ = get_parameter("ground_threshold").as_double();
        min_ground_points_ = get_parameter("min_ground_points").as_int();
        min_height_ = get_parameter("min_height").as_double();
        use_tf_ = get_parameter("use_tf").as_bool();
        target_frame_ = get_parameter("target_frame").as_string();

        // Инициализация сегментатора
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setDistanceThreshold(ground_threshold_);
        seg_.setMaxIterations(1000);

        // Подписка и публикация
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->cloudCallback(msg);
            });

        filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        // Логирование параметров
        RCLCPP_INFO(get_logger(), "Ground filter initialized with parameters:");
        RCLCPP_INFO(get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Ground threshold: %.2f m", ground_threshold_);
        RCLCPP_INFO(get_logger(), "  Min height: %.2f m", min_height_);
        RCLCPP_INFO(get_logger(), "  Min ground points: %d", min_ground_points_);
        RCLCPP_INFO(get_logger(), "  Use TF: %s", use_tf_ ? "true" : "false");
        if (use_tf_) {
            RCLCPP_INFO(get_logger(), "  Target frame: %s", target_frame_.c_str());
        }
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Конвертация в PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(get_logger(), "Received empty point cloud");
            return;
        }

        // TF преобразование при необходимости
        if (use_tf_) {
            try {
                geometry_msgs::msg::TransformStamped transform = 
                    tf_buffer_.lookupTransform(
                        target_frame_, 
                        msg->header.frame_id, 
                        msg->header.stamp,
                        rclcpp::Duration::from_seconds(0.1));
                
                Eigen::Affine3f transform_eigen = 
                    tf2::transformToEigen(transform.transform).cast<float>();
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
                    new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(*cloud, *transformed_cloud, transform_eigen);
                cloud = transformed_cloud;
            } 
            catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
                return;
            }
        }

        // Обработка облака
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles = processCloud(cloud);

        // Публикация результата
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*obstacles, output);
        output.header = msg->header;
        filtered_pub_->publish(output);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Сегментация плоскости
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        
        seg_.setInputCloud(cloud);
        seg_.segment(*inliers, *coefficients);

        // Извлечение препятствий
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        
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

        // Фильтрация по высоте
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(obstacles);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_height_, std::numeric_limits<float>::max());
        pass.filter(*obstacles);

        return obstacles;
    }

    // Члены класса
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    pcl::SACSegmentation<pcl::PointXYZ> seg_;

    // Параметры
    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double ground_threshold_;
    double min_height_;
    int min_ground_points_;
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