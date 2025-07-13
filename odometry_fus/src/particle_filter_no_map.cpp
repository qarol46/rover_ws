#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <random>
#include <algorithm>
#include <memory>

struct Particle {
    geometry_msgs::msg::Pose pose;
    double weight;
    std::vector<float> last_scan;
};

class ParticleFilterNoMap : public rclcpp::Node {
public:
    ParticleFilterNoMap() : Node("particle_filter_no_map"), tf_broadcaster_(this) {
    // Параметры
    declare_parameter("num_particles", 100);
    declare_parameter("odom_topic", "/odom");
    declare_parameter("laser_topic", "/scan");
    declare_parameter("particles_topic", "/particles");
    declare_parameter("estimated_pose_topic", "/estimated_pose");
    declare_parameter("world_frame", "odom");
    declare_parameter("robot_frame", "base_link");
    
    num_particles_ = get_parameter("num_particles").as_int();
    
    // Инициализация генератора случайных чисел
    std::random_device rd;
    gen_ = std::mt19937(rd());
    
    // Инициализация частиц
    initialize_particles();
    
    // Подписки с правильными QoS
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(), 
        rclcpp::QoS(10),  // Стандартные настройки для одометрии
        std::bind(&ParticleFilterNoMap::odom_callback, this, std::placeholders::_1));
        
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("laser_topic").as_string(), 
        rclcpp::SensorDataQoS(),  // Специальные настройки для сенсорных данных
        std::bind(&ParticleFilterNoMap::laser_callback, this, std::placeholders::_1));
    
    // Публикаторы
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
        get_parameter("particles_topic").as_string(), 10);
        
    estimated_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("estimated_pose_topic").as_string(), 
        rclcpp::QoS(10).reliable());  // Гарантированная доставка
    }   

private:
    double getYawFromQuaternion(const tf2::Quaternion& q) {
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void initialize_particles() {
        particles_.resize(num_particles_);
        
        // Небольшой шум вокруг начала координат
        std::normal_distribution<> x_dist(0.0, 0.1);
        std::normal_distribution<> y_dist(0.0, 0.1);
        std::normal_distribution<> theta_dist(0.0, 0.05);
        
        for (auto& p : particles_) {
            p.pose.position.x = x_dist(gen_);
            p.pose.position.y = y_dist(gen_);
            
            tf2::Quaternion q;
            q.setRPY(0, 0, theta_dist(gen_));
            p.pose.orientation = tf2::toMsg(q);
            
            p.weight = 1.0 / num_particles_;
            p.last_scan.clear();
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!prev_odom_) {
            prev_odom_ = msg;
            return;
        }

        // Вычисляем изменение в локальной системе робота
        tf2::Quaternion q_prev, q_current;
        tf2::fromMsg(prev_odom_->pose.pose.orientation, q_prev);
        tf2::fromMsg(msg->pose.pose.orientation, q_current);

        double yaw_prev = getYawFromQuaternion(q_prev);
        double yaw_current = getYawFromQuaternion(q_current);

        // Разница в локальных координатах
        double dx = msg->pose.pose.position.x - prev_odom_->pose.pose.position.x;
        double dy = msg->pose.pose.position.y - prev_odom_->pose.pose.position.y;
        double dtheta = yaw_current - yaw_prev;

        // Преобразуем в локальное смещение
        double local_dx = dx * cos(yaw_prev) + dy * sin(yaw_prev);
        double local_dy = -dx * sin(yaw_prev) + dy * cos(yaw_prev);

        // Обновляем частицы
        for (auto& p : particles_) {
            double yaw_p = getYawFromQuaternion(tf2::Quaternion(
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w
            ));

            // Применяем движение в локальной системе частицы
            p.pose.position.x += local_dx * cos(yaw_p) - local_dy * sin(yaw_p);
            p.pose.position.y += local_dx * sin(yaw_p) + local_dy * cos(yaw_p);

            // Обновляем ориентацию
            yaw_p += dtheta;
            tf2::Quaternion q_p;
            q_p.setRPY(0, 0, yaw_p);
            p.pose.orientation = tf2::toMsg(q_p);
        }
        prev_odom_ = msg;
        publish_particles();
    }   

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!last_odom_) return;
        
        // Первый скан - просто сохраняем
        if (particles_[0].last_scan.empty()) {
            for (auto& p : particles_) {
                p.last_scan = msg->ranges;
            }
            return;
        }
        
        // Обновляем веса частиц на основе согласованности последовательных сканов
        update_weights(msg);
        
        // Ресемплинг
        resample();
        
        // Публикуем частицы и оценку позы
        publish_particles();
        publish_estimated_pose();
    }

    void update_weights(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        double total_weight = 0.0;
        const int step = 10;  // Подвыборка лучей для ускорения
        
        for (auto& p : particles_) {
            double weight = 0.0;
            int count = 0;
            
            // Для каждого луча (с шагом)
            for (size_t i = 0; i < scan->ranges.size(); i += step) {
                if (i >= p.last_scan.size()) continue;
                
                float curr_range = scan->ranges[i];
                float prev_range = p.last_scan[i];
                
                // Игнорируем inf и nan значения
                if (std::isfinite(curr_range) && std::isfinite(prev_range)) {
                    double diff = fabs(curr_range - prev_range);
                    weight += exp(-diff * diff / (2 * 0.1 * 0.1));  // Гауссово ядро
                    count++;
                }
            }
            
            p.weight = (count > 0) ? weight / count : 0.0;
            total_weight += p.weight;
            
            // Сохраняем текущий скан для следующей итерации
            p.last_scan = scan->ranges;
        }
        
        // Нормализация весов
        if (total_weight > 1e-6) {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        } else {
            // Если все веса нулевые, сброс к равномерному распределению
            double uniform_weight = 1.0 / particles_.size();
            for (auto& p : particles_) {
                p.weight = uniform_weight;
            }
        }
    }

    void resample() {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);
        
        // Вычисляем кумулятивные веса
        std::vector<double> cum_weights(particles_.size());
        cum_weights[0] = particles_[0].weight;
        for (size_t i = 1; i < particles_.size(); ++i) {
            cum_weights[i] = cum_weights[i-1] + particles_[i].weight;
        }
        
        // Систематический ресемплинг
        std::uniform_real_distribution<> dist(0.0, 1.0 / num_particles_);
        double start = dist(gen_);
        double step = 1.0 / num_particles_;
        
        size_t i = 0;
        for (int j = 0; j < num_particles_; ++j) {
            double target = start + j * step;
            while (i < particles_.size() - 1 && target > cum_weights[i]) {
                i++;
            }
            new_particles.push_back(particles_[i]);
        }
        
        // Добавляем небольшой шум для разнообразия
        std::normal_distribution<> pos_noise(0.0, 0.01);
        std::normal_distribution<> angle_noise(0.0, 0.005);
        
        for (auto& p : new_particles) {
            p.pose.position.x += pos_noise(gen_);
            p.pose.position.y += pos_noise(gen_);
            
            tf2::Quaternion q;
            tf2::fromMsg(p.pose.orientation, q);
            double yaw = getYawFromQuaternion(q) + angle_noise(gen_);
            q.setRPY(0, 0, yaw);
            p.pose.orientation = tf2::toMsg(q);
            
            p.weight = 1.0 / num_particles_;
        }
        
        particles_ = std::move(new_particles);
    }

    void publish_particles() {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = now();
        pose_array.header.frame_id = get_parameter("world_frame").as_string();
        
        for (const auto& p : particles_) {
            pose_array.poses.push_back(p.pose);
        }
        
        particles_pub_->publish(pose_array);
    }

    void publish_estimated_pose() {
        // Вычисляем средневзвешенную позу
        double sum_x = 0.0, sum_y = 0.0;
        double sum_cos = 0.0, sum_sin = 0.0;
        
        for (const auto& p : particles_) {
            sum_x += p.weight * p.pose.position.x;
            sum_y += p.weight * p.pose.position.y;
            
            tf2::Quaternion q;
            tf2::fromMsg(p.pose.orientation, q);
            double yaw = getYawFromQuaternion(q);
            sum_cos += p.weight * cos(yaw);
            sum_sin += p.weight * sin(yaw);
        }
        
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.header.frame_id = get_parameter("world_frame").as_string();
        
        pose_msg.pose.pose.position.x = sum_x;
        pose_msg.pose.pose.position.y = sum_y;
        
        double avg_yaw = atan2(sum_sin, sum_cos);
        tf2::Quaternion q;
        q.setRPY(0, 0, avg_yaw);
        pose_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Простая ковариация
        pose_msg.pose.covariance[0] = 0.1;  // x
        pose_msg.pose.covariance[7] = 0.1;  // y
        pose_msg.pose.covariance[35] = 0.1; // yaw
        
        estimated_pose_pub_->publish(pose_msg);
        
        // Публикуем TF
        //geometry_msgs::msg::TransformStamped transform;
        //transform.header = pose_msg.header;
        //transform.child_frame_id = get_parameter("robot_frame").as_string();
        //transform.transform.translation.x = pose_msg.pose.pose.position.x;
        //transform.transform.translation.y = pose_msg.pose.pose.position.y;
        //transform.transform.translation.z = 0.0;
        //transform.transform.rotation = pose_msg.pose.pose.orientation;
        //tf_broadcaster_.sendTransform(transform);
    }

    // Параметры
    int num_particles_;
    
    // Данные
    std::vector<Particle> particles_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    nav_msgs::msg::Odometry::SharedPtr prev_odom_;
    
    // Генератор случайных чисел
    std::mt19937 gen_;
    
    // ROS интерфейсы
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNoMap>());
    rclcpp::shutdown();
    return 0;
}