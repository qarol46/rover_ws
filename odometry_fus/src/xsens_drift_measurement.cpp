#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <vector>
#include <numeric>

class XsensDriftAnalyzer : public rclcpp::Node
{
public:
    XsensDriftAnalyzer() : Node("xsens_drift_analyzer")
    {
        // Параметры
        measurement_duration_ = 600.0;  // 10 минут в секундах
        report_interval_ = 60.0;        // Интервал отчетов (1 минута)

        // Подписка на топик IMU
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&XsensDriftAnalyzer::imu_callback, this, std::placeholders::_1));

        start_time_ = this->now();
        last_report_time_ = start_time_;

        RCLCPP_INFO(this->get_logger(), "Xsens MTi-G-710 Drift Analyzer initialized. Waiting for IMU data...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (measurement_complete_) return;

        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();

        // Получаем текущий угол рысканья (yaw) из кватерниона
        double roll, pitch, yaw;
        tf2::Quaternion quat(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Сохраняем начальный угол
        if (!initial_yaw_set_) {
            initial_yaw_ = yaw;
            current_yaw_ = yaw;
            initial_yaw_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial yaw angle: %.6f rad", initial_yaw_);
            return;
        }

        current_yaw_ = yaw;

        // Проверяем, прошла ли очередная минута
        if ((current_time - last_report_time_).seconds() >= report_interval_) {
            int minutes = static_cast<int>(elapsed_time / 60);
            int seconds = static_cast<int>(elapsed_time) % 60;

            // Вычисляем разницу углов
            double angle_diff = calculate_angle_difference(initial_yaw_, current_yaw_);
            yaw_differences_.push_back(angle_diff);

            // Выводим отчет за минуту
            char report[256];
            snprintf(report, sizeof(report),
                     "After %d min %d sec: Angle drift = %.6f rad (%.4f deg)",
                     minutes, seconds, angle_diff, angle_diff * (180.0 / M_PI));
            minute_reports_.push_back(report);
            RCLCPP_INFO(this->get_logger(), "%s", report);

            last_report_time_ = current_time;
        }

        // Проверяем завершение измерений
        if (elapsed_time >= measurement_duration_) {
            measurement_complete_ = true;
            calculate_final_drift();
        }
    }

    double calculate_angle_difference(double start_angle, double end_angle)
    {
        double diff = end_angle - start_angle;
        // Нормализуем разницу в диапазон [-π, π]
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }

    void calculate_final_drift()
    {
        // Вычисляем среднюю разницу углов за минуту
        double sum = std::accumulate(yaw_differences_.begin(), yaw_differences_.end(), 0.0,
            [](double acc, double val) { return acc + std::abs(val); });
        double avg_diff_per_minute = sum / yaw_differences_.size();

        // Преобразуем в скорость дрейфа (рад/сек)
        double drift_rate = avg_diff_per_minute / 60.0;

        // Выводим итоговый отчет
        RCLCPP_INFO(this->get_logger(), "\n======================================");
        RCLCPP_INFO(this->get_logger(), "Xsens MTi-G-710 Z-axis Drift Analysis Results:");
        RCLCPP_INFO(this->get_logger(), "Measurement duration: %.1f minutes", measurement_duration_/60.0);

        for (const auto& report : minute_reports_) {
            RCLCPP_INFO(this->get_logger(), "%s", report.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "\nFinal Results:");
        RCLCPP_INFO(this->get_logger(), "Average angular drift per minute: %.8f rad/min", avg_diff_per_minute);
        RCLCPP_INFO(this->get_logger(), "Average drift rate: %.8f rad/sec", drift_rate);
        RCLCPP_INFO(this->get_logger(), "                     %.8f deg/sec", drift_rate * (180.0 / M_PI));
        RCLCPP_INFO(this->get_logger(), "======================================\n");
    }

    // Параметры
    double measurement_duration_;
    double report_interval_;
    bool measurement_complete_ = false;

    // Данные IMU
    double initial_yaw_ = 0.0;
    double current_yaw_ = 0.0;
    bool initial_yaw_set_ = false;
    std::vector<double> yaw_differences_;
    std::vector<std::string> minute_reports_;

    // Временные метки
    rclcpp::Time start_time_;
    rclcpp::Time last_report_time_;

    // Подписка
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XsensDriftAnalyzer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}