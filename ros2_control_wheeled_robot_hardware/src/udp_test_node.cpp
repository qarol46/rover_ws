#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include "rclcpp/rclcpp.hpp"

class UdpTestNode : public rclcpp::Node {
public:
    UdpTestNode() : Node("udp_test_node") {
        // Инициализация с параметрами из вашего hardware_interface
        udp_socket_ = std::make_unique<Eth_Socket>();
        if (!udp_socket_->Initialize("192.168.1.100", 8888, 8889)) {
            RCLCPP_ERROR(get_logger(), "UDP initialization failed!");
            throw std::runtime_error("UDP init failed");
        }

        // Таймер для тестовой отправки данных
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->test_communication(); });
    }

private:
    void test_communication() {
        // Тест отправки
        double test_speeds[2] = {0.5, -0.3}; // Пример скоростей
        if (udp_socket_->SendWheelSpeeds(test_speeds)) {
            RCLCPP_INFO(get_logger(), "Sent speeds: left=%.2f, right=%.2f", 
                       test_speeds[0], test_speeds[1]);
        }

        // Тест приема
        double speeds[6], positions[6];
        if (udp_socket_->GetWheelStates(speeds, positions)) {
            RCLCPP_INFO(get_logger(), "Received: ");
            for (int i = 0; i < 6; ++i) {
                RCLCPP_INFO(get_logger(), "Wheel %d: speed=%.2f, pos=%.2f", 
                           i, speeds[i], positions[i]);
            }
        }
    }

    std::unique_ptr<Eth_Socket> udp_socket_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}