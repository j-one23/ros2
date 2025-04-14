#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"  // 올바른 메시지 타입 include
#include <memory>
#include <chrono>
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_2", qos_profile);
    int count = 0;
    rclcpp::WallRate loop_rate(5.0);
    while (rclcpp::ok())
    {
        std_msgs::msg::Int32 message;
        message.data = count++;
        RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);
        mypub->publish(message);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}