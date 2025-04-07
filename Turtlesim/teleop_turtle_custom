#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("node_pub1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", qos_profile);//토픽 이름 바꾸기

    geometry_msgs::msg::Twist twi;

    rclcpp::WallRate loop_rate(1.0); // 반복 주파수를 저장하는 객체 (단위 Hz)

    while (rclcpp::ok())
    {
        twi.linear.x = 0;
        twi.angular.z =0;
        char c;
        std::cout<<"키 입력>>";
        std::cin>>c;
        if(c == 'f')
            twi.linear.x = 1;
        else if(c == 'b')
            twi.linear.x = -1;
        else if(c == 'l')
            twi.angular.z = -1;    
        else if(c == 'r')
            twi.angular.z = 1;
        RCLCPP_INFO(node->get_logger(), "Publish: x %f y %f ", twi.linear.x,twi.angular.z);
        mypub->publish(twi);
        // rclcpp::spin_some(node);
        
        loop_rate.sleep(); // 반복 주파수에서 남은 시간만큼 sleep
    }
    rclcpp::shutdown();
    return 0;
}

