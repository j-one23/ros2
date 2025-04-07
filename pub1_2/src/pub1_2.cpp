#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("node_pub1");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1", qos_profile);

    geometry_msgs::msg::Vector3 vec;

    //rclcpp::WallRate loop_rate(1.0); // 반복 주파수를 저장하는 객체 (단위 Hz)

    while (rclcpp::ok())
    {
        double x,y,z;
        std::cout<<"x값 입력 >>";
        std::cin>>x;
        std::cout<<"y값 입력 >>";
        std::cin>>y;
        std::cout<<"z값 입력 >>";
        std::cin>>z;
        vec.x = x;
        vec.y = y;
        vec.z = z;
        RCLCPP_INFO(node->get_logger(), "Publish: x %f y %f  z %f", vec.x,vec.y,vec.z);
        mypub->publish(vec);
        // rclcpp::spin_some(node);
        //loop_rate.sleep(); // 반복 주파수에서 남은 시간만큼 sleep
    }
    rclcpp::shutdown();
    return 0;
}
