#include "psub2_2/pub.hpp"
Pub::Pub() : Node("mypub"), count_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
    timer_ = this->create_wall_timer(50ms, std::bind(&Pub::publish_msg, this));
}
void Pub::publish_msg()
{
    auto msg = geometry_msgs::msg::Vector3();
    static int m = 0;
    msg.x= msg.y= msg.z= m++;
    RCLCPP_INFO(this->get_logger(), "Published message: x: %lf, y: %lf, z: %lf", msg.x,msg.y,msg.z);
    pub_->publish(msg);
    }