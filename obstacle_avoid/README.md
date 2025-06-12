### 장애물 회피 주행

이전에 jetson에서 사용하던 dxl의 sub파일에서 필요한 부분을 수정하여 사용

```
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "line_tracer/dxl.hpp"
#include <memory>
#include <functional>
using namespace std::placeholders;
void mysub_callback(rclcpp::Node::SharedPtr node, Dxl& dxl, const
geometry_msgs::msg::Vector3::SharedPtr msg)
{
RCLCPP_INFO(node->get_logger(), "Received message: %lf,%lf", msg->x,msg->y);
dxl.setVelocity((int)msg->x, (int)msg->y);
}
int main(int argc, char* argv[])
{
rclcpp::init(argc, argv);
Dxl dxl;
auto node = std::make_shared<rclcpp::Node>("node_dxlsub");
if(!dxl.open())
{
RCLCPP_ERROR(node->get_logger(), "dynamixel open error");
rclcpp::shutdown();
return -1;
}
auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> fn;
fn = std::bind(mysub_callback, node, dxl, _1);
auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>("cmd_vel",qos_profile,fn);
rclcpp::spin(node);
dxl.close();
rclcpp::shutdown();
return 0;
}
```

