#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
   nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
   h264parse ! rtph264pay pt=96 ! \
   udpsink host = 203.234.58.167 port = 8001 sync=false";

cv::VideoWriter writer;
cv::VideoWriter writer2;
        
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    writer << frame;
    writer2 << frame;
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");
    writer2.open("video.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(640, 360), true);
    if(!writer2.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer2 open failed!"); rclcpp::shutdown(); return -1; }
    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);
    if(!writer.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); rclcpp::shutdown(); return -1; }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    writer2.release();
    return 0;
}

