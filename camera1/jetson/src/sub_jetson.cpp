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
	udpsink host=203.234.58.167 port=8001 sync=false";

cv::VideoWriter writer;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat raw = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (raw.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty image received");
        return;
    }

    cv::Mat gray;
    cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
    cv::Mat binary_bgr;
    cv::cvtColor(binary, binary_bgr, cv::COLOR_GRAY2BGR);
    writer << binary_bgr;
    RCLCPP_INFO(node->get_logger(), "Received Binary Image: %s, %d x %d", msg->format.c_str(), binary.rows, binary.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_binary");

    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}