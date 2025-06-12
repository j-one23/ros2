#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;

cv::VideoWriter writer_original;
cv::VideoWriter writer_gray;
cv::VideoWriter writer_binary;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Failed to decode image.");
        return;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    cv::Size frame_size = frame.size();
    if (frame_size.width != 640 || frame_size.height != 360) {
        cv::resize(frame, frame, cv::Size(640, 360));
        cv::resize(gray, gray, cv::Size(640, 360));
        cv::resize(binary, binary, cv::Size(640, 360));
    }

    cv::imshow("Original Image", frame);
    cv::imshow("Grayscale Image", gray);
    cv::imshow("Binary Image", binary);

    writer_original.write(frame);
    writer_gray.write(gray);
    writer_binary.write(binary);

    cv::waitKey(1);

    RCLCPP_INFO(node->get_logger(), "Received Image: %s, size: %d x %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  
    double fps = 30.0;  
    cv::Size frame_size(640, 360);  

    writer_original.open("output_original.avi", fourcc, fps, frame_size, true);
    writer_gray.open("output_gray.avi", fourcc, fps, frame_size, false);  // Grayscale video (1 channel)
    writer_binary.open("output_binary.avi", fourcc, fps, frame_size, false);  // Binary video (1 channel)

    if (!writer_original.isOpened() || !writer_gray.isOpened() || !writer_binary.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open the video writers.");
        return -1;
    }

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);
    
    rclcpp::spin(node);

    writer_original.release();
    writer_gray.release();
    writer_binary.release();

    rclcpp::shutdown();
    return 0;
}
