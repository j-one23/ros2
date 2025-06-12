#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/vector3.hpp"
//#include "dxl/dxl.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <chrono>
#include<math.h>
#define SPEED 100
#define GAIN 0.2 //0.25

using namespace std::chrono_literals;
using std::placeholders::_1;

class ld : public rclcpp::Node{
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;
    geometry_msgs::msg::Vector3 vel;
   // rclcpp::TimerBase::SharedPtr timer_;
    void subcallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
        cv::Mat frame, stats, centroids, labels, org;
        frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
        org = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
        cv::imshow("wsl",frame);
        cv::imshow("org", org);
        RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
        frame = frame(cv::Rect(cv::Point(0,frame.rows/4*3),cv::Point(frame.cols,frame.rows)));
        static cv::Point po(frame.cols/2,frame.rows/2);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        cv::threshold(frame, frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        int lable =  cv::connectedComponentsWithStats(frame, labels, stats, centroids);
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        int index,a=1;
        static bool b = true;
        double mindistance;
        for(int i = 1;i<lable;i++){
            if(stats.at<int>(i,4)<50){
                a++;
                continue;
            }
            if(i == a){
                index = i;
                mindistance = sqrt(pow((po.x - centroids.at<double>(i,0)),2)+pow((po.y - centroids.at<double>(i,1)),2));
            }
            else if(sqrt(pow((po.x-centroids.at<double>(i,0)),2)+pow((po.y- centroids.at<double>(i,1)),2))<mindistance){
                mindistance = sqrt(pow((po.x-centroids.at<double>(i,0)),2)+pow((po.y- centroids.at<double>(i,1)),2));
                index = i;
            }
         }
        if(po.y>frame.rows-35){
                b = false;
            }
        if(mindistance < 100){
            b = true;
            }
         for(int i = 1;i<lable;i++){
                cv::Scalar sc = i == index && b ? cv::Scalar(0,0,255):cv::Scalar(255,0,0);
                if(stats.at<int>(i,4)<50) sc = cv::Scalar(0,255,255);
                cv::rectangle(frame,cv::Rect(stats.at<int>(i,0),stats.at<int>(i,1),stats.at<int>(i,2),stats.at<int>(i,3)),sc);
                cv::rectangle(frame,cv::Rect(centroids.at<double>(i,0),centroids.at<double>(i,1),3,3),sc);
            } 
            cv::rectangle(frame,cv::Rect(po.x,po.y,3,3),cv::Scalar(0,255,0));
            if(b){        
            po = cv::Point(centroids.at<double>(index,0),centroids.at<double>(index,1));
            }
            cv::circle(frame, po, 6, cv::Scalar(0, 0, 255), -1);
            vel.y =  -1*SPEED -  (frame.cols/2 -po.x)*GAIN;
            vel.x = SPEED -  (frame.cols/2 -po.x)*GAIN;
            pub->publish(vel);
            cv::imshow("wsl",frame);
            cv::waitKey(1);
        }
    public:
    ld(std::string s): Node(s){
        vel.x = 0;
        vel.y = 0;
        vel.z = 0;
        pub = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", rclcpp::QoS(rclcpp::KeepLast(10)));
        sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::QoS(
            rclcpp::KeepLast(10))/*.best_effort()*/,std::bind(&ld::subcallback, this, _1));
     //   timer_ = this->create_wall_timer(40ms,std::bind(&jw::pubcallback ,this));
    }
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ld>("node_dxlpub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
