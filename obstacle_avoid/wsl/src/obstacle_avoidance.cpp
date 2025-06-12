#include <iostream>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <signal.h>
#include <cmath>
using namespace std;
using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)
#define DISTANCE 1
#define GAIN 0.5
#define SPEED 50

using std::placeholders::_1;
using namespace std::chrono_literals;

class RPLidarScanSubscriber : public rclcpp::Node
{
public:
    RPLidarScanSubscriber() : Node("rplidar_scan_subscriber"),count_(0)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::SensorDataQoS());
        rplidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", qos_profile, std::bind(&RPLidarScanSubscriber::ScanCallback, this, _1));

        auto qos_profile_1 = rclcpp::QoS(rclcpp::KeepLast(10));
        dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("cmd_vel", qos_profile_1);
        timer_ = this->create_wall_timer(100ms, std::bind(&RPLidarScanSubscriber::publish_velcmd_msg, this));
    }

private:
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        Mat lidar(500, 500, CV_8UC1, Scalar(255));
        arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2),
            Point(lidar.cols / 2, lidar.rows / 2- DISTANCE * 100), Scalar(0, 255, 255));
        int count = scan->scan_time / scan->time_increment;

        
        bool flag_R = false, flag_L = false; //일정거리안에 물체가 들어왔는지 판단하는 flag

        vector <float> distance_R, angle_R, distance_L, angle_L; //일정거리에 들어온 물체의 거리값, 각도 저장
        vector <Point> distance_pts_R, distance_pts_L;  //일정거리에 들어온 물체의 좌표값

        cvtColor(lidar, lidar, COLOR_GRAY2BGR);
        for (int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            
            float range = scan->ranges[i];
            double range_x, range_y;
            range_x = lidar.cols / 2 + ((range / 3.0) * sin(degree * M_PI / 180.) * (lidar.cols / 2));
            range_y = lidar.rows / 2 + ((range / 3.0) * cos(degree * M_PI / 180.) * (lidar.rows / 2));
           
            if (degree >= 90 && degree <= 180) {
                if (range < DISTANCE) {
                    distance_R.push_back(range);
                    distance_pts_R.push_back(Point(range_x, range_y));
                    angle_R.push_back(degree);
                    flag_R = true;
                    circle(lidar, Point(range_x, range_y), 1, Scalar(0, 0, 255), -1);
                }
                else {
                    circle(lidar, Point(range_x, range_y), 1, Scalar(0, 0, 255), -1);
                }
            }
            else if (degree <= -90 && degree >= -180) {
                if (range < DISTANCE) {
                    distance_L.push_back(range);
                    distance_pts_L.push_back(Point(range_x, range_y));
                    angle_L.push_back(degree);
                    flag_L = true;
                    circle(lidar, Point(range_x, range_y), 1, Scalar(0, 0, 255), -1);
                }
                else {
                    circle(lidar, Point(range_x, range_y), 1, Scalar(0, 0, 255), -1);
                }
            }
            else {
                circle(lidar, Point(range_x, range_y), 1, Scalar(255,0,0), -1);
            }
        }

        int min_index_R = 0, min_index_L = 0;
        float min_distance_L = 0.f, min_distance_R = 0.f;

        if (distance_L.size() > 0) {
            min_distance_L = distance_L.at(0);
            for (int i = 1; i < distance_L.size(); i++) {
                if (min_distance_L > distance_L[i]) {
                    min_distance_L = distance_L[i];
                    min_index_L = i;
                }
            }
        }
        if (distance_R.size() > 0) {
            min_distance_R = distance_R.at(0);
            for (int i = 1; i < distance_R.size(); i++) {
                if (min_distance_R > distance_R[i]) {
                    min_distance_R = distance_R[i];
                    min_index_R = i;
                }
            }
        }

        float angle_arrow = 0.f;
        if (flag_L&&flag_R) {
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), distance_pts_L[min_index_L], Scalar(0, 255, 0));
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), distance_pts_R[min_index_R], Scalar(255, 0, 0));
            cout << "ROI 왼쪽"<< DISTANCE <<"m반경에 들어오는 객체의 갯수 : " << distance_L.size() << '\t' << "최소거리 : " 
                << min_distance_L << '\t' << "각도 : " << angle_L[min_index_L] << '\t' << endl;
            cout << "ROI 오른쪽" << DISTANCE << "m반경에 들어오는 객체의 갯수 : " << distance_R.size() << '\t' 
                << "최소거리 : " << min_distance_R << '\t' << "각도 : " << angle_R[min_index_R] << '\t' << endl;
            angle_arrow = angle_R[min_index_R] + angle_L[min_index_L];
            // if (angle_arrow != 0) {
            //     error = (((DISTANCE * 100) - distance_pts_L[min_index_L].x) - (distance_pts_R[min_index_R].x - (DISTANCE * 100))) / 2;
            // }
            //error = (((DISTANCE * 100) - distance_pts_L[min_index_L].x) - (distance_pts_R[min_index_R].x - (DISTANCE * 100))) / 2;
            error = (((((lidar.cols / 2) - (DISTANCE * 100))) - distance_pts_L[min_index_L].x) - (distance_pts_R[min_index_R].x - ((lidar.cols / 2) + (DISTANCE * 100)))) / 2.0;
            // else {
            //     error = (lidar.cols / 2 + 40) - distance_pts_R[min_index_R].x;
            // }
        }
        else if (flag_R && !flag_L) {
            if (angle_R[min_index_R] != 0) {
                arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), distance_pts_R[min_index_R], Scalar(255, 0, 0));
            }
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2),
                Point(lidar.cols / 2 - DISTANCE*100, lidar.rows / 2), Scalar(255, 0, 255));
            cout << "ROI 오른쪽" << DISTANCE << "m반경에 들어오는 객체의 갯수 : " << distance_R.size() << '\t' 
                << "최소거리 : " << min_distance_R << '\t' << "각도 : " << angle_R[min_index_R] << '\t' << endl;
            error =  -((DISTANCE * 100) - distance_pts_R[min_index_R].x);
        }
        else if (!flag_R && flag_L) {
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), distance_pts_L[min_index_L], Scalar(0, 255, 0));
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2),
                Point(lidar.cols / 2 + (DISTANCE ) * 100, lidar.rows / 2), Scalar(153, 0, 0));
            cout << "ROI 왼쪽" << DISTANCE << "m반경에 들어오는 객체의 갯수 : " << distance_L.size() << '\t' << "최소거리 : " 
                << min_distance_L << '\t' << "각도 : " << angle_L[min_index_L] << '\t' << endl;
            error = (DISTANCE * 100) - distance_pts_L[min_index_L].x;

        }
        else {
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), 
                Point(lidar.cols / 2 + (DISTANCE ) * 100, lidar.rows / 2), Scalar(255, 0, 0));
            arrowedLine(lidar, Point(lidar.cols / 2, lidar.rows / 2), 
                Point(lidar.cols / 2 - DISTANCE * 100, lidar.rows / 2), Scalar(255, 0, 0));
            error = 0.f;
        }
        if (error > 0) {
            cout << "좌회전중 error: " << error/2.f << endl;
        }
        else if (error < 0) {
            cout << "우회전중 error: " << error/2.f << endl;
        }
        else {
            cout << "직진중 error: " << error/2.f << endl;
        }

        imshow("lidar", lidar);
        waitKey(10);
    }

    void publish_velcmd_msg() {
        msg.x = SPEED -  error*GAIN;
        msg.y =  -1*SPEED - error*GAIN;
        dynamixel_publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dynamixel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rplidar_subscriber_;
    geometry_msgs::msg::Vector3 msg;
    double error;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPLidarScanSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}