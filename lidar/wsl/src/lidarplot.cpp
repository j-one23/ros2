#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI) //라디안 디그리 변환

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  cv::Mat lidar(500,500,CV_8UC3,cv::Scalar(255,255,255));
  int count = scan->scan_time / scan->time_increment;
  //1회 스캔 전체 시간을 1개의 포인트마다 측정되는 시간 간격으로 나누면,
  //스캔 1회(=한 토픽 메시지)에 포함된 거리 측정 포인트 총 개수가 나옵니다.
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);//각 포인트 각도 계산
    //                    레이저 스캔 시작 각도 + 한 포인트마다 각도 변화량
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    double deg_ =(degree)*M_PI/180 ; //각도 -> 라디안으로
    double dis_ =scan->ranges[i]; //각도에 대한 거리
    //극좌표를 직교좌표로 변환 (각, 거리) -> x,y좌표
    double lidarx = 250+(sin(deg_)*dis_*250/10); //x좌표, x = r * sin(θ)
    double lidary = 250+(cos(deg_)*dis_*250/10); // y좌표, y = r * cos(θ)
    //250/12
    cv::rectangle(lidar,cv::Rect(lidarx,lidary,3,3),cv::Scalar(0,0,255),-1);
  }
  cv::rectangle(lidar,cv::Rect(lidar.cols/2-1,lidar.rows/2-1,3,3),cv::Scalar(0,0,0),-1);
  cv::imshow("lidar",lidar);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
