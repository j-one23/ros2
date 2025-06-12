#include "line_tracer/vision.hpp"
#include "line_tracer/dxl.hpp"

bool ctrl_c_pressed = false;
void ctrlc_handler(int){ ctrl_c_pressed = true; }

bool mode = false;
#define UNDER200 0.27
#define OVER200 0.27
#define RPM 80
int main()
{
    
    string src = "nvarguscamerasrc sensor-id=0 ! \
	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12, framerate=(fraction)30/1 ! \
	nvvidconv flip-method=0 ! video/x-raw, \
	width=(int)640, height=(int)360, format=(string)BGRx ! \
	videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 
    
    
    VideoCapture source("/home/jetson/ros2_ws/simulation/5_lt_cw_100rpm_out.mp4"); 
    if (!source.isOpened()){ cout << "Camera error" << endl; return -1; }
    ///home/jetson/ros2_ws/simulation/5_lt_cw_100rpm_out.mp4
    ///home/jetson/ros2_ws/simulation/7_lt_ccw_100rpm_in.mp4
    

    string dst0 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.42 port=8000 sync=false";
    VideoWriter writer0(dst0,0, (double)30,Size(640,360),true);
    if(!writer0.isOpened()) {cerr<<"Writer open failed!"<<endl; return -1;}

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.42 port=8001 sync=false";
    VideoWriter writer1(dst1,0, (double)30,Size(640,360),false);
    if(!writer1.isOpened()) {cerr<<"Writer open failed!"<<endl; return -1;}

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.42 port=8002 sync=false";
    VideoWriter writer2(dst2,0, (double)30,Size(640,90),true);
    if(!writer2.isOpened()) {cerr<<"Writer open failed!"<<endl; return -1;}

    TickMeter tm;                  // 시간 측정 객체 생성
    signal(SIGINT, ctrlc_handler); //시그널핸들러

    Mat frame, cutthred;
    Mat labels, stats, centroids;
    int target; // 따라갈 객체
    Point center(320, 45); // 초기 중심점 설정 (하단 영상 중앙)
    int error;// 위치오차
    double closest = DBL_MAX;
    Dxl mx;
    int lvel = 0, rvel = 0;
    if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }

    while (true) {

        tm.start();
        source >> frame;
        if (frame.empty()){ cerr << "frame empty!" << endl; break; }

        cutthred = processFrame(frame);

        int labeling = connectedComponentsWithStats(cutthred, labels, stats, centroids);

        get_target(labeling, stats, center, centroids, closest, target, error, cutthred);

        cvtColor(cutthred, cutthred, COLOR_GRAY2BGR);

        draw_target(labeling, stats, cutthred, target, center, centroids);

        writer0 << frame;    // 컬러 원본
        //writer1 << thred;    // 이진화
        writer2 << cutthred; // 하단 1/4 이진화

        if(mx.kbhit())// 없으면제어멈춤
        { 
            char ch= mx.getch();
            if(ch== 'q') break;
            else if(ch== 's') {
                mode= true;
            }
        }
        get_Velocity( lvel, rvel, error, UNDER200, OVER200, RPM);
        if(mode) {
            mx.setVelocity(lvel, rvel);
        }
        if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출

        tm.stop();
        //double injury_time = 30 - tm.getTimeMilli(); // 모자란 시간
        //if(injury_time < 0){injury_time = -injury_time;}
        //else usleep(injury_time * 1000);
        cout << "err: " << error << " lvel: " << lvel << " rvel: " << rvel;
        cout << " time: " << tm.getTimeMilli() << " ms." << endl;//+ injury_time 
        tm.reset();  
    }
    mx.close(); // 장치닫기
    return 0;
}