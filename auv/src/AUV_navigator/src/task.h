#ifndef __TASK_H__
#define __TASK_H__

#include <stdint.h>
#include <iostream>
#include <thread>
#include <chrono>

#include "CommWithGroundStation.h"
#include "CommWithMCU.h"
#include "thread_safe_queue.h"
#include "config.h"
#include "IAPSystem.h"
#include "VideoSystem.h"


// #include "yolov5.h"
// #include "logging.h"
// #include "cv_draw.h"

#include "yolov8_custom.h"
#include "yolov5.h"
// #include "utils/logging.h"
// #include "draw/cv_draw.h"
 


#include <opencv2/core.hpp>       // 核心功能（包括基本数据结构）
#include <opencv2/highgui.hpp>    // GUI相关功能
#include <opencv2/imgproc.hpp>    // 图像处理相关功能
#include <opencv2/videoio.hpp>    // 视频输入输出相关功能
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/opencv.hpp>



float getRollAnlge(RovSystem& rov);   //获取机器滚转角， 单位： 度
float getPitchAnle(RovSystem& rov);    //获取机器俯仰角， 单位： 度
float getYawAnle(RovSystem& rov);      //获取机器偏航角， 单位： 度
int16_t getGyroX(RovSystem& rov);  //获取机器 x 方向角速度，单位 °/s
int16_t getGyroY(RovSystem& rov);  //获取机器 y 方向角速度，单位 °/s
int16_t getGyroZ(RovSystem& rov);  //获取机器 z 方向角速度，单位 °/s
int16_t getDepth(RovSystem& rov);  //单位 cm 向下为负 ， 例如 ROV再水下30cm，此函数返回值-30
int16_t getDepthvelocity(RovSystem& rov);  //单位 cm/s，向上为正  




void setForwardThrottle(int16_t throttle, CommWithGroundStation& GroundStationComm);   //  控制ROV前后运动，throttle油门 范围  -1000 ~ 1000，向前为正
void setRightThrottle(int16_t throttle, CommWithGroundStation& GroundStationComm);   //  控制ROV左右平移，throttle 范围  -1000 ~ 1000，向右为正
void setDepthSpeed(int16_t speed, CommWithGroundStation& GroundStationComm);   //  控制ROV上下运动，速度 范围  -30 ~ 30 cm/s，向上为正
void setYawAngularvelocity(int16_t velocity, CommWithGroundStation& GroundStationComm);   //  控制ROV 航向角角速度，  范围  -80 ~ 80 °/s，顺时针为正
void setDepth(int16_t depth, CommWithGroundStation& GroundStationComm); //  设定ROV 深度，并且在到达相应深度后，锁定在设定深度上。范围 -1000cm ~ 1000cm   向上为正，例如要下潜到1米，设置为 -100
void setYawAngle(int16_t angle, CommWithGroundStation& GroundStationComm);  //  设定ROV偏航角度，并且在转到对应偏航角上，锁定偏航角。范围 -180° ~ 180°    
void lockDepth(CommWithGroundStation& GroundStationComm);  //将ROV深度锁定为当前深度
void lockYawAnlge(CommWithGroundStation& GroundStationComm);  //将ROV偏航角锁定为当前偏航角

 
void setPWM(int16_t pwm[8], CommWithGroundStation& GroundStationComm);   // 8路pwm，频率50HZ，每个值代表脉宽，范围0-20000us, 一般舵机脉宽范围是500us -- 2500 us

void setYawAngularvelocity_B(int16_t velocity, CommWithGroundStation& GroundStationComm);










#define ZERO_FOREARD_THR -30


// bool checkForClass(const std::vector<Detection>& objects, const std::string& target_class_name); 
// uint8_t task2Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov8Custom& yolov8); 
uint8_t task2Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov5& yolov5); 
uint8_t task3Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov8Custom& yolov8, Yolov5& yolov5); 
uint8_t task4Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov8Custom& yolov8, Yolov5& yolov5); 






cv::Rect getMaxHeightObject(const std::vector<Detection>& objects, const std::string& target_class_name); 

std::vector<KeyPoint> getMaxHeightBoxKeypointsByClassWithMinScore(const std::vector<Detection>& objects, 
                                                                  const std::vector<std::map<int, KeyPoint>>& kps, 
                                                                  const std::string& targetClassName, 
                                                                  float minScore); 


double euclideanDistance(const cv::Point& p1, const cv::Point& p2);
Detection findClosestObject(const std::vector<Detection>& objects, const std::string& targetClassName, const cv::Point& targetPoint, double maxDistance);
#endif
