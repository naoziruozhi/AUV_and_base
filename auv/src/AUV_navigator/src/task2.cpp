#include <iostream>
#include <thread>
#include <chrono>

#include "CommWithGroundStation.h"
#include "CommWithMCU.h"
#include "thread_safe_queue.h"
#include "config.h"
#include "IAPSystem.h"
#include "VideoSystem.h"


 
#include "task.h"
 

#include <opencv2/core.hpp>       // 核心功能（包括基本数据结构）
#include <opencv2/highgui.hpp>    // GUI相关功能
#include <opencv2/imgproc.hpp>    // 图像处理相关功能
#include <opencv2/videoio.hpp>    // 视频输入输出相关功能
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/opencv.hpp>

std::string target_class_name = "red_ball"; // 指定类别名称


extern int record_land_depth ;
uint8_t task2Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov5& yolov5) 
{
    static int rovAutoControlState = 10;
 
    static auto time_last = std::chrono::high_resolution_clock::now();
    static auto time_now  = std::chrono::high_resolution_clock::now();
    
    time_last = time_now;
    time_now = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_last).count();

    std::cout << "duration1 is -----------> " << duration1 << std::endl;
    // 读取视频帧
    cv::Mat img;
    auto time_1 = std::chrono::high_resolution_clock::now();
    cap_down >> img;
    auto time_2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(time_2 - time_1).count();
    // std::cout << "duration2 is -----------> " << duration2 << std::endl;


    if (img.empty())
    {
        std::cout << "Video end." << std::endl;
        // break;
    }

 
    cv::Mat img_show = img.clone();

    // 检测结果
    std::vector<Detection> objects_v5;
    
    // 运行模型
    yolov5.Run(img, objects_v5);





    for (size_t i = 0; i < objects_v5.size(); ++i) 
    {
        const auto& object = objects_v5[i];
    

        cv::rectangle(img_show, object.box, object.color, 2);
        std::string draw_string = object.className + " " + std::to_string(object.confidence);
        cv::putText(img_show, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    object.color, 2);

    }
   
    static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
    GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;

    std::cout << "rovAutoControlState is ---------------> " << rovAutoControlState << std::endl;


    
    // rovAutoControlState = -1;
 
    switch(rovAutoControlState)
    {
        
        case -1:
        {
            ;
        }break;
        case 10:
        {




            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, target_class_name);

 
            float set_yaw_vel =  0 - 0.02*(img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0));
            if(set_yaw_vel > 80) set_yaw_vel = 80;
            if(set_yaw_vel < -80) set_yaw_vel = -80;

            // std::cout << "dir error  is --------------------> " << (img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0)) << std::endl;
            
            float set_depth_vel = 0.08 * (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0));
            if(set_depth_vel > 30) set_depth_vel = 30;
            if(set_depth_vel < -30) set_depth_vel = -30;
            // std::cout << "height error  is --------------------> " << (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0)) << std::endl;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = set_yaw_vel/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = set_depth_vel / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            if(abs((img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0))) < 50 && abs((img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0))) < 50)
            {
                rovAutoControlState = 11;
                stateEntryTime = std::chrono::steady_clock::now();
            }



 
        }break;

        case 11:
        {
            int forward_speed_set = 250;
            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, target_class_name);
            // std::cout << "max rectangle height is --------------------> " << rectangle_result.height << std::endl;
 
            float set_yaw_vel =  0 - 0.02*(img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0));
            if(set_yaw_vel > 80) set_yaw_vel = 80;
            if(set_yaw_vel < -80) set_yaw_vel = -80;

            // std::cout << "dir error  is --------------------> " << (img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0)) << std::endl;
            
            float set_depth_vel = 0.08 * (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0));
            if(set_depth_vel > 30) set_depth_vel = 30;
            if(set_depth_vel < -30) set_depth_vel = -30;
            // std::cout << "height error  is --------------------> " << (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0)) << std::endl;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = set_yaw_vel/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = set_depth_vel / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            if(rectangle_result.height > 250)
            {
                rovAutoControlState = 12;
                stateEntryTime = std::chrono::steady_clock::now();
            }

        }break;

        case 12:
        {

            static float delay_time = 2.5;
            static int forward_speed_set = 200;

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            auto currentTime = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= delay_time ) 
            {
                rovAutoControlState = 13;   
                stateEntryTime = std::chrono::steady_clock::now(); // 开始计时
            }
        }break;

        case 13:
        {
            static float delay_time = 3;
            static int forward_speed_set = -300;

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            auto currentTime = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= delay_time ) 
            {
                rovAutoControlState = 14;   
                stateEntryTime = std::chrono::steady_clock::now(); // 开始计时
            }
        }break;

        case 14:
        {
            static float delay_time = 2;
            static int forward_speed_set = 0;

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 40/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            auto currentTime = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= delay_time ) 
            {
                rovAutoControlState = 15;   
                stateEntryTime = std::chrono::steady_clock::now(); // 开始计时
            }

        }break;

        case 15:
        {
            static float delay_time = 0.5;
            static int forward_speed_set = 0;

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            auto currentTime = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= delay_time ) 
            {
                rovAutoControlState = 30;   
                stateEntryTime = std::chrono::steady_clock::now(); // 开始计时
            }

        }break;

        default:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode
        }break;
 


    }


    // cv::imshow("result", img_show);
    cv::waitKey(1);

    if(rovAutoControlState == 30)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

