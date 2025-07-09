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
 



 
uint8_t rectangle_num = 4;
uint8_t pass_rectangle_num = 0;




 

// 函数：找到特定类别中height最大的目标，返回其坐标
cv::Rect getMaxHeightObject(const std::vector<Detection>& objects, const std::string& target_class_name) 
{
    cv::Rect maxHeightRect;
    int maxHeight = 0;

    for (const auto& obj : objects) 
    {
        if (obj.className == target_class_name) 
        {
            if (obj.box.height > maxHeight) 
            {
                maxHeight = obj.box.height;
                maxHeightRect = obj.box;
            }
        }
    }

    return maxHeightRect;
}

// 函数返回指定类别中最大高度框的keypoint score大于阈值的KeyPoint集
std::vector<KeyPoint> getMaxHeightBoxKeypointsByClassWithMinScore(const std::vector<Detection>& objects, 
                                                                  const std::vector<std::map<int, KeyPoint>>& kps, 
                                                                  const std::string& targetClassName, 
                                                                  float minScore) 
{
    int maxHeightIndex = -1;
    int maxHeight = 0;
    bool found = false;

    // 遍历检测结果，寻找指定类别中最大高度的框
    for (int i = 0; i < objects.size(); ++i) 
    {
        if (objects[i].className == targetClassName && objects[i].box.height > maxHeight) 
        {
            maxHeight = objects[i].box.height;
            maxHeightIndex = i;
            found = true;
        }
    }

    if (!found) 
    {
        throw std::runtime_error("No detections found for the specified class.");
    }

    // 提取对应的KeyPoint集，但只包括分数高于阈值的KeyPoints
    std::vector<KeyPoint> validKeyPoints;
    const auto& keypointsMap = kps[maxHeightIndex];
    for (const auto& pair : keypointsMap) 
    {
        const KeyPoint& kp = pair.second;
        if (kp.score > minScore) 
        {
            validKeyPoints.push_back(kp);
        }
    }

    return validKeyPoints;
}

// 计算两点之间的欧氏距离
double euclideanDistance(const cv::Point& p1, const cv::Point& p2) 
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 寻找最接近的对象
Detection findClosestObject(const std::vector<Detection>& objects, const std::string& targetClassName, const cv::Point& targetPoint, double maxDistance) 
{
    Detection closestObject;
    double minDistance = std::numeric_limits<double>::max();  // Initialize with the maximum possible double value

    for (const auto& object : objects) 
    {
        if (object.className == targetClassName) 
        {
            cv::Point objectCenter = cv::Point(object.box.x + object.box.width / 2, object.box.y + object.box.height / 2);
            double distance = euclideanDistance(targetPoint, objectCenter);

            if (distance < minDistance && distance < maxDistance) 
            {
                minDistance = distance;
                closestObject = object;
            }
        }
    }

    if (minDistance == std::numeric_limits<double>::max()) 
    {
        throw std::runtime_error("No suitable object found within the distance threshold.");
    }

    return closestObject;
}


uint8_t task3Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm, cv::VideoCapture& cap_up, cv::VideoCapture& cap_down, Yolov8Custom& yolov8, Yolov5& yolov5) 
{
    static int rovAutoControlState = 2;

    static auto time_last = std::chrono::high_resolution_clock::now();
    static auto time_now  = std::chrono::high_resolution_clock::now();
    
    time_last = time_now;
    time_now = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_last).count();

    // std::cout << "duration1 is -----------> " << duration1 << std::endl;
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





    // std::cout << "Number of objects-------------------->: " << objects.size() << std::endl;

    // std::cout << "Number of kps-------------------->: " << kps.size() << std::endl;
    cv::Mat img_show = img.clone();








    // for (size_t i = 0; i < objects.size(); ++i) 
    // {
    //     const auto& object = objects[i];
    //     const auto& keypoints_map = kps[i];

    //     // cv::rectangle(img_show, object.box, object.color, 2);
    //     // std::string draw_string = object.className + " " + std::to_string(object.confidence);
    //     // cv::putText(img_show, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //     //             object.color, 2);



    //     for (const auto& keypoint_item : keypoints_map) 
    //     {
    //     // cv::putText(img_show, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //     //             object.color, 2);
    //         if(keypoint_item.second.score > 0.1)
    //         {
    //             std::string draw_string = std::to_string(keypoint_item.second.id);
    //             cv::putText(img_show, draw_string, cv::Point(keypoint_item.second.x, keypoint_item.second.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //                     object.color, 2);

                
    //             cv::circle(img_show, cv::Point(keypoint_item.second.x, keypoint_item.second.y), 5, object.color, -1);
    //         }

    //     }

    // }
 







 
    // 检测结果
    std::vector<Detection> objects_v5;
    
    // 运行模型
    yolov5.Run(img, objects_v5);




    if(rovAutoControlState != 5)
    {
        for (size_t i = 0; i < objects_v5.size(); ++i) 
        {
            const auto& object = objects_v5[i];
        

            cv::rectangle(img_show, object.box, object.color, 2);
            std::string draw_string = object.className + " " + std::to_string(object.confidence);
            cv::putText(img_show, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        object.color, 2);

        }
    }

    // else if()

 








    // 绘制框，显示结果
    // DrawDetections(img, objects);
    // cv::imshow("result", img);
    // cv::waitKey(1);
    // std::string front1_target_class_name = "front1"; // 指定类别名称
    // static std::deque<bool> front1_recent_detections(10, false); // 最近十帧的检测记录
    // static int front1_count_detected = 0; // 记录 "front1" 检测到的次数


    
     
    // // 遍历所有检测到的物体
    // for (const auto& obj : objects) 
    // {
    //     // 输出物体种类名称
    //     // std::cout << "Class: " << obj.className << std::endl;

    //     // 输出物体的坐标，包括 x, y, width, height
    //     // std::cout << "Coordinates: "
    //     //           << "x=" << obj.box.x
    //     //           << ", y=" << obj.box.y
    //     //           << ", width=" << obj.box.width
    //     //           << ", height=" << obj.box.height << std::endl;
    
    //     if(obj.className == "front1")
    //     {
    
    //         // std::cout << "class id is -----------> " << obj.class_id << std::endl;
    //     }
    // }

    // bool front1_detected = checkForClass(objects, front1_target_class_name);

    // bool result_front1 = updateDetectionHistory(front1_recent_detections, front1_count_detected, front1_detected);


    // static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
    

    static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
    GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;

    std::cout << "rovAutoControlState is ---------------> " << rovAutoControlState << std::endl;
    std::cout << "pass_rectangle_num is ---------------> " << pass_rectangle_num << std::endl;
    
    
    // rovAutoControlState = -1;
 
    switch(rovAutoControlState)
    {




        case -1:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0 / 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode
        }break;




        case 0:    //先左右转动找最大框，代表最近的框    左转
        {
            
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = -20 / 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            auto currentTime = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= 0.3) //计时超过2s 进入下一个状态
            {
                rovAutoControlState = 1;
                stateEntryTime = std::chrono::steady_clock::now();
            }





        }break;


        case 1: //先左右转动找最大框，代表最近的框    右转
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 20/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            auto currentTime = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= 0.6) //计时超过2s 进入下一个状态
            {
                rovAutoControlState = 2;
                stateEntryTime = std::chrono::steady_clock::now();
            }
        }break;

        case 2:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            rovAutoControlState = 3;
            stateEntryTime = std::chrono::steady_clock::now();
   
            

            

        }break;


        case 3:   //角度和深度方向对框，满足两个阈值进入下一个状态，需要增加如果一致不满足阈值的处理
        {
            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, "rectangle");
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
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode


            if(abs((img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0))) < 50 && abs((img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0))) < 50)
            {
                rovAutoControlState = 4;
                stateEntryTime = std::chrono::steady_clock::now();
            }




        }break;


        case 4:  //在对框的基础上前进，在框的高度大于一定的阈值下进入下一个状态
        {


            int forward_speed_set = 250;
            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, "rectangle");
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


            if(rectangle_result.height > 370)
            {
                rovAutoControlState = 5;
                stateEntryTime = std::chrono::steady_clock::now();
            }

        }break;

        case 5:
        {
   
            




            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, "rectangle");
            // std::cout << "max rectangle height is --------------------> " << rectangle_result.height << std::endl;
            int forward_speed_set = 1 * (480 - rectangle_result.height);
            if(forward_speed_set > 1000) forward_speed_set = 1000;
            if(forward_speed_set < -1000) forward_speed_set = -1000;


            int right_speed_set = 0.55*(rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0);


            std::cout << "widht error is ----------------> " <<rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0 << std::endl;
            if(right_speed_set > 1000) right_speed_set = 1000;
            if(right_speed_set < -1000) right_speed_set = -1000;
             
            float set_yaw_vel = 0;
            // float set_yaw_vel =  0 - 0.02*(img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0));
            // if(set_yaw_vel > 80) set_yaw_vel = 80;
            // if(set_yaw_vel < -80) set_yaw_vel = -80;

            // std::cout << "dir error  is --------------------> " << (img.cols/2.0 - (rectangle_result.x + rectangle_result.width / 2.0)) << std::endl;
            
            float set_depth_vel = 0.08 * (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0));
            if(set_depth_vel > 30) set_depth_vel = 30;
            if(set_depth_vel < -30) set_depth_vel = -30;



 
            std::vector<Detection> objects;
            // 关键点
            std::vector<std::map<int, KeyPoint>> kps;


 
            yolov8.Run(img, objects, kps);
           


            try 
            {
                auto keypoints = getMaxHeightBoxKeypointsByClassWithMinScore(objects, kps, "rectangle", 0.1);
                // 这里可以根据需要处理keypoints
                
                std::cout << "Number of keypoints: " << keypoints.size() << std::endl;

                if(keypoints.size() == 4)
                {
                    float y_diff_0_3 = 0.0;
                    float y_diff_1_2 = 0.0;
                    KeyPoint kp0, kp1, kp2, kp3;

                    // 查找具体的keypoints
                    for (const auto& kp : keypoints) {
                        if (kp.id == 0) kp0 = kp;
                        else if (kp.id == 1) kp1 = kp;
                        else if (kp.id == 2) kp2 = kp;
                        else if (kp.id == 3) kp3 = kp;
                    }

                    // 计算高度差
                    y_diff_0_3 = std::abs(kp0.y - kp3.y);
                    y_diff_1_2 = std::abs(kp1.y - kp2.y);


                    std::cout << "height error is -------------------------------------------->  : " << y_diff_0_3 - y_diff_1_2 << std::endl;


                    cv::Point targetPoint0(kp0.x, kp0.y); // 指定的点
                    cv::Point targetPoint1(kp1.x, kp1.y); // 指定的点
                    cv::Point targetPoint2(kp2.x, kp2.y); // 指定的点
                    cv::Point targetPoint3(kp3.x, kp3.y); // 指定的点
                    double maxDistance = 100.0; // 设定的阈值

                    try 
                    {
                        Detection closest0 = findClosestObject(objects_v5, "point", targetPoint0, maxDistance);
                        Detection closest1 = findClosestObject(objects_v5, "point", targetPoint1, maxDistance);
                        Detection closest2 = findClosestObject(objects_v5, "point", targetPoint2, maxDistance);
                        Detection closest3 = findClosestObject(objects_v5, "point", targetPoint3, maxDistance);



                        // cv::rectangle(img_show, closest0.box, closest0.color, 2);
                        // cv::rectangle(img_show, closest1.box, closest1.color, 2);
                        // cv::rectangle(img_show, closest2.box, closest2.color, 2);
                        // cv::rectangle(img_show, closest3.box, closest3.color, 2);




                        float b0_pointx = closest0.box.x + closest0.box.width / 2.0; 
                        float b0_pointy = closest0.box.y + closest0.box.height / 2.0; 

                        float b1_pointx = closest1.box.x + closest1.box.width / 2.0; 
                        float b1_pointy = closest1.box.y + closest1.box.height / 2.0; 

                        float b2_pointx = closest2.box.x + closest2.box.width / 2.0; 
                        float b2_pointy = closest2.box.y + closest2.box.height / 2.0; 

                        float b3_pointx = closest3.box.x + closest3.box.width / 2.0; 
                        float b3_pointy = closest3.box.y + closest3.box.height / 2.0; 


                        cv::circle(img_show, cv::Point(b0_pointx, b0_pointy), 5, closest0.color, -1);
                        cv::circle(img_show, cv::Point(b1_pointx, b1_pointy), 5, closest1.color, -1);
                        cv::circle(img_show, cv::Point(b2_pointx, b2_pointy), 5, closest2.color, -1);
                        cv::circle(img_show, cv::Point(b3_pointx, b3_pointy), 5, closest3.color, -1);


                        float by_diff_0_3 = std::abs(b0_pointy - b3_pointy);
                        float by_diff_1_2 = std::abs(b1_pointy - b2_pointy);

                        std::cout << "bbb height error is -------------------------------------------->  : " << by_diff_0_3 - by_diff_1_2 << std::endl;
                        if(std::abs(by_diff_0_3 - by_diff_1_2) < 45)
                        {
                            rovAutoControlState = 6;
                            stateEntryTime = std::chrono::steady_clock::now();

                        }
                        // std::cout << "Closest object class: " << closest.className << " at distance: " << euclideanDistance(targetPoint, cv::Point(closest.box.x + closest.box.width / 2, closest.box.y + closest.box.height / 2)) << std::endl;


                        set_yaw_vel =  0 - 0.015*(by_diff_0_3 - by_diff_1_2);
                        if(set_yaw_vel > 80) set_yaw_vel = 80;
                        if(set_yaw_vel < -80) set_yaw_vel = -80;


                        // right_speed_set = 


                    } 
                    catch (const std::exception& e) 
                    {
                        std::cerr << "Error: " << e.what() << std::endl;
                    }

                }


                for (const auto& keypoint_item : keypoints) 
                {
                 
                    std::string draw_string = std::to_string(keypoint_item.id);
                    cv::putText(img_show, draw_string, cv::Point(keypoint_item.x, keypoint_item.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            (255,0,0), 2);

                    
                    cv::circle(img_show, cv::Point(keypoint_item.x, keypoint_item.y), 5, (255,0,0), -1);

                   
                }



            } 
            catch (const std::exception& e) 
            {
                std::cerr << "Error: " << e.what() << std::endl;
            }

            // std::cout << "height error  is --------------------> " << (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0)) << std::endl;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = set_yaw_vel/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = set_depth_vel / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = right_speed_set / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] =  (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

 
        }break;

        case 6:
        {





            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, "rectangle");
            // std::cout << "max rectangle height is --------------------> " << rectangle_result.height << std::endl;
            int forward_speed_set = 1 * (480 - rectangle_result.height);
            if(forward_speed_set > 1000) forward_speed_set = 1000;
            if(forward_speed_set < -1000) forward_speed_set = -1000;


            int right_speed_set = 0.55*(rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0);


            std::cout << "widht error is ----------------> " <<rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0 << std::endl;
            if(right_speed_set > 1000) right_speed_set = 1000;
            if(right_speed_set < -1000) right_speed_set = -1000;
             
            float set_yaw_vel = 0;
        
            
            float set_depth_vel = 0.08 * (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0));
            if(set_depth_vel > 30) set_depth_vel = 30;
            if(set_depth_vel < -30) set_depth_vel = -30;

            if(rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0 < 60)
            {
                rovAutoControlState = 7;
                stateEntryTime = std::chrono::steady_clock::now();
            }


            // std::cout << "height error  is --------------------> " << (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0)) << std::endl;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = set_yaw_vel/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = set_depth_vel / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = right_speed_set / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] =  (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

        }break;

        case 7:
        {

            cv::Rect rectangle_result = getMaxHeightObject(objects_v5, "rectangle");
            // std::cout << "max rectangle height is --------------------> " << rectangle_result.height << std::endl;
            int forward_speed_set = 250;
   

            int right_speed_set = 0.55*(rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0);


            std::cout << "widht error is ----------------> " <<rectangle_result.x + rectangle_result.width / 2.0 - img.cols/2.0 << std::endl;
            if(right_speed_set > 1000) right_speed_set = 1000;
            if(right_speed_set < -1000) right_speed_set = -1000;
             
            float set_yaw_vel = 0;
        
            
            float set_depth_vel = 0.08 * (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0));
            if(set_depth_vel > 30) set_depth_vel = 30;
            if(set_depth_vel < -30) set_depth_vel = -30;

            if(rectangle_result.height > 600)
            {
                rovAutoControlState = 8;
                stateEntryTime = std::chrono::steady_clock::now();
            }


            // std::cout << "height error  is --------------------> " << (img.rows/2.0 - (rectangle_result.y + rectangle_result.height / 2.0)) << std::endl;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = set_yaw_vel/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = set_depth_vel / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = right_speed_set / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] =  (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

        }break;


        case 8:
        {

            int forward_speed_set = 800;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] =  (forward_speed_set + ZERO_FOREARD_THR) / 1000.0 * 520; //forward
            // GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode
            auto currentTime = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= 2) 
            {
                pass_rectangle_num ++;
                if(pass_rectangle_num == rectangle_num)
                {
                    rovAutoControlState = 30;
                }
                else
                {
                    rovAutoControlState = 2;
                }
                
                
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

