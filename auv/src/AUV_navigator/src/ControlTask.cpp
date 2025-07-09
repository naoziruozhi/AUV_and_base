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
 


 

// // void controlTask(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm)    
// // {

// //     auto next = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);  

// //     static int rovAutoControlState = 0;
// //     static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
// //     std::this_thread::sleep_for(std::chrono::seconds(3));



// //     // model file path
// //     const char *model_file = "../demo/demo4/yolov5_letterbox/weights/best_int4.rknn";
 
// //     // 参数：是否录像、绘制文字
// //     const bool record = false;


// //     // 打开摄像头
// //     cv::VideoCapture cap("/dev/video4"); // 0 表示默认摄像头
// //     if(!cap.isOpened()) 
// //     {
// //         std::cerr << "Error opening video stream" << std::endl;
// //     }
// //     // 获取视频尺寸、帧率
// //     int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
// //     int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
// //     int fps = cap.get(cv::CAP_PROP_FPS);

// //     // 初始化
// //     Yolov5 yolo;
// //     // 加载模型
// //     yolo.LoadModel(model_file);
// //     // 视频帧
// //     cv::Mat img;
// //     cv::namedWindow("result", cv::WINDOW_AUTOSIZE); 
// //     while (true) 
// //     {
// //         // 读取视频帧
// //         auto time_1 = std::chrono::high_resolution_clock::now();
// //         cap >> img;
// //         auto time_2 = std::chrono::high_resolution_clock::now();
// //         if (img.empty())
// //         {
// //             std::cout << "Video end." << std::endl;
// //             break;
// //         }

// //         // 检测结果
// //         std::vector<Detection> objects;
        
// //         // 运行模型
// //         yolo.Run(img, objects);
// //         // 绘制框，显示结果
// //         DrawDetections(img, objects);
// //         cv::imshow("result", img);
// //         auto time_3 = std::chrono::high_resolution_clock::now();
// //         auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_2 - time_1).count();
// //         auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(time_3 - time_2).count();
// //         // auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(time_4 - time_3).count();
// //         std::cout << "duration1 is -----------> " << duration1 << std::endl;
// //         std::cout << "duration2 is -----------> " << duration2 << std::endl;
// //         cv::waitKey(1);

// //     }

 
// //     cap.release();
 
// // }










#define UP_CAM  "/dev/video0"
#define DOWN_CAM  "/dev/video4"


int record_land_depth = 0;
int record_land_depth_all = 0;
int record_cnt = 0;

uint8_t task0Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm) 
{
    static int rovAutoControlState = -1;
    static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
    // std::cout << "getDepth(MCUComm.RovState) is --------->  " << getDepth(MCUComm.RovState) << std::endl;
    switch(rovAutoControlState)
    {
        case -1:
        {
            
            record_land_depth = getDepth(MCUComm.RovState);
            rovAutoControlState = 0;
            stateEntryTime = std::chrono::steady_clock::now();

        }break;
        case 0:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 0;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;



            auto currentTime = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= 15) //计时超过2s 进入下一个状态
            {
                rovAutoControlState = 1;
                stateEntryTime = std::chrono::steady_clock::now();
            }


        }break;

        case 1:
        {

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 2/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode
            

            auto currentTime = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - stateEntryTime).count() >= 0.1) //计时超过2s 进入下一个状态
            {
                rovAutoControlState = 2;
                stateEntryTime = std::chrono::steady_clock::now();
            }
            
            // rovAutoControlState = 2;
            // stateEntryTime = std::chrono::steady_clock::now();
        }break;

        case 2:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0/ 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            rovAutoControlState = 3;
            stateEntryTime = std::chrono::steady_clock::now();
        }break;

        default:
        {
            ;
        }break;

 

    }

    if(rovAutoControlState == 3)
    {
        return 1;
    }
    else
    {
        return 0;
    }

     
}









uint8_t task1Control(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm) 
{
    static int rovAutoControlState = 0;
    static auto stateEntryTime = std::chrono::steady_clock::now(); // Time when a state is entered
    // std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaa" << std::endl;
    std::cout << "rovAutoControlState is ---------------> " << rovAutoControlState << std::endl;
    switch(rovAutoControlState)
    {
        case 0:
        {

            
        
            int depthSet = -40;     

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0; //yaw vel
            // std::cout << "GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] is --------------> " << GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] <<endl;
            std::cout << "depth is ------------------> " << (getDepth(MCUComm.RovState) - record_land_depth) << std::endl;
            int depth_vel_set = 0;
            if(depthSet > (getDepth(MCUComm.RovState) - record_land_depth))
            {
                depth_vel_set = 10;

            }

            else if(depthSet < (getDepth(MCUComm.RovState) - record_land_depth))
            {
                depth_vel_set = -10;
            }

            else
            {

                depth_vel_set = 0;
            }

            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0 / 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = depth_vel_set / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode
          
                
            if(abs((getDepth(MCUComm.RovState) - record_land_depth) - depthSet) < 5) 
            {
                rovAutoControlState = 1;
                GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0; //depth vel
                stateEntryTime = std::chrono::steady_clock::now();
                
            }
        }break;

        default:
        {
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[1] = 0 / 80.0 * 520; //yaw_vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[2] = 0 / 30.0 * 520; //depth vel
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[3] = 0 / 1000.0 * 520; //right
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[4] = ZERO_FOREARD_THR / 1000.0 * 520; //forward
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[5] = 400;
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[6] = 0 / 80.0 * 520;   //roll speed
            GroundStationComm.GroundStationRemote.OriginalRemoteChannel[8] = 0;  // mode

            std::cout << "i am here --------------> " << std::endl;
        }break;

     

  
    }

    if(rovAutoControlState == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }


}



void controlTask(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm)    
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    auto next = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);  
    static int rovTaskState = 0;

    // model file path
    const char *v8model_file = "../demo/demo7/v8_pose/weights/all_data_pose.rknn";

    // 参数：是否录像、绘制文字
    // const bool record = false;
    nn_model_type_e model_type = NN_YOLOV8_POSE;
    Yolov8Custom yolov8(model_type);
    yolov8.LoadModel(v8model_file);



    const char *v5model_file = "../demo/demo4/yolov5_letterbox/weights/all_data_detection.rknn";
    // 初始化
    Yolov5 yolov5;
    // 加载模型
    yolov5.LoadModel(v5model_file);

    // // 打开摄像头
    // cv::VideoCapture cap_up(UP_CAM); // 0 表示默认摄像头

    // cap_up.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap_up.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap_up.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // int fourcc_up = static_cast<int>(cap_up.get(cv::CAP_PROP_FOURCC));
    // char fourcc_str_up[] = {
    //     static_cast<char>(fourcc_up & 0xFF),
    //     static_cast<char>((fourcc_up >> 8) & 0xFF),
    //     static_cast<char>((fourcc_up >> 16) & 0xFF),
    //     static_cast<char>((fourcc_up >> 24) & 0xFF),
    //     '\0'
    // };

    char pipeline_desc_up[1024];

    
   
    snprintf(pipeline_desc_up, sizeof(pipeline_desc_up),
            "v4l2src device=%s ! image/jpeg, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! appsink",
            UP_CAM);



    // std::string pipeline_up = "v4l2src device=/dev/video0 ! image/jpeg, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
    cv::VideoCapture cap_up(pipeline_desc_up, cv::CAP_GSTREAMER);






    if(!cap_up.isOpened()) 
    {
        std::cerr << "Error opening video stream" << std::endl;
    }
    // 获取视频尺寸、帧率
    int width_up = cap_up.get(cv::CAP_PROP_FRAME_WIDTH);
    int height_up = cap_up.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps_up = cap_up.get(cv::CAP_PROP_FPS);
    // std::cout << "Current format: " << fourcc_str_up << std::endl;
    // std::cout << "width_up: " << width_up << std::endl;
    // std::cout << "height_up: " << height_up << std::endl;
    // std::cout << "fps_up: " << fps_up << std::endl;
    // while(1);


    // // 打开摄像头
    // cv::VideoCapture cap_down(DOWN_CAM); // 0 表示默认摄像头

    // cap_down.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap_down.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap_down.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    // int fourcc_down = static_cast<int>(cap_down.get(cv::CAP_PROP_FOURCC));
    // char fourcc_str_down[] = {
    //     static_cast<char>(fourcc_down & 0xFF),
    //     static_cast<char>((fourcc_down >> 8) & 0xFF),
    //     static_cast<char>((fourcc_down >> 16) & 0xFF),
    //     static_cast<char>((fourcc_down >> 24) & 0xFF),
    //     '\0'
    // };

    char pipeline_desc_down[1024];
    snprintf(pipeline_desc_down, sizeof(pipeline_desc_down),
            "v4l2src device=%s ! image/jpeg, width=1280, height=720, framerate=30/1 ! jpegdec ! videoconvert ! appsink",
            DOWN_CAM);
    cv::VideoCapture cap_down(pipeline_desc_down, cv::CAP_GSTREAMER);

    if(!cap_down.isOpened()) 
    {
        std::cerr << "Error opening video stream" << std::endl;
    }
    // 获取视频尺寸、帧率
    int width_down = cap_down.get(cv::CAP_PROP_FRAME_WIDTH);
    int height_down = cap_down.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps_down = cap_down.get(cv::CAP_PROP_FPS);

    // std::cout << "Current format: " << fourcc_str_down << std::endl;
    // std::cout << "width_down: " << width_down << std::endl;
    // std::cout << "height_down: " << height_down << std::endl;
    // std::cout << "fps_down: " << fps_down << std::endl;
    // while(1);
    // // 初始化
    // Yolov5 yolo;
    // // 加载模型
    // yolo.LoadModel(model_file);
    // 视频帧
    // cv::Mat img;
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE); 
    

    while(1)
    {
        std::cout << "rovTaskState is ---------------> " << rovTaskState << std::endl;
        // std::cout << "yaw is --------------> " << getYawAnle(MCUComm.RovState) << std::endl;
        // std::cout << "depth is --------------> " << getDepth(MCUComm.RovState) << std::endl;
        switch(rovTaskState)
        {

            case 0:
            {
                
                uint8_t task0_return = task0Control(GroundStationComm, MCUComm);
                if(task0_return == 1)
                {
                    rovTaskState = 1;
                }
                
            }break;

            case 1:
            {
                uint8_t task1_return = task1Control(GroundStationComm, MCUComm);
                if(task1_return == 1)
                {
                    rovTaskState = 2;
                }
            }break;

            case 2:
            {
                uint8_t task2_return = task2Control(GroundStationComm, MCUComm, cap_up, cap_down, yolov5);
                if(task2_return == 1)
                {
                    rovTaskState = 3;
                }
            }break;

            case 3:
            {
                uint8_t task3_return = task3Control(GroundStationComm, MCUComm, cap_up, cap_down, yolov8, yolov5);
                if(task3_return == 1)
                {
                    rovTaskState = 4;
                }
            }break;

            case 4:
            {
                uint8_t task4_return = task4Control(GroundStationComm, MCUComm, cap_up, cap_down, yolov8, yolov5);
                if(task4_return == 1)
                {
                    rovTaskState = 5;
                }
            }break;
            default:
            {
                

            }break;


        }

        std::this_thread::sleep_until(next);
        next += std::chrono::milliseconds(1);  
    }
        
  

}





// void controlTask(CommWithGroundStation& GroundStationComm, CommWithMCU& MCUComm)    
// {
//     std::this_thread::sleep_for(std::chrono::seconds(3));
//     auto next = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);  
//     static int rovTaskState = 0;

//     // model file path
//     const char *model_file = "../demo/demo7/v8_pose/weights/v8pose_m_int.rknn";

//     // 参数：是否录像、绘制文字
//     // const bool record = false;
//     nn_model_type_e model_type = NN_YOLOV8_POSE;
//     Yolov8Custom yolo(model_type);
//     yolo.LoadModel(model_file);
//     // 检测框
//     std::vector<Detection> objects;
//     // 关键点
//     std::vector<std::map<int, KeyPoint>> kps;

//     // cv::Mat img = cv::imread("/home/cat/code/small/competition/AutoControl/ROVControl/demo/demo7/v8_pose/image_0090.jpg", cv::IMREAD_COLOR);


//     // 打开摄像头
//     // cv::VideoCapture cap_up("/dev/video0"); // 0 表示默认摄像头
//     // if(!cap_up.isOpened()) 
//     // {
//     //     std::cerr << "Error opening video stream" << std::endl;
//     // }
//     // // 获取视频尺寸、帧率
//     // int width_up = cap_up.get(cv::CAP_PROP_FRAME_WIDTH);
//     // int height_up = cap_up.get(cv::CAP_PROP_FRAME_HEIGHT);
//     // int fps_up = cap_up.get(cv::CAP_PROP_FPS);

//     // // 打开摄像头
//     // // cv::VideoCapture cap_down(DOWN_CAM); // 0 表示默认摄像头
//     // if(!cap_down.isOpened()) 
//     // {
//     //     std::cerr << "Error opening video stream" << std::endl;
//     // }
//     // // 获取视频尺寸、帧率
//     // int width_down = cap_down.get(cv::CAP_PROP_FRAME_WIDTH);
//     // int height_down = cap_down.get(cv::CAP_PROP_FRAME_HEIGHT);
//     // int fps_down = cap_down.get(cv::CAP_PROP_FPS);


//     // // 初始化
//     // Yolov5 yolo;
//     // // 加载模型
//     // yolo.LoadModel(model_file);
//     // // 视频帧
//     // // cv::Mat img;
//     // cv::namedWindow("result", cv::WINDOW_AUTOSIZE); 
    

//     while(1)
//     {
//         cv::Mat img = cv::imread("/home/cat/code/small/competition/AutoControl/ROVControl/demo/demo7/v8_pose/image_0090.jpg", cv::IMREAD_COLOR);
//         auto time_1 = std::chrono::high_resolution_clock::now();
//         std::vector<Detection> objects;
//         // 关键点
//         std::vector<std::map<int, KeyPoint>> kps;
//         yolo.Run(img, objects, kps);
//         auto time_2 = std::chrono::high_resolution_clock::now();
//         std::cout << "Number of objects-------------------->: " << objects.size() << std::endl;

//         std::cout << "Number of kps-------------------->: " << kps.size() << std::endl;
//         for (size_t i = 0; i < objects.size(); ++i) 
//         {
//             const auto& object = objects[i];
//             const auto& keypoints_map = kps[i];

//             cv::rectangle(img, object.box, object.color, 2);
//             std::string draw_string = object.className + " " + std::to_string(object.confidence);
//             cv::putText(img, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                         object.color, 2);



//             for (const auto& keypoint_item : keypoints_map) 
//             {
//             // cv::putText(img, draw_string, cv::Point(object.box.x, object.box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//             //             object.color, 2);
//                 std::string draw_string = std::to_string(keypoint_item.second.id);
//                 cv::putText(img, draw_string, cv::Point(keypoint_item.second.x, keypoint_item.second.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                         object.color, 2);
//                 cv::circle(img, cv::Point(keypoint_item.second.x, keypoint_item.second.y), 5, object.color, -1);
//             }

//         }
//         auto time_3 = std::chrono::high_resolution_clock::now();
//         auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(time_2 - time_1).count();
//         auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(time_3 - time_2).count();
//         std::cout << "duration1 is -----------> " << duration1 << std::endl;
//         std::cout << "duration2 is -----------> " << duration2 << std::endl;
//         cv::imwrite("result2.jpg", img);
//         std::this_thread::sleep_until(next);
//         next += std::chrono::milliseconds(1);  
//     }
        
  

// }

