#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include "eco.hpp"   // 你的ECO跟踪器头文件，根据实际路径调整

int main(int argc, char **argv)
{
//1. 初始化 ROS 节点与发布器
    ros::init(argc, argv, "eco_tracker_node");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher heartbeat_pub = nh.advertise<std_msgs::Header>("/user_heartbeat", 10);


    // GStreamer管道，用你的设备和参数替换
    char pipeline_desc_up[1024];
    snprintf(pipeline_desc_up, sizeof(pipeline_desc_up),
            "v4l2src device=/dev/video0 ! image/jpeg, width=640, height=480, framerate=30/1 ! jpegdec ! videoconvert ! appsink");

 //2. 打开摄像头（用 GStreamer 管道）
    cv::VideoCapture cap("v4l2src device=/dev/video0 ! videoconvert ! appsink", cv::CAP_GSTREAMER);
    if (!cap.isOpened()) 
    {
        ROS_ERROR("Cannot open camera");
        return -1;
    }


//3. 选择初始目标区域 ROI
    cv::Mat frame;
    // 读取第一帧
    cap >> frame;
    if (frame.empty())
    {
        ROS_ERROR("Failed to read frame from camera");
        return -1;
    }
    // 选择ROI
    cv::Rect2f roi = cv::selectROI("Video", frame);
    cv::destroyAllWindows();

    // 初始化ECO跟踪器
    eco::ECO *tracker = new eco::ECO();
    eco::EcoParameters parameters;
    tracker->init(frame, roi, parameters);

    ros::Rate loop_rate(30);

    bool reinitializeTracker = false;

//5. 主循环：目标跟踪 + 控制 + 显示
    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty()) break;

        if (reinitializeTracker)
        {
            roi = cv::selectROI("Video", frame);
            cv::destroyAllWindows();
            if (roi.width == 0 || roi.height == 0)
            {
                ROS_WARN("Invalid ROI selected, continue tracking with old ROI");
            }
            else
            {
                tracker->init(frame, roi, parameters);
            }
            reinitializeTracker = false;
        }


//6. 计算偏差并发布 /cmd_vel——ECO 跟踪器已经输出了目标的 roi（矩形框），你现在要根据这个框的位置来生成控制命令：
// 目标中心是跟踪目标中心  图像中心是整个识别区域中心 
        tracker->update(frame, roi);

        float centerX = roi.x + roi.width / 2;
        float centerY = roi.y + roi.height / 2;
//这两个误差是目标在图像中偏离中心的位置
        float error_x = centerX - frame.cols / 2;   // 横向偏差
        float error_y = frame.rows / 2 - centerY;   // 纵向偏差（深度）

        float kp_linear = 0.001f;
        float kp_angular = 0.003f;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = kp_linear * (320-roi.height)*3;
        cmd_vel.linear.z = kp_linear * error_y * 6;
        cmd_vel.angular.z = kp_angular * error_x;

        vel_pub.publish(cmd_vel);


        std_msgs::Header hb_msg;
        hb_msg.stamp = ros::Time::now();
        heartbeat_pub.publish(hb_msg);

        // 显示跟踪框
        cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);
        cv::imshow("Video", frame);

        char key = (char)cv::waitKey(1);
        if (key == 's' || key == 'S')
            reinitializeTracker = true;
        else if (key == 27)  // ESC退出
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete tracker;
    cap.release();
    cv::destroyAllWindows();

    return 0;
}

