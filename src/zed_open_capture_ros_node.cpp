#include <string>
#include <thread>
#include <atomic>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <zed_open_capture_ros/ZedOpenCaptureConfig.h>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

#include <zed-open-capture/sensorcapture.hpp>
#include <zed-open-capture/videocapture.hpp>

sl_oc::video::VideoCapture video_capture;
sl_oc::sensors::SensorCapture sensor_capture;
sl_oc::video::VideoParams video_params;

std::atomic<bool> running(true);

// Dynamic reconfigure callback
void reconfigCallback(zed_open_capture_ros::ZedOpenCaptureConfig &config, uint32_t level) {
    video_params.res = static_cast<sl_oc::video::RESOLUTION>(config.resolution);
    video_params.fps = static_cast<sl_oc::video::FPS>(config.fps);
    video_params.verbose = config.verbose;

    ROS_INFO("Reconfigure request: Resolution=%d, FPS=%d, Verbose=%s", config.resolution, config.fps, config.verbose ? "true" : "false");

    video_capture.initializeVideo();
}

void imuPublisher(ros::Publisher &imu_pub) {
    while (running) {
        auto imu_data = sensor_capture.getLastIMUData(10000);
        if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.linear_acceleration.x = imu_data.aX;
            imu_msg.linear_acceleration.y = imu_data.aY;
            imu_msg.linear_acceleration.z = imu_data.aZ;
            imu_msg.angular_velocity.x = imu_data.gX;
            imu_msg.angular_velocity.y = imu_data.gY;
            imu_msg.angular_velocity.z = imu_data.gZ;
            imu_pub.publish(imu_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_open_capture_ros_node");
    ros::NodeHandle nh;

    ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("/zed/left/image_raw", 1);
    ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("/zed/right/image_raw", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/zed/imu/data_raw", 1);

    if (!sensor_capture.initializeSensors()) {
        ROS_ERROR("Failed to initialize sensor capture");
        return 1;
    }

    std::thread imu_thread(imuPublisher, std::ref(imu_pub));

    dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig> server;
    dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig>::CallbackType f;
    f = boost::bind(&reconfigCallback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(60);

    while (ros::ok()) {
        auto frame = video_capture.getLastFrame(1000);
        if (frame.data) {
            cv::Mat left_rgb, right_rgb;
            cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
            cv::Mat frameBGR(frame.height, frame.width, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            cv::Mat left_image, right_image;
            left_rgb = frameBGR(cv::Rect(0, 0, frame.width / 2, frame.height));
            right_rgb = frameBGR(cv::Rect(frame.width / 2, 0, frame.width / 2, frame.height));

            sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_rgb).toImageMsg();
            sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_rgb).toImageMsg();

            left_msg->header.stamp = ros::Time::now();
            right_msg->header.stamp = ros::Time::now();

            left_pub.publish(left_msg);
            right_pub.publish(right_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    running = false;
    imu_thread.join();

    return 0;
}
