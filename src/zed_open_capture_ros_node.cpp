// zed_open_capture_node.cpp

#include <string>
#include <thread>
#include <atomic>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <zed_open_capture_ros/ZedOpenCaptureConfig.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

#include <zed-open-capture/sensorcapture.hpp>
#include <zed-open-capture/videocapture.hpp>

std::unique_ptr<sl_oc::video::VideoCapture> video_capture;
sl_oc::sensors::SensorCapture sensor_capture;
sl_oc::video::VideoParams video_params;

std::atomic<bool> running(true);

diagnostic_updater::Updater *diagnostic_updater_ptr;
int imu_count = 0;
int image_count = 0;

// Dynamic reconfigure callback
void reconfigCallback(zed_open_capture_ros::ZedOpenCaptureConfig &config, uint32_t level) {
    video_params.res = static_cast<sl_oc::video::RESOLUTION>(config.resolution);
    video_params.fps = static_cast<sl_oc::video::FPS>(config.fps);
    video_params.verbose = config.verbose;

    ROS_INFO("Reconfigure request: Resolution=%d, FPS=%d, Verbose=%s", config.resolution, config.fps, config.verbose ? "true" : "false");

    video_capture = std::make_unique<sl_oc::video::VideoCapture>(video_params);
    bool success = video_capture->initializeVideo(-1);
    ROS_INFO("Video re-initialization success: %s", success ? "true" : "false");
}

void imuPublisher(ros::Publisher &imu_pub) {
    while (running) {
        auto imu_data = sensor_capture.getLastIMUData(10000);
        if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "zed_imu";

            imu_msg.linear_acceleration.x = imu_data.aX;
            imu_msg.linear_acceleration.y = imu_data.aY;
            imu_msg.linear_acceleration.z = imu_data.aZ;
            imu_msg.angular_velocity.x = imu_data.gX;
            imu_msg.angular_velocity.y = imu_data.gY;
            imu_msg.angular_velocity.z = imu_data.gZ;
            imu_pub.publish(imu_msg);

            imu_count++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void publishStaticTransforms() {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;

    geometry_msgs::TransformStamped left_tf;
    left_tf.header.stamp = ros::Time::now();
    left_tf.header.frame_id = "zed_base";
    left_tf.child_frame_id = "zed_left";
    left_tf.transform.translation.x = 0.0;
    left_tf.transform.translation.y = 0.0;
    left_tf.transform.translation.z = 0.0;
    left_tf.transform.rotation.w = 1.0;
    transforms.push_back(left_tf);

    geometry_msgs::TransformStamped right_tf = left_tf;
    right_tf.child_frame_id = "zed_right";
    right_tf.transform.translation.y = 0.06;
    transforms.push_back(right_tf);

    geometry_msgs::TransformStamped imu_tf = left_tf;
    imu_tf.child_frame_id = "zed_imu";
    imu_tf.transform.translation.z = 0.02;
    transforms.push_back(imu_tf);

    static_broadcaster.sendTransform(transforms);
}

void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("IMU messages published", imu_count);
    stat.add("Image messages published", image_count);
    imu_count = 0;
    image_count = 0;
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "ZED Open Capture node running");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_open_capture_ros_node");
    ros::NodeHandle nh;

    ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("/zed/left/image_raw", 1);
    ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("/zed/right/image_raw", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/zed/imu/data_raw", 1);

    camera_info_manager::CameraInfoManager left_info_mgr(nh, "zed_left");
    camera_info_manager::CameraInfoManager right_info_mgr(nh, "zed_right");

    int res, fps;
    nh.param("resolution", res, 2);
    nh.param("fps", fps, 1);

    video_params.res = static_cast<sl_oc::video::RESOLUTION>(res);
    video_params.fps = static_cast<sl_oc::video::FPS>(fps);
    video_params.verbose = false;

    if (!sensor_capture.initializeSensors()) {
        ROS_ERROR("Failed to initialize sensor capture");
        return 1;
    }

    video_capture = std::make_unique<sl_oc::video::VideoCapture>(video_params);
    if (!video_capture->initializeVideo(-1)) {
        ROS_ERROR("Failed to initialize video capture");
        return 1;
    }

    publishStaticTransforms();

    std::thread imu_thread(imuPublisher, std::ref(imu_pub));

    dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig> server;
    dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig>::CallbackType f;
    f = boost::bind(&reconfigCallback, _1, _2);
    server.setCallback(f);

    diagnostic_updater::Updater updater;
    updater.setHardwareID("zed_open_capture_node");
    updater.add("ZED Diagnostic", diagnosticsCallback);
    diagnostic_updater_ptr = &updater;

    ros::Rate loop_rate(60);

    while (ros::ok()) {
        auto frame = video_capture->getLastFrame(1000);
        if (frame.data) {
            cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
            cv::Mat frameBGR;
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            cv::Mat left_rgb = frameBGR(cv::Rect(0, 0, frame.width / 2, frame.height));
            cv::Mat right_rgb = frameBGR(cv::Rect(frame.width / 2, 0, frame.width / 2, frame.height));

            ros::Time stamp = ros::Time::now();
            sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_rgb).toImageMsg();
            sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_rgb).toImageMsg();

            left_msg->header.stamp = stamp;
            right_msg->header.stamp = stamp;
            left_msg->header.frame_id = "zed_left";
            right_msg->header.frame_id = "zed_right";

            left_pub.publish(left_msg);
            right_pub.publish(right_msg);

            image_count += 2;
        }

        updater.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    running = false;
    imu_thread.join();

    return 0;
}