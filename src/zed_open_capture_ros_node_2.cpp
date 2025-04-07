#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <memory> // Required for std::unique_ptr
#include <stdexcept> // For exceptions
#include <cmath> // For std::round

// Assuming these headers exist and define the necessary types/enums
#include <zed-open-capture/videocapture.hpp>
#include <zed-open-capture/sensorcapture.hpp>

// Define sensor availability (though not used in this refactored version,
// could be used for conditional compilation if desired)
#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

namespace zed_ros_wrapper {

// Helper struct to hold camera configuration
struct CameraConfig {
    sl_oc::video::RESOLUTION resolution = sl_oc::video::RESOLUTION::HD1080;
    sl_oc::video::FPS fps_enum = sl_oc::video::FPS::FPS_30;
    double target_fps = 30.0; // Actual numeric FPS target
    sl_oc::VERBOSITY verbosity = sl_oc::VERBOSITY::INFO;
};

/**
 * @brief Manages the connection and data retrieval from the ZED camera hardware
 * using the zed-open-capture library. Does not interact with ROS directly.
 */
class StereoCamera {
public:
    StereoCamera() = default; // Constructor doesn't do heavy lifting

    // Prevent copying
    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;

    ~StereoCamera() {
        // unique_ptr handles deletion automatically.
        // Explicitly call deinitialization if the library requires it.
        // Note: The original library might not have explicit deinit,
        // relying on object destruction. Check library docs if unsure.
        if (video_capture_) {
            // video_capture_->close(); // If such a method exists
        }
        if (sensor_capture_) {
            // sensor_capture_->close(); // If such a method exists
        }
        ROS_INFO("StereoCamera resources released.");
    }

    /**
     * @brief Initializes video and sensor capture based on configuration.
     * @param config Camera configuration parameters.
     * @return True if initialization was successful, false otherwise.
     */
    bool initialize(const CameraConfig& config) {
        sl_oc::video::VideoParams video_params;
        video_params.res = config.resolution;
        video_params.fps = config.fps_enum;
        video_params.verbose = config.verbosity;

        ROS_INFO("Initializing ZED video capture...");
        video_capture_ = std::make_unique<sl_oc::video::VideoCapture>(video_params);
        if (!video_capture_ || !video_capture_->initializeVideo(-1)) {
            ROS_ERROR("Failed to initialize ZED video capture.");
            video_capture_.reset(); // Ensure pointer is null if failed
            return false;
        }
        camera_sn_ = video_capture_->getSerialNumber();
        ROS_INFO("Video Capture connected to ZED camera SN: %d", camera_sn_);

        ROS_INFO("Initializing ZED sensor capture...");
        sensor_capture_ = std::make_unique<sl_oc::sensors::SensorCapture>(config.verbosity);
        // Use the serial number acquired by VideoCapture
        if (!sensor_capture_ || !sensor_capture_->initializeSensors(camera_sn_)) {
            ROS_WARN("Failed to initialize ZED sensor capture. Continuing without sensors.");
            sensor_capture_.reset(); // Allow operation without sensors if video works
        } else {
            ROS_INFO("Sensor Capture initialized for ZED camera SN: %d", camera_sn_);
            // Enable sensor synchronization if both are available
            video_capture_->enableSensorSync(sensor_capture_.get());
            ROS_INFO("Sensor synchronization enabled.");
        }

        return true; // Video initialized successfully (sensor optional)
    }

    /**
     * @brief Retrieves the latest video frame.
     * @param num_attempts Number of times to try fetching before returning empty frame.
     * @return The latest sl_oc::video::Frame. frame.data will be nullptr if no new frame.
     */
    sl_oc::video::Frame getLastFrame(int num_attempts = 1) const {
        if (!video_capture_) {
            return {}; // Return empty frame
        }
        return video_capture_->getLastFrame(num_attempts);
    }

    /**
     * @brief Retrieves the latest IMU data.
     * @return The latest sl_oc::sensors::data::Imu. imu.valid may not be NEW_VAL.
     */
    sl_oc::sensors::data::Imu getLastIMUData() const {
        if (!sensor_capture_) {
            return {}; // Return empty data
        }
        // Sensor acquisition runs at its own rate, check validity in the caller
        return sensor_capture_->getLastIMUData();
    }

    // --- Getters for camera state ---
    bool isVideoInitialized() const { return video_capture_ != nullptr; }
    bool isSensorInitialized() const { return sensor_capture_ != nullptr; }
    int getSerialNumber() const { return camera_sn_; }
    // !TODO: Add methods to get actual sensor rate (e.g., 400 Hz) if the library provides it.
    double getSensorRateHz() const { return 400.0; } // Placeholder, get from library if possible


private:
    std::unique_ptr<sl_oc::video::VideoCapture> video_capture_;
    std::unique_ptr<sl_oc::sensors::SensorCapture> sensor_capture_;
    int camera_sn_ = -1;
};


/**
 * @brief ROS Node class to manage interaction between StereoCamera and ROS.
 */
class ZedRosNode {
public:
    ZedRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          image_transport_(nh), // Initialize ImageTransport
          last_frame_timestamp_ns_(0)
    {
        ROS_INFO("Initializing ZedRosNode...");
    }

    ~ZedRosNode() = default; // unique_ptr for zed_camera_ handles cleanup

    /**
     * @brief Initializes ROS parameters, camera connection, publishers, and timers.
     * @return True on success, false on failure.
     */
    bool init() {
        if (!loadParameters()) {
            return false;
        }

        zed_camera_ = std::make_unique<StereoCamera>();
        if (!zed_camera_->initialize(camera_config_)) {
            ROS_FATAL("Failed to initialize StereoCamera hardware interface.");
            return false; // Critical failure if video doesn't start
        }

        setupPublishers();
        setupTimers();

        ROS_INFO("ZedRosNode initialized successfully.");
        return true;
    }

private:

    /**
     * @brief Loads parameters from the ROS parameter server.
     * @return True on success, false if configuration is invalid.
     */
    bool loadParameters() {
        ROS_INFO("Loading ROS parameters...");
        int resolution_param = 1; // Default: HD1080
        bool verbose_param = false;

        private_nh_.param("resolution", resolution_param, 1);
        private_nh_.param("verbose", verbose_param, false);
        private_nh_.param<std::string>("left_frame_id", left_frame_id_, "zed_left_camera_frame");
        private_nh_.param<std::string>("right_frame_id", right_frame_id_, "zed_right_camera_frame");
        private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "zed_imu_link"); // More conventional IMU frame

        if (!configureCameraParams(resolution_param)) {
            return false;
        }
        camera_config_.verbosity = verbose_param ? sl_oc::VERBOSITY::INFO : sl_oc::VERBOSITY::OFF;

        ROS_INFO("Parameters Loaded:");
        ROS_INFO("  Resolution: %d", static_cast<int>(camera_config_.resolution));
        ROS_INFO("  FPS: %.2f", camera_config_.target_fps);
        ROS_INFO("  Verbosity: %d", static_cast<int>(camera_config_.verbosity));
        ROS_INFO("  Left Frame ID: %s", left_frame_id_.c_str());
        ROS_INFO("  Right Frame ID: %s", right_frame_id_.c_str());
        ROS_INFO("  IMU Frame ID: %s", imu_frame_id_.c_str());

        return true;
    }

    /**
     * @brief Sets camera resolution and FPS enums based on the integer parameter.
     * @param resolution_param Integer specifying the desired mode.
     * @return True if parameter is valid, false otherwise.
     */
    bool configureCameraParams(int resolution_param) {
         switch (resolution_param) {
            case 0: // HD2K @ 15
                camera_config_.resolution = sl_oc::video::RESOLUTION::HD2K;
                camera_config_.fps_enum = sl_oc::video::FPS::FPS_15;
                camera_config_.target_fps = 15.0;
                break;
            case 1: // HD1080 @ 30
                camera_config_.resolution = sl_oc::video::RESOLUTION::HD1080;
                camera_config_.fps_enum = sl_oc::video::FPS::FPS_30;
                camera_config_.target_fps = 30.0;
                break;
            case 2: // HD720 @ 60
                camera_config_.resolution = sl_oc::video::RESOLUTION::HD720;
                camera_config_.fps_enum = sl_oc::video::FPS::FPS_60;
                camera_config_.target_fps = 60.0;
                break;
            case 3: // VGA @ 100
                camera_config_.resolution = sl_oc::video::RESOLUTION::VGA;
                camera_config_.fps_enum = sl_oc::video::FPS::FPS_100;
                camera_config_.target_fps = 100.0;
                break;
            default:
                ROS_FATAL("Unknown resolution parameter value: %d", resolution_param);
                return false;
         }
         return true;
    }

    /**
     * @brief Sets up ROS publishers.
     */
    void setupPublishers() {
        ROS_INFO("Setting up ROS publishers...");
        // Queue size 10, latched topics can sometimes be useful for static info but not needed here.
        left_image_pub_ = image_transport_.advertise("left/image_raw", 10);
        right_image_pub_ = image_transport_.advertise("right/image_raw", 10);

        if (zed_camera_->isSensorInitialized()) {
            imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 100); // Standard topic name is often data_raw
        }
    }

    /**
     * @brief Sets up ROS timers for polling camera data.
     */
    void setupTimers() {
        ROS_INFO("Setting up ROS timers...");
        // Timer for images: Poll slightly faster than the target frame rate
        // to reduce latency, but avoid excessive polling. Factor of 1.5-2.0 is common.
        double image_poll_rate = camera_config_.target_fps * 1.5;
        double image_timer_duration = 1.0 / image_poll_rate;
        ROS_INFO("Setting image timer poll rate: %.2f Hz (Duration: %.4f s)", image_poll_rate, image_timer_duration);
        image_timer_ = nh_.createTimer(ros::Duration(image_timer_duration), &ZedRosNode::imageCallback, this);


        if (zed_camera_->isSensorInitialized()) {
            // Timer for IMU: Poll slightly faster than the expected sensor rate (e.g., 400 Hz)
            double sensor_native_rate = zed_camera_->getSensorRateHz(); // Get from camera if possible
            double sensor_poll_rate = sensor_native_rate * 1.5; // Poll ~600Hz for 400Hz sensor
            double sensor_timer_duration = 1.0 / sensor_poll_rate;

            // Avoid excessively high rates if sensor rate is unknown or very high
             if (sensor_timer_duration < 0.001) { // Limit polling to max 1000 Hz
                 sensor_timer_duration = 0.001;
                 ROS_WARN("Calculated sensor poll rate too high, capping at 1000 Hz.");
             }

            ROS_INFO("Setting sensor timer poll rate: %.2f Hz (Duration: %.4f s)", 1.0 / sensor_timer_duration, sensor_timer_duration);
            sensor_timer_ = nh_.createTimer(ros::Duration(sensor_timer_duration), &ZedRosNode::sensorCallback, this);
        }
    }

    /**
     * @brief ROS Timer callback to process and publish images.
     */
    void imageCallback(const ros::TimerEvent& /*event*/) {
        if (!zed_camera_->isVideoInitialized()) return;

        const sl_oc::video::Frame frame = zed_camera_->getLastFrame(1);

        // Check if frame is valid and *new*
        if (frame.data == nullptr || frame.timestamp <= last_frame_timestamp_ns_) {
            return; // No new data or invalid frame
        }
        last_frame_timestamp_ns_ = frame.timestamp;

        // Assuming YUV2 format (YUYV) based on original code
        // Check frame.data_format if available in sl_oc::video::Frame to be sure
        cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR; // Allocate destination matrix

        try {
             cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
        } catch (const cv::Exception& e) {
            ROS_ERROR("OpenCV exception during YUV->BGR conversion: %s", e.what());
            return;
        }


        // Split into left and right images (assuming side-by-side format)
        // Use cv::Rect more safely by checking dimensions
        if (frameBGR.cols < frame.width || frameBGR.rows < frame.height) {
             ROS_ERROR("BGR frame dimensions mismatch! Expected %dx%d, Got %dx%d",
                       frame.width, frame.height, frameBGR.cols, frameBGR.rows);
             return;
        }
        int half_width = frame.width / 2;
        cv::Mat left_image = frameBGR(cv::Rect(0, 0, half_width, frame.height));
        cv::Mat right_image = frameBGR(cv::Rect(half_width, 0, half_width, frame.height));

        // Create ROS timestamp from nanoseconds
        // Note: Assumes camera timestamp is based on ROS time epoch (or close enough)
        ros::Time timestamp_ros = ros::Time().fromNSec(frame.timestamp);

        // Publish Left Image
        publishImage(left_image, left_image_pub_, left_frame_id_, timestamp_ros);

        // Publish Right Image
        publishImage(right_image, right_image_pub_, right_frame_id_, timestamp_ros);
    }

     /**
     * @brief Helper to publish an image message.
     */
    void publishImage(const cv::Mat& img, image_transport::Publisher& pub, const std::string& frame_id, const ros::Time& stamp) {
        if (pub.getNumSubscribers() == 0) return; // Don't do work if no one is listening

        try {
            // Use cv_bridge to convert cv::Mat to sensor_msgs::Image
            std_msgs::Header header;
            header.stamp = stamp;
            header.frame_id = frame_id;
            cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, img);

            sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();
            pub.publish(img_msg);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR("Std exception during image publishing: %s", e.what());
        }
    }


    /**
     * @brief ROS Timer callback to process and publish IMU data.
     */
    void sensorCallback(const ros::TimerEvent& /*event*/) {
        if (!zed_camera_->isSensorInitialized() || imu_pub_.getNumSubscribers() == 0) {
            return; // No sensor or no subscribers
        }

        const sl_oc::sensors::data::Imu imuData = zed_camera_->getLastIMUData();

        // Check if the data is valid and new (according to library's definition)
        // The original code used NEW_VAL, assuming this means synchronized/valid new data.
        if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
            sensor_msgs::Imu imu_msg;

            // Assuming timestamp is nanoseconds, create ROS time
            imu_msg.header.stamp = ros::Time().fromNSec(imuData.timestamp);
            imu_msg.header.frame_id = imu_frame_id_; // Use parameterized frame_id

            // Populate IMU data - Check units expected by ROS (rad/s, m/s^2)
            // Assuming library provides data in correct units. Add conversions if needed.
            imu_msg.angular_velocity.x = imuData.gX;
            imu_msg.angular_velocity.y = imuData.gY;
            imu_msg.angular_velocity.z = imuData.gZ;

            imu_msg.linear_acceleration.x = imuData.aX;
            imu_msg.linear_acceleration.y = imuData.aY;
            imu_msg.linear_acceleration.z = imuData.aZ;

            // Orientation is not provided by this sensor data structure in the snippet
            // Set covariance matrices to unknown (all zeros, first element -1)
            // Or provide fixed covariances if you have estimates.
            imu_msg.orientation_covariance[0] = -1.0;
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;
            // !TODO: Populate covariances if known estimates are available.

            imu_pub_.publish(imu_msg);
        }
    }


    // ROS Members
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    ros::Publisher imu_pub_;
    ros::Timer image_timer_;
    ros::Timer sensor_timer_;

    // Camera Interface
    std::unique_ptr<StereoCamera> zed_camera_;
    CameraConfig camera_config_;

    // Parameters
    std::string left_frame_id_;
    std::string right_frame_id_;
    std::string imu_frame_id_;

    // State
    uint64_t last_frame_timestamp_ns_; // Use unsigned 64-bit for nanosecond timestamps

}; // class ZedRosNode

} // namespace zed_ros_wrapper


// --- Main Function ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_ros_node"); // Node name

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // Private node handle for parameters

    ROS_INFO("Starting ZED ROS Node...");

    zed_ros_wrapper::ZedRosNode node(nh, private_nh);

    if (!node.init()) {
        ROS_FATAL("Failed to initialize ZED ROS Node. Shutting down.");
        return 1; // Indicate failure
    }

    ROS_INFO("ZED ROS Node running.");
    ros::spin(); // Process callbacks

    ROS_INFO("ZED ROS Node shutting down.");
    return 0; // Indicate success
}
