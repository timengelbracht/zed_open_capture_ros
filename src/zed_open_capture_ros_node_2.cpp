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
#include <mutex> // Required for std::recursive_mutex, std::lock_guard
#include <atomic> // Required for std::atomic_bool

// Dynamic Reconfigure Includes
#include <dynamic_reconfigure/server.h>
// IMPORTANT: Replace with your actual package name if different
#include <zed_open_capture_ros/ZedOpenCaptureConfig.h>

// Assuming these headers exist and define the necessary types/enums
#include <zed-open-capture/videocapture.hpp>
#include <zed-open-capture/sensorcapture.hpp>

#define VIDEO_MOD_AVAILABLE 1
#define SENSORS_MOD_AVAILABLE 1

namespace zed_ros_wrapper {

// Helper struct to hold the validated hardware configuration being used
struct CameraHWConfig {
    sl_oc::video::RESOLUTION resolution = sl_oc::video::RESOLUTION::HD1080;
    sl_oc::video::FPS fps_enum = sl_oc::video::FPS::FPS_30;
    double actual_fps = 30.0; // Actual numeric FPS for this mode
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
        // Ensure stop is called if not already
        if(is_initialized_) {
            stop();
        }
        ROS_INFO("StereoCamera instance destroyed.");
    }

    /**
     * @brief Initializes video and sensor capture based on configuration.
     * @param hw_config Validated hardware configuration parameters.
     * @return True if initialization was successful, false otherwise.
     */
    bool initialize(const CameraHWConfig& hw_config) {
        if(is_initialized_) {
            ROS_WARN("StereoCamera already initialized. Please call stop() first.");
            return false;
        }
        config_ = hw_config; // Store config used for initialization
        sl_oc::video::VideoParams video_params;
        video_params.res = config_.resolution;
        video_params.fps = config_.fps_enum;
        video_params.verbose = config_.verbosity;

        ROS_INFO("Initializing ZED video capture (Res: %d, FPS: %.1f)...", static_cast<int>(config_.resolution), config_.actual_fps);
        try {
             video_capture_ = std::make_unique<sl_oc::video::VideoCapture>(video_params);
        } catch (const std::exception& e) {
             ROS_ERROR("Exception during VideoCapture construction: %s", e.what());
             return false;
        }

        if (!video_capture_ || !video_capture_->initializeVideo(-1)) {
            ROS_ERROR("Failed to initialize ZED video capture object.");
            video_capture_.reset(); // Ensure pointer is null if failed
            return false;
        }
        camera_sn_ = video_capture_->getSerialNumber();
        ROS_INFO("Video Capture connected to ZED camera SN: %d", camera_sn_);

        ROS_INFO("Initializing ZED sensor capture...");
         try {
             sensor_capture_ = std::make_unique<sl_oc::sensors::SensorCapture>(config_.verbosity);
         } catch (const std::exception& e) {
             ROS_ERROR("Exception during SensorCapture construction: %s", e.what());
             // Allow continuing without sensors
             sensor_capture_.reset();
         }

        // Use the serial number acquired by VideoCapture
        if (!sensor_capture_ || !sensor_capture_->initializeSensors(camera_sn_)) {
            ROS_WARN("Failed to initialize ZED sensor capture object. Continuing without sensors.");
            sensor_capture_.reset(); // Allow operation without sensors if video works
        } else {
            ROS_INFO("Sensor Capture initialized for ZED camera SN: %d", camera_sn_);
            // Enable sensor synchronization if both are available
            video_capture_->enableSensorSync(sensor_capture_.get());
            ROS_INFO("Sensor synchronization enabled.");
        }

        is_initialized_ = true;
        return true; // Video initialized successfully (sensor optional)
    }

     /**
      * @brief Stops the camera capture and releases resources.
      */
     void stop() {
         if(!is_initialized_) {
             return;
         }
         ROS_INFO("Stopping camera capture interface...");
         // Add explicit stop/release methods if the library needs them before destruction
         // e.g., video_capture_->stopStream(); sensor_capture_->stopCapture();

         // Reset smart pointers to release underlying objects
         video_capture_.reset();
         sensor_capture_.reset();
         is_initialized_ = false;
         camera_sn_ = -1;
         ROS_INFO("Camera capture interface stopped.");
     }


    /**
     * @brief Retrieves the latest video frame.
     * @param num_attempts Number of times to try fetching before returning empty frame.
     * @return The latest sl_oc::video::Frame. frame.data will be nullptr if no new frame.
     */
    sl_oc::video::Frame getLastFrame(int num_attempts = 1) const {
        // Use atomic bool for thread-safe check
        if (!is_initialized_.load() || !video_capture_) {
            return {}; // Return empty frame
        }
        return video_capture_->getLastFrame(num_attempts);
    }

    /**
     * @brief Retrieves the latest IMU data.
     * @return The latest sl_oc::sensors::data::Imu. imu.valid may not be NEW_VAL.
     */
    sl_oc::sensors::data::Imu getLastIMUData() const {
        // Use atomic bool for thread-safe check
        if (!is_initialized_.load() || !sensor_capture_) {
            return {}; // Return empty data
        }
        // Sensor acquisition runs at its own rate, check validity in the caller
        return sensor_capture_->getLastIMUData();
    }

    // --- Getters for camera state ---
    bool isVideoInitialized() const { return is_initialized_.load() && video_capture_ != nullptr; }
    bool isSensorInitialized() const { return is_initialized_.load() && sensor_capture_ != nullptr; }
    int getSerialNumber() const { return camera_sn_; }
    double getActualFps() const { return is_initialized_.load() ? config_.actual_fps : 0.0; }
    // !TODO: Get sensor rate from library if possible
    double getSensorRateHz() const { return 400.0; } // Placeholder


private:
    std::unique_ptr<sl_oc::video::VideoCapture> video_capture_;
    std::unique_ptr<sl_oc::sensors::SensorCapture> sensor_capture_;
    int camera_sn_ = -1;
    CameraHWConfig config_; // Config used for the current initialization
    std::atomic_bool is_initialized_{false}; // Flag indicating if initialized (thread-safe read)
};


/**
 * @brief ROS Node class to manage interaction between StereoCamera and ROS,
 * including dynamic reconfiguration.
 */
class ZedRosNode {
public:
    ZedRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : nh_(nh),
          private_nh_(private_nh),
          image_transport_(nh), // Initialize ImageTransport
          last_frame_timestamp_ns_(0)
    {
        ROS_INFO("Constructing ZedRosNode...");
    }

    ~ZedRosNode() {
        // Ensure camera is stopped cleanly if node is destroyed
        if(zed_camera_) {
            zed_camera_->stop();
        }
        ROS_INFO("ZedRosNode destroyed.");
    }

    /**
     * @brief Loads static parameters and sets up the dynamic reconfigure server.
     * Actual camera initialization is triggered by the first DynRec callback.
     * @return True on success, false on failure.
     */
    bool start() {
        if (!loadStaticParameters()) { // Load parameters not handled by DynRec first
             ROS_ERROR("Failed to load static parameters.");
             return false;
        }
        setupPublishers(); // Setup publishers early

        // Setup dynamic reconfigure server FIRST
        // It will trigger the first callback with default values.
        setupDynamicReconfigure();

        ROS_INFO("ZedRosNode setup complete. Waiting for initial dynamic reconfigure callback to initialize camera...");
        return true;
    }

private:

    // --- Dynamic Reconfigure Section ---

    // Using recursive mutex as reconfiguration might call methods that also lock (like setupTimers)
    std::recursive_mutex config_mutex_;
    std::unique_ptr<dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig>> dyn_rec_server_;
    // Stores the latest config requested & validated via dynamic reconfigure
    zed_open_capture_ros::ZedOpenCaptureConfig current_validated_config_;
     // Stores the hardware config currently running or attempted
    CameraHWConfig current_hw_config_;


    void setupDynamicReconfigure() {
        ROS_INFO("Setting up dynamic reconfigure server...");
        // Pass the recursive mutex to the server constructor
        dyn_rec_server_ = std::make_unique<dynamic_reconfigure::Server<zed_open_capture_ros::ZedOpenCaptureConfig>>(config_mutex_, private_nh_);
        // Use boost::bind for member function callback
        // Note: Using 'this' captures the pointer; ensure ZedRosNode outlives the server.
        auto callback = boost::bind(&ZedRosNode::dynamicReconfigureCallback, this, _1, _2);
        dyn_rec_server_->setCallback(callback);
        ROS_INFO("Dynamic reconfigure server setup complete.");
        // The server automatically calls the callback once upon setup with defaults from the .cfg file
    }

    /**
     * @brief Callback for dynamic parameter changes. Validates config, applies changes,
     * potentially re-initializes the camera. Runs in a separate thread.
     * @param config The incoming configuration from dynamic_reconfigure client.
     * @param level Bitmask indicating level of parameters changed (not used here).
     */
    void dynamicReconfigureCallback(zed_open_capture_ros::ZedOpenCaptureConfig &config, uint32_t /*level*/) {
        // Server ensures this callback itself isn't called concurrently,
        // but we need the mutex to protect against timer callbacks accessing shared resources.
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);

        ROS_INFO("Dynamic reconfigure callback triggered.");
        ROS_DEBUG("Requested Config -> Res: %d, FPS: %d, Verbose: %d", config.resolution, config.fps, config.verbose);

        // Validate resolution and FPS combination, determine target hardware config
        CameraHWConfig target_hw_config;
        bool config_valid = validateConfiguration(config, target_hw_config);
        // Note: validateConfiguration might modify 'config.fps' to reflect reality

        if (!config_valid) {
            ROS_ERROR("Invalid hardware configuration requested via dynamic reconfigure. Ignoring change.");
            // Overwrite the incoming 'config' with the last known valid one before returning it
            // This prevents rqt_reconfigure showing invalid state.
            config = current_validated_config_;
            return;
        }

        // Determine if camera needs re-initialization
        // Re-initialize if this is the first time OR if resolution/FPS hardware enum changed
        bool first_initialization = !zed_camera_; // Check if camera object exists yet
        bool reinit_needed = first_initialization ||
                             target_hw_config.resolution != current_hw_config_.resolution ||
                             target_hw_config.fps_enum != current_hw_config_.fps_enum ||
                             target_hw_config.verbosity != current_hw_config_.verbosity;

        ROS_INFO("Callback -> First Init: %d, Reinit Needed: %d", first_initialization, reinit_needed);

        if (reinit_needed) {
            ROS_INFO("Camera (Re)Initialization required. Applying new configuration...");

            // Stop current operations cleanly before changing hardware state
            if (!first_initialization) {
                ROS_INFO("Stopping existing timers and camera...");
                image_timer_.stop();
                sensor_timer_.stop();
                zed_camera_->stop(); // Explicitly stop the camera interface
            }

            // Store the hardware config we are *trying* to apply
            current_hw_config_ = target_hw_config;

            // Attempt to initialize/re-initialize camera object
            if (!zed_camera_) { // Create if it's the first time
                 zed_camera_ = std::make_unique<StereoCamera>();
            }
            bool init_success = zed_camera_->initialize(current_hw_config_);

            if (init_success) {
                ROS_INFO("Camera (Re)Initialized Successfully (Res=%d, FPS=%.1f)",
                    static_cast<int>(current_hw_config_.resolution), current_hw_config_.actual_fps);

                // Reset state dependent on frame rate/timestamps
                last_frame_timestamp_ns_ = 0;

                // (Re)Start timers with potentially new rates based on the new config
                setupTimers();
            } else {
                ROS_ERROR("Failed to re-initialize camera with new settings. Node may be unusable.");
                image_timer_.stop(); // Ensure timers are stopped if init failed
                sensor_timer_.stop();
                zed_camera_.reset(); // Destroy failed camera object
                // Keep current_hw_config as it reflects the last attempt
            }
        } else {
             ROS_INFO("No camera re-initialization required for this configuration change (e.g., only verbose changed).");
             // Handle non-reinit changes here if any (e.g., update logging level)
        }

         // Store the latest validated config from DynRec (potentially modified by validateConfiguration)
         // This helps rqt_reconfigure reflect the actual running state.
        current_validated_config_ = config;
        ROS_INFO("Dynamic reconfigure callback finished.");
    }


    /**
     * @brief Validates DynRec config, resolves FPS/Resolution based on typical ZED capabilities,
     * populates target HW config struct. Modifies input 'requested_config' if FPS is clamped.
     * @param requested_config Config from DynRec callback (may be modified on output).
     * @param target_hw_config Output struct with validated HW settings to be applied.
     * @return True if config is valid and applicable, false otherwise.
     */
    bool validateConfiguration(zed_open_capture_ros::ZedOpenCaptureConfig &requested_config, CameraHWConfig& target_hw_config) {
        target_hw_config.verbosity = requested_config.verbose ? sl_oc::VERBOSITY::INFO : sl_oc::VERBOSITY::OFF;

        double valid_fps = 0;
        sl_oc::video::FPS valid_fps_enum;

        // Determine the *valid* FPS for the requested resolution
        // Based on common ZED/ZED2/ZED Mini capabilities. Adjust if needed for specific model.
        switch (requested_config.resolution) {
           case 0: // HD2K
               target_hw_config.resolution = sl_oc::video::RESOLUTION::HD2K;
               valid_fps = 15.0; // Typically only 15 FPS
               valid_fps_enum = sl_oc::video::FPS::FPS_15;
               break;
           case 1: // HD1080
               target_hw_config.resolution = sl_oc::video::RESOLUTION::HD1080;
               // Allow 15 or 30 FPS
               if (requested_config.fps <= 22) { // Midpoint between 15 and 30
                    valid_fps = 15.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_15;
               } else {
                    valid_fps = 30.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_30;
               }
               break;
           case 2: // HD720
               target_hw_config.resolution = sl_oc::video::RESOLUTION::HD720;
                // Allow 15, 30, or 60 FPS
               if (requested_config.fps <= 22) { // Midpoint 15/30
                    valid_fps = 15.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_15;
               } else if (requested_config.fps <= 45) { // Midpoint 30/60
                    valid_fps = 30.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_30;
               } else {
                    valid_fps = 60.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_60;
               }
               break;
           case 3: // VGA
               target_hw_config.resolution = sl_oc::video::RESOLUTION::VGA;
                // Allow 15, 30, 60, 100 FPS
               if (requested_config.fps <= 22) {
                    valid_fps = 15.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_15;
               } else if (requested_config.fps <= 45) {
                    valid_fps = 30.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_30;
               } else if (requested_config.fps <= 80) { // Midpoint 60/100
                    valid_fps = 60.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_60;
               } else {
                    valid_fps = 100.0;
                    valid_fps_enum = sl_oc::video::FPS::FPS_100;
               }
               break;
           default:
               ROS_ERROR("Invalid resolution parameter value in DynRec Cfg: %d", requested_config.resolution);
               return false; // Invalid resolution index
        }

        // Check if requested FPS matches the *validated* FPS for the resolution
        // We use the validated FPS determined above based on typical ZED capabilities.
        int requested_fps_rounded = static_cast<int>(std::round(requested_config.fps));
        int valid_fps_rounded = static_cast<int>(std::round(valid_fps));

        if (requested_fps_rounded != valid_fps_rounded) {
             ROS_WARN("Requested FPS %d is not directly supported for resolution mode %d. Using validated rate %.1f FPS.",
                      requested_config.fps, requested_config.resolution, valid_fps);
             // Update the config object that dynamic_reconfigure sends back to client
             // to show the *actual* FPS being used.
             requested_config.fps = valid_fps_rounded;
        }

        target_hw_config.fps_enum = valid_fps_enum;
        target_hw_config.actual_fps = valid_fps;

        ROS_INFO("Validated Config -> Res: %d, FPS Enum: %d, Actual FPS: %.1f",
                 static_cast<int>(target_hw_config.resolution), static_cast<int>(target_hw_config.fps_enum), target_hw_config.actual_fps);

        return true; // Configuration is valid (potentially adjusted)
    }

    // --- Parameter Loading (Static Only) ---
    bool loadStaticParameters() {
        ROS_INFO("Loading static ROS parameters (Frame IDs)...");
        private_nh_.param<std::string>("left_frame_id", left_frame_id_, "zed_left_camera_frame");
        private_nh_.param<std::string>("right_frame_id", right_frame_id_, "zed_right_camera_frame");
        private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "zed_imu_link");
        ROS_INFO("Static Parameters Loaded:");
        ROS_INFO("  Left Frame ID: %s", left_frame_id_.c_str());
        ROS_INFO("  Right Frame ID: %s", right_frame_id_.c_str());
        ROS_INFO("  IMU Frame ID: %s", imu_frame_id_.c_str());
        return true;
    }

    // --- Publisher & Timer Setup ---
    void setupPublishers() {
        ROS_INFO("Setting up ROS publishers...");
        // Queue size 1 allows dropping older messages if processing is slow
        left_image_pub_ = image_transport_.advertise("left/image_raw", 1);
        right_image_pub_ = image_transport_.advertise("right/image_raw", 1);
        // IMU publishers - queue size 100 is reasonable for high-rate sensor
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
    }

    /**
     * @brief Stops existing timers and starts new ones based on current_hw_config_.
     * Assumes config_mutex_ is already locked if called from DynRec callback.
     */
    void setupTimers() {
        // Stop existing timers before potentially re-creating them
        image_timer_.stop();
        sensor_timer_.stop();

        // Ensure camera is valid and we have an FPS target before setting timers
        // Check the pointer itself first, then the init status
        if (!zed_camera_ || !zed_camera_->isVideoInitialized() || current_hw_config_.actual_fps <= 0) {
             ROS_WARN("Cannot setup timers, camera not initialized or invalid FPS (%.1f).", current_hw_config_.actual_fps);
             return;
        }

        ROS_INFO("Setting up ROS timers based on current config (FPS: %.1f)...", current_hw_config_.actual_fps);

        // Timer for images: Poll slightly faster than the target frame rate
        double image_poll_rate = current_hw_config_.actual_fps * 1.5; // Poll ~1.5x FPS
        double image_timer_duration = (image_poll_rate > 1.0) ? (1.0 / image_poll_rate) : 1.0; // Prevent zero/negative/too slow rate, min 1Hz poll
        if (image_timer_duration < 0.002) image_timer_duration = 0.002; // Max ~500Hz poll rate sanity check

        ROS_INFO("Setting image timer poll rate: %.2f Hz (Duration: %.4f s)", 1.0/image_timer_duration, image_timer_duration);
        // Use one-shot=false, auto-start=true
        image_timer_ = nh_.createTimer(ros::Duration(image_timer_duration), &ZedRosNode::imageCallback, this, false, true);


        if (zed_camera_->isSensorInitialized()) {
            // Timer for IMU: Poll slightly faster than the expected sensor rate (e.g., 400 Hz)
            double sensor_native_rate = zed_camera_->getSensorRateHz(); // Get from camera if possible
            double sensor_poll_rate = sensor_native_rate * 1.5; // Poll ~1.5x sensor rate
            double sensor_timer_duration = (sensor_poll_rate > 1.0) ? (1.0 / sensor_poll_rate) : 0.1; // Min 10Hz poll

             // Limit polling to max ~1000 Hz for sanity
             if (sensor_timer_duration < 0.001) {
                 sensor_timer_duration = 0.001;
             }
            ROS_INFO("Setting sensor timer poll rate: %.2f Hz (Duration: %.4f s)", 1.0/sensor_timer_duration, sensor_timer_duration);
             // Use one-shot=false, auto-start=true
            sensor_timer_ = nh_.createTimer(ros::Duration(sensor_timer_duration), &ZedRosNode::sensorCallback, this, false, true);
        } else {
             ROS_INFO("Sensor capture not initialized, IMU timer not started.");
        }
    }

    // --- Callbacks (Image & Sensor) ---

    /**
     * @brief ROS Timer callback to process and publish images. Thread-safe.
     */
    void imageCallback(const ros::TimerEvent& /*event*/) {
        // Lock mutex as we access the camera object shared with DynRec callback
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);

        // Check camera pointer and initialization status within lock
        if (!zed_camera_ || !zed_camera_->isVideoInitialized()) {
            // ROS_DEBUG_THROTTLE(5.0, "Image callback skipped, camera not ready.");
            return; // Camera might be re-initializing
        }

        const sl_oc::video::Frame frame = zed_camera_->getLastFrame(1);

        // Check if frame is valid and *new*
        if (frame.data == nullptr || frame.timestamp <= last_frame_timestamp_ns_) {
            // ROS_DEBUG("Image callback skipped, no new frame data.");
            return; // No new data or invalid frame
        }
        uint64_t current_ts = frame.timestamp; // Store before potential mutation if needed
        last_frame_timestamp_ns_ = current_ts;

        // --- Image Processing ---
        // Assuming YUV2 format (YUYV) based on original code
        // Check frame.data_format if available in sl_oc::video::Frame to be sure
        cv::Mat frameYUV(frame.height, frame.width, CV_8UC2, frame.data);
        cv::Mat frameBGR; // Allocate destination matrix

        try {
             cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
        } catch (const cv::Exception& e) {
            ROS_ERROR_THROTTLE(1.0, "OpenCV exception during YUV->BGR conversion: %s", e.what()); // Throttle errors
            return;
        }


        // Split into left and right images (assuming side-by-side format)
        if (frameBGR.cols < frame.width || frameBGR.rows < frame.height) {
             ROS_ERROR_THROTTLE(1.0,"BGR frame dimensions mismatch! Expected %dx%d, Got %dx%d",
                       frame.width, frame.height, frameBGR.cols, frameBGR.rows);
             return;
        }
        int half_width = frame.width / 2;
        // Use range check for safety, although Rect constructor should handle it
        if (half_width <= 0 || frame.height <= 0) {
             ROS_ERROR_THROTTLE(1.0,"Invalid frame dimensions for splitting: %dx%d", frame.width, frame.height);
             return;
        }
        cv::Mat left_image = frameBGR(cv::Rect(0, 0, half_width, frame.height));
        cv::Mat right_image = frameBGR(cv::Rect(half_width, 0, half_width, frame.height));

        // Create ROS timestamp from nanoseconds
        // Note: Assumes camera timestamp is based on ROS time epoch (or close enough)
        ros::Time timestamp_ros = ros::Time().fromNSec(current_ts);

        // Publish images (publishImage helper is below)
        publishImage(left_image, left_image_pub_, left_frame_id_, timestamp_ros);
        publishImage(right_image, right_image_pub_, right_frame_id_, timestamp_ros);
    }

     /**
     * @brief Helper to publish an image message. Checks subscribers. Minimal state access.
     */
    void publishImage(const cv::Mat& img, image_transport::Publisher& pub, const std::string& frame_id, const ros::Time& stamp) {
        if (pub.getNumSubscribers() == 0) return; // Don't do work if no one is listening

        try {
            // Use cv_bridge to convert cv::Mat to sensor_msgs::Image
            std_msgs::Header header;
            header.stamp = stamp;
            header.frame_id = frame_id;
            // BGR8 is the standard color format in ROS for OpenCV BGR images
            cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, img);

            sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();
            pub.publish(img_msg);
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_THROTTLE(1.0, "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "Std exception during image publishing: %s", e.what());
        }
    }


    /**
     * @brief ROS Timer callback to process and publish IMU data. Thread-safe.
     */
    void sensorCallback(const ros::TimerEvent& /*event*/) {
        // Lock mutex as we access the camera object shared with DynRec callback
        std::lock_guard<std::recursive_mutex> lock(config_mutex_);

        // Check camera pointer, init status, *and* sensor status specifically
        if (!zed_camera_ || !zed_camera_->isSensorInitialized() || imu_pub_.getNumSubscribers() == 0) {
            // ROS_DEBUG_THROTTLE(5.0, "IMU callback skipped, sensor not ready or no subscribers.");
            return; // Camera might be re-initializing or sensor failed
        }

        const sl_oc::sensors::data::Imu imuData = zed_camera_->getLastIMUData();

        // Check if the data is valid and new (according to library's definition)
        if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
            sensor_msgs::Imu imu_msg;

            // Assuming timestamp is nanoseconds, create ROS time
            imu_msg.header.stamp = ros::Time().fromNSec(imuData.timestamp);
            imu_msg.header.frame_id = imu_frame_id_; // Use parameterized frame_id

            // Populate IMU data - Check units expected by ROS (rad/s, m/s^2)
            // Add conversions here if library units differ from ROS standard units.
            imu_msg.angular_velocity.x = imuData.gX;
            imu_msg.angular_velocity.y = imuData.gY;
            imu_msg.angular_velocity.z = imuData.gZ;

            imu_msg.linear_acceleration.x = imuData.aX;
            imu_msg.linear_acceleration.y = imuData.aY;
            imu_msg.linear_acceleration.z = imuData.aZ;

            // Orientation is not provided by this sensor data structure
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 1.0; // Valid quaternion for unknown orientation
            imu_msg.orientation_covariance[0] = -1.0; // Indicate unknown orientation

            // Set covariance matrices to unknown (all zeros, first element -1)
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;
            // !TODO: Populate covariances with estimated values if available.

            imu_pub_.publish(imu_msg);
        }
    }


    // --- Member Variables ---
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    ros::Publisher imu_pub_;
    ros::Timer image_timer_;
    ros::Timer sensor_timer_;

    // Camera Interface & Configuration State (Protected by config_mutex_)
    std::unique_ptr<StereoCamera> zed_camera_;
    CameraHWConfig current_hw_config_; // Config currently running/attempted on HW

    // Static Parameters
    std::string left_frame_id_;
    std::string right_frame_id_;
    std::string imu_frame_id_;

    // Image Callback State
    uint64_t last_frame_timestamp_ns_;

}; // class ZedRosNode

} // namespace zed_ros_wrapper


// --- Main Function ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_ros_node"); // Node name

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // Private node handle for parameters

    ROS_INFO("Starting ZED ROS Node...");

    // Create the node object
    zed_ros_wrapper::ZedRosNode node(nh, private_nh);

    // Start performs static setup and dynamic reconfigure server setup.
    // Actual camera initialization happens within the first DynRec callback.
    if (!node.start()) {
        ROS_FATAL("Failed to setup ZED ROS Node. Shutting down.");
        return 1; // Indicate failure
    }

    ROS_INFO("ZED ROS Node setup complete, spinning to process callbacks.");
    // ros::spin() processes messages and timer events in the main thread.
    // Dynamic reconfigure callback runs in a separate thread managed by its server.
    ros::spin();

    ROS_INFO("ZED ROS Node shutting down.");
    return 0; // Indicate success
}
