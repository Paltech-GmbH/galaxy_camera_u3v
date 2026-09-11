#include <chrono>
#include <ctime>
#include <atomic>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <string>
#include "galaxy_camera_u3v/GxIAPI.h"
#include "galaxy_camera_u3v/DxImageProc.h"
#include "galaxy_camera_u3v/gx_utils.h"
#include <unistd.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <opencv2/core.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "galaxy_camera_u3v/visibility_control.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std::chrono_literals;

#define FRAME_RATE 30.0
#define BAYER_ENCODING "BAYER"

namespace camera {

//
// this is a read only camera publisher
//
class U3vImagePub: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit U3vImagePub(const rclcpp::NodeOptions &options)
  : Node("u3v_image_pub", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  // : Node("u3v_image_pub", options),

  camera_info_url_("package://galaxy_camera_u3v/camera_info/mer2_630_60u3c.yaml")
  {
    // this flag is used control if certain parameters can be updated
    is_initialising_ = true;

    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();
    auto sensor_qos = rclcpp::SensorDataQoS();

    // ros2 parameter call backs
    parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&U3vImagePub::on_set_parameters_callback, this, std::placeholders::_1));

    acquisition_role_ = this->declare_parameter<std::string>("acquisition_role","standalone");
    RCLCPP_INFO(this->get_logger(), "starting with acquisition role: %s on thread: %s", acquisition_role_.c_str(), string_thread_id().c_str());

    // determine the acquisition role this camera plays - there is a leader and followers

    // initial camera info
    topic_ = this->declare_parameter<std::string>("topic","stereo/right");
    RCLCPP_INFO(this->get_logger(),"parameter topic_: %s", topic_.c_str());

    RCLCPP_INFO(this->get_logger(),"Using camera_info_url_: %s", camera_info_url_.c_str());
    camera_info_manager::CameraInfoManager cim(this, topic_, camera_info_url_);
    camera_info_ = cim.getCameraInfo();

    // variables for GX Library
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t num_devices;

    // initialise library
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "GXInitLib failed ... ");
      exit (-1);
    }
    lib_initialised_ = true;

    // Get device enumerated number - should be at least 1
    status = GXUpdateDeviceList(&num_devices, 1000);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg);
      exit (-2);
    }
    if(num_devices <= 0){
      RCLCPP_ERROR(this->get_logger(),"no camera devices found ... plug in and try again");
      exit (-3);
    }
    RCLCPP_INFO(this->get_logger(), "Found %d cameras ....", num_devices);

    device_sn_ = this->declare_parameter<std::string>("device_sn","FDS20110008");
    RCLCPP_INFO(this->get_logger(),"parameter device_sn_: %s", device_sn_.c_str());

    // open the camera (single daheng camera - open by index, as per original)
    status = open_device();
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error opening camera: %s", error_msg);
      exit (-5);
    }
    RCLCPP_DEBUG(this->get_logger(), "gx_dev_handle_0: %p", gx_dev_handle_);


    // status = GXGetInt(this->gx_dev_handle_, GX_INT_PAYLOAD_SIZE, &this->payload_size_);
    // if (status != GX_STATUS_SUCCESS) {
    //   auto error_msg = GetErrorString(status);
    //   RCLCPP_ERROR(this->get_logger(), "error getting payload_size_: %s", error_msg);
    //   exit (-7);
    // }
    // this->declare_parameter<int64_t>("pixel_format", GX_PIXEL_FORMAT_BAYER_RG10);
    this->declare_parameter<int64_t>("pixel_format", GX_PIXEL_FORMAT_BAYER_RG8);

    CameraDeviceInfo camera_info;
    status = GetCameraInfo(gx_dev_handle_, &camera_info);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting device camera info: %s", error_msg);
      exit (-6);
    }
    RCLCPP_INFO(this->get_logger(),"camera - vendor_name: %s model_name: %s serial_number: %s device_version: %s firmware_version: %s color_filter: %d",
      camera_info.vendor_name.c_str(),
      camera_info.model_name.c_str(),
      camera_info.serial_number.c_str(),
      camera_info.device_version.c_str(),
      camera_info.device_firmware_version.c_str(),
      camera_info.color_filter
    );

    if (!camera_info.color_filter) {
      RCLCPP_ERROR(this->get_logger(), "camera not bayer color .. exiting");
      exit(-6);
    }

    status = GXGetEnum(this->gx_dev_handle_, GX_ENUM_PIXEL_SIZE, &this->pixel_size_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting pixel_size: %s", error_msg);
      exit (-6);
    }

    status = GXGetEnum(this->gx_dev_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &this->color_filter_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting color_filter: %s", error_msg);
      exit (-6);
    }

    ImageFormat image_format;
    status = GetImageFormat(gx_dev_handle_, &image_format);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting image format: %s", error_msg);
      exit (-6);
    }
    RCLCPP_INFO(get_logger(),"image format - sensor width: %ld sensor height: %ld width max: %ld height max: %ld width: %ld height: %ld offset x: %ld offset y:%ld",
      image_format.sensor_width, image_format.sensor_height,
      image_format.width_max, image_format.height_max,
      image_format.width, image_format.height,
      image_format.offset_x, image_format.offset_y
    );

    // remember the sensor dimensions so reinit can resize buffers consistently
    sensor_width_ = image_format.sensor_width;
    sensor_height_ = image_format.sensor_height;

    // not implemented on test device
    // status = GXSetEnum(this->gx_dev_handle_, GX_ENUM_DEAD_PIXEL_CORRECT, GX_DEAD_PIXEL_CORRECT_ON);
    // if (status != GX_STATUS_SUCCESS) {
    //   auto error_msg = GetErrorString(status);
    //   RCLCPP_ERROR(this->get_logger(), "error setting GX_DEAD_PIXEL_CORRECT_ON: %s", error_msg);
    //   exit (-6);
    // }

    // iamge encoding to publish
    this->declare_parameter<std::string>("image_encoding", "BGR8");

    // camera parameters - these are the default values. Values will be set on the camera if appropriate
    this->declare_parameter<int64_t>("acquisition_mode", GX_ACQ_MODE_CONTINUOUS);
    this->declare_parameter<int64_t>("trigger_mode",GX_TRIGGER_MODE_OFF);
    // leader send the trigger out on line2 for the followers (hard wired)
    if (acquisition_role_.compare("leader") == 0) {
      this->declare_parameter<int64_t>("trigger_mode",GX_TRIGGER_MODE_OFF);
      // this->declare_parameter<int64_t>("trigger_source",GX_TRIGGER_SOURCE_SOFTWARE);
      this->declare_parameter<int64_t>("line_selector", GX_ENUM_LINE_SELECTOR_LINE2);
      this->declare_parameter<int64_t>("line_mode", GX_ENUM_LINE_MODE_OUTPUT);
      // this->declare_parameter<int64_t>("line_source", GX_ENUM_LINE_SOURCE_STROBE);
      this->declare_parameter<int64_t>("line_source", GX_ENUM_LINE_SOURCE_TIMER1_ACTIVE);
    }
    if (acquisition_role_.compare("follower") == 0) {
      this->declare_parameter<int64_t>("trigger_mode",GX_TRIGGER_MODE_ON);
      this->declare_parameter<int64_t>("trigger_source",GX_TRIGGER_SOURCE_LINE3);
      this->declare_parameter<int64_t>("line_selector", GX_ENUM_LINE_SELECTOR_LINE3);
      this->declare_parameter<int64_t>("line_mode", GX_ENUM_LINE_MODE_INPUT);
    }
    this->declare_parameter<double_t>("auto_exposure_time_min", 8.0); // microseconds
    this->declare_parameter<double_t>("auto_exposure_time_max", 5000.0); //microseconds
    this->declare_parameter<int64_t>("exposure_auto", GX_EXPOSURE_AUTO_CONTINUOUS);
    this->declare_parameter<int64_t>("exposure_mode", GX_EXPOSURE_MODE_TIMED);
    // this->declare_parameter<double_t>("exposure_time", 5000.0);
    this->declare_parameter<int64_t>("expected_gray_value", 30);
    this->declare_parameter<double_t>("current_acquisition_frame_rate",0.0);
    this->declare_parameter<double_t>("gain",0.0); // read only - gets updated periodically
    this->declare_parameter<int64_t>("gain_auto", GX_GAIN_AUTO_CONTINUOUS);
    // this->declare_parameter<int64_t>("gain_auto", GX_GAIN_AUTO_OFF);
    this->declare_parameter<double_t>("auto_gain_min", 0.0); // dB
    this->declare_parameter<double_t>("auto_gain_max", 24.0); // dB
    this->declare_parameter<int64_t>("balance_ratio_selector", GX_BALANCE_RATIO_SELECTOR_RED);
    this->declare_parameter<double_t>("balance_ratio",1.0); // read only when continuous - gets updated periodically
    this->declare_parameter<int64_t>("balance_white_auto", GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    this->declare_parameter<bool>("gamma_enable", true);
    this->declare_parameter<int64_t>("gamma_mode", GX_GAMMA_SELECTOR_SRGB);
    int roi_width=3088;
    int roi_height=2064;
    this->declare_parameter<int64_t>("awb_roi_width", roi_width);
    this->declare_parameter<int64_t>("awb_roi_height", roi_height);
    this->declare_parameter<int64_t>("awb_roi_offset_x", 0); // int16_t(2048/2 - (roi_width/2))
    this->declare_parameter<int64_t>("awb_roi_offset_y", 0); // int16_t(1536/2 - (roi_height/2))
    this->declare_parameter<int64_t>("awb_lamp_house", GX_AWB_LAMP_HOUSE_ADAPTIVE);

    this->declare_parameter<int64_t>("acquisition_frame_rate_mode",GX_ACQUISITION_FRAME_RATE_MODE_ON);
    // this->declare_parameter<double_t>("acquisition_frame_rate", 56.0);
    this->declare_parameter<double_t>("acquisition_frame_rate", 10.0);

    // health-check tuning: how low the *measured* fps may fall before we declare
    // the camera dead and reinitialise. Default 1.0 fps as requested.
    this->declare_parameter<double_t>("min_healthy_fps", 1.0);
    // number of consecutive 1-second health windows below threshold before reinit
    this->declare_parameter<int64_t>("unhealthy_windows_before_reinit", 2);

    status = GXStreamOn(gx_dev_handle_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error stream on (%s): %s", device_sn_.c_str(), error_msg);
      exit (-8);
    }
    streaming_ = true;

    // register an offline callback for fast disconnect detection
    register_offline_callback();

    // setup c style buffers for the camera
    this->RGB_image_buf_ = new u_char[sensor_height_ * sensor_width_ * 3];
    this->image_buf_ = new u_char[this->payload_size_];

    // publishers
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    // initialise are start the timer to work out the frames per second)
    auto start_time = std::chrono::steady_clock::now();
    frame_time_ = start_time;
    frame_count_ = 0;
    frame_timer_ = this->create_wall_timer(1s, std::bind(&U3vImagePub::frame_timer_callback, this));

    param_timer_ = this->create_wall_timer(1s, std::bind(&U3vImagePub::param_timer_callback, this));

    callback_group_capture_timer_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    capture_timer_ = this->create_wall_timer(10ms, std::bind(&U3vImagePub::capture_timer_callback, this), callback_group_capture_timer_);

    // dedicated callback group + timer for reinitialisation so it never runs
    // re-entrantly with capture and never blocks other callbacks
    callback_group_reinit_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // just use for testing - trigger should come from the controller
    // auto frame_duration = 1000000us/FRAME_RATE;
    // trigger_timer_ = this->create_wall_timer(frame_duration, std::bind(&U3vImagePub::trigger_timer_callback, this));


    capture_trigger_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>("/capture_trigger", qos, std::bind(&U3vImagePub::capture_trigger_callback, this, std::placeholders::_1));

    // Named to match depthai_ros_driver's ~/start_camera and ~/stop_camera so
    // deep sleep can drive every camera through the same pair of calls.
    stop_camera_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_camera",
        std::bind(&U3vImagePub::stop_camera_cb, this, std::placeholders::_1, std::placeholders::_2));
    start_camera_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_camera",
        std::bind(&U3vImagePub::start_camera_cb, this, std::placeholders::_1, std::placeholders::_2));

    is_initialising_ = false;
  }

  CAMERA_LOCAL
  ~U3vImagePub(){

    if (this->gx_dev_handle_ != NULL) {
      RCLCPP_INFO(this->get_logger(), "closing gx_dev_handle_");
      unregister_offline_callback();
      GXStreamOff(this->gx_dev_handle_);
      GXCloseDevice(this->gx_dev_handle_);
      gx_dev_handle_ = NULL;
    }
    if (lib_initialised_) {
      GXCloseLib();
    }

    if (RGB_image_buf_ != NULL)
      delete[] RGB_image_buf_;
    if (image_buf_ != NULL)
      delete[] image_buf_;

    RCLCPP_INFO(this->get_logger(),"finished");
  }

private:
  bool is_initialising_;
  double last_record = 0;
  int rec_fps = 4;

  std::string acquisition_role_; // camera maybe a leader or a follower

  rclcpp::CallbackGroup::SharedPtr callback_group_capture_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_reinit_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
  // rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::TimerBase::SharedPtr info_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr reinit_timer_;

  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr capture_trigger_sub_;

  // ros2 camera
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;

  std::string topic_;

  std::string camera_info_url_;
  camera_info_manager::CameraInfo camera_info_;

  std::string image_encoding_;

  // camera related
  std::string device_sn_;
  rclcpp::Time trigger_timestamp_;
  std::string trigger_source_;
  GX_DEV_HANDLE gx_dev_handle_ = NULL;

  int64_t color_filter_;
  int64_t pixel_size_;
  int64_t payload_size_;

  int64_t sensor_width_ = 0;
  int64_t sensor_height_ = 0;

  u_char *RGB_image_buf_ = NULL;
  u_char *image_buf_ = NULL;

  std::chrono::time_point<std::chrono::steady_clock> frame_time_;

  uint16_t frame_count_;

  // ---- reinit / health-check state ----
  bool lib_initialised_ = false;
  bool streaming_ = false;
  std::atomic<bool> reinit_in_progress_{false};
  /// Set while the camera is deliberately stopped (deep sleep). Distinct from
  /// streaming_: it tells the health check that zero fps is expected, so it
  /// does not "recover" the camera we just switched off.
  std::atomic<bool> stopped_on_request_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_camera_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_camera_srv_;
  // set by the SDK offline callback - means the device dropped off the bus
  std::atomic<bool> device_offline_{false};
  GX_EVENT_CALLBACK_HANDLE offline_cb_handle_ = NULL;
  int unhealthy_window_count_ = 0;

  // open the single daheng camera by index (original behaviour, unchanged)
  CAMERA_LOCAL
  GX_STATUS open_device() {
    GX_OPEN_PARAM gx_open_param;
    gx_open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    gx_open_param.openMode = GX_OPEN_INDEX;
    gx_open_param.pszContent = const_cast<char*>("1");
    return GXOpenDevice(&gx_open_param, &this->gx_dev_handle_);
  }

  // static trampoline for the SDK offline callback
  static void GX_STDC on_device_offline(void *user_param) {
    auto *self = reinterpret_cast<U3vImagePub*>(user_param);
    if (self != nullptr) {
      self->device_offline_.store(true);
    }
  }

  CAMERA_LOCAL
  void register_offline_callback() {
    if (gx_dev_handle_ == NULL) return;
    GX_STATUS status = GXRegisterDeviceOfflineCallback(
        gx_dev_handle_, this, &U3vImagePub::on_device_offline, &offline_cb_handle_);
    if (status != GX_STATUS_SUCCESS) {
      RCLCPP_WARN(get_logger(), "could not register device offline callback: %s",
                  GetErrorString(status));
      offline_cb_handle_ = NULL;
    }
  }

  CAMERA_LOCAL
  void unregister_offline_callback() {
    if (gx_dev_handle_ != NULL && offline_cb_handle_ != NULL) {
      GXUnregisterDeviceOfflineCallback(gx_dev_handle_, offline_cb_handle_);
      offline_cb_handle_ = NULL;
    }
  }

  CAMERA_LOCAL
  void update_payload_size() {
    auto status = GXGetInt(gx_dev_handle_, GX_INT_PAYLOAD_SIZE, &payload_size_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(get_logger(), "error getting payload_size_: %s", error_msg);
      // do not exit here - during reinit a transient failure should not kill the node
      return;
    }

    RCLCPP_INFO(get_logger(),"payload_size_: %ld", payload_size_);
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter &parameter: parameters){

      if (parameter.get_name() == "acquisition_role" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        if (!acquisition_role_.empty()) {
          result.successful = false;
          result.reason = "acquisition_role can't be changed, once set!";
        } else {
          if (!parameter.as_string().compare("leader") == 0
           && !parameter.as_string().compare("follower") == 0
           && !parameter.as_string().compare("standalone") == 0){
            result.successful = false;
            result.reason = "acquisition_role can be either standalone, leader or follower!";
          }
        }
      }
      else if (parameter.get_name() == "pixel_format" &&
          parameter.get_type() ==  rclcpp::ParameterType::PARAMETER_INTEGER){
          result = param_gx_set_enum(GX_ENUM_PIXEL_FORMAT, parameter.as_int());
          update_payload_size();
      }
      else if (parameter.get_name() == "image_encoding" &&
          parameter.get_type() ==  rclcpp::ParameterType::PARAMETER_STRING){
          image_encoding_ = parameter.as_string();
      }
      else if (parameter.get_name() == "topic" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        if (!topic_.empty()) {
          result.successful = false;
          result.reason = "topic can't be changed, once set!";
        }
      }
      else if (parameter.get_name() == "device_sn" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        if (!device_sn_.empty()) {
          result.successful = false;
          result.reason = "device_sn can't be changed, once set!";
        }
      }
      else if (parameter.get_name() == "acquisition_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_ACQUISITION_MODE, parameter.as_int());
      }
      else if (parameter.get_name() == "trigger_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_TRIGGER_MODE, parameter.as_int());
      }
      else if (parameter.get_name() == "trigger_source" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_TRIGGER_SOURCE, parameter.as_int());
      }
      else if (parameter.get_name() == "line_selector" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_LINE_SELECTOR, parameter.as_int());
      }
      else if (parameter.get_name() == "line_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_LINE_MODE, parameter.as_int());
      }
      else if (parameter.get_name() == "line_source" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_LINE_SOURCE, parameter.as_int());
      }
      else if (parameter.get_name() == "exposure_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_EXPOSURE_MODE, parameter.as_int());
      }
      // else if (parameter.get_name() == "exposure_time" &&
      //     parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      //     result = param_gx_set_float(GX_FLOAT_EXPOSURE_TIME, parameter.as_double());
      // }
      else if (parameter.get_name() == "exposure_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_EXPOSURE_AUTO, parameter.as_int());
      }
      else if (parameter.get_name() == "auto_exposure_time_min" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, parameter.as_double());
      }
      else if (parameter.get_name() == "auto_exposure_time_max" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, parameter.as_double());
      }
      else if (parameter.get_name() == "expected_gray_value" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_GRAY_VALUE, parameter.as_int());
      }
      else if (parameter.get_name() == "acquisition_frame_rate_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_ACQUISITION_FRAME_RATE_MODE, parameter.as_int());
      }
      else if (parameter.get_name() == "acquisition_frame_rate" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_ACQUISITION_FRAME_RATE, parameter.as_double());
      }
      else if (parameter.get_name() == "gain" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_GAIN, parameter.as_double());
      }
      else if (parameter.get_name() == "gain_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_GAIN_AUTO, parameter.as_int());
      }
      else if (parameter.get_name() == "auto_gain_min" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_GAIN_MIN, parameter.as_double());
      }
      else if (parameter.get_name() == "auto_gain_max" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_GAIN_MAX, parameter.as_double());
      }
      else if (parameter.get_name() == "balance_ratio_selector" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_BALANCE_RATIO_SELECTOR, parameter.as_int());
      }
      else if (parameter.get_name() == "balance_ratio" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_BALANCE_RATIO, parameter.as_double());
      }
      else if (parameter.get_name() == "balance_white_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_BALANCE_WHITE_AUTO, parameter.as_int());
      }
      // else if (parameter.get_name() == "awb_lamp_house" &&
      //     parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      //     result = param_gx_set_enum(GX_ENUM_AWB_LAMP_HOUSE, parameter.as_int());
      // }
      else if (parameter.get_name() == "awb_roi_width" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_WIDTH, parameter.as_int());
      }
      else if (parameter.get_name() == "awb_roi_height" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_HEIGHT, parameter.as_int());
      }
      // else if (parameter.get_name() == "awb_roi_offset_x" &&
      //     parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      //     result = param_gx_set_int(GX_INT_AWBROI_OFFSETX, parameter.as_int());
      // }
      // else if (parameter.get_name() == "awb_roi_offset_y" &&
      //     parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      //     result = param_gx_set_int(GX_INT_AWBROI_OFFSETY, parameter.as_int());
      // }

      if (!result.successful) {
        RCLCPP_WARN(this->get_logger(), "parameter %s not set - %s",parameter.get_name().c_str(), result.reason.c_str());
      } else {
        RCLCPP_DEBUG(this->get_logger(), "parameter set %s: %s", parameter.get_name().c_str(), parameter.value_to_string().c_str());
      }
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_enum(GX_FEATURE_ID_CMD feature_id, int64_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto status = GXSetEnum(gx_dev_handle_, feature_id, n_value);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_int(GX_FEATURE_ID_CMD feature_id, int64_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_writeable = false;
    status = GXIsWritable(gx_dev_handle_, feature_id, &is_writeable);
    if (status == GX_STATUS_SUCCESS) {
      if(is_writeable){
        status = GXSetInt(gx_dev_handle_, feature_id, n_value);
      }
      // } else {
      //   RCLCPP_WARN(get_logger(),"not writable param_gx_set_int %d %ld", feature_id, n_value);
      // }
    }
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_float(GX_FEATURE_ID_CMD feature_id, double_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_writeable = false;
    // silently fail if not writeable - timer callback updates parameter with actual values
    status = GXIsWritable(gx_dev_handle_, feature_id, &is_writeable);
    if (status == GX_STATUS_SUCCESS) {
      if(is_writeable){
        status = GXSetFloat(gx_dev_handle_, feature_id, n_value);
      }
      // } else {
      //   RCLCPP_WARN(get_logger(),"not writable param_gx_set_float %d %f", feature_id, n_value);
      // }
    }
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  // push the current ROS parameter values back onto the camera. Used both at
  // startup (implicitly via the param_timer) and explicitly after a reinit so
  // the freshly reopened device gets the right configuration immediately.
  CAMERA_LOCAL
  void reapply_camera_settings() {
    if (gx_dev_handle_ == NULL) return;

    auto set_enum = [&](const std::string &name, GX_FEATURE_ID_CMD id) {
      if (this->has_parameter(name))
        param_gx_set_enum(id, this->get_parameter(name).as_int());
    };
    auto set_int = [&](const std::string &name, GX_FEATURE_ID_CMD id) {
      if (this->has_parameter(name))
        param_gx_set_int(id, this->get_parameter(name).as_int());
    };
    auto set_float = [&](const std::string &name, GX_FEATURE_ID_CMD id) {
      if (this->has_parameter(name))
        param_gx_set_float(id, this->get_parameter(name).as_double());
    };

    set_enum("pixel_format", GX_ENUM_PIXEL_FORMAT);
    update_payload_size();

    set_enum("acquisition_mode", GX_ENUM_ACQUISITION_MODE);
    set_enum("trigger_mode", GX_ENUM_TRIGGER_MODE);
    if (this->has_parameter("trigger_source"))
      set_enum("trigger_source", GX_ENUM_TRIGGER_SOURCE);
    if (this->has_parameter("line_selector"))
      set_enum("line_selector", GX_ENUM_LINE_SELECTOR);
    if (this->has_parameter("line_mode"))
      set_enum("line_mode", GX_ENUM_LINE_MODE);
    if (this->has_parameter("line_source"))
      set_enum("line_source", GX_ENUM_LINE_SOURCE);

    set_enum("exposure_mode", GX_ENUM_EXPOSURE_MODE);
    set_enum("exposure_auto", GX_ENUM_EXPOSURE_AUTO);
    set_float("auto_exposure_time_min", GX_FLOAT_AUTO_EXPOSURE_TIME_MIN);
    set_float("auto_exposure_time_max", GX_FLOAT_AUTO_EXPOSURE_TIME_MAX);
    set_int("expected_gray_value", GX_INT_GRAY_VALUE);

    set_enum("gain_auto", GX_ENUM_GAIN_AUTO);
    set_float("auto_gain_min", GX_FLOAT_AUTO_GAIN_MIN);
    set_float("auto_gain_max", GX_FLOAT_AUTO_GAIN_MAX);

    set_enum("balance_ratio_selector", GX_ENUM_BALANCE_RATIO_SELECTOR);
    set_enum("balance_white_auto", GX_ENUM_BALANCE_WHITE_AUTO);

    set_int("awb_roi_width", GX_INT_AWBROI_WIDTH);
    set_int("awb_roi_height", GX_INT_AWBROI_HEIGHT);

    if (this->has_parameter("gamma_enable"))
      GXSetBool(gx_dev_handle_, GX_BOOL_GAMMA_ENABLE, this->get_parameter("gamma_enable").as_bool());
    set_enum("gamma_mode", GX_ENUM_GAMMA_MODE);

    set_enum("acquisition_frame_rate_mode", GX_ENUM_ACQUISITION_FRAME_RATE_MODE);
    set_float("acquisition_frame_rate", GX_FLOAT_ACQUISITION_FRAME_RATE);
  }

  CAMERA_LOCAL
  void param_timer_callback(){
    RCLCPP_INFO_ONCE(get_logger(),"first param_timer_callback ...");
    // skip touching the device while a reinit is underway, the handle is gone,
    // or acquisition is deliberately stopped
    if (reinit_in_progress_.load() || gx_dev_handle_ == NULL || stopped_on_request_.load()) {
      return;
    }
    update_changed_enum_param("acquisition_mode", GX_ENUM_ACQUISITION_MODE);
    update_changed_enum_param("trigger_mode", GX_ENUM_TRIGGER_MODE);
    update_changed_enum_param("exposure_mode", GX_ENUM_EXPOSURE_MODE);
    // update_changed_float_param("exposure_time", GX_FLOAT_EXPOSURE_TIME);
    update_changed_enum_param("exposure_auto", GX_ENUM_EXPOSURE_AUTO);
    update_changed_float_param("auto_exposure_time_min", GX_FLOAT_AUTO_EXPOSURE_TIME_MIN);
    update_changed_float_param("auto_exposure_time_max", GX_FLOAT_AUTO_EXPOSURE_TIME_MAX);
    update_changed_int_param("expected_gray_value", GX_INT_GRAY_VALUE);
    update_changed_enum_param("acquisition_frame_rate_mode", GX_ENUM_ACQUISITION_FRAME_RATE_MODE);
    update_changed_float_param("acquisition_frame_rate", GX_FLOAT_ACQUISITION_FRAME_RATE);
    update_changed_float_param("current_acquisition_frame_rate", GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE);
    update_changed_float_param("gain", GX_FLOAT_GAIN);
    update_changed_enum_param("gain_auto", GX_ENUM_GAIN_AUTO);
    update_changed_float_param("auto_gain_min", GX_FLOAT_AUTO_GAIN_MIN);
    update_changed_float_param("auto_gain_max", GX_FLOAT_AUTO_GAIN_MAX);
    update_changed_enum_param("balance_ratio_selector", GX_ENUM_BALANCE_RATIO_SELECTOR);
    update_changed_float_param("balance_ratio", GX_FLOAT_BALANCE_RATIO);
    update_changed_enum_param("balance_white_auto", GX_ENUM_BALANCE_WHITE_AUTO);
    update_changed_enum_param("awb_lamp_house", GX_ENUM_AWB_LAMP_HOUSE);
    update_changed_bool_param("gamma_enable", GX_BOOL_GAMMA_ENABLE);
    update_changed_enum_param("gamma_mode", GX_ENUM_GAMMA_MODE);
    // update_changed_enum_param("saturation_mode", GX_ENUM_BALANCE_WHITE_AUTO);
  }

  CAMERA_LOCAL
  void update_changed_enum_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      int64_t p_value = param.as_int();
      int64_t gx_value = 0;
      status = GXGetEnum(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        // RCLCPP_INFO(get_logger(),"update_changed_enum_param %s %d p %ld gx %ld", param_name.c_str(), feature_id, p_value, gx_value );
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void update_changed_bool_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      bool p_value = param.as_bool();
      bool gx_value = true;
      status = GXGetBool(gx_dev_handle_, feature_id, &gx_value);
      GXSetBool(gx_dev_handle_, feature_id, true);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        // RCLCPP_INFO(get_logger(),"update_changed_enum_param %s %d p %ld gx %ld", param_name.c_str(), feature_id, p_value, gx_value );
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void update_changed_int_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      int64_t p_value = param.as_int();
      int64_t gx_value = 0;
      status = GXGetInt(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void update_changed_float_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      double p_value = param.as_double();
      double gx_value = 0.0;
      status = GXGetFloat(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  // Runs every 1 second. Computes the achieved fps over the last window and
  // uses it as a health signal. Because successful frames keep the rate up,
  // the normal timeout/success churn (status -14 then 0) does not trip this -
  // only a genuine stall (camera disconnected => stays at -14) drives fps to 0.
  CAMERA_LOCAL
  void frame_timer_callback() {
    auto end_time = std::chrono::steady_clock::now();
    auto frame_count = frame_count_;
    frame_count_ = 0;
    auto frame_time = frame_time_;
    frame_time_=end_time;

    auto elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-frame_time).count()/1000.0;
    auto fps = (elapsed_s > 0.0) ? (frame_count/elapsed_s) : 0.0;

    RCLCPP_DEBUG(this->get_logger(), "fps: %f", fps);

    // ---- health check ----
    // don't evaluate health while still initialising or already recovering,
    // nor while the camera is deliberately stopped -- zero fps is the point.
    if (is_initialising_ || reinit_in_progress_.load() || stopped_on_request_.load()) {
      unhealthy_window_count_ = 0;
      return;
    }

    // fast path: the SDK told us the device went offline
    if (device_offline_.load()) {
      RCLCPP_ERROR(get_logger(), "%s device reported OFFLINE - reinitialising", topic_.c_str());
      trigger_reinit();
      return;
    }

    // expected rate from the configured acquisition_frame_rate parameter
    double configured_fps = 0.0;
    if (this->has_parameter("acquisition_frame_rate")) {
      configured_fps = this->get_parameter("acquisition_frame_rate").as_double();
    }
    double min_healthy_fps = this->get_parameter("min_healthy_fps").as_double();
    int unhealthy_limit = static_cast<int>(this->get_parameter("unhealthy_windows_before_reinit").as_int());

    // only police fps if the camera is actually supposed to be streaming
    // continuously (in trigger mode there may legitimately be no frames)
    bool continuous = !trigger_mode();

    if (continuous && configured_fps > 0.0 && fps < min_healthy_fps) {
      unhealthy_window_count_++;
      RCLCPP_WARN(get_logger(),
        "%s low fps: measured %.2f < threshold %.2f (configured %.2f) [%d/%d]",
        topic_.c_str(), fps, min_healthy_fps, configured_fps,
        unhealthy_window_count_, unhealthy_limit);

      if (unhealthy_window_count_ >= unhealthy_limit) {
        RCLCPP_ERROR(get_logger(),
          "%s fps below %.2f for %d windows - reinitialising camera",
          topic_.c_str(), min_healthy_fps, unhealthy_window_count_);
        trigger_reinit();
      }
    } else {
      // healthy window resets the counter
      unhealthy_window_count_ = 0;
    }
  }

  CAMERA_LOCAL
  void stop_camera_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                      std_srvs::srv::Trigger::Response::SharedPtr res) {
    if (stopped_on_request_.load()) {
      res->success = true;
      res->message = "already stopped";
      return;
    }
    if (reinit_in_progress_.load()) {
      res->success = false;
      res->message = "reinitialising - try again shortly";
      return;
    }
    // Set this first: it is what stops the health check from reinitialising
    // the camera when the fps it is about to see drops to zero.
    stopped_on_request_.store(true);
    if (capture_timer_) {
      capture_timer_->cancel();
    }
    if (gx_dev_handle_ != NULL && streaming_) {
      GXStreamOff(gx_dev_handle_);
      streaming_ = false;
    }
    RCLCPP_INFO(get_logger(), "%s acquisition stopped on request", topic_.c_str());
    res->success = true;
    res->message = "acquisition stopped";
  }

  CAMERA_LOCAL
  void start_camera_cb(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr res) {
    if (!stopped_on_request_.load()) {
      res->success = true;
      res->message = "already running";
      return;
    }
    if (gx_dev_handle_ == NULL) {
      // The device went away while we were stopped; the reinit path owns
      // recovery, so hand it over rather than duplicating the reopen logic.
      stopped_on_request_.store(false);
      trigger_reinit();
      res->success = false;
      res->message = "device handle gone - reinitialising";
      return;
    }
    GX_STATUS status = GXStreamOn(gx_dev_handle_);
    if (status != GX_STATUS_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "GXStreamOn failed on start_camera: %s", GetErrorString(status));
      stopped_on_request_.store(false);
      trigger_reinit();
      res->success = false;
      res->message = "stream on failed - reinitialising";
      return;
    }
    streaming_ = true;
    // Give the health check a clean window rather than judging the camera on
    // the seconds it spent switched off.
    frame_count_ = 0;
    frame_time_ = std::chrono::steady_clock::now();
    unhealthy_window_count_ = 0;
    stopped_on_request_.store(false);
    if (capture_timer_) {
      capture_timer_->reset();
    }
    RCLCPP_INFO(get_logger(), "%s acquisition started on request", topic_.c_str());
    res->success = true;
    res->message = "acquisition started";
  }

  // schedule the reinit on a dedicated callback group so it can't run
  // re-entrantly with the capture timer and won't block other callbacks
  CAMERA_LOCAL
  void trigger_reinit() {
    bool expected = false;
    if (!reinit_in_progress_.compare_exchange_strong(expected, true)) {
      return; // already reinitialising
    }

    // stop capturing while we rebuild the device
    if (capture_timer_) {
      capture_timer_->cancel();
    }

    unhealthy_window_count_ = 0;

    // one-shot timer in the reinit callback group
    reinit_timer_ = this->create_wall_timer(
        200ms,
        [this]() {
          reinit_timer_->cancel();
          do_reinit();
        },
        callback_group_reinit_);
  }

  CAMERA_LOCAL
  void do_reinit() {
    RCLCPP_WARN(get_logger(), "reinitialising camera %s ...", device_sn_.c_str());

    // 1. tear down the current handle (ignore errors - device may be gone)
    unregister_offline_callback();
    if (gx_dev_handle_ != NULL) {
      if (streaming_) {
        GXStreamOff(gx_dev_handle_);
        streaming_ = false;
      }
      GXCloseDevice(gx_dev_handle_);
      gx_dev_handle_ = NULL;
    }
    device_offline_.store(false);

    // 2. refresh device list and reopen (by index, single camera)
    GX_STATUS status = GX_STATUS_ERROR;
    for (int attempt = 0; attempt < 1000 && rclcpp::ok(); ++attempt) {
      uint32_t num_devices = 0;
      GXUpdateDeviceList(&num_devices, 1000);
      if (num_devices > 0) {
        status = open_device();
        if (status == GX_STATUS_SUCCESS) {
          break;
        }
        RCLCPP_WARN(get_logger(), "reopen attempt %d failed: %s",
                    attempt, GetErrorString(status));
      } else {
        RCLCPP_WARN(get_logger(), "reopen attempt %d - no devices found yet", attempt);
      }
      std::this_thread::sleep_for(500ms);
    }

    if (status != GX_STATUS_SUCCESS || gx_dev_handle_ == NULL) {
      RCLCPP_ERROR(get_logger(), "could not reopen %s - retrying shortly", device_sn_.c_str());
      reinit_timer_ = this->create_wall_timer(
          2s,
          [this]() { reinit_timer_->cancel(); do_reinit(); },
          callback_group_reinit_);
      return;
    }

    // 3. refresh derived values and reapply all settings
    GXGetEnum(gx_dev_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter_);
    GXGetEnum(gx_dev_handle_, GX_ENUM_PIXEL_SIZE, &pixel_size_);
    reapply_camera_settings();

    // 4. re-register offline callback on the new handle
    register_offline_callback();

    // 5. restart the stream
    status = GXStreamOn(gx_dev_handle_);
    if (status != GX_STATUS_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "GXStreamOn failed after reinit: %s", GetErrorString(status));
      unregister_offline_callback();
      GXCloseDevice(gx_dev_handle_);
      gx_dev_handle_ = NULL;
      reinit_timer_ = this->create_wall_timer(
          2s,
          [this]() { reinit_timer_->cancel(); do_reinit(); },
          callback_group_reinit_);
      return;
    }
    streaming_ = true;

    // 6. resume normal operation
    frame_count_ = 0;
    frame_time_ = std::chrono::steady_clock::now();
    unhealthy_window_count_ = 0;
    reinit_in_progress_.store(false);
    if (capture_timer_) {
      capture_timer_->reset();
    }
    RCLCPP_INFO(get_logger(), "camera %s reinitialised successfully", device_sn_.c_str());
  }

  // CAMERA_LOCAL
  // void trigger_timer_callback() {
  //   this->trigger_timestamp_ = rclcpp::Clock().now();

  //   if(triggered_) {
  //     RCLCPP_DEBUG(this->get_logger(),"last trigger not finished triggered_: %d", triggered_);
  //     return;
  //   }

  //   // GX_STATUS status = GXSendCommand(this->gx_dev_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
  //   triggered_ = true;

  //   // if (status != GX_STATUS_SUCCESS) {
  //   //   auto error_msg = GetErrorString(status);
  //   //   RCLCPP_WARN(this->get_logger(), "unable to trigger gx_dev_handle_(%d): %s", this->gx_dev_handle_, error_msg);
  //   // }
  // }

  CAMERA_LOCAL
  bool trigger_mode() {
    auto param = get_parameter("trigger_mode");
    if (param.as_int() == GX_TRIGGER_MODE_OFF){
      return false;
    } else {
      return true;
    }
  }

  CAMERA_LOCAL
  void capture_trigger_callback(const sensor_msgs::msg::TimeReference::SharedPtr msg) {
    if (acquisition_role_.compare("follower") == 0) {
      RCLCPP_ERROR(this->get_logger(),"Unable to use software triggering on a camera with acquisition role follower!");
      return;
    }

    // don't trigger while reinitialising or with a dead handle
    if (reinit_in_progress_.load() || gx_dev_handle_ == NULL) {
      return;
    }

    // if it was started in continuous and we receive capture trigger message enable it
    if (!trigger_mode()) {
      set_parameter(rclcpp::Parameter("trigger_mode", GX_TRIGGER_MODE_ON));
      set_parameter(rclcpp::Parameter("trigger_source", GX_TRIGGER_SOURCE_SOFTWARE));
      // started in timer
      capture_timer_->cancel();
    }

    this->trigger_timestamp_ = msg->time_ref;
    this->trigger_source_ = msg->source;

    GX_STATUS status = GXSendCommand(this->gx_dev_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_WARN(this->get_logger(), "unable to trigger gx_dev_handle_(%p): %s", this->gx_dev_handle_, error_msg);
    }

    this->capture_device(topic_, gx_dev_handle_, RGB_image_buf_, image_buf_, color_filter_, payload_size_);
  }

  CAMERA_LOCAL
  void capture_timer_callback() {
    RCLCPP_INFO_ONCE(this->get_logger(),"capture_timer_callback started on thread: %s", string_thread_id().c_str());
    // skip if we're rebuilding the device or it's gone
    if (reinit_in_progress_.load() || gx_dev_handle_ == NULL) {
      return;
    }
    trigger_timestamp_ = rclcpp::Clock().now();
    this->capture_device(topic_, gx_dev_handle_, RGB_image_buf_, image_buf_, color_filter_, payload_size_);
  }

  CAMERA_LOCAL
  void capture_device(std::string topic, GX_DEV_HANDLE gx_dev_handle, u_char * RGB_image_buf, u_char * image_buf, int64_t color_filter, int64_t payload_size) {

    auto stamp = this->trigger_timestamp_;
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER frame_buffer = NULL;

    status = GXDQBuf(gx_dev_handle, &frame_buffer, 25);
    if (status == GX_STATUS_TIMEOUT) {
      // status -14: normal between frames at low fps. A sustained run of these
      // (camera disconnected) shows up as fps -> 0 in frame_timer_callback,
      // which triggers the reinit. So nothing to do here but return.
      RCLCPP_DEBUG(get_logger(), "%s timeout handle %p capture", topic.c_str(), gx_dev_handle);
      return;
    } else  if (status != GX_STATUS_SUCCESS) {
      // a hard (non-timeout) error almost always means the device is gone -
      // flag offline so the health check reinitialises promptly
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s error GXDQBuf: %s", topic.c_str(), error_msg);
      device_offline_.store(true);
      return;
    }

    if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
      // incomplete / invalid frame - link is still alive, just a bad frame.
      // Don't count it as a frame and don't requeue it as published.
      RCLCPP_WARN(get_logger(),"%s abnormal camera acquisition - code: %d", topic.c_str(), frame_buffer->nStatus);
    } else if (frame_buffer->nPixelFormat != GX_PIXEL_FORMAT_BAYER_RG8 && frame_buffer->nPixelFormat != GX_PIXEL_FORMAT_BAYER_RG10) {
      RCLCPP_ERROR(get_logger(),"%s unknown pixel format %d", topic.c_str(), frame_buffer->nPixelFormat);
    } else if (frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG8 && image_encoding_=="BAYER_RGGB16") {
      RCLCPP_ERROR(get_logger(),"image_encoding %s with pixel_format BAYER_RG8 capture not supported", image_encoding_.c_str());
    } else {

      RCLCPP_DEBUG(get_logger(), "%s handle %p capture frame_id: %ld timestamp: %lu.%.10lu", topic.c_str(), gx_dev_handle, frame_buffer->nFrameID, uint64_t(stamp.seconds()),stamp.nanoseconds());

      // Initialize a shared pointer to an Image message.
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      // msg->header.stamp = stamp + rclcpp::Duration(0,get_parameter("exposure_time").as_double()*1000);
      msg->header.stamp = rclcpp::Clock().now();
      double unix_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
      double now_nanosec = (msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec) - 70000000;
      msg->header.stamp.sec = static_cast<int32_t>(now_nanosec / 1e9);
      msg->header.stamp.nanosec = static_cast<uint32_t>(fmod(now_nanosec, 1e9));
      std::time_t time_t_timestamp = static_cast<std::time_t>(unix_timestamp);
      double fractional_seconds = unix_timestamp - static_cast<double>(time_t_timestamp);
      std::ostringstream timestamp_stream;
      timestamp_stream << std::put_time(std::localtime(&time_t_timestamp), "%Y-%m-%d_%H_%M_%S");
      int milliseconds = static_cast<int>(fractional_seconds * 1000);
      timestamp_stream << "." << std::setfill('0') << std::setw(3) << milliseconds;
      if (!trigger_source_.empty()) {
        msg->header.frame_id = trigger_source_;
      } else {
        msg->header.frame_id = device_sn_;
      }
      msg->is_bigendian = false;
      msg->height = frame_buffer->nHeight;
      msg->width = frame_buffer->nWidth;

      if (image_encoding_ == "RGB8") {
        // convert image to rgb8
        PixelFormatConvert(frame_buffer, color_filter, RGB_image_buf);

        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth * 3;
        msg->encoding = sensor_msgs::image_encodings::RGB8;
        msg->step = frame_buffer->nWidth*3;
        msg->data.resize(msg_size);
        memcpy(&msg->data[0],RGB_image_buf,msg_size);
      } else if (image_encoding_ == "BGR8") {
        // convert image to BGR8
        PixelFormatConvert(frame_buffer, GX_COLOR_FILTER_BAYER_BG, RGB_image_buf);
        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth * 3;
        msg->encoding = sensor_msgs::image_encodings::BGR8;
        msg->step = frame_buffer->nWidth*3;
        msg->data.resize(msg_size);
        memcpy(&msg->data[0],RGB_image_buf,msg_size);
      } else if (image_encoding_ == "BAYER_RGGB8") {
        msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
        msg->step = frame_buffer->nWidth;
        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth;
        if (frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG8) {
          msg->data.resize(msg_size);
          memcpy(&msg->data[0],frame_buffer->pImgBuf,msg_size);
        } else {
          Raw16toRaw8(frame_buffer, color_filter, image_buf);
          msg->data.resize(msg_size);
          memcpy(&msg->data[0], image_buf, msg_size);
        }
      } else if (image_encoding_ == "BAYER_RGGB16"
              && frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG10) {
        // Raw10PackedtoRaw16(frame_buffer, image_buf);
        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth * 2;
        msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
        msg->step = frame_buffer->nWidth*2;
        msg->data.resize(msg_size);
        // memcpy(&msg->data[0],image_buf,msg_size);
        memcpy(&msg->data[0],frame_buffer->pImgBuf,msg_size);
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s invalid encoding. Not publishing image!", image_encoding_.c_str());
        // requeue before returning so we don't leak the buffer
        GXQBuf(gx_dev_handle, frame_buffer);
        return;
      }
      frame_count_++;
      pub_->publish(*std::move(msg));
      pub_info_->publish(camera_info_);
    }

    status = GXQBuf(gx_dev_handle, frame_buffer);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s error GXQBuf: %s", topic.c_str(), error_msg);
    }
  }
};
} // end namespace camera

RCLCPP_COMPONENTS_REGISTER_NODE(camera::U3vImagePub)