/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_uav_hw_api/api.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/timer_handler.h>

//}

namespace mrs_uav_o3de_api
{

/* class Api //{ */

/**
 * @brief Connects the MRS UAV System to the O3DE multirotor simulator.
 *
 * The simulator only speaks standard messages, so the commands are translated here:
 *   attitude rate -> geometry_msgs/TwistStamped: twist.angular = body rate, twist.linear.z = throttle
 *   attitude      -> geometry_msgs/PoseStamped: pose.orientation = attitude, pose.position.z = throttle
 * The simulator cuts the throttle when the commands stop, so not forwarding them disarms the UAV.
 */
class Api : public mrs_uav_hw_api::MrsUavHwApi {

public:
  ~Api() {};

  void initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();

  // | --------------------- status methods --------------------- |

  mrs_msgs::msg::HwApiStatus       getStatus();
  mrs_msgs::msg::HwApiCapabilities getCapabilities();

  uint8_t airborne(const bool connected);

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool &request);
  std::tuple<bool, std::string> callbackOffboard(void);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_timers_;

  std::atomic<bool> is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  // | ------------------------- params ------------------------- |

  mrs_msgs::msg::HwApiCapabilities _capabilities_;

  double      _utm_x_;
  double      _utm_y_;
  std::string _utm_zone_;
  double      _amsl_;

  double _input_timeout_;

  std::string _world_frame_;
  std::string _body_frame_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odom_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>   sh_imu_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range> sh_range_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Bool>     sh_on_ground_;

  void callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackRangefinder(const sensor_msgs::msg::Range::ConstSharedPtr msg);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<geometry_msgs::msg::TwistStamped> ph_attitude_rate_cmd_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseStamped>  ph_attitude_cmd_;

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<mrs_lib::ThreadTimer> timer_main_;

  void timerMain();

  // | ------------------------ variables ----------------------- |

  std::mutex   mutex_status_;
  bool         armed_    = false;
  bool         offboard_ = false;
  rclcpp::Time last_cmd_time_;

  // | ------------------------- methods ------------------------ |

  bool updateCmdTime(void);
  void publishPositionData(const nav_msgs::msg::Odometry &odom);
  void publishBatteryState(void);
  void publishMagneticField(void);
  void publishRC(void);
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void Api::initialize(const rclcpp::Node::SharedPtr &node, std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {

  node_            = node;
  clock_           = node_->get_clock();
  common_handlers_ = common_handlers;

  cbgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const std::string uav_name = common_handlers_->getUavName();

  _world_frame_ = uav_name + "/" + common_handlers_->getWorldFrameName();
  _body_frame_  = uav_name + "/" + common_handlers_->getBodyFrameName();

  last_cmd_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------- loading parameters ------------------- |

  std::shared_ptr<mrs_lib::ParamLoader> param_loader = common_handlers_->plugin_param_loader;

  param_loader->loadParam("input_timeout", _input_timeout_);

  param_loader->loadParam("gnss/utm_x", _utm_x_);
  param_loader->loadParam("gnss/utm_y", _utm_y_);
  param_loader->loadParam("gnss/utm_zone", _utm_zone_);
  param_loader->loadParam("gnss/amsl", _amsl_);

  if (!param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ---------------------- capabilities ---------------------- |

  // all the outputs are implemented, but the simulator takes only the attitude and attitude rate commands
  _capabilities_          = common_handlers_->requested_capabilities;
  _capabilities_.api_name = "O3DE";

  _capabilities_.accepts_actuator_cmd              = false;
  _capabilities_.accepts_control_group_cmd         = false;
  _capabilities_.accepts_acceleration_hdg_rate_cmd = false;
  _capabilities_.accepts_acceleration_hdg_cmd      = false;
  _capabilities_.accepts_velocity_hdg_rate_cmd     = false;
  _capabilities_.accepts_velocity_hdg_cmd          = false;
  _capabilities_.accepts_position_cmd              = false;
  _capabilities_.accepts_feedforward               = false;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.node_name                           = "O3deHwApi";
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbgrp_subs_;

  sh_odom_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/simulator_odom_in", &Api::callbackOdom, this);

  sh_on_ground_ = mrs_lib::SubscriberHandler<std_msgs::msg::Bool>(shopts, "~/simulator_on_ground_in");

  if (_capabilities_.produces_imu) {
    sh_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/simulator_imu_in", &Api::callbackImu, this);
  }

  if (_capabilities_.produces_distance_sensor) {
    sh_range_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>(shopts, "~/simulator_rangefinder_in", &Api::callbackRangefinder, this);
  }

  // | ----------------------- publishers ----------------------- |

  if (_capabilities_.accepts_attitude_rate_cmd) {
    ph_attitude_rate_cmd_ = mrs_lib::PublisherHandler<geometry_msgs::msg::TwistStamped>(node_, "~/simulator_attitude_rate_cmd_out");
  }

  if (_capabilities_.accepts_attitude_cmd) {
    ph_attitude_cmd_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseStamped>(node_, "~/simulator_attitude_cmd_out");
  }

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions opts;

  opts.node           = node_;
  opts.autostart      = true;
  opts.callback_group = cbgrp_timers_;

  timer_main_ = std::make_shared<mrs_lib::ThreadTimer>(opts, rclcpp::Rate(10.0, clock_), std::bind(&Api::timerMain, this));

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* destroy() //{ */

void Api::destroy() {

  timer_main_->stop();
}

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus Api::getStatus() {

  mrs_msgs::msg::HwApiStatus status;

  const bool    connected = sh_odom_.hasMsg();
  const uint8_t on_air    = airborne(connected);

  std::scoped_lock lock(mutex_status_);

  // stamped under the mutex: a status not newer than the last one is dropped, so it must not outrun the state it carries
  status.stamp     = clock_->now();
  status.connected = connected;
  status.armed     = armed_;
  status.offboard  = offboard_;
  status.mode      = offboard_ ? "OFFBOARD" : "NORMAL";
  status.airborne  = on_air;

  return status;
}

//}

/* airborne() //{ */

uint8_t Api::airborne(const bool connected) {

  // the simulator does not publish it while it holds the UAV for the ground to appear
  if (!connected || !sh_on_ground_.hasMsg()) {
    return mrs_msgs::msg::HwApiStatus::AIRBORNE_UNKNOWN;
  }

  return sh_on_ground_.getMsg()->data ? mrs_msgs::msg::HwApiStatus::AIRBORNE_NO : mrs_msgs::msg::HwApiStatus::AIRBORNE_YES;
}

//}

/* getCapabilities() //{ */

mrs_msgs::msg::HwApiCapabilities Api::getCapabilities() {

  _capabilities_.stamp = clock_->now();

  return _capabilities_;
}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> Api::callbackArming(const bool &request) {

  std::scoped_lock lock(mutex_status_);

  armed_ = request;

  // stopping the commands is what cuts the motors in the simulator
  if (!request) {
    offboard_ = false;
  }

  const std::string message = request ? "armed" : "disarmed";

  RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());

  return {true, message};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> Api::callbackOffboard(void) {

  std::scoped_lock lock(mutex_status_);

  if (!armed_) {
    return {false, "Cannot switch to offboard, not armed."};
  }

  if ((clock_->now() - last_cmd_time_).seconds() > _input_timeout_) {
    return {false, "Cannot switch to offboard, missing control input."};
  }

  offboard_ = true;

  RCLCPP_INFO(node_->get_logger(), "offboard set");

  return {true, "Offboard set"};
}

//}

// | --------------------- input callbacks -------------------- |

/* updateCmdTime() //{ */

// @return true if the command should be passed to the simulator
bool Api::updateCmdTime(void) {

  std::scoped_lock lock(mutex_status_);

  last_cmd_time_ = clock_->now();

  return offboard_;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool Api::callbackAttitudeRateCmd(const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  if (updateCmdTime()) {

    geometry_msgs::msg::TwistStamped cmd;

    cmd.header.stamp    = msg->stamp;
    cmd.header.frame_id = _body_frame_;
    cmd.twist.angular   = msg->body_rate;
    cmd.twist.linear.z  = msg->throttle;

    ph_attitude_rate_cmd_.publish(cmd);
  }

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool Api::callbackAttitudeCmd(const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {

  if (!_capabilities_.accepts_attitude_cmd) {
    return false;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  if (updateCmdTime()) {

    geometry_msgs::msg::PoseStamped cmd;

    cmd.header.stamp     = msg->stamp;
    cmd.header.frame_id  = _world_frame_;
    cmd.pose.orientation = msg->orientation;
    cmd.pose.position.z  = msg->throttle;

    ph_attitude_cmd_.publish(cmd);
  }

  return true;
}

//}

/* unsupported inputs //{ */

bool Api::callbackActuatorCmd([[maybe_unused]] const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackControlGroupCmd([[maybe_unused]] const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackAccelerationHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackAccelerationHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackVelocityHdgRateCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackVelocityHdgCmd([[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg) {
  return false;
}

bool Api::callbackPositionCmd([[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg) {
  return false;
}

void Api::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOdom() //{ */

void Api::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting simulator odometry");

  // the frames are the simulator's own, the MRS UAV System expects its names
  nav_msgs::msg::Odometry odom = *msg;

  odom.header.frame_id = _world_frame_;
  odom.child_frame_id  = _body_frame_;

  common_handlers_->publishers.publishStatus(getStatus());

  if (_capabilities_.produces_position) {

    geometry_msgs::msg::PointStamped position;

    position.header = odom.header;
    position.point  = odom.pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  if (_capabilities_.produces_orientation) {

    geometry_msgs::msg::QuaternionStamped orientation;

    orientation.header     = odom.header;
    orientation.quaternion = odom.pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  if (_capabilities_.produces_velocity) {

    geometry_msgs::msg::Vector3Stamped velocity;

    velocity.header.stamp    = odom.header.stamp;
    velocity.header.frame_id = _body_frame_;
    velocity.vector          = odom.twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  if (_capabilities_.produces_angular_velocity) {

    geometry_msgs::msg::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp    = odom.header.stamp;
    angular_velocity.header.frame_id = _body_frame_;
    angular_velocity.vector          = odom.twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }

  if (_capabilities_.produces_odometry) {
    common_handlers_->publishers.publishOdometry(odom);
  }

  if (_capabilities_.produces_ground_truth) {
    common_handlers_->publishers.publishGroundTruth(odom);
  }

  publishPositionData(odom);

  if (_capabilities_.produces_magnetometer_heading) {

    mrs_msgs::msg::Float64Stamped heading;

    heading.header.stamp = odom.header.stamp;

    try {
      heading.value = mrs_lib::AttitudeConverter(odom.pose.pose.orientation).getHeading();
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
      RCLCPP_WARN(node_->get_logger(), "exception caught: '%s'", e.what());
      return;
    }

    common_handlers_->publishers.publishMagnetometerHeading(heading);
  }
}

//}

/* callbackImu() //{ */

void Api::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {

  if (!is_initialized_ || !_capabilities_.produces_imu) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting IMU");

  sensor_msgs::msg::Imu imu = *msg;

  imu.header.frame_id = _body_frame_;

  common_handlers_->publishers.publishIMU(imu);
}

//}

/* callbackRangefinder() //{ */

void Api::callbackRangefinder(const sensor_msgs::msg::Range::ConstSharedPtr msg) {

  if (!is_initialized_ || !_capabilities_.produces_distance_sensor) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting rangefinder");

  common_handlers_->publishers.publishDistanceSensor(*msg);
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Api::timerMain() {

  if (!is_initialized_) {
    return;
  }

  publishBatteryState();

  publishRC();

  publishMagneticField();

  std::scoped_lock lock(mutex_status_);

  if (offboard_ && (clock_->now() - last_cmd_time_).seconds() > _input_timeout_) {

    RCLCPP_WARN(node_->get_logger(), "control input timed out, leaving offboard");

    offboard_ = false;
  }
}

//}

// | ------------------------- methods ------------------------ |

/* publishPositionData() //{ */

void Api::publishPositionData(const nav_msgs::msg::Odometry &odom) {

  const double amsl = odom.pose.pose.position.z + _amsl_;

  if (_capabilities_.produces_altitude) {

    mrs_msgs::msg::HwApiAltitude altitude;

    altitude.stamp = odom.header.stamp;
    altitude.amsl  = amsl;

    common_handlers_->publishers.publishAltitude(altitude);
  }

  if (_capabilities_.produces_gnss_status) {

    mrs_msgs::msg::GpsInfo gnss_status;

    gnss_status.stamp              = odom.header.stamp;
    gnss_status.fix_type           = mrs_msgs::msg::GpsInfo::GPS_FIX_TYPE_3D_FIX;
    gnss_status.satellites_visible = 12;
    gnss_status.h_acc              = 0.5;
    gnss_status.v_acc              = 0.8;

    common_handlers_->publishers.publishGNSSStatus(gnss_status);
  }

  if (!_capabilities_.produces_gnss && !_capabilities_.produces_rtk) {
    return;
  }

  double lat;
  double lon;

  mrs_lib::UTMtoLL(odom.pose.pose.position.y + _utm_y_, odom.pose.pose.position.x + _utm_x_, _utm_zone_, lat, lon);

  if (_capabilities_.produces_gnss) {

    sensor_msgs::msg::NavSatFix gnss;

    gnss.header.stamp    = odom.header.stamp;
    gnss.header.frame_id = _body_frame_;
    gnss.latitude        = lat;
    gnss.longitude       = lon;
    gnss.altitude        = amsl;

    gnss.position_covariance[0]   = 0.5;  // east variance [m^2]
    gnss.position_covariance[4]   = 0.5;  // north variance [m^2]
    gnss.position_covariance[8]   = 1.5;  // up variance [m^2]
    gnss.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    common_handlers_->publishers.publishGNSS(gnss);
  }

  if (_capabilities_.produces_rtk) {

    mrs_msgs::msg::RtkGps rtk;

    rtk.header.stamp      = odom.header.stamp;
    rtk.gps.latitude      = lat;
    rtk.gps.longitude     = lon;
    rtk.gps.altitude      = amsl;
    rtk.fix_type.fix_type = mrs_msgs::msg::RtkFixType::RTK_FIX;

    common_handlers_->publishers.publishRTK(rtk);
  }
}

//}

/* publishBatteryState() //{ */

void Api::publishBatteryState(void) {

  if (_capabilities_.produces_battery_state) {

    sensor_msgs::msg::BatteryState msg;

    msg.capacity = 100;
    msg.current  = 10.0;
    msg.voltage  = 15.8;
    msg.charge   = 0.8;

    common_handlers_->publishers.publishBatteryState(msg);
  }
}

//}

/* publishMagneticField() //{ */

void Api::publishMagneticField(void) {

  if (_capabilities_.produces_magnetic_field) {

    sensor_msgs::msg::MagneticField msg;

    msg.header.stamp    = clock_->now();
    msg.header.frame_id = _body_frame_;

    // roughly the Earth's magnetic field in central Europe, fixed in the body frame as in the MRS multirotor simulator [T]
    msg.magnetic_field.x = 2.1e-5;
    msg.magnetic_field.y = 0.1e-5;
    msg.magnetic_field.z = 4.3e-5;

    msg.magnetic_field_covariance = {1e-12, 0.0, 0.0, 0.0, 1e-12, 0.0, 0.0, 0.0, 1e-12};

    common_handlers_->publishers.publishMagneticField(msg);
  }
}

//}

/* publishRC() //{ */

void Api::publishRC(void) {

  if (_capabilities_.produces_rc_channels) {

    mrs_msgs::msg::HwApiRcChannels rc;

    rc.stamp    = clock_->now();
    rc.channels = std::vector<double>(8, 0.0);

    common_handlers_->publishers.publishRcChannels(rc);
  }

  if (_capabilities_.produces_rc_rssi) {

    mrs_msgs::msg::HwApiRcRssi rssi;

    rssi.stamp = clock_->now();
    rssi.rssi  = 0;

    common_handlers_->publishers.publishRcRssi(rssi);
  }
}

//}

}  // namespace mrs_uav_o3de_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_o3de_api::Api, mrs_uav_hw_api::MrsUavHwApi)
