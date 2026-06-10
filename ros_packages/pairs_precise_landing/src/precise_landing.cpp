/* include //{ */

#include <memory>
#include <mutex>
#include <atomic>
#include <optional>
#include <string>
#include <sstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/median_filter.h>
#include <pairs_lib/geometry/misc.h>
#include <pairs_lib/geometry/cyclic.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/subscriber_handler.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/service_client_handler.h>
#include <pairs_lib/service_server_handler.h>
#include <pairs_lib/node.h>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pairs_msgs/msg/trajectory_reference.hpp>
#include <pairs_msgs/msg/tracker_command.hpp>
#include <pairs_msgs/msg/uav_state.hpp>
#include <pairs_msgs/msg/reference.hpp>
#include <pairs_msgs/msg/reference_stamped.hpp>
#include <pairs_msgs/msg/control_manager_diagnostics.hpp>
#include <pairs_msgs/srv/string.hpp>
#include <pairs_msgs/srv/path_srv.hpp>
#include <pairs_msgs/srv/float64_stamped_srv.hpp>

//}

using namespace Eigen;

namespace pairs_precise_landing
{

/* using //{ */

using vec2_t = pairs_lib::geometry::vec_t<2>;
using vec3_t = pairs_lib::geometry::vec_t<3>;

using radians  = pairs_lib::geometry::radians;
using sradians = pairs_lib::geometry::sradians;

//}

/* STRUCTURES and TYPEDEFS //{ */

[[maybe_unused]] enum {

  IDLE_STATE,
  ALIGN_STATE,
  DESCEND_STATE,
  ALIGN2_STATE,
  LANDING_STATE,
  REPEAT_STATE,
  ABORT_STATE,

} States_t;

const char *state_names[7] = {
    "IDLING", "ALIGNING", "DESCENDING", "ALIGNING2", "LANDING", "REPEATING", "ABORTING",
};

[[maybe_unused]] enum {

  DESCEND_TRAJECTORY,
  LANDING_TRAJECTORY,
  REPEAT_TRAJECTORY,
  ABORT_TRAJECTORY,

} Trajectory_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class PreciseLanding //{ */

class PreciseLanding : public pairs_lib::Node {

public:
  PreciseLanding(rclcpp::NodeOptions options);

  bool is_initialized_ = false;

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void initialize();

  std::string _uav_name_;

  std::string _frame_id_;

  std::unique_ptr<pairs_lib::Transformer> transformer_;

  pairs_lib::PublisherHandler<pairs_msgs::msg::TrajectoryReference> ph_trajectory_reference_;

  pairs_lib::SubscriberHandler<pairs_msgs::msg::TrackerCommand>                   sh_tracker_cmd_;
  pairs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped>     sh_landing_pad_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::UavState>                         sh_uav_state_;
  pairs_lib::SubscriberHandler<std_msgs::msg::Float64>                            sh_mass_estimate_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::ControlManagerDiagnostics>        sh_ctrl_diag_;

  void callbackTimeoutPad(const std::string &topic_name, const rclcpp::Time &last_msg);

  void callbackLandingPad(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger> service_server_land_;
  pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger> service_server_stop_;

  pairs_lib::ServiceClientHandler<pairs_msgs::srv::String>            sch_switch_controller_;
  pairs_lib::ServiceClientHandler<pairs_msgs::srv::String>            sch_switch_tracker_;
  pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_arming_;
  pairs_lib::ServiceClientHandler<pairs_msgs::srv::Float64StampedSrv> sch_set_min_z;
  pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_enable_min_height_check_;
  pairs_lib::ServiceClientHandler<pairs_msgs::srv::PathSrv>           sch_path_;

  // params loaded from config file
  double _trajectory_dt_;

  double inital_mass_estimate_;

  std::atomic<bool> see_landing_pad_ = false;

  rclcpp::Time timeouter_;

  std::string controller_;
  std::string tracker_;

  bool   _heading_relative_to_pad_enabled_;
  double _heading_relative_to_pad_;

  // aligning params
  double _aligning_speed_;
  double _aligning_height_;
  double _aligning_radius_;
  double _aligning_timeout_;

  // descending params
  double _descending_speed_;
  double _descending_timeout_;
  double _descending_height_;

  // aligning2 params
  double _aligning2_timeout_;

  double _aligning2_criterion_initial_radius_;
  double _aligning2_criterion_radius_increase_rate_;
  double _aligning2_criterion_radius_limit_;

  double _aligning2_in_alignment_duration_;

  rclcpp::Time aligning2_in_radius_time_;
  bool         aligning2_in_radius_ = false;
  double       aligning2_current_radius_;

  // landing params
  rclcpp::Time landing_since_;

  double _landing_speed_;
  double _landing_height_;
  double _landing_mass_factor_;
  int    _landing_repeat_threshold_;
  bool   _landing_disarming_vision_enabled_;
  double _landing_disarming_vision_distance_;

  // repeating params
  double _repeating_speed_;
  double _repeating_height_;
  double _repeating_timeout_;

  // aborting params
  double aborting_height_;

  bool callbackLand(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool callbackAbort(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool setController(const std::string &desired_controller);

  bool setTracker(const std::string &desired_tracker);

  void disarm(void);

  bool setMinZ(const double state);

  bool enableMinHeightCheck(const bool state);

  void gotoPath(const double x, const double y, const double z, const double hdg, const std::string &frame);

  bool alignmentCheck(const double &desired_height, const double &position_hor_thr, const double &position_vert_thr, const double &heading_thr);

  bool shouldTimeout(const double &timeout);

  std::optional<pairs_msgs::msg::ReferenceStamped> getTransformedTrackerCmd(const std::string &frame_id);

  std::optional<pairs_msgs::msg::ReferenceStamped> getTransformedLandingPad(const std::string &frame_id);

  std::optional<pairs_msgs::msg::ReferenceStamped> getTransformedUavState(const std::string &frame_id);

  // state machine
  int        current_state_, previous_state_;
  std::mutex mutex_state_;

  int repeat_landing_counter;

  rclcpp::TimerBase::SharedPtr state_machine_timer_;
  void                         stateMachineTimer();

public:
  double _main_rate_;

  void changeState(int newState);

  std::optional<pairs_msgs::msg::TrajectoryReference> createTrajectory(int trajectoryType);
};

//}

/* PreciseLanding() //{ */

PreciseLanding::PreciseLanding(rclcpp::NodeOptions options) : pairs_lib::Node("PreciseLanding", options) {
  node_  = this_node_ptr();
  clock_ = node_->get_clock();
  initialize();
}

//}

/* initialize() //{ */

void PreciseLanding::initialize() {

  pairs_lib::ParamLoader param_loader(node_, "PreciseLanding");

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("frame_id", _frame_id_);

  param_loader.loadParam("rate", _main_rate_);

  param_loader.loadParam("trajectory_dt", _trajectory_dt_);

  param_loader.loadParam("controller", controller_);
  param_loader.loadParam("tracker", tracker_);

  param_loader.loadParam("desired_heading/relative_to_pad/enabled", _heading_relative_to_pad_enabled_);
  param_loader.loadParam("desired_heading/relative_to_pad/heading", _heading_relative_to_pad_);

  // aligning params
  param_loader.loadParam("stages/aligning/speed", _aligning_speed_);
  param_loader.loadParam("stages/aligning/height", _aligning_height_);
  param_loader.loadParam("stages/aligning/radius", _aligning_radius_);
  param_loader.loadParam("stages/aligning/timeout", _aligning_timeout_);

  // descending params
  param_loader.loadParam("stages/descending/speed", _descending_speed_);
  param_loader.loadParam("stages/descending/timeout", _descending_timeout_);
  param_loader.loadParam("stages/descending/height", _descending_height_);

  // aligning2 params
  param_loader.loadParam("stages/aligning2/timeout", _aligning2_timeout_);

  param_loader.loadParam("stages/aligning2/criterion/initial_radius", _aligning2_criterion_initial_radius_);
  param_loader.loadParam("stages/aligning2/criterion/radius_increase_rate", _aligning2_criterion_radius_increase_rate_);
  param_loader.loadParam("stages/aligning2/criterion/limit_radius", _aligning2_criterion_radius_limit_);

  param_loader.loadParam("stages/aligning2/in_alignment_duration", _aligning2_in_alignment_duration_);

  // landing params
  param_loader.loadParam("stages/landing/speed", _landing_speed_);
  param_loader.loadParam("stages/landing/height", _landing_height_);
  param_loader.loadParam("stages/landing/disarming/mass_factor", _landing_mass_factor_);
  param_loader.loadParam("stages/landing/repeat_threshold", _landing_repeat_threshold_);
  param_loader.loadParam("stages/landing/disarming/vision/enabled", _landing_disarming_vision_enabled_);
  param_loader.loadParam("stages/landing/disarming/vision/distance", _landing_disarming_vision_distance_);

  // repeating params
  param_loader.loadParam("stages/repeating/speed", _repeating_speed_);
  param_loader.loadParam("stages/repeating/height", _repeating_height_);
  param_loader.loadParam("stages/repeating/timeout", _repeating_timeout_);

  // aborting params
  param_loader.loadParam("stages/aborting/height", aborting_height_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: Could not load all parameters!");
    rclcpp::shutdown();
    return;
  }

  transformer_ = std::make_unique<pairs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- publishers ----------------------- |

  ph_trajectory_reference_ = pairs_lib::PublisherHandler<pairs_msgs::msg::TrajectoryReference>(node_, "~/trajectory_reference_out");

  // | --------------------- service clients -------------------- |

  sch_switch_controller_       = pairs_lib::ServiceClientHandler<pairs_msgs::srv::String>(node_, "~/switch_controller_out");
  sch_switch_tracker_          = pairs_lib::ServiceClientHandler<pairs_msgs::srv::String>(node_, "~/switch_tracker_out");
  sch_arming_                  = pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/arming_out");
  sch_set_min_z                = pairs_lib::ServiceClientHandler<pairs_msgs::srv::Float64StampedSrv>(node_, "~/set_min_z_out");
  sch_enable_min_height_check_ = pairs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/enable_min_height_check_out");
  sch_path_                    = pairs_lib::ServiceClientHandler<pairs_msgs::srv::PathSrv>(node_, "~/path_out");

  // | --------------------- service servers -------------------- |

  service_server_land_ = pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/land_in", std::bind(&PreciseLanding::callbackLand, this, std::placeholders::_1, std::placeholders::_2));
  service_server_stop_ = pairs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_, "~/abort_in", std::bind(&PreciseLanding::callbackAbort, this, std::placeholders::_1, std::placeholders::_2));

  // | ----------------------- subscribers ---------------------- |

  {
    pairs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node_;
    shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = rclcpp::Duration::from_seconds(1.0);
    shopts.threadsafe         = true;
    shopts.autostart          = true;

    sh_landing_pad_ = pairs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(
        shopts, "~/landing_pad_in", &PreciseLanding::callbackLandingPad, this, &PreciseLanding::callbackTimeoutPad, this);
  }

  {
    pairs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node_;
    shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = pairs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;

    sh_tracker_cmd_   = pairs_lib::SubscriberHandler<pairs_msgs::msg::TrackerCommand>(shopts, "~/tracker_cmd_in");
    sh_uav_state_     = pairs_lib::SubscriberHandler<pairs_msgs::msg::UavState>(shopts, "~/uav_state_in");
    sh_mass_estimate_ = pairs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "~/mass_estimate_in");
    sh_ctrl_diag_     = pairs_lib::SubscriberHandler<pairs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/ctrl_diag_in");
  }

  // state machine
  current_state_  = IDLE_STATE;
  previous_state_ = IDLE_STATE;

  // | ------------------------- timers ------------------------- |

  // start timers
  state_machine_timer_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / _main_rate_)),
                                                  std::bind(&PreciseLanding::stateMachineTimer, this));

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackLandingPad() //{ */

void PreciseLanding::callbackLandingPad([[maybe_unused]] const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "[PreciseLanding]: getting landing pad pose");

  see_landing_pad_ = true;
}

//}

/* callbackTimeoutPad() //{ */

void PreciseLanding::callbackTimeoutPad([[maybe_unused]] const std::string &topic_name, [[maybe_unused]] const rclcpp::Time &last_msg) {

  see_landing_pad_ = false;
}

//}

// | ------------------------ services ------------------------ |

/* callbackAbort() //{ */

bool PreciseLanding::callbackAbort([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   const std::shared_ptr<std_srvs::srv::Trigger::Response>                  res) {

  res->success = true;
  res->message = "aborting landing";

  {
    std::scoped_lock lock(mutex_state_);

    changeState(ABORT_STATE);
  }

  return true;
}

//}

/* callbackLand() //{ */

bool PreciseLanding::callbackLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  const std::shared_ptr<std_srvs::srv::Trigger::Response>                  res) {

  {
    std::stringstream ss;

    if (!sh_uav_state_.hasMsg()) {
      ss << "missing UAV state";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!sh_landing_pad_.hasMsg()) {
      ss << "missing landing pad detections";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "missing tracker cmd";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!sh_mass_estimate_.hasMsg()) {
      ss << "missing estimated mass";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!sh_ctrl_diag_.hasMsg()) {
      ss << "missing control manager diagnostics";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!sh_ctrl_diag_.getMsg()->flying_normally) {
      ss << "not flying normally";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }

    if (!see_landing_pad_) {
      ss << "landing pad not detected";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: " << ss.str());
      res->message = ss.str();
      res->success = false;
      return true;
    }
  }

  {
    std::scoped_lock lock(mutex_state_);

    if (current_state_ == IDLE_STATE) {

      changeState(ALIGN_STATE);

      res->success = true;
      res->message = "landing has started";

    } else {

      res->success = false;
      res->message = "landing already in process";
    }
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void PreciseLanding::changeState(int newState) {

  // just for the info log
  RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: Switching states: %s -> %s", state_names[current_state_], state_names[newState]);

  previous_state_ = current_state_;
  current_state_  = newState;

  timeouter_ = clock_->now();

  // if changing to idle, stop the drone
  switch (newState) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      repeat_landing_counter = 0;

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      if (!setController(controller_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to switch controller");
        changeState(IDLE_STATE);
        return;
      }

      if (!setTracker(tracker_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to switch tracker");
        changeState(IDLE_STATE);
        return;
      }

      if (!enableMinHeightCheck(false)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to disable the min height check");
        changeState(IDLE_STATE);
        return;
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      auto mass_estimate = sh_mass_estimate_.getMsg();

      inital_mass_estimate_ = mass_estimate->data;

      break;
    }

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE: {

      aligning2_in_radius_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      aligning2_in_radius_      = false;
      aligning2_current_radius_ = _aligning2_criterion_initial_radius_;

      break;
    }

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE: {

      // log when we started the landing
      landing_since_ = clock_->now();

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      if (repeat_landing_counter++ >= _landing_repeat_threshold_) {

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: Exceeded the number of landing attempts, aborting");

        changeState(ABORT_STATE);
      }

      break;
    }

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE: {

      RCLCPP_DEBUG(node_->get_logger(), "[PreciseLanding]: aborting");

      break;
    }

      //}
  }
}

//}

/* createTrajectory() //{ */

std::optional<pairs_msgs::msg::TrajectoryReference> PreciseLanding::createTrajectory(int trajectoryType) {

  auto tracker_cmd = getTransformedTrackerCmd(_frame_id_);
  auto landing_pad = getTransformedLandingPad(_frame_id_);

  if (!tracker_cmd || !landing_pad) {
    return {};
  }

  double init_x   = tracker_cmd->reference.position.x;
  double init_y   = tracker_cmd->reference.position.y;
  double init_z   = tracker_cmd->reference.position.z;
  double init_hdg = tracker_cmd->reference.heading;

  double landing_pad_x   = landing_pad->reference.position.x;
  double landing_pad_y   = landing_pad->reference.position.y;
  double landing_pad_z   = landing_pad->reference.position.z;
  double landing_pad_hdg = landing_pad->reference.heading;

  // prepare the trajectory
  pairs_msgs::msg::TrajectoryReference trajectory;
  trajectory.fly_now     = true;
  trajectory.use_heading = true;
  trajectory.header      = tracker_cmd->header;

  /* DESCEND_TRAJECTORY //{ */

  if (trajectoryType == DESCEND_TRAJECTORY) {

    double target_distance, direction, desired_height, desired_vector, desired_heading;

    desired_height  = landing_pad_z + _descending_height_;
    desired_vector  = desired_height - init_z;
    target_distance = std::abs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double step_size = _descending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;

      point.heading = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      pairs_msgs::msg::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* LANDING_TRAJECTORY //{ */

  } else if (trajectoryType == LANDING_TRAJECTORY) {

    double desired_heading;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double target_distance = init_z - landing_pad_z - _landing_height_;
    double direction       = -1;
    double step_size       = _landing_speed_ * _trajectory_dt_;
    int    n_steps         = int(floor(target_distance / step_size));

    // the first point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      pairs_msgs::msg::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z + _landing_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* REPEAT_TRAJECTORY //{ */

  } else if (trajectoryType == REPEAT_TRAJECTORY) {

    double target_distance, desired_height, desired_vector, direction, desired_heading;

    desired_height  = landing_pad_z + _repeating_height_;
    desired_vector  = desired_height - init_z;
    target_distance = std::abs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double step_size = _repeating_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      pairs_msgs::msg::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      pairs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* ABORT_TRAJECTORY //{ */

  } else if (trajectoryType == ABORT_TRAJECTORY) {

    pairs_msgs::msg::Reference point;

    point.position.x = init_x;
    point.position.y = init_y;
    point.position.z = landing_pad_z + aborting_height_;
    point.heading    = init_hdg;

    trajectory.points.push_back(point);

    return trajectory;

    //}
  }

  return trajectory;
}

//}

// | ------------------------- setters ------------------------ |

/* setController() //{ */

bool PreciseLanding::setController(const std::string &desired_controller) {

  auto request   = std::make_shared<pairs_msgs::srv::String::Request>();
  request->value = desired_controller;

  RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: switching to controller: \"%s\"", desired_controller.c_str());

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_switch_controller_.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for setController() returned false: %s",
                           (*res)->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for setController() failed!");
    return false;
  }

  return true;
}

//}

/* setTracker() //{ */

bool PreciseLanding::setTracker(const std::string &desired_tracker) {

  auto request   = std::make_shared<pairs_msgs::srv::String::Request>();
  request->value = desired_tracker;

  RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: switching to tracker: \"%s\"", desired_tracker.c_str());

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_switch_tracker_.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for setTracker() returned false: %s", (*res)->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for setTracker() failed!");
    return false;
  }

  return true;
}

//}

/* disarm() //{ */

void PreciseLanding::disarm(void) {

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_arming_.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for disarm() returned false: %s", (*res)->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for disarm() failed!");
  }
}

//}

/* setMinZ() //{ */

bool PreciseLanding::setMinZ(const double z) {

  auto request                 = std::make_shared<pairs_msgs::srv::Float64StampedSrv::Request>();
  request->header.frame_id     = _frame_id_;
  request->value               = z;

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_set_min_z.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for setMinZ() returned false: %s", (*res)->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for setMinZ() failed!");
    return false;
  }

  return true;
}

//}

/* enableMinHeightCheck() //{ */

bool PreciseLanding::enableMinHeightCheck(const bool state) {

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: %s min height check", state ? "enabling" : "disabling");

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_enable_min_height_check_.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for enableMinHeightCheck() returned false: %s",
                           (*res)->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for enableMinHeightCheck() failed!");
    return false;
  }

  return true;
}

//}

/* gotoPath() //{ */

void PreciseLanding::gotoPath(const double x, const double y, const double z, const double hdg, const std::string &frame) {

  auto request                                      = std::make_shared<pairs_msgs::srv::PathSrv::Request>();
  request->path.fly_now                             = true;
  request->path.header.frame_id                     = frame;
  request->path.max_execution_time                  = 0.9;
  request->path.max_deviation_from_path             = 0.5;
  request->path.use_heading                         = true;

  request->path.override_constraints                = true;
  request->path.override_max_acceleration_vertical  = 2.0;
  request->path.override_max_acceleration_horizontal = 2.0;
  request->path.override_max_velocity_vertical      = _aligning_speed_;
  request->path.override_max_velocity_horizontal    = _aligning_speed_;
  request->path.override_max_jerk_vertical          = 20.0;
  request->path.override_max_jerk_horizontal        = 20.0;

  pairs_msgs::msg::Reference point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  point.heading    = hdg;

  request->path.points.push_back(point);

  // TODO(ros2): verify ServiceClientHandler API
  auto res = sch_path_.callSync(request);

  if (res) {
    if (!(*res)->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: service call for gotoPath() returned false: %s", (*res)->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: service call for gotoPath() failed!");
  }
}

//}

// | -------------------- support routines -------------------- |

/* shouldTimeout() //{ */

bool PreciseLanding::shouldTimeout(const double &timeout) {

  if (timeouter_.nanoseconds() == 0) {
    return false;
  }

  if ((clock_->now() - timeouter_).seconds() > timeout) {
    return true;
  } else {
    return false;
  }
}

//}

/* getTransformedTrackerCmd() //{ */

std::optional<pairs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedTrackerCmd([[maybe_unused]] const std::string &frame_id) {

  if (!sh_tracker_cmd_.hasMsg()) {
    return {};
  }

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  pairs_msgs::msg::ReferenceStamped out;

  {
    out.header             = tracker_cmd->header;
    out.reference.position = tracker_cmd->position;
    out.reference.heading  = tracker_cmd->heading;

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      out = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: could not transform tracker cmd to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* getTransformedLandingPad() //{ */

std::optional<pairs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedLandingPad([[maybe_unused]] const std::string &frame_id) {

  if (!sh_landing_pad_.hasMsg()) {
    return {};
  }

  auto landing_pad = sh_landing_pad_.getMsg();

  pairs_msgs::msg::ReferenceStamped out;

  {
    out.header             = landing_pad->header;
    out.reference.position = landing_pad->pose.pose.position;
    out.reference.heading  = pairs_lib::AttitudeConverter(landing_pad->pose.pose.orientation).getHeading();

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      out = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: could not transform landing pad to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* getTransformedUavState() //{ */

std::optional<pairs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedUavState([[maybe_unused]] const std::string &frame_id) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  pairs_msgs::msg::ReferenceStamped out;

  {
    out.header             = uav_state->header;
    out.reference.position = uav_state->pose.position;
    out.reference.heading  = pairs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      out = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: could not transform UAV state to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* alignmentCheck() //{ */

bool PreciseLanding::alignmentCheck(const double &desired_height, const double &position_hor_thr, const double &position_vert_thr, const double &heading_thr) {

  auto uav_state   = getTransformedUavState(_frame_id_);
  auto landing_pad = getTransformedLandingPad(_frame_id_);

  if (!landing_pad || !uav_state) {
    return false;
  }

  double tar_x, tar_y, tar_z, tar_heading;
  double cur_x, cur_y, cur_z, cur_heading;

  tar_x = landing_pad->reference.position.x;
  tar_y = landing_pad->reference.position.y;
  tar_z = landing_pad->reference.position.z + desired_height;

  cur_x       = uav_state->reference.position.x;
  cur_y       = uav_state->reference.position.y;
  cur_z       = uav_state->reference.position.z;
  cur_heading = uav_state->reference.heading;

  if (_heading_relative_to_pad_enabled_) {
    tar_heading = landing_pad->reference.heading + _heading_relative_to_pad_;
  } else {
    tar_heading = cur_heading;
  }

  double horizontal_error = std::hypot(cur_x - tar_x, cur_y - tar_y);
  double vertical_error   = std::abs(cur_z - tar_z);
  double heading_error    = std::abs(radians::diff(cur_heading, tar_heading));

  if (horizontal_error < position_hor_thr && vertical_error < position_vert_thr && heading_error < heading_thr) {
    return true;
  } else {
    return false;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* stateMachineTimer() //{ */

void PreciseLanding::stateMachineTimer() {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_state_);

  RCLCPP_INFO_ONCE(node_->get_logger(), "[PreciseLanding]: stateMachineTimer() running");

  // | ------- check for non normal flying for all states ------- |

  if (current_state_ != IDLE_STATE) {

    auto ctrl_diag = sh_ctrl_diag_.getMsg();

    if (!ctrl_diag->flying_normally) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: not flying normally, aborting");

      changeState(IDLE_STATE);

      return;
    }
  }

  // | ------------------- state machine logic ------------------ |

  switch (current_state_) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      if (shouldTimeout(_aligning_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: landing timed out, ABORTING");

        changeState(ABORT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);
      auto tracker_cmd = getTransformedTrackerCmd(_frame_id_);

      double des_x = landing_pad->reference.position.x;
      double des_y = landing_pad->reference.position.y;
      double des_z = landing_pad->reference.position.z + _aligning_height_;
      double des_heading;

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      if (_heading_relative_to_pad_enabled_) {
        des_heading = landing_pad->reference.heading + _heading_relative_to_pad_;
      } else {
        des_heading = tracker_cmd->reference.heading;
      }

      gotoPath(des_x, des_y, des_z, des_heading, _frame_id_);

      if (alignmentCheck(_aligning_height_, _aligning_radius_, 0.2, 0.1)) {

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: aligned with the landing pad, DESCENDING");

        changeState(DESCEND_STATE);

        return;
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      if (shouldTimeout(_descending_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: timed out, re-ALIGNING");

        changeState(REPEAT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      // publish the trajectory
      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      if (alignmentCheck(_descending_height_, 0.1, 0.2, 0.1)) {

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: correct height reached, ALIGNING for landing");

        changeState(ALIGN2_STATE);

        return;
      }

      break;
    }

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE: {

      if (shouldTimeout(_aligning2_timeout_)) {

        RCLCPP_WARN(node_->get_logger(), "[PreciseLanding]: Aligning for landing took too long, ABORTING.");

        changeState(ABORT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[PreciseLanding]: alignment radius criterion: %.1f cm", aligning2_current_radius_ * 100.0);

      if (alignmentCheck(_descending_height_, aligning2_current_radius_, 0.1, 0.1)) {

        if (!aligning2_in_radius_) {

          aligning2_in_radius_      = true;
          aligning2_in_radius_time_ = clock_->now();
        }

      } else {

        aligning2_current_radius_ += (1.0 / _main_rate_) * _aligning2_criterion_radius_increase_rate_;

        if (aligning2_current_radius_ > _aligning2_criterion_radius_limit_) {
          aligning2_current_radius_ = _aligning2_criterion_radius_limit_;
        }

        if (aligning2_in_radius_) {

          aligning2_in_radius_      = false;
          aligning2_in_radius_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
          RCLCPP_WARN(node_->get_logger(), "[PreciseLanding]: alignment disturbed");
        }
      }

      // | ---------------- check the alignment time ---------------- |

      if (aligning2_in_radius_) {

        double alignemnt_held_for = (clock_->now() - aligning2_in_radius_time_).seconds();

        if (alignemnt_held_for > _aligning2_in_alignment_duration_) {

          RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: alignment finished, LANDING");

          changeState(LANDING_STATE);

          return;

        } else {

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "[PreciseLanding]: alignment holds for %.2f/%.2f s", alignemnt_held_for,
                               _aligning2_in_alignment_duration_);
        }
      }

      break;
    }

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE: {

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(LANDING_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      auto estimated_mass = sh_mass_estimate_.getMsg();

      if (estimated_mass->data < (_landing_mass_factor_ * inital_mass_estimate_)) {

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: disarming using mass estimator threshold");

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: landing finished");

        disarm();

        changeState(IDLE_STATE);

        return;
      }

      if (_landing_disarming_vision_enabled_ && alignmentCheck(0.0, 0.1, _landing_disarming_vision_distance_, 1.0)) {

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: disarming using vision-based threshold");

        RCLCPP_INFO(node_->get_logger(), "[PreciseLanding]: landing finished");

        disarm();

        changeState(IDLE_STATE);

        return;
      }

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      if (shouldTimeout(_repeating_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "[PreciseLanding]: repeating timed out, ABORTING");

        changeState(ABORT_STATE);

        return;
      }

      auto trajectory = createTrajectory(REPEAT_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      // | -------------------- check the height -------------------- |
      if (alignmentCheck(_repeating_height_, 0.2, 0.2, 0.1)) {

        changeState(ALIGN_STATE);

        return;
      }

      break;
    }

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE: {

      auto trajectory = createTrajectory(ABORT_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      changeState(IDLE_STATE);

      break;
    }

      //}
  }
}

//}

}  // namespace pairs_precise_landing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pairs_precise_landing::PreciseLanding)
