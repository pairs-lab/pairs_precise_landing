/* include //{ */

#include <memory>
#include <map>
#include <optional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <pairs_lib/lkf.h>
#include <pairs_lib/param_loader.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/geometry/cyclic.h>
#include <pairs_lib/geometry/misc.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/subscriber_handler.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/node.h>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>

//}

namespace pairs_landing_pad_estimation
{

/* using //{ */

using vec2_t = pairs_lib::geometry::vec_t<2>;
using vec3_t = pairs_lib::geometry::vec_t<3>;

using radians  = pairs_lib::geometry::radians;
using sradians = pairs_lib::geometry::sradians;

//}

/* defines //{ */

#define STATE_X 0
#define STATE_Y 1
#define STATE_Z 2
#define STATE_HEADING 3

//}

/* LKF helpers //{ */

// Define the LKF we will be using
const int _n_states_       = 4;
const int _n_inputs_       = 0;
const int _n_measurements_ = 4;

using lkf_t = pairs_lib::LKF<_n_states_, _n_inputs_, _n_measurements_>;

using A_t        = lkf_t::A_t;
using B_t        = lkf_t::B_t;
using H_t        = lkf_t::H_t;
using Q_t        = lkf_t::Q_t;
using x_t        = lkf_t::x_t;
using P_t        = lkf_t::P_t;
using R_t        = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class LandingPadEstimation() //{ */

class LandingPadEstimation : public pairs_lib::Node {

public:
  LandingPadEstimation(rclcpp::NodeOptions options);

  bool is_initialized_ = false;

  void iterate(const double dt);
  void publish(void);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void initialize();

  // params
  double           _prediction_rate_;
  std::string      _uav_name_;
  std::vector<int64_t> _tag_ids_;
  std::string      _estimation_frame_;
  std::string      _full_estimation_frame_;
  std::string      _body_frame_;
  std::string      _full_body_frame_;
  double           _correction_timeout_;
  double           max_relative_distance_;
  bool             _autoprefix_uav_name_;

  double relative_x, relative_y, relative_z;
  double relative_roll, relative_pitch, relative_yaw;

  std::unique_ptr<pairs_lib::Transformer> transformer_;

  pairs_lib::SubscriberHandler<apriltag_msgs::msg::AprilTagDetectionArray> sh_tag_detections_;

  pairs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped> ph_pose_;
  pairs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped> ph_measurement_;

  void callbackTagDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg);

  // lkf matrices
  A_t A_;
  R_t R_;
  Q_t Q_;
  H_t H_;
  B_t B_;

  std::unique_ptr<lkf_t> lkf_;

  std::optional<statecov_t> statecov_;
  rclcpp::Time              time_last_correction_;
  std::mutex                mutex_statecov_;

  rclcpp::TimerBase::SharedPtr timer_main_;
  rclcpp::Time                 timer_main_last_time_;
  void                         timerMain();
};

//}

/* LandingPadEstimation() //{ */

LandingPadEstimation::LandingPadEstimation(rclcpp::NodeOptions options) : pairs_lib::Node("LandingPadEstimation", options) {
  node_  = this_node_ptr();
  clock_ = node_->get_clock();
  initialize();
}

//}

/* initialize() //{ */

void LandingPadEstimation::initialize() {

  // | ----------------------- load params ---------------------- |

  pairs_lib::ParamLoader param_loader(node_, "LandingPadEstimation");

  param_loader.loadParam("prediction_rate", _prediction_rate_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("tag_ids", _tag_ids_);
  param_loader.loadParam("estimation_frame", _estimation_frame_);
  param_loader.loadParam("body_frame", _body_frame_);
  param_loader.loadParam("correction_timeout", _correction_timeout_);
  param_loader.loadParam("max_relative_distance", max_relative_distance_);
  param_loader.loadParam("transformer/autoprefix_uav_name", _autoprefix_uav_name_);

  param_loader.loadParam("relative_transform/translation/x", relative_x);
  param_loader.loadParam("relative_transform/translation/y", relative_y);
  param_loader.loadParam("relative_transform/translation/z", relative_z);

  param_loader.loadParam("relative_transform/rotation/roll", relative_roll);
  param_loader.loadParam("relative_transform/rotation/pitch", relative_pitch);
  param_loader.loadParam("relative_transform/rotation/yaw", relative_yaw);

  // state matrix
  param_loader.loadMatrixStatic("lkf/A", A_);

  // input matrix
  param_loader.loadMatrixStatic("lkf/B", B_);

  // measurement noise
  param_loader.loadMatrixStatic("lkf/R", R_);

  // process covariance
  param_loader.loadMatrixStatic("lkf/Q", Q_);

  // measurement mapping
  param_loader.loadMatrixStatic("lkf/H", H_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[LandingPadEstimation]: Could not load all parameters!");
    rclcpp::shutdown();
    return;
  }

  _full_estimation_frame_ = _uav_name_ + "/" + _estimation_frame_;
  _full_body_frame_       = _uav_name_ + "/" + _body_frame_;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscriberHandlerOptions shopts;
  shopts.node      = node_;
  shopts.node_name = "LandingPadEstimation";

  sh_tag_detections_ = pairs_lib::SubscriberHandler<apriltag_msgs::msg::AprilTagDetectionArray>(shopts, "~/tag_detections_in",
                                                                                                &LandingPadEstimation::callbackTagDetections, this);

  // | ----------------------- publishers ----------------------- |

  ph_pose_        = pairs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(node_, "~/estimated_pose_out");
  ph_measurement_ = pairs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(node_, "~/measurement_pose_out");

  // | ----------------------- transfomer ----------------------- |

  transformer_ = std::make_unique<pairs_lib::Transformer>(node_);

  if (_autoprefix_uav_name_) {
    transformer_->setDefaultPrefix(_uav_name_);
  }

  transformer_->retryLookupNewest(true);

  // | --------------------------- lkf -------------------------- |

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | ------------------------- timers ------------------------- |

  timer_main_last_time_ = clock_->now();

  timer_main_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / _prediction_rate_)),
                                         std::bind(&LandingPadEstimation::timerMain, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "[LandingPadEstimation]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackTagDetections() //{ */

void LandingPadEstimation::callbackTagDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "[LandingPadEstimation]: receiving detections");

  // | ----------------- retrive the tag's pose ----------------- |

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> tag_pose;

  // TODO(ros2 apriltag): verify pose source. The ROS1 apriltag_ros::AprilTagDetection carried a full
  // PoseWithCovarianceStamped (PnP result). On Jazzy, apriltag_msgs::msg::AprilTagDetection only carries
  // {id, centre, corners} and NO pose. The apriltag detector node on ROS2 instead broadcasts a tf for each
  // detected tag (typically frame "<family>:<id>", e.g. "tag36h11:0"). We therefore preserve the downstream
  // estimation pipeline unchanged and recover each detected tag's pose by looking it up over tf into the
  // detection-array's header frame. Adjust the tag-frame naming convention below to match the detector config.
  std::map<int, apriltag_msgs::msg::AprilTagDetection> detection_map;

  for (auto tag : msg->detections) {
    detection_map.insert(std::pair(tag.id, tag));
  }

  for (auto desired_id : _tag_ids_) {
    if (detection_map.find(desired_id) != detection_map.end()) {

      const std::string camera_frame = msg->header.frame_id;

      // TODO(ros2 apriltag): confirm the tag frame naming used by the apriltag detector (family:id).
      const std::string tag_frame = "tag36h11:" + std::to_string(desired_id);

      // build an identity pose in the tag frame and transform it into the camera/detection frame
      geometry_msgs::msg::PoseWithCovarianceStamped tag_identity;
      tag_identity.header.frame_id      = tag_frame;
      tag_identity.header.stamp         = msg->header.stamp;
      tag_identity.pose.pose.position.x = 0.0;
      tag_identity.pose.pose.position.y = 0.0;
      tag_identity.pose.pose.position.z = 0.0;
      tag_identity.pose.pose.orientation.w = 1.0;

      auto tf_result = transformer_->transformSingle(tag_identity, camera_frame);

      if (!tf_result) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandingPadEstimation]: could not look up the pose of tag '%s'", tag_frame.c_str());
        break;
      }

      tag_pose = tf_result.value();
      break;
    }
  }

  if (!tag_pose) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandingPadEstimation]: tags with the right ids not found");
    return;
  }

  // | ------------------- check for outliers ------------------- |

  {
    auto result = transformer_->transformSingle(tag_pose.value(), _full_body_frame_);

    if (!result) {
      RCLCPP_ERROR(node_->get_logger(), "[LandingPadEstimation]: could not transform the tag detection to '%s'", _full_body_frame_.c_str());
      return;
    }

    if (std::hypot(result->pose.pose.position.x, result->pose.pose.position.y, result->pose.pose.position.z) > max_relative_distance_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandingPadEstimation]: detection too far from the UAV");
      return;
    }
  }

  // | ------------------ publish for debugging ----------------- |

  ph_measurement_.publish(tag_pose.value());

  // | ------------------------- offset ------------------------- |

  {
    geometry_msgs::msg::TransformStamped offset_tf;
    offset_tf.header                  = tag_pose.value().header;
    offset_tf.transform.rotation      = pairs_lib::AttitudeConverter(relative_roll, relative_pitch, relative_yaw);
    offset_tf.transform.translation.x = relative_x;
    offset_tf.transform.translation.y = relative_y;
    offset_tf.transform.translation.z = relative_z;

    Eigen::Isometry3d offset_eig = tf2::transformToEigen(offset_tf);

    geometry_msgs::msg::TransformStamped tag_tf;
    tag_tf.header                  = tag_pose.value().header;
    tag_tf.transform.rotation      = tag_pose.value().pose.pose.orientation;
    tag_tf.transform.translation.x = tag_pose.value().pose.pose.position.x;
    tag_tf.transform.translation.y = tag_pose.value().pose.pose.position.y;
    tag_tf.transform.translation.z = tag_pose.value().pose.pose.position.z;

    Eigen::Isometry3d tag_eig = tf2::transformToEigen(tag_tf);

    Eigen::Isometry3d prod = tag_eig * offset_eig;

    Eigen::Affine3d affine(prod);

    auto tf = tf2::eigenToTransform(affine);

    tag_pose.value().pose.pose.position.x  = tf.transform.translation.x;
    tag_pose.value().pose.pose.position.y  = tf.transform.translation.y;
    tag_pose.value().pose.pose.position.z  = tf.transform.translation.z;
    tag_pose.value().pose.pose.orientation = tf.transform.rotation;
  }

  // | ------------------- transform the pose ------------------- |

  auto result = transformer_->transformSingle(tag_pose.value(), _full_estimation_frame_);

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "[LandingPadEstimation]: could not transform the tag detection to '%s'", _full_estimation_frame_.c_str());
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped tag_world_ = result.value();

  RCLCPP_INFO_ONCE(node_->get_logger(), "[LandingPadEstimation]: receiving the right AprilTag");

  // | -------------------------- fuse -------------------------- |

  auto statecov = pairs_lib::get_mutexed(mutex_statecov_, statecov_);

  if (!statecov) {

    statecov = statecov_t();

    statecov->x << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z,
        pairs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading();

    statecov->P = P_t::Identity();

    RCLCPP_INFO(node_->get_logger(), "[LandingPadEstimation]: statecov initialized");
  }

  // create the measurement vector
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(_n_measurements_);

  double mes_heading = sradians::unwrap(pairs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading(), statecov->x[STATE_HEADING]);

  measurement << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z, mes_heading;

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[LandingPadEstimation]: measurement: " << measurement.transpose());

  try {
    statecov = lkf_->correct(*statecov, measurement, R_);

    statecov->stamp = tag_world_.header.stamp;
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "[LandingPadEstimation]: correction step failed");
    return;
  }

  rclcpp::Time time_last_correction = clock_->now();

  RCLCPP_DEBUG(node_->get_logger(), "[LandingPadEstimation]: correct: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2],
               statecov->x[3]);

  {
    std::scoped_lock lock(mutex_statecov_);

    statecov_             = statecov;
    time_last_correction_ = time_last_correction;
  }
}

//}

// --------------------------------------------------------------
// |                           models                           |
// --------------------------------------------------------------

/* publish() //{ */

void LandingPadEstimation::publish() {

  auto [statecov, time_last_correction] = pairs_lib::get_mutexed(mutex_statecov_, statecov_, time_last_correction_);

  if (!statecov) {
    return;
  }

  if (time_last_correction.nanoseconds() == 0 || (clock_->now() - time_last_correction).seconds() > _correction_timeout_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[LandingPadEstimation]: landing pad detections timeouted");

    {
      std::scoped_lock lock(mutex_statecov_);
      time_last_correction_ = rclcpp::Time(0);
      statecov_             = {};
    }

    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  pose.header.frame_id = _uav_name_ + "/" + _estimation_frame_;
  pose.header.stamp    = statecov->stamp;

  pose.pose.pose.position.x  = statecov->x[STATE_X];
  pose.pose.pose.position.y  = statecov->x[STATE_Y];
  pose.pose.pose.position.z  = statecov->x[STATE_Z];
  pose.pose.pose.orientation = pairs_lib::AttitudeConverter(0, 0, 0).setHeading(statecov->x[STATE_HEADING]);

  ph_pose_.publish(pose);
}

//}

/* iterate() //{ */

void LandingPadEstimation::iterate(const double dt) {

  auto statecov = pairs_lib::get_mutexed(mutex_statecov_, statecov_);

  if (!statecov) {
    return;
  }

  if (dt < 0.001 || dt > 1.0) {
    return;
  }

  try {
    statecov = lkf_->predict(*statecov, Eigen::VectorXd::Zero(_n_inputs_), Q_, dt);

    statecov->stamp = clock_->now();
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "[LandingPadEstimation]: prediction step failed");
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "[LandingPadEstimation]: predict: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2],
               statecov->x[3]);

  pairs_lib::set_mutexed(mutex_statecov_, statecov, statecov_);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void LandingPadEstimation::timerMain() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "[LandingPadEstimation]: timerMain() spinning");

  const rclcpp::Time now = clock_->now();
  const double       dt  = (now - timer_main_last_time_).seconds();
  timer_main_last_time_  = now;

  iterate(dt);

  publish();
}

//}

}  // namespace pairs_landing_pad_estimation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pairs_landing_pad_estimation::LandingPadEstimation)
