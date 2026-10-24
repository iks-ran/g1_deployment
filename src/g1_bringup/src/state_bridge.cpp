// Bridge Unitree G1 topics to standard ROS 2 topics for robot_state_publisher / RViz.
//
// - /lowstate (unitree_hg/LowState, ~1 kHz)            -> /joint_states (29 joints, throttled)
// - /odommodestate (unitree_go/SportModeState, 500 Hz) -> TF odom -> pelvis, and /g1/odom (throttled)
// - /utlidar/cloud_livox_mid360 (frame livox_frame)    -> /g1/cloud (robot stamp -> host clock)
// - /utlidar/imu_livox_mid360 (EMPTY frame_id)         -> /g1/imu (robot stamp -> host clock, frame livox_imu)
//
// The cloud (~20k points, ~440 KB) is read with a Reliable subscription: a best-effort reader loses the whole
// sample when one UDP fragment is dropped (measured on a loopback bag replay: 0 Hz best effort vs 9.3 Hz reliable).
// The robot offers it Reliable. Readers of /g1/cloud should subscribe Reliable too (RViz config does).
//
// Clocks: the robot's clock is not synchronized with the host. Lidar and IMU keep their robot stamps, converted to
// host time with an offset tracked from the 200 Hz lidar IMU (ClockOffset). Stamping on receipt would be wrong for
// the cloud: its stamp is the scan START (per-point `time` runs 0..100 ms), ~100 ms before it arrives.
// /lowstate and /odommodestate carry no usable stamp, so joint states and odom TF are stamped on receipt.
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

#include "g1_bringup/clock_offset.hpp"

namespace g1_bringup
{

namespace
{

// LowState.motor_state index order on the G1 (29 DoF) = actuated-joint order of g1_29dof_rev_1_0.urdf.
std::vector<std::string> joint_names()
{
  std::vector<std::string> names;
  for (const char * side : {"left", "right"}) {
    for (const char * j : {"hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"}) {
      names.push_back(std::string(side) + "_" + j + "_joint");
    }
  }
  for (const char * j : {"waist_yaw", "waist_roll", "waist_pitch"}) {
    names.push_back(std::string(j) + "_joint");
  }
  for (const char * side : {"left", "right"}) {
    for (const char * j : {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
        "wrist_yaw"})
    {
      names.push_back(std::string(side) + "_" + j + "_joint");
    }
  }
  return names;
}

// Rotate vector v by the inverse of unit quaternion (w, x, y, z): world -> body.
std::array<double, 3> rotate_inverse(double w, double x, double y, double z, const std::array<double, 3> & v)
{
  const double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
  const double r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
  const double r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);
  return {r00 * v[0] + r10 * v[1] + r20 * v[2],
    r01 * v[0] + r11 * v[1] + r21 * v[2],
    r02 * v[0] + r12 * v[1] + r22 * v[2]};
}

int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

class StateBridge : public rclcpp::Node
{
public:
  explicit StateBridge(const rclcpp::NodeOptions & options)
  : Node("g1_state_bridge", options),
    joint_names_(joint_names()),
    js_period_ns_(period_ns(declare_parameter("joint_state_rate", 100.0))),
    odom_period_ns_(period_ns(declare_parameter("odom_rate", 100.0))),
    odom_frame_(declare_parameter("odom_frame", "odom")),
    base_frame_(declare_parameter("base_frame", "pelvis")),
    imu_frame_(declare_parameter("imu_frame", "livox_imu")),
    clock_offset_(declare_parameter("clock_offset_window", 2.0))
  {
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("g1/odom", 10);
    // Reliable publishers: compatible with both reliable (RViz default) and best-effort readers.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("g1/cloud", 5);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("g1/imu", 5);
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
      "lowstate", rclcpp::SensorDataQoS(),
      [this](unitree_hg::msg::LowState::ConstSharedPtr msg) {on_lowstate(*msg);});
    odom_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      "odommodestate", rclcpp::SensorDataQoS(),
      [this](unitree_go::msg::SportModeState::ConstSharedPtr msg) {on_odom(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "utlidar/cloud_livox_mid360", rclcpp::QoS(2).reliable(),
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {on_cloud(std::move(msg));});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "utlidar/imu_livox_mid360", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {on_imu(std::move(msg));});
  }

private:
  static int64_t period_ns(double rate) {return rate > 0 ? static_cast<int64_t>(1e9 / rate) : 0;}

  void on_imu(sensor_msgs::msg::Imu::UniquePtr msg)
  {
    const int64_t receive_ns = now().nanoseconds();
    const int64_t stamp = stamp_ns(msg->header.stamp);
    int64_t host_ns = receive_ns;
    if (stamp != 0) {
      clock_offset_.add(receive_ns, stamp);
      host_ns = *clock_offset_.to_host_ns(stamp);
      log_offset();
    }
    msg->header.stamp = rclcpp::Time(host_ns, RCL_ROS_TIME);
    // The MID-360 IMU publishes no frame_id and no orientation (all-zero quaternion); acceleration is
    // in g (Livox convention, as FAST-LIO etc. expect), angular velocity in rad/s. Kept as is.
    msg->header.frame_id = imu_frame_;
    msg->orientation_covariance[0] = -1.0;  // REP-145: orientation not provided
    imu_pub_->publish(std::move(msg));
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    auto host_ns = clock_offset_.to_host_ns(stamp_ns(msg->header.stamp));
    if (!host_ns) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "dropping cloud: no clock offset yet (no /utlidar/imu_livox_mid360)");
      return;
    }
    msg->header.stamp = rclcpp::Time(*host_ns, RCL_ROS_TIME);
    cloud_pub_->publish(std::move(msg));
  }

  void log_offset()
  {
    const int64_t off = *clock_offset_.offset_ns();
    if (!logged_offset_ns_ || std::llabs(off - *logged_offset_ns_) > 50000000LL) {
      RCLCPP_INFO(get_logger(), "%s host - robot = %+.3f s",
        logged_offset_ns_ ? "clock offset JUMPED to" : "clock offset", off * 1e-9);
      logged_offset_ns_ = off;
    }
  }

  void on_lowstate(const unitree_hg::msg::LowState & msg)
  {
    const rclcpp::Time t = now();
    if (t.nanoseconds() - last_js_ns_ < js_period_ns_) {
      return;
    }
    last_js_ns_ = t.nanoseconds();
    auto js = std::make_unique<sensor_msgs::msg::JointState>();
    js->header.stamp = t;
    js->name = joint_names_;
    const size_t n = joint_names_.size();
    js->position.resize(n);
    js->velocity.resize(n);
    js->effort.resize(n);
    for (size_t i = 0; i < n; ++i) {
      js->position[i] = msg.motor_state[i].q;
      js->velocity[i] = msg.motor_state[i].dq;
      js->effort[i] = msg.motor_state[i].tau_est;
    }
    js_pub_->publish(std::move(js));
  }

  void on_odom(const unitree_go::msg::SportModeState & msg)
  {
    const rclcpp::Time t = now();
    if (t.nanoseconds() - last_odom_ns_ < odom_period_ns_) {
      return;
    }
    const auto & q = msg.imu_state.quaternion;  // (w, x, y, z)
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-6) {
      return;  // estimator not running yet
    }
    last_odom_ns_ = t.nanoseconds();
    const double w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;

    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = t;
    odom->header.frame_id = odom_frame_;
    odom->child_frame_id = base_frame_;
    auto & p = odom->pose.pose.position;
    p.x = msg.position[0];
    p.y = msg.position[1];
    p.z = msg.position[2];
    auto & o = odom->pose.pose.orientation;
    o.w = w;
    o.x = x;
    o.y = y;
    o.z = z;
    // SportModeState.velocity is in the odom frame; nav_msgs twist is in the child frame.
    const auto lin = rotate_inverse(w, x, y, z, {msg.velocity[0], msg.velocity[1], msg.velocity[2]});
    odom->twist.twist.linear.x = lin[0];
    odom->twist.twist.linear.y = lin[1];
    odom->twist.twist.linear.z = lin[2];
    odom->twist.twist.angular.x = msg.imu_state.gyroscope[0];
    odom->twist.twist.angular.y = msg.imu_state.gyroscope[1];
    odom->twist.twist.angular.z = msg.imu_state.gyroscope[2];

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom->header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = p.x;
    tf.transform.translation.y = p.y;
    tf.transform.translation.z = p.z;
    tf.transform.rotation = o;
    odom_pub_->publish(std::move(odom));
    tf_br_->sendTransform(tf);
  }

  const std::vector<std::string> joint_names_;
  const int64_t js_period_ns_;
  const int64_t odom_period_ns_;
  const std::string odom_frame_;
  const std::string base_frame_;
  const std::string imu_frame_;
  ClockOffset clock_offset_;
  std::optional<int64_t> logged_offset_ns_;
  int64_t last_js_ns_{0};
  int64_t last_odom_ns_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace g1_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(g1_bringup::StateBridge)
