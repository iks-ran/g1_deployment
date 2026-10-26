// Motion compensation of a MID-360 scan: rotation from the lidar's own gyro, translation from leg odometry.
//
// The robot publishes the cloud raw: points are in the lidar frame at their own measurement time (per-point `time`,
// ns after the header stamp = scan start, 0..100 ms). Like FAST-LIO2 (UndistortPcl), every point is moved into the
// lidar frame at the scan END (stamp + max per-point time):
//   p_end = R_end<-t p + t_end(t)
// R_end<-t = G(T)^T G(t), G = Exp(w dt) integrated from the gyro (body rates; the MID-360 IMU axes are the lidar
// axes). t_end(t) = position of the lidar origin at time t in the scan-end lidar frame, relative to the scan end
// (FAST-LIO2 integrates it from its filter velocity + acceleration; here it comes from leg odometry via TF). Without
// it only rotation is compensated (an IMU alone cannot give velocity): <= v * 0.1 s, ~7 cm at 0.73 m/s.
// Gyro and cloud stamps must be on the same clock (both are robot stamps). Output fields: x y z intensity (float32).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace g1_bringup
{

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major

class GyroBuffer
{
public:
  explicit GyroBuffer(int64_t keep_ns)
  : keep_ns_(keep_ns) {}

  void add(int64_t t_ns, const Vec3 & w)
  {
    if (!samples_.empty() && t_ns <= samples_.back().t) {
      return;  // out of order / duplicate
    }
    samples_.push_back({t_ns, w});
    while (samples_.front().t < t_ns - keep_ns_) {
      samples_.pop_front();
    }
  }

  bool empty() const {return samples_.empty();}
  int64_t first_ns() const {return samples_.front().t;}
  int64_t last_ns() const {return samples_.back().t;}

  // Angular velocity at t: linear interpolation, held constant beyond the ends. Requires !empty().
  Vec3 at(int64_t t_ns) const
  {
    auto it = std::lower_bound(samples_.begin(), samples_.end(), t_ns,
        [](const Sample & s, int64_t t) {return s.t < t;});
    if (it == samples_.begin()) {
      return it->w;
    }
    if (it == samples_.end()) {
      return samples_.back().w;
    }
    const auto & b = *it;
    const auto & a = *(it - 1);
    const double f = static_cast<double>(t_ns - a.t) / static_cast<double>(b.t - a.t);
    return {a.w[0] + f * (b.w[0] - a.w[0]), a.w[1] + f * (b.w[1] - a.w[1]), a.w[2] + f * (b.w[2] - a.w[2])};
  }

private:
  struct Sample
  {
    int64_t t;
    Vec3 w;
  };
  int64_t keep_ns_;
  std::deque<Sample> samples_;
};

// t_end(s) sampled every step_s from the scan start (s = 0); linear interpolation, held beyond the last sample.
struct Translation
{
  double step_s{0.005};
  std::vector<Vec3> t;

  Vec3 at(double s) const
  {
    const double x = std::max(0.0, s / step_s);
    const size_t i = std::min(t.size() - 1, static_cast<size_t>(x));
    if (i + 1 >= t.size()) {
      return t.back();
    }
    const double f = x - static_cast<double>(i);
    return {t[i][0] + f * (t[i + 1][0] - t[i][0]), t[i][1] + f * (t[i + 1][1] - t[i][1]),
      t[i][2] + f * (t[i + 1][2] - t[i][2])};
  }
};

namespace detail
{

inline Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
    }
  }
  return c;
}

inline Mat3 transpose(const Mat3 & a)
{
  return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

// Rodrigues: rotation matrix of rotation vector r.
inline Mat3 exp_so3(const Vec3 & r)
{
  const double th = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  const double a = th < 1e-9 ? 1.0 : std::sin(th) / th;
  const double b = th < 1e-9 ? 0.5 : (1 - std::cos(th)) / (th * th);
  const double x = r[0], y = r[1], z = r[2];
  return {1 - b * (y * y + z * z), -a * z + b * x * y, a * y + b * x * z,
    a * z + b * x * y, 1 - b * (x * x + z * z), -a * x + b * y * z,
    -a * y + b * x * z, a * x + b * y * z, 1 - b * (x * x + y * y)};
}

inline const sensor_msgs::msg::PointField * field(const sensor_msgs::msg::PointCloud2 & c, const std::string & n)
{
  for (const auto & f : c.fields) {
    if (f.name == n) {
      return &f;
    }
  }
  return nullptr;
}

// Points are packed (22-byte step on the robot): unaligned, read with memcpy.
inline float read_f32(const sensor_msgs::msg::PointCloud2 & c, size_t i, uint32_t off)
{
  float v;
  std::memcpy(&v, &c.data[i * c.point_step + off], 4);
  return v;
}

}  // namespace detail

// Scan duration = max per-point time (s). Empty string on success, else why the cloud cannot be deskewed.
inline std::string scan_span(const sensor_msgs::msg::PointCloud2 & cloud, double time_scale, double & span)
{
  using sensor_msgs::msg::PointField;
  const auto * fx = detail::field(cloud, "x");
  const auto * fy = detail::field(cloud, "y");
  const auto * fz = detail::field(cloud, "z");
  const auto * ft = detail::field(cloud, "time");
  if (!fx || !fy || !fz || !ft || fx->datatype != PointField::FLOAT32 || fy->datatype != PointField::FLOAT32 ||
    fz->datatype != PointField::FLOAT32 || ft->datatype != PointField::FLOAT32)
  {
    return "expected FLOAT32 fields x, y, z, time";
  }
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (cloud.data.size() < n * cloud.point_step) {
    return "data shorter than width * height * point_step";
  }
  span = 0.0;
  for (size_t i = 0; i < n; ++i) {
    span = std::max(span, static_cast<double>(detail::read_f32(cloud, i, ft->offset)) * time_scale);
  }
  if (!(span >= 0.0 && span < 0.3)) {
    return "per-point time spans " + std::to_string(span) + " s (wrong time unit?)";
  }
  return "";
}

// Writes the deskewed cloud (x y z intensity, scan-end lidar frame) into `out`: fields, data, size; the caller sets
// the header (stamp = scan end). scan_start_ns: the input's header stamp on the gyro's clock. time_scale: seconds per
// unit of the `time` field. trans: leg-odometry translation, or nullptr for rotation only. max_extrapolation_ns: how
// far past the last gyro sample the scan may end (the rate is held constant there).
// Returns an empty string on success, else why nothing was written.
inline std::string deskew_to_end(
  const sensor_msgs::msg::PointCloud2 & in, sensor_msgs::msg::PointCloud2 & out, int64_t scan_start_ns,
  const GyroBuffer & gyro, double time_scale, const Translation * trans, int64_t max_extrapolation_ns = 20000000)
{
  using sensor_msgs::msg::PointField;
  double span = 0.0;
  const std::string err = scan_span(in, time_scale, span);
  if (!err.empty()) {
    return err;
  }
  if (trans && trans->t.empty()) {
    return "empty translation";
  }
  const int64_t span_ns = static_cast<int64_t>(span * 1e9);
  if (gyro.empty() || gyro.first_ns() > scan_start_ns ||
    gyro.last_ns() < scan_start_ns + span_ns - max_extrapolation_ns)
  {
    return "no gyro samples covering the scan";
  }
  const auto * fx = detail::field(in, "x");
  const auto * fy = detail::field(in, "y");
  const auto * fz = detail::field(in, "z");
  const auto * ft = detail::field(in, "time");
  const auto * fi = detail::field(in, "intensity");
  if (fi && fi->datatype != PointField::FLOAT32) {
    fi = nullptr;
  }

  // G[k] = rotation (scan-start frame <- frame at t = k * dt), integrated on a 0.1 ms grid (midpoint rate);
  // M[k] = G[K]^T G[k] = rotation (scan-end frame <- frame at t), K = the grid point of the scan end.
  constexpr double dt = 1e-4;
  const size_t grid = static_cast<size_t>(span / dt) + 2;
  std::vector<Mat3> M(grid);
  M[0] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (size_t k = 1; k < grid; ++k) {
    const Vec3 w = gyro.at(scan_start_ns + static_cast<int64_t>((k - 0.5) * dt * 1e9));
    M[k] = detail::mul(M[k - 1], detail::exp_so3({w[0] * dt, w[1] * dt, w[2] * dt}));
  }
  auto grid_index = [&](double t) {
      return std::min(grid - 1, static_cast<size_t>(std::max(0.0, std::round(t / dt))));
    };
  const Mat3 end_inv = detail::transpose(M[grid_index(span)]);
  for (auto & m : M) {
    m = detail::mul(end_inv, m);
  }

  const size_t n = static_cast<size_t>(in.width) * in.height;
  out.height = 1;
  out.width = static_cast<uint32_t>(n);
  out.is_bigendian = false;
  out.is_dense = in.is_dense;
  out.point_step = 16;
  out.row_step = out.point_step * out.width;
  out.fields.clear();
  for (const char * name : {"x", "y", "z", "intensity"}) {
    PointField f;
    f.name = name;
    f.offset = static_cast<uint32_t>(4 * out.fields.size());
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    out.fields.push_back(f);
  }
  out.data.resize(n * out.point_step);
  for (size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(detail::read_f32(in, i, ft->offset)) * time_scale;
    const Mat3 & m = M[grid_index(t)];
    const double x = detail::read_f32(in, i, fx->offset);
    const double y = detail::read_f32(in, i, fy->offset);
    const double z = detail::read_f32(in, i, fz->offset);
    const Vec3 tr = trans ? trans->at(t) : Vec3{0, 0, 0};
    const float o[4] = {
      static_cast<float>(m[0] * x + m[1] * y + m[2] * z + tr[0]),
      static_cast<float>(m[3] * x + m[4] * y + m[5] * z + tr[1]),
      static_cast<float>(m[6] * x + m[7] * y + m[8] * z + tr[2]),
      fi ? detail::read_f32(in, i, fi->offset) : 0.0f};
    std::memcpy(&out.data[i * out.point_step], o, sizeof(o));
  }
  return "";
}

}  // namespace g1_bringup
