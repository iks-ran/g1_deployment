// Robot clock -> host clock offset, tracked from (host receive time, robot stamp) pairs.
//
// The robot's clock is not synchronized with the host (~33 min apart on 2026-10-24, drifting ~1 s/h), so robot
// stamps are converted with  host_stamp = robot_stamp + offset.  Each sample gives  receive - stamp = offset +
// latency (latency >= 0), so the offset is the lower envelope: the minimum over a sliding window short enough to
// follow the drift. Feed it a frequent message stamped at the time of measurement (the MID-360 IMU at 200 Hz), NOT
// the cloud: its stamp is the scan START, ~100 ms before it arrives.
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

namespace g1_bringup
{

class ClockOffset
{
public:
  explicit ClockOffset(double window_s = 2.0)
  : window_ns_(static_cast<int64_t>(window_s * 1e9)) {}

  void add(int64_t receive_ns, int64_t stamp_ns)
  {
    const int64_t off = receive_ns - stamp_ns;
    // Monotonic min queue: offsets increasing front to back, front = window minimum.
    while (!mins_.empty() && mins_.back().second >= off) {
      mins_.pop_back();
    }
    mins_.emplace_back(receive_ns, off);
    while (mins_.front().first < receive_ns - window_ns_) {
      mins_.pop_front();
    }
  }

  // Current estimate, or nullopt before the first sample.
  std::optional<int64_t> offset_ns() const
  {
    if (mins_.empty()) {
      return std::nullopt;
    }
    return mins_.front().second;
  }

  std::optional<int64_t> to_host_ns(int64_t stamp_ns) const
  {
    auto off = offset_ns();
    if (!off) {
      return std::nullopt;
    }
    return stamp_ns + *off;
  }

private:
  int64_t window_ns_;
  std::deque<std::pair<int64_t, int64_t>> mins_;  // (receive_ns, receive_ns - stamp_ns)
};

}  // namespace g1_bringup
