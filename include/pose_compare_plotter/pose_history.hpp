// Copyright 2026. Licensed under the MIT License.
#ifndef POSE_COMPARE_PLOTTER__POSE_HISTORY_HPP_
#define POSE_COMPARE_PLOTTER__POSE_HISTORY_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pose_compare_plotter
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kStaleSeconds = 1.0;
constexpr std::size_t kMaxSamples = 20000;

enum class Quantity {Position, Orientation, Velocity, Quaternion};

struct Sample
{
  double received = 0.0;  // Seconds on a shared local steady clock, not ROS time.
  double stamp = 0.0;
  std::array<double, 3> position{};
  std::array<double, 3> rpy{};
  std::array<double, 3> velocity{kNan, kNan, kNan};
  std::array<double, 4> quaternion{};  // Original values, including quaternion sign.
};

struct Stream
{
  std::string topic;
  std::string frame;
  std::deque<Sample> samples;
  std::array<double, 3> origin{};
  bool relative = false;
  std::size_t accepted = 0;
  std::size_t rejected = 0;
  std::size_t frame_changes = 0;
  double last_received = -1.0;

  void prune(double now, double history_seconds)
  {
    while (!samples.empty() &&
      (samples.front().received < now - history_seconds || samples.size() > kMaxSamples))
    {
      samples.pop_front();
    }
  }

  bool append(const geometry_msgs::msg::PoseStamped & message, double now, double history)
  {
    const auto & p = message.pose.position;
    const auto & q = message.pose.orientation;
    Sample sample;
    sample.received = now;
    sample.stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
    sample.position = {p.x, p.y, p.z};
    sample.quaternion = {q.x, q.y, q.z, q.w};
    const auto finite = [](double v) {return std::isfinite(v);};
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::all_of(sample.position.begin(), sample.position.end(), finite) ||
      !std::all_of(sample.quaternion.begin(), sample.quaternion.end(), finite) ||
      !std::isfinite(norm) || norm < 1e-9)
    {
      ++rejected;
      return false;
    }
    // Normalize only for angle calculation; quaternion plots retain the message values.
    const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
    sample.rpy = {
      std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / kPi,
      std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)) * 180 / kPi,
      std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / kPi};

    if (accepted > 0 && frame != message.header.frame_id) {
      samples.clear();
      origin = {};
      relative = false;
      ++frame_changes;
    }
    frame = message.header.frame_id;
    if (!samples.empty()) {
      const auto & previous = samples.back();
      const double dt = sample.stamp - previous.stamp;
      // Do not invent speed across absent/reset/duplicate timestamps or a stream outage.
      if (sample.stamp > 0 && previous.stamp > 0 && dt > 1e-6 && dt <= 0.25 &&
        now - previous.received <= 0.5)
      {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          sample.velocity[axis] = (sample.position[axis] - previous.position[axis]) / dt;
        }
      }
    }
    samples.push_back(sample);
    last_received = now;
    ++accepted;
    prune(now, history);
    return true;
  }

  bool fresh(double now) const
  {
    return !samples.empty() && now - last_received <= kStaleSeconds;
  }

  double rate(double now) const
  {
    if (!fresh(now) || samples.size() < 2) {return 0.0;}
    auto first = std::lower_bound(samples.begin(), samples.end(), now - 2.0,
        [](const Sample & s, double t) {return s.received < t;});
    const auto count = std::distance(first, samples.end());
    if (count < 2) {return 0.0;}
    const double duration = samples.back().received - first->received;
    return duration > 0 ? (count - 1) / duration : 0.0;
  }

  bool zero(double now)
  {
    if (!fresh(now)) {return false;}
    origin = samples.back().position;
    relative = true;
    return true;
  }

  double value(const Sample & sample, Quantity quantity, std::size_t axis) const
  {
    switch (quantity) {
      case Quantity::Position: return sample.position[axis] - (relative ? origin[axis] : 0);
      case Quantity::Orientation: return sample.rpy[axis];
      case Quantity::Velocity: return sample.velocity[axis];
      case Quantity::Quaternion: return sample.quaternion[axis];
    }
    return kNan;
  }
};
}  // namespace pose_compare_plotter
#endif  // POSE_COMPARE_PLOTTER__POSE_HISTORY_HPP_
