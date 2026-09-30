# Pose comparison viewer
<img width="1903" height="982" alt="image" src="https://github.com/user-attachments/assets/6f923c7b-be30-4d00-8413-028cc2ca5b82" />

C++ / Qt window for comparing all three pose streams while moving a tracked
robot by hand:

| Stream | Default topic | Line |
| --- | --- | --- |
| A: Raw MoCap | `/mocap/pop/pose` | Solid |
| B: Vision input | `/mavros/vision_pose/pose` | Dashed |
| C: FCU EKF output | `/mavros/local_position/pose` | Dotted |

The viewer subscribes to `geometry_msgs/msg/PoseStamped` using best-effort,
volatile QoS, compatible with both best-effort and reliable publishers.
It does not publish poses, transforms, or vehicle commands.

## Build and run

From the basestation workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select pose_compare_plotter
source install/setup.bash
ros2 run pose_compare_plotter pose_compare_plotter
```

Build dependencies include Qt 5 Widgets (`qtbase5-dev`). If missing on another
machine, run `rosdep install --from-paths src/basestation_bundle/pose_compare_plotter
--ignore-src -r -y` from the workspace. A desktop display is required. For SSH,
use a working X11-forwarded session, or run the viewer on the desktop computer
that can discover the topics. Use the same ROS domain / middleware configuration
as your existing `ros2 topic echo` commands.

Topic names and the initial visible history can be changed:

```bash
ros2 run pose_compare_plotter pose_compare_plotter --ros-args \
  -p raw_topic:=/mocap/pop/pose \
  -p vision_topic:=/mavros/vision_pose/pose \
  -p ekf_topic:=/mavros/local_position/pose \
  -p history_seconds:=30.0
```

## Reading the window

- **Position / XYZ:** three plots in meters. Red is X, green is Y, blue is Z.
  Each plot overlays A, B, and C, with the latest value next to its line legend.
- **Orientation / RPY:** roll, pitch, yaw in degrees, calculated from normalized
  quaternions. Roll/yaw wrap at ±180 degrees; lines break at a wrap. RPY has the
  usual singularity near ±90 degrees pitch; use quaternions there.
- **Velocity (derived):** unsmoothed position differences divided by each topic's
  own message-header time differences. These are estimates, not subscriptions
  to velocity topics. Noise and timestamp jitter can create spikes. Velocity
  is omitted for the first sample, absent/duplicate/backward timestamps,
  header gaps above 0.25 s, or reception gaps above 0.5 s.
- **Quaternion / XYZW:** the four components exactly as received. `q` and `-q`
  represent the same orientation; a sign change alone is not physical motion.

Full topic names, message frame IDs, receipt rates, accepted/invalid counts,
and time since the last valid message appear above the plots. After one second
without a valid sample, a stream is marked **STALE**. Last-value legends then
show old values; their age is shown in that stream's status box.

**Zero position** subtracts each visible stream's latest position from its
positions. Hold the drone still when clicking it. It requires fresh samples
from every checked stream and does not rotate axes or alter orientation/velocity.
If the FCU is disconnected, A and B continue plotting. Uncheck C to zero A/B
without waiting for EKF data. When C becomes available, check it and zero again.
The EKF may use a different origin or heading; zeroing removes origin offsets
only. Frame IDs are displayed independently for all three sources.
**Absolute position** returns to the original positions. A frame ID change
clears that stream's history and zero offset, and increments its frame-change
counter. **Clear history** clears all histories and zero offsets.

**Pause plots** freezes a snapshot while subscriptions and the status boxes
continue updating. Resume returns to the current rolling window. **Save PNG**
saves the displayed window. A/B/C checkboxes hide individual streams when lines
overlap. Window length is adjustable from 5 to 120 seconds; increasing it cannot
recover already discarded samples. Each stream is capped at 20,000 samples.

The horizontal axis is local steady-clock receipt time, in seconds relative to
the right edge (0 s). Reception outages above 0.5 s break lines. The streams
are **not synchronized, transformed, or automatically aligned**. Network delay
can shift curves; the bridge may also replace timestamps. This viewer is for
axis/sign checks, not precise latency measurement. Data remains in memory;
PNG export does not record the original ROS messages.

## Bench procedure

With the drone disarmed and propellers removed:

1. Mark the intended room +X and +Y directions. Hold the drone still and click
   **Zero position**.
2. Move slowly 20 cm along +X and return. Repeat along +Y, then lift vertically.
   Observe which plot changes and its sign for each topic.
3. With the drone bridge's identity rotation, expect B's X/Y/Z displacement
   and velocity directions to agree with A. If a 180-degree yaw conversion is
   enabled instead, X/Y signs oppose A and Z agrees. Restart the bridge after
   rebuilding it to load a changed rotation. The physically marked room
   directions determine whether the selected mapping is correct.
4. Hold level with the physical nose along intended +world X. In the desired
   ROS FLU body convention, the correctly aligned world orientation should be
   near zero roll/pitch/yaw. Turning the nose toward +world Y should give +90° yaw.
5. Pause or save an image after each movement. A source that stops publishing
   (for example, a bridge rejecting poses at its tilt gate) is marked stale.

Seeing Z near 0.15 m at rest does not by itself prove that Z is up: test a lift.
The raw `optitrack`, processed `world`, and EKF frame IDs (often `map`) are
kept distinct intentionally.

## Troubleshooting: WAITING / no samples

An SSH terminal on the Pi and a terminal on the basestation run ROS nodes on
separate computers. A successful echo on the Pi does not establish reception
on the basestation. Check the topic from the same ROS environment used to start
the viewer.

In the diagnosed basestation setup, the viewer had `ROS_DISCOVERY_SERVER` set,
while the local VRPN and MoCap bridge publishers did not. A read-only comparison
received both streams with the publishers' settings and neither stream with the
viewer's settings. Restarting only the viewer without that variable restored
live plots.

This is a conditional workaround, not a required launch prefix. If all three
topics are reachable with your current ROS settings, use the normal launch
command above. Reconnecting the battery may restart the FCU/MAVROS stream, but
it does not change the basestation terminal's discovery environment.

When the same discovery mismatch occurs, launch from the basestation desktop terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/basestation/base_ws/install/setup.bash
env -u ROS_DISCOVERY_SERVER ros2 run pose_compare_plotter pose_compare_plotter
```

This removes the variable only for that command. Use it when the publishers use
default discovery; a network configured around a discovery server instead needs
matching server settings. Keep the ROS domain and middleware consistent too
(`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` in this setup).
No rebuild is needed for an environment change.

To verify reception with the same setting and an explicit message type:

```bash
env -u ROS_DISCOVERY_SERVER ros2 topic echo /mavros/vision_pose/pose \
  geometry_msgs/msg/PoseStamped --qos-reliability best_effort --once
```

`ros2 topic list` and type inference may use a daemon started with older discovery
settings. The explicit message type above avoids depending on type inference.
An SSH connection itself does not forward ROS topic traffic.

## Verification

```bash
colcon test --packages-select pose_compare_plotter --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover the supplied quaternions, zero offsets, invalid input, frame changes,
timestamp discontinuities, bounded history, Qt controls/rendering, and actual
ROS reception of all three topics with both QoS types, plus zeroing while the EKF is absent. GUI tests use Qt's offscreen platform and
isolated test topics in a separate localhost-only ROS domain.
