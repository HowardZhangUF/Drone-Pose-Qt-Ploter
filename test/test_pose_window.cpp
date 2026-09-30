// Copyright 2026. Licensed under the MIT License.
#define POSE_COMPARE_PLOTTER_NO_MAIN
#include "../src/pose_compare_plotter.cpp"
#include <cstdlib>
#include <gtest/gtest.h>
#include <QTest>

using pose_compare_plotter::PoseNode;
using pose_compare_plotter::PoseWindow;

namespace
{
class PoseWindowTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int argc = 1;
    static char program[] = "test_pose_window";
    static char * argv[] = {program, nullptr};
    static QApplication application(argc, argv);
    if (!rclcpp::ok()) {rclcpp::init(argc, argv);}
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("raw_topic", "/pose_compare_plotter_test/raw"),
      rclcpp::Parameter("vision_topic", "/pose_compare_plotter_test/vision"),
      rclcpp::Parameter("ekf_topic", "/pose_compare_plotter_test/ekf")});
    node = std::make_shared<PoseNode>(options);
  }
  std::shared_ptr<PoseNode> node;
};
}  // namespace

TEST_F(PoseWindowTest, ReceivesBestEffortAndReliableTopicsAndControlsRemainResponsive)
{
  PoseWindow window(node);
  window.show();
  QCoreApplication::processEvents();
  auto * zero = window.findChild<QPushButton *>("zero");
  ASSERT_NE(zero, nullptr);
  EXPECT_FALSE(zero->isEnabled());
  auto publisher = std::make_shared<rclcpp::Node>("pose_compare_plotter_test_publisher");
  auto raw = publisher->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/pose_compare_plotter_test/raw", rclcpp::SensorDataQoS());
  auto vision = publisher->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/pose_compare_plotter_test/vision", rclcpp::QoS(10).reliable());
  auto ekf = publisher->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/pose_compare_plotter_test/ekf", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(publisher);
  geometry_msgs::msg::PoseStamped message;
  message.pose.orientation.w = 1;
  message.header.stamp.sec = 100;
  for (int i = 0; i < 200 && (node->streams[0].accepted < 3 ||
    node->streams[1].accepted < 3 || node->streams[2].accepted < 3); ++i)
  {
    message.header.frame_id = "optitrack";
    message.pose.position.x = 0.2;
    raw->publish(message);
    message.header.frame_id = "world";
    message.pose.position.x = -0.2;
    vision->publish(message);
    message.header.frame_id = "map";
    message.pose.position.x = 1.2;
    ekf->publish(message);
    executor.spin_some();
    QTest::qWait(10);
  }
  ASSERT_GE(node->streams[0].accepted, 3u);
  ASSERT_GE(node->streams[1].accepted, 3u);
  ASSERT_GE(node->streams[2].accepted, 3u);
  EXPECT_EQ(node->streams[2].topic, "/pose_compare_plotter_test/ekf");
  EXPECT_DOUBLE_EQ(node->streams[2].samples.back().position[0], 1.2);
  window.refresh();
  EXPECT_TRUE(zero->isEnabled());
  EXPECT_TRUE(window.findChild<QLabel *>("raw_status")->text().contains("optitrack"));
  EXPECT_TRUE(window.findChild<QLabel *>("vision_status")->text().contains("world"));
  EXPECT_TRUE(window.findChild<QLabel *>("ekf_status")->text().contains("map"));
  EXPECT_TRUE(window.findChild<QLabel *>("ekf_status")->text().contains("FCU EKF"));
  QTest::mouseClick(zero, Qt::LeftButton);
  EXPECT_TRUE(node->streams[0].relative);
  EXPECT_TRUE(node->streams[1].relative);
  EXPECT_TRUE(node->streams[2].relative);
  EXPECT_DOUBLE_EQ(node->streams[2].value(node->streams[2].samples.back(),
    pose_compare_plotter::Quantity::Position, 0), 0.0);
  auto * pause = window.findChild<QPushButton *>("pause");
  QTest::mouseClick(pause, Qt::LeftButton);
  EXPECT_FALSE(zero->isEnabled());
  EXPECT_TRUE(window.findChild<QLabel *>("mode")->text().contains("PAUSED"));
  const auto previous = node->streams[0].accepted;
  raw->publish(message);
  for (int i = 0; i < 50 && node->streams[0].accepted == previous; ++i) {
    executor.spin_some();
    QTest::qWait(10);
  }
  EXPECT_GT(node->streams[0].accepted, previous);
  QTest::mouseClick(pause, Qt::LeftButton);
  QTest::mouseClick(window.findChild<QPushButton *>("clear"), Qt::LeftButton);
  EXPECT_TRUE(node->streams[0].samples.empty());
  EXPECT_TRUE(node->streams[1].samples.empty());
  EXPECT_TRUE(node->streams[2].samples.empty());
  EXPECT_FALSE(zero->isEnabled());
}

TEST_F(PoseWindowTest, RendersAllTabsAndMarksStoppedStreamsStale)
{
  // Simulated data only, inserted into viewer memory; nothing is published to flight topics.
  const double now = node->now();
  for (int i = 0; i <= 180; ++i) {
    geometry_msgs::msg::PoseStamped message;
    const double time = i * 0.1;
    message.header.stamp.sec = 100 + i / 10;
    message.header.stamp.nanosec = (i % 10) * 100000000;
    message.header.frame_id = "optitrack";
    message.pose.position.x = 0.25 * std::sin(time);
    message.pose.position.y = 0.15 * std::cos(time * 0.8);
    message.pose.position.z = 0.15 + 0.05 * std::sin(time * 0.4);
    const double yaw = 0.15 * std::sin(time * 0.5);
    message.pose.orientation.z = std::sin(yaw / 2);
    message.pose.orientation.w = std::cos(yaw / 2);
    node->streams[0].append(message, now - 18 + time, 30);
    message.header.frame_id = "world";
    message.pose.position.x *= -1;
    message.pose.position.y *= -1;
    message.pose.orientation.z = std::cos(yaw / 2);
    message.pose.orientation.w = -std::sin(yaw / 2);
    node->streams[1].append(message, now - 18 + time, 30);
    // Distinct origin and a small tracking difference make the EKF curve visible.
    message.header.frame_id = "map";
    message.pose.position.x += 0.04 + 0.02 * std::sin(time * 1.1);
    message.pose.position.y += 0.04;
    message.pose.position.z += 0.025;
    node->streams[2].append(message, now - 18 + time, 30);
  }
  PoseWindow window(node);
  window.show();
  auto * tabs = window.findChild<QTabWidget *>("plots");
  ASSERT_EQ(tabs->count(), 4);
  for (int i = 0; i < tabs->count(); ++i) {
    tabs->setCurrentIndex(i);
    window.refresh();
    QCoreApplication::processEvents();
    const QPixmap picture = window.grab();
    EXPECT_FALSE(picture.isNull());
    EXPECT_GE(picture.width(), 1000);
    if (const char * directory = std::getenv("POSE_PLOTTER_PREVIEW_DIR")) {
      EXPECT_TRUE(picture.save(QString("%1/pose_plotter_%2.png").arg(directory).arg(i)));
    }
  }
  node->streams[2].last_received = now - 2;
  window.refresh();
  EXPECT_TRUE(window.findChild<QLabel *>("ekf_status")->text().contains("STALE"));
  EXPECT_FALSE(window.findChild<QPushButton *>("zero")->isEnabled());
}

TEST_F(PoseWindowTest, MissingEkfDoesNotBlockMocapAndCanBeHiddenForZeroing)
{
  geometry_msgs::msg::PoseStamped message;
  message.pose.orientation.w = 1;
  message.pose.position.x = 0.25;
  message.header.frame_id = "optitrack";
  node->streams[0].append(message, node->now(), 30);
  message.header.frame_id = "world";
  node->streams[1].append(message, node->now(), 30);
  PoseWindow window(node);
  window.show();
  QCoreApplication::processEvents();
  auto * zero = window.findChild<QPushButton *>("zero");
  auto * show_ekf = window.findChild<QCheckBox *>("show_ekf");
  ASSERT_NE(show_ekf, nullptr);
  EXPECT_TRUE(window.findChild<QLabel *>("raw_status")->text().contains("LIVE"));
  EXPECT_TRUE(window.findChild<QLabel *>("vision_status")->text().contains("LIVE"));
  EXPECT_TRUE(window.findChild<QLabel *>("ekf_status")->text().contains("WAITING"));
  EXPECT_FALSE(zero->isEnabled());
  show_ekf->setChecked(false);
  EXPECT_TRUE(zero->isEnabled());
  QTest::mouseClick(zero, Qt::LeftButton);
  EXPECT_TRUE(node->streams[0].relative);
  EXPECT_TRUE(node->streams[1].relative);
  EXPECT_FALSE(node->streams[2].relative);
  show_ekf->setChecked(true);
  EXPECT_FALSE(zero->isEnabled());
  message.header.frame_id = "map";
  message.pose.position.x = 3.5;
  node->streams[2].append(message, node->now(), 30);
  window.refresh();
  EXPECT_TRUE(zero->isEnabled());
  QTest::mouseClick(zero, Qt::LeftButton);
  EXPECT_TRUE(node->streams[2].relative);
  EXPECT_DOUBLE_EQ(node->streams[2].origin[0], 3.5);
  for (const char * name : pose_compare_plotter::kVisibilityNames) {
    window.findChild<QCheckBox *>(name)->setChecked(false);
  }
  EXPECT_FALSE(zero->isEnabled());
}
