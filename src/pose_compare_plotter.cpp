// Copyright 2026. Licensed under the MIT License.
// Read-only ROS 2 pose comparison. All ROS callbacks and painting run on the Qt thread.
#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pose_compare_plotter/pose_history.hpp"

namespace pose_compare_plotter
{
using SteadyClock = std::chrono::steady_clock;
constexpr std::size_t kStreamCount = 3;
const std::array<const char *, kStreamCount> kLetters{"A", "B", "C"};
const std::array<const char *, kStreamCount> kStatusNames{
  "raw_status", "vision_status", "ekf_status"};
const std::array<const char *, kStreamCount> kVisibilityNames{
  "show_raw", "show_vision", "show_ekf"};
const std::array<const char *, kStreamCount> kSourceDescriptions{
  "A · MOCAP · solid", "B · VISION · dashed", "C · FCU EKF · dotted"};
const std::array<Qt::PenStyle, kStreamCount> kLineStyles{
  Qt::SolidLine, Qt::DashLine, Qt::DotLine};

class PoseNode : public rclcpp::Node
{
public:
  explicit PoseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pose_compare_plotter", options), started_(SteadyClock::now())
  {
    streams[0].topic = declare_parameter<std::string>("raw_topic", "/mocap/pop/pose");
    streams[1].topic = declare_parameter<std::string>("vision_topic", "/mavros/vision_pose/pose");
    streams[2].topic = declare_parameter<std::string>("ekf_topic", "/mavros/local_position/pose");
    history_seconds = declare_parameter<double>("history_seconds", 30.0);
    if (!std::isfinite(history_seconds) || history_seconds < 5 || history_seconds > 120) {
      throw std::invalid_argument("history_seconds must be between 5.0 and 120.0");
    }
    for (std::size_t i = 0; i < streams.size(); ++i) {
      subscriptions_[i] = create_subscription<geometry_msgs::msg::PoseStamped>(
        streams[i].topic, rclcpp::SensorDataQoS().keep_last(50),
        [this, i](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          if (!streams[i].append(*message, now(), history_seconds)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Ignoring invalid pose on %s (nonfinite values or invalid quaternion)",
              streams[i].topic.c_str());
          }
        });
      streams[i].topic = subscriptions_[i]->get_topic_name();
    }
    RCLCPP_INFO(get_logger(), "Read-only pose viewer: A=%s, B=%s, C=%s",
      streams[0].topic.c_str(), streams[1].topic.c_str(), streams[2].topic.c_str());
  }

  double now() const
  {
    return std::chrono::duration<double>(SteadyClock::now() - started_).count();
  }

  std::array<Stream, kStreamCount> streams;
  double history_seconds;

private:
  SteadyClock::time_point started_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, kStreamCount> subscriptions_;
};

class Plot : public QWidget
{
public:
  Plot(Quantity quantity, std::size_t axis, const QString & title,
    const QColor & color, QWidget * parent = nullptr)
  : QWidget(parent), quantity_(quantity), axis_(axis), title_(title), color_(color)
  {
    setMinimumSize(780, 125);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void showData(const std::array<Stream, kStreamCount> * streams, double end, double seconds,
    const std::array<bool, kStreamCount> & visible)
  {
    streams_ = streams;
    end_ = end;
    seconds_ = seconds;
    visible_ = visible;
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), Qt::white);
    const QRectF graph(78, 35, width() - 98, height() - 66);
    if (!streams_ || graph.width() < 1 || graph.height() < 1) {return;}
    const double start = end_ - seconds_;
    double low = 0.0, high = 0.0;
    bool has_values = false;
    for (std::size_t source = 0; source < kStreamCount; ++source) {
      if (!visible_[source]) {continue;}
      const auto & stream = (*streams_)[source];
      for (const auto & sample : stream.samples) {
        if (sample.received < start || sample.received > end_) {continue;}
        const double value = stream.value(sample, quantity_, axis_);
        if (!std::isfinite(value)) {continue;}
        low = std::min(low, value);
        high = std::max(high, value);
        has_values = true;
      }
    }
    if (quantity_ == Quantity::Orientation) {
      low = axis_ == 1 ? -100 : -190;
      high = -low;
    } else if (quantity_ == Quantity::Quaternion) {
      low = std::min(low, -1.1);
      high = std::max(high, 1.1);
    } else {
      const double padding = std::max((high - low) * 0.12, 0.025);
      low -= padding;
      high += padding;
    }
    const auto pixel = [&](double time, double value) {
        return QPointF(graph.left() + (time - start) / seconds_ * graph.width(),
          graph.bottom() - (value - low) / (high - low) * graph.height());
      };
    painter.setPen(color_);
    QFont title_font = painter.font();
    title_font.setBold(true);
    painter.setFont(title_font);
    painter.drawText(QRectF(12, 4, width() * 0.42, 25), Qt::AlignVCenter, title_);
    title_font.setBold(false);
    painter.setFont(title_font);
    for (int tick = 0; tick <= 4; ++tick) {
      const double value = low + (high - low) * tick / 4;
      const double y = pixel(start, value).y();
      painter.setPen(QColor("#e4e8ef"));
      painter.drawLine(QPointF(graph.left(), y), QPointF(graph.right(), y));
      painter.setPen(QColor("#64748b"));
      painter.drawText(QRectF(0, y - 9, 69, 18), Qt::AlignRight | Qt::AlignVCenter,
        QString::number(value, 'g', 4));
    }
    for (int tick = 0; tick <= 5; ++tick) {
      const double time = start + seconds_ * tick / 5;
      const double x = pixel(time, low).x();
      painter.setPen(QColor("#e4e8ef"));
      painter.drawLine(QPointF(x, graph.top()), QPointF(x, graph.bottom()));
      painter.setPen(QColor("#64748b"));
      painter.drawText(QRectF(x - 28, graph.bottom() + 4, 56, 18), Qt::AlignCenter,
        QString::number(time - end_, 'f', 0) + " s");
    }
    painter.setPen(QPen(QColor("#a9b6c6"), 1));
    painter.drawLine(pixel(start, 0), pixel(end_, 0));

    for (std::size_t source = 0; source < kStreamCount; ++source) {
      const auto & stream = (*streams_)[source];
      const QColor color = source == 0 ? color_ :
        (source == 1 ? color_.lighter(125) : color_.darker(135));
      const QPen line(color, source == 0 ? 2.0 : 2.6, kLineStyles[source]);
      const double legend_x = width() - 500 + source * 165;
      painter.setPen(line);
      painter.drawLine(QPointF(legend_x, 17), QPointF(legend_x + 28, 17));
      painter.setPen(QColor("#334155"));
      QString latest = "--";
      if (!stream.samples.empty()) {
        const double value = stream.value(stream.samples.back(), quantity_, axis_);
        if (std::isfinite(value)) {latest = QString::number(value, 'f', 3);}
      }
      painter.drawText(QRectF(legend_x + 34, 5, 126, 24), Qt::AlignVCenter,
        QString("%1: %2").arg(kLetters[source]).arg(visible_[source] ? latest : "hidden"));
      if (!visible_[source]) {continue;}

      painter.save();
      painter.setClipRect(graph);
      painter.setPen(line);
      QPainterPath path;
      bool connected = false;
      double previous_time = 0, previous_value = 0;
      QPointF last_point;
      for (const auto & sample : stream.samples) {
        if (sample.received < start || sample.received > end_) {continue;}
        const double value = stream.value(sample, quantity_, axis_);
        if (!std::isfinite(value)) {connected = false; continue;}
        const QPointF point = pixel(sample.received, value);
        const bool angle_wrap = quantity_ == Quantity::Orientation &&
          std::abs(value - previous_value) > 180;
        if (!connected || sample.received - previous_time > 0.5 || angle_wrap) {
          path.moveTo(point);
        } else {
          path.lineTo(point);
        }
        previous_time = sample.received;
        previous_value = value;
        last_point = point;
        connected = true;
      }
      painter.drawPath(path);
      if (connected) {
        painter.setBrush(color);
        painter.drawEllipse(last_point, 3, 3);
      }
      painter.restore();
    }
    if (!has_values) {
      painter.setPen(QColor("#64748b"));
      painter.drawText(graph, Qt::AlignCenter,
        quantity_ == Quantity::Velocity ? "Waiting for valid position / timestamp differences" :
        "No samples in this time window");
    }
  }

private:
  Quantity quantity_;
  std::size_t axis_;
  QString title_;
  QColor color_;
  const std::array<Stream, kStreamCount> * streams_ = nullptr;
  std::array<bool, kStreamCount> visible_{true, true, true};
  double end_ = 0, seconds_ = 30;
};

class PoseWindow : public QWidget
{
public:
  explicit PoseWindow(const std::shared_ptr<PoseNode> & node) : node_(node)
  {
    setWindowTitle("MoCap / Vision / FCU EKF — Live Comparison");
    resize(1180, 860);
    setStyleSheet(
      "QWidget {font-family: 'DejaVu Sans'; font-size: 12px; color: #17283c;}"
      "QPushButton {padding: 7px 12px;} QTabWidget::pane {border: 1px solid #dce3ec;}"
      "QTabBar::tab {padding: 9px 18px;} QDoubleSpinBox {padding: 5px;}");
    auto * layout = new QVBoxLayout(this);
    auto * title = new QLabel("Pose comparison", this);
    title->setStyleSheet("font-size: 24px; font-weight: bold; padding: 3px 0;");
    layout->addWidget(title);
    auto * subtitle = new QLabel(
      "A solid = raw MoCap    •    B dashed = vision pose    •    C dotted = FCU EKF pose", this);
    subtitle->setWordWrap(true);
    layout->addWidget(subtitle);
    auto * statuses = new QHBoxLayout;
    for (std::size_t i = 0; i < kStreamCount; ++i) {
      status_[i] = new QLabel(this);
      status_[i]->setObjectName(kStatusNames[i]);
      status_[i]->setTextFormat(Qt::PlainText);
      status_[i]->setWordWrap(true);
      status_[i]->setMinimumHeight(90);
      status_[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
      statuses->addWidget(status_[i], 1);
    }
    layout->addLayout(statuses);
    auto * controls = new QHBoxLayout;
    auto * pause = new QPushButton("Pause plots", this);
    pause->setObjectName("pause");
    pause->setCheckable(true);
    zero_ = new QPushButton("Zero position", this);
    zero_->setObjectName("zero");
    zero_->setToolTip("Hold still. Zero the visible streams; each must have fresh data. "
      "Uncheck a missing stream to zero the others. No rotation is applied.");
    auto * absolute = new QPushButton("Absolute position", this);
    auto * clear = new QPushButton("Clear history", this);
    clear->setObjectName("clear");
    auto * save = new QPushButton("Save PNG", this);
    for (auto * button : {pause, zero_, absolute, clear, save}) {controls->addWidget(button);}
    controls->addStretch();
    controls->addWidget(new QLabel("Window:", this));
    auto * duration = new QDoubleSpinBox(this);
    duration->setRange(5, 120);
    duration->setDecimals(0);
    duration->setSuffix(" s");
    duration->setValue(node_->history_seconds);
    controls->addWidget(duration);
    for (std::size_t i = 0; i < kStreamCount; ++i) {
      visible_[i] = new QCheckBox(kLetters[i], this);
      visible_[i]->setObjectName(kVisibilityNames[i]);
      visible_[i]->setToolTip(kSourceDescriptions[i]);
      visible_[i]->setChecked(true);
      controls->addWidget(visible_[i]);
      connect(visible_[i], &QCheckBox::toggled, this, [this](bool) {refresh();});
    }
    layout->addLayout(controls);
    mode_ = new QLabel(this);
    mode_->setObjectName("mode");
    mode_->setWordWrap(true);
    layout->addWidget(mode_);

    auto * tabs = new QTabWidget(this);
    tabs->setObjectName("plots");
    const std::array<QColor, 4> colors{
      QColor("#d33345"), QColor("#148449"), QColor("#2463d4"), QColor("#8756b6")};
    const std::array<QStringList, 4> titles{
      QStringList{"X position [m]", "Y position [m]", "Z position [m]"},
      QStringList{"Roll about X [deg]", "Pitch about Y [deg]", "Yaw about Z [deg]"},
      QStringList{"X velocity [m/s]", "Y velocity [m/s]", "Z velocity [m/s]"},
      QStringList{"Quaternion X", "Quaternion Y", "Quaternion Z", "Quaternion W"}};
    const std::array<QString, 4> names{
      "Position / XYZ", "Orientation / RPY", "Velocity (derived)", "Quaternion / XYZW"};
    const std::array<QString, 4> notes{
      "Compare A / B / C by moving one physical direction at a time. Zero position removes origin offsets only.",
      "Angles use each topic's own frame. Yaw / roll wrap at ±180°; plot lines break at wraps.",
      "Estimated velocity = Δposition / Δheader time, without smoothing. Not a measured velocity topic.",
      "Original message components. q and −q represent the same orientation; sign flips can be harmless."};
    for (std::size_t group = 0; group < names.size(); ++group) {
      auto * page = new QWidget(tabs);
      auto * page_layout = new QVBoxLayout(page);
      auto * note = new QLabel(notes[group], page);
      note->setWordWrap(true);
      page_layout->addWidget(note);
      for (int axis = 0; axis < titles[group].size(); ++axis) {
        auto * plot = new Plot(static_cast<Quantity>(group), axis, titles[group][axis], colors[axis], page);
        plots_.push_back(plot);
        page_layout->addWidget(plot, 1);
      }
      tabs->addTab(page, names[group]);
    }
    layout->addWidget(tabs, 1);
    auto * footer = new QLabel(
      "Time: seconds before the right edge, using local receipt time. Streams are not timestamp-synchronized.\n"
      "Bench check: hold still → Zero position → move along one chosen room axis → return → repeat.", this);
    footer->setWordWrap(true);
    footer->setStyleSheet("color: #576980; padding: 5px 0;");
    layout->addWidget(footer);

    connect(pause, &QPushButton::toggled, this, [this, pause, absolute, clear](bool paused) {
        paused_ = paused;
        if (paused) {snapshot_ = node_->streams; paused_at_ = node_->now();}
        pause->setText(paused ? "Resume plots" : "Pause plots");
        absolute->setEnabled(!paused);
        clear->setEnabled(!paused);
        refresh();
      });
    connect(zero_, &QPushButton::clicked, this, [this]() {
        const double now = node_->now();
        if (canZero(now)) {
          for (std::size_t i = 0; i < kStreamCount; ++i) {
            if (visible_[i]->isChecked()) {node_->streams[i].zero(now);}
          }
        }
        refresh();
      });
    connect(absolute, &QPushButton::clicked, this, [this]() {
        for (auto & stream : node_->streams) {stream.relative = false;}
        refresh();
      });
    connect(clear, &QPushButton::clicked, this, [this]() {
        for (auto & stream : node_->streams) {
          const std::string topic = stream.topic;
          stream = Stream{};
          stream.topic = topic;
        }
        refresh();
      });
    connect(save, &QPushButton::clicked, this, [this]() {
        const QString filename = QFileDialog::getSaveFileName(
          this, "Save plot window", "pose_comparison.png", "PNG image (*.png)");
        if (!filename.isEmpty() && !grab().save(filename, "PNG")) {
          QMessageBox::warning(this, "Save failed", "Could not save the PNG file.");
        }
      });
    connect(duration, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
      this, [this](double seconds) {node_->history_seconds = seconds; refresh();});
    auto * redraw = new QTimer(this);
    connect(redraw, &QTimer::timeout, this, [this]() {refresh();});
    redraw->start(50);
    refresh();
  }

  void refresh()
  {
    const double now = node_->now();
    for (std::size_t i = 0; i < kStreamCount; ++i) {
      auto & stream = node_->streams[i];
      stream.prune(now, node_->history_seconds);
      const bool fresh = stream.fresh(now);
      const QString state = stream.accepted == 0 ? "WAITING" : (fresh ? "LIVE" : "STALE");
      const QString age = stream.last_received < 0 ? "--" :
        QString::number(now - stream.last_received, 'f', 1) + " s";
      status_[i]->setText(QString("%1  %2\n%3\nFrame: %4  |  %5 Hz  |  Age: %6\n"
          "Accepted: %7  |  Invalid: %8  |  Frame changes: %9")
        .arg(kSourceDescriptions[i]).arg(state)
        .arg(QString::fromStdString(stream.topic))
        .arg(stream.frame.empty() ? "(not supplied)" : QString::fromStdString(stream.frame))
        .arg(stream.rate(now), 0, 'f', 1).arg(age)
        .arg(static_cast<qulonglong>(stream.accepted))
        .arg(static_cast<qulonglong>(stream.rejected))
        .arg(static_cast<qulonglong>(stream.frame_changes)));
      status_[i]->setStyleSheet(QString(
          "background: %1; border: 1px solid %2; border-radius: 5px; padding: 8px;")
        .arg(fresh ? "#eef8f2" : "#fff8eb").arg(fresh ? "#b3dac3" : "#e8cc94"));
    }
    const auto & shown = paused_ ? snapshot_ : node_->streams;
    zero_->setEnabled(canZero(now));
    mode_->setText(QString("%1  •  Position A: %2  /  B: %3  /  C: %4  •  No frame rotation applied by viewer")
      .arg(paused_ ? "PLOTS PAUSED — subscriptions continue" : "LIVE PLOTS")
      .arg(shown[0].relative ? "change since zero" : "absolute")
      .arg(shown[1].relative ? "change since zero" : "absolute")
      .arg(shown[2].relative ? "change since zero" : "absolute"));
    for (auto * plot : plots_) {
      plot->showData(&shown, paused_ ? paused_at_ : now, node_->history_seconds,
        {visible_[0]->isChecked(), visible_[1]->isChecked(), visible_[2]->isChecked()});
    }
  }

private:
  bool canZero(double now) const
  {
    if (paused_) {return false;}
    bool any_visible = false;
    for (std::size_t i = 0; i < kStreamCount; ++i) {
      if (visible_[i]->isChecked()) {
        any_visible = true;
        if (!node_->streams[i].fresh(now)) {return false;}
      }
    }
    return any_visible;
  }

  std::shared_ptr<PoseNode> node_;
  std::array<Stream, kStreamCount> snapshot_;
  std::array<QLabel *, kStreamCount> status_{};
  std::array<QCheckBox *, kStreamCount> visible_{};
  QPushButton * zero_ = nullptr;
  QLabel * mode_ = nullptr;
  std::vector<Plot *> plots_;
  bool paused_ = false;
  double paused_at_ = 0;
};
}  // namespace pose_compare_plotter

#ifndef POSE_COMPARE_PLOTTER_NO_MAIN
int main(int argc, char ** argv)
{
  try {
    auto arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char *> qt_arguments;
    for (auto & argument : arguments) {qt_arguments.push_back(argument.data());}
    int qt_argc = static_cast<int>(qt_arguments.size());
    qt_arguments.push_back(nullptr);
    QApplication app(qt_argc, qt_arguments.data());
    auto node = std::make_shared<pose_compare_plotter::PoseNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    pose_compare_plotter::PoseWindow window(node);
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, &app, [&]() {
        if (!rclcpp::ok()) {app.quit(); return;}
        executor.spin_some(std::chrono::milliseconds(2));
      });
    ros_timer.start(10);
    window.show();
    const int result = app.exec();
    executor.remove_node(node);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "pose_compare_plotter: %s\n", error.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
}
#endif
