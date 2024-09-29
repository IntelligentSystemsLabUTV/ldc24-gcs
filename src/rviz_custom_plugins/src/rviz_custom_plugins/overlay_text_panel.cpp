#include <rviz_custom_plugins/overlay_text_panel.hpp>

namespace rviz_custom_plugins
{

OverlayTextPanel::OverlayTextPanel(QWidget * parent)
: rviz_common::Panel(parent), node_(std::make_shared<rclcpp::Node>("string_display_node"))
{
  // Set up the UI
  auto * layout = new QVBoxLayout;
  label_ = new QLabel("Waiting for messages...");
  layout->addWidget(label_);
  setLayout(layout);

  // Subscribe to the string topic
  std::string topic_name = "/string_topic";
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    topic_name, 10,
    std::bind(&OverlayTextPanel::stringCallback, this, std::placeholders::_1));

  // Connect the signal to the slot to safely update the label
  connect(this, &OverlayTextPanel::updateText, label_, &QLabel::setText);

  // Timer to spin the ROS node
  QTimer * timer = new QTimer(this);
  connect(
    timer, &QTimer::timeout, this, [this]() {
      rclcpp::spin_some(node_);
    });
  timer->start(100);  // Spin every 100 milliseconds
}

void OverlayTextPanel::stringCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string color;
  if (msg->data == "INIT") {
    color = "#000000";     // Black
  } else if (msg->data == "TAKEOFF" || msg->data == "EMERGENCY_LANDING" || msg->data == "ARM" ||
    msg->data == "DISARM")
  {
    color = "#feb000";     // Dark yellow
  } else if (msg->data == "EXPLORE") {
    color = "#00008b";     // Dark blue
  } else if (msg->data == "TRACK") {
    color = "#008b8b";     // Dark cyan
  } else if (msg->data == "FOLLOWME") {
    color = "#cc8400";     // Dark orange
  } else if (msg->data == "RTB") {
    color = "#4b0082";     // Dark purple
  } else if (msg->data == "COMPLETED") {
    color = "#006400";     // Dark green
  } else {
    color = "#8b0000";     // Dark red
  }
  QString text = QString("<font color='%1' style='font-size:%2px;'>%3</font>")
    .arg(color.c_str())
    .arg(20)
    .arg(msg->data.c_str());
  emit updateText(text);
}

}  // namespace rviz_custom_plugins

// Register the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::OverlayTextPanel, rviz_common::Panel)
