#include <rviz_custom_plugins/button_panel.hpp>

namespace rviz_custom_plugins
{

ButtonPanel::ButtonPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  // Initialize ROS2 node
  node_ = std::make_shared<rclcpp::Node>("rviz_button_node");

  // Create publishers for two different topics
  publisher_start_ = node_->create_publisher<std_msgs::msg::String>("/start", 10);
  publisher_stop_ = node_->create_publisher<std_msgs::msg::String>("/stop", 10);

  // Create the first button
  button_start_ = new QPushButton("START", this);
  connect(button_start_, SIGNAL(clicked()), this, SLOT(startButtonClicked()));
  button_start_->setStyleSheet("color: green; font-weight: bold;");

  // Create the second button
  button_stop_ = new QPushButton("STOP", this);
  connect(button_stop_, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
  button_stop_->setStyleSheet("color: red; font-weight: bold;");

  // Arrange the buttons vertically
  auto layout = new QHBoxLayout();
  layout->addWidget(button_start_);
  layout->addWidget(button_stop_);
  setLayout(layout);
}

void ButtonPanel::startButtonClicked()
{
  // Publish empty message on the first topic
  std_msgs::msg::Empty empty_msg;
  std_msgs::msg::String str_msg;
  str_msg.set__data("start");
  publisher_start_->publish(str_msg);
}

void ButtonPanel::stopButtonClicked()
{
  // Publish empty message on the second topic
  std_msgs::msg::Empty empty_msg;
  std_msgs::msg::String str_msg;
  str_msg.set__data("stop");
  publisher_stop_->publish(str_msg);
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::ButtonPanel, rviz_common::Panel)
