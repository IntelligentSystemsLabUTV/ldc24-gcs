#ifndef RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
#define RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

namespace rviz_custom_plugins
{

class OverlayTextPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  OverlayTextPanel(QWidget * parent = nullptr);

  void stringCallback(const std_msgs::msg::String::SharedPtr msg);

Q_SIGNALS:
  void updateText(const QString & new_text);

private:
  QLabel * label_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rviz_common::properties::StringProperty * topic_property_;
};

} // namespace rviz_custom_plugins

#endif // RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
