#ifndef RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
#define RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace rviz_custom_plugins
{

class OverlayTextPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  OverlayTextPanel(QWidget * parent = nullptr);

  void onInitialize() override;

  // Override the save and load methods to remember the topic
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void updateTopic();

private:
  void subscribe();
  void unsubscribe();
  void callback(const std_msgs::msg::String::SharedPtr msg);

  QLabel * label_;
  QLabel * topic_info_label_;
  QLineEdit * topic_input_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Node::SharedPtr node_;

  QString topic_name_ = "";
};

}  // namespace rviz_custom_plugins

#endif  // RVIZ_CUSTOM_PLUGINS__OVERLAY_TEXT_PANEL_HPP_
