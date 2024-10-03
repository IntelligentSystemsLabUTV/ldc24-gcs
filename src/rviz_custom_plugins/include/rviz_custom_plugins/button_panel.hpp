#ifndef RVIZ_CUSTOM_PLUGINS__BUTTON_PANEL_HPP_
#define RVIZ_CUSTOM_PLUGINS__BUTTON_PANEL_HPP_

#include <dua_qos_cpp/dua_qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace rviz_custom_plugins
{

class ButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  ButtonPanel(QWidget * parent = nullptr);

private Q_SLOTS:
  void startButtonClicked();
  void stopButtonClicked();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_start_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_stop_;
  QPushButton * button_start_;
  QPushButton * button_stop_;
};

} // namespace rviz_custom_plugins

#endif  // RVIZ_CUSTOM_PLUGINS__BUTTON_PANEL_HPP_
