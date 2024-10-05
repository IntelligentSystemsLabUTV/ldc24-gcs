#ifndef RVIZ_CUSTOM_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
#define RVIZ_CUSTOM_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>

#include <QDialog>
#include <QImage>
#include <QLabel>
#include <QObject>

#include <rviz_common/ros_topic_display.hpp>

#include <dua_qos_cpp/dua_qos.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <dua_interfaces/msg/visual_targets.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rviz_custom_plugins
{

using geometry_msgs::msg::Pose;
using sensor_msgs::msg::Image;

typedef std::tuple<std::string, Pose, Image> Info;
typedef std::vector<Info> Infos;

class VisualTargetsDisplay
  : public rviz_common::RosTopicDisplay<dua_interfaces::msg::VisualTargets>
{
  Q_OBJECT

public:
  VisualTargetsDisplay();
  ~VisualTargetsDisplay() override;

protected:
  void onInitialize() override;
  void processMessage(dua_interfaces::msg::VisualTargets::ConstSharedPtr msg) override;

private:
  void createInteractiveMarker(
    const geometry_msgs::msg::Pose & pose,
    const std::string & id);
  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
    const std::string & id);
  void showImage(const std::string & id);

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unordered_map<std::string, Infos> map_;
  std::mutex mutex_;
};

}  // namespace rviz_custom_plugins

#endif  // RVIZ_CUSTOM_PLUGINS__VISUAL_TARGETS_DISPLAY_HPP_
