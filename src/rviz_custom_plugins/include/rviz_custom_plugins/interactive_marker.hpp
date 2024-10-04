#ifndef RVIZ_CUSTOM_PLUGINS__INTERACTIVE_MARKER_HPP_
#define RVIZ_CUSTOM_PLUGINS__INTERACTIVE_MARKER_HPP_

#include <rviz_common/display.hpp>
#include <rviz_rendering/objects/mesh_resource_marker.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QMessageBox>
#include <QImage>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>

namespace rviz_custom_plugins
{

class InteractiveMarker : public rviz_common::Display
{
Q_OBJECT

public:
  InteractiveMarker();
  virtual ~InteractiveMarker() = default;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private:
  void processMarkerMessage(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void createInteractiveMarker(const vision_msgs::msg::Detection2D &detection, const sensor_msgs::msg::Image &image);
  void showImagePopup(const sensor_msgs::msg::Image &image);

  rviz_common::properties::StringProperty *topic_property_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr marker_subscriber_;
};

}  // namespace rviz_custom_plugins

#endif  // RVIZ_CUSTOM_PLUGINS__INTERACTIVE_MARKER_HPP_
