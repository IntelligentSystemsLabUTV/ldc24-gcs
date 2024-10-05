#include "rviz_custom_plugins/visual_targets_display.hpp"

#include <memory>
#include <string>

#include <QApplication>
#include <QLabel>
#include <QMetaObject>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <sensor_msgs/image_encodings.hpp>

namespace rviz_custom_plugins
{

VisualTargetsDisplay::VisualTargetsDisplay()
: rviz_common::RosTopicDisplay<dua_interfaces::msg::VisualTargets>()
{
}

VisualTargetsDisplay::~VisualTargetsDisplay()
{
  if (initialized()) {
    server_->clear();
    server_->applyChanges();
  }
}

void VisualTargetsDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  auto node = ros_node_abstraction->get_raw_node();
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "visual_targets", node);
}

void VisualTargetsDisplay::processMessage(dua_interfaces::msg::VisualTargets::ConstSharedPtr msg)
{
  mutex_.lock();
  server_->clear();
  for (const auto & detection : msg->targets.detections) {
    if (!detection.results.empty()) {
      std::string id = detection.results[0].hypothesis.class_id;
      std::replace(id.begin(), id.end(), ' ', '_');
      map_[id].push_back(msg->image);
      createInteractiveMarker(detection.results[0].pose.pose, id);
    }
  }
  server_->applyChanges();
  mutex_.unlock();
}

void VisualTargetsDisplay::createInteractiveMarker(
  const geometry_msgs::msg::Pose & pose,
  const std::string & id)
{
  // Create a marker for the COLLADA model
  visualization_msgs::msg::Marker mesh_marker;
  mesh_marker.header.set__stamp(rclcpp::Time(0));
  mesh_marker.header.set__frame_id("map");
  mesh_marker.set__type(visualization_msgs::msg::Marker::MESH_RESOURCE);
  mesh_marker.set__action(visualization_msgs::msg::Marker::ADD);
  mesh_marker.set__pose(pose);
  mesh_marker.scale.set__x(1.0);
  mesh_marker.scale.set__y(1.0);
  mesh_marker.scale.set__z(1.0);
  mesh_marker.color.set__r(1.0);
  mesh_marker.color.set__g(1.0);
  mesh_marker.color.set__b(1.0);
  mesh_marker.color.set__a(1.0);
  std::string mesh_resource = "file:////home/neo/workspace/src/gcs_bringup/dae/" + id + ".dae";
  mesh_marker.set__mesh_resource(mesh_resource);
  mesh_marker.set__mesh_use_embedded_materials(true);

  // Create a control for the marker
  visualization_msgs::msg::InteractiveMarkerControl mesh_control;
  mesh_control.set__name(id);
  mesh_control.set__orientation(pose.orientation);
  mesh_control.set__orientation_mode(visualization_msgs::msg::InteractiveMarkerControl::FIXED);
  mesh_control.set__interaction_mode(visualization_msgs::msg::InteractiveMarkerControl::BUTTON);
  mesh_control.set__always_visible(true);
  mesh_control.markers.push_back(mesh_marker);

  // Create an interactive marker
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.set__stamp(rclcpp::Time(0));
  int_marker.header.set__frame_id("map");
  int_marker.set__pose(pose);
  int_marker.set__name(id);
  int_marker.set__description(id);
  int_marker.set__scale(1.0);
  int_marker.controls.push_back(mesh_control);

  // Insert the interactive marker into the server and set the callback
  server_->insert(
    int_marker,
    std::bind(&VisualTargetsDisplay::processFeedback, this, std::placeholders::_1, id));
}

void VisualTargetsDisplay::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback,
  const std::string & id)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
    // Since this callback is in a different thread, use Qt's signal-slot mechanism
    QMetaObject::invokeMethod(
      this,
      [this, id]() {this->showImage(id);},
      Qt::QueuedConnection);
  }
}

void VisualTargetsDisplay::showImage(const std::string & id)
{
  mutex_.lock();
  // Create a dialog to display the images
  QDialog * dialog = new QDialog();
  dialog->setAttribute(Qt::WA_DeleteOnClose);  // Ensure the dialog is deleted when closed
  // Set the dialog title using the class_id in bold, uppercase and larger font
  std::string class_id = id;
  std::replace(class_id.begin(), class_id.end(), '_', ' ');
  std::transform(class_id.begin(), class_id.end(), class_id.begin(), ::toupper);
  QString title = "<b><font size='10'>" + QString::fromStdString(class_id) + "</font></b>";
  dialog->setWindowTitle(title);
  // Create a layout to display the images
  const std::vector<sensor_msgs::msg::Image> & images = map_[id];
  if (images.empty()) {
    mutex_.unlock();
    return;
  }
  // Iterate over the images and create a label for each image
  QHBoxLayout * main_layout = new QHBoxLayout(dialog);
  for (const auto & image : images) {
    // Convert sensor_msgs::msg::Image to QImage
    QImage qimage;
    const auto & encoding = image.encoding;
    if (encoding == sensor_msgs::image_encodings::RGB8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGB888);
    } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGBA8888);
    } else if (encoding == sensor_msgs::image_encodings::MONO8) {
      qimage = QImage(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_Grayscale8);
    } else if (encoding == sensor_msgs::image_encodings::BGR8) {
      // Need to convert BGR to RGB
      QImage temp(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGB888);
      qimage = temp.rgbSwapped();
    } else if (encoding == sensor_msgs::image_encodings::BGRA8) {
      // Need to convert BGRA to RGBA
      QImage temp(
        image.data.data(),
        image.width,
        image.height,
        QImage::Format_RGBA8888);
      qimage = temp.rgbSwapped();
    } else {
      // Log a warning using RCLCPP_WARN
      RCLCPP_WARN(
        rviz_ros_node_.lock()->get_raw_node()->get_logger(),
        "Unsupported image encoding: %s",
        encoding.c_str());
      continue;
    }
    // Ensure the image data is copied since the original data may be released
    qimage = qimage.copy();
    // Create a label to display the image
    QLabel * image_label = new QLabel(dialog);
    image_label->setPixmap(QPixmap::fromImage(qimage));
    image_label->setAlignment(Qt::AlignCenter);
    // Add the label to the layout
    main_layout->addWidget(image_label);
  }
  // Set the layout for the dialog
  dialog->setLayout(main_layout);
  // Show the dialog
  dialog->show();

  mutex_.unlock();
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::VisualTargetsDisplay, rviz_common::Display)
