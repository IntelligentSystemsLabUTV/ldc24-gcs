#include <rviz_custom_plugins/interactive_marker.hpp>

namespace rviz_custom_plugins
{

InteractiveMarker::InteractiveMarker()
{
  // Property for subscribing to the visual target topic
  topic_property_ = new rviz_common::properties::StringProperty(
    "Visual Target Topic", "/visual_target_topic",
    "The topic on which to subscribe to receive marker locations and images", this);
}

void InteractiveMarker::onInitialize()
{
  // Initialize the ROS2 node for subscribing to topics
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // Subscribe to the visual target topic
  marker_subscriber_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    topic_property_->getStdString(), 10,
    std::bind(&InteractiveMarker::processMarkerMessage, this, std::placeholders::_1));
}

void InteractiveMarker::onEnable()
{
  // Here you can handle what happens when the plugin is enabled
}

void InteractiveMarker::onDisable()
{
  // Here you can handle what happens when the plugin is disabled
}

void InteractiveMarker::processMarkerMessage(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // Iterate over each detection in the message
  for (const auto & detection : msg->detections) {
    // Create an interactive marker for each detection
    createInteractiveMarker(detection, msg->image);
  }
}

void InteractiveMarker::createInteractiveMarker(
  const vision_msgs::msg::Detection2D & detection,
  const sensor_msgs::msg::Image & image)
{
  // Create a MeshResourceMarker to render the COLLADA model at the specified location
  auto mesh_marker = std::make_shared<rviz_rendering::MeshResourceMarker>(
    scene_manager_, scene_node_);

  // Set the pose and orientation using the detection info
  const auto & position = detection.results[0].pose.pose.position;
  const auto & orientation = detection.results[0].pose.pose.orientation;

  mesh_marker->setPosition(Ogre::Vector3(position.x, position.y, position.z));
  mesh_marker->setOrientation(
    Ogre::Quaternion(
      orientation.w, orientation.x, orientation.y,
      orientation.z));
  mesh_marker->setVisible(true);

  // Set up an interaction callback for showing a pop-up with the image
  connect(
    this, &InteractiveMarker::onMouseEvent, [this, image](
      rviz_common::ViewportMouseEvent & event) {
      if (event.leftDown()) {
        showImagePopup(image);
      }
    });
}

void InteractiveMarker::showImagePopup(const sensor_msgs::msg::Image & image)
{
  // Convert the ROS image message to a Qt image
  QImage qimage;
  if (image.encoding == "rgb8") {
    qimage = QImage(&image.data[0], image.width, image.height, QImage::Format_RGB888);
  }
  // Add more conversions if needed based on different encodings
  else {
    // Handle unsupported encoding types
    RCLCPP_WARN(node_->get_logger(), "Unsupported image encoding: %s", image.encoding.c_str());
    return;
  }

  // Create a QMessageBox to show the image
  QMessageBox msgBox;
  msgBox.setWindowTitle("Marker Clicked!");
  msgBox.setText("Displaying image for clicked marker.");

  // Add an image to the message box
  QLabel * imageLabel = new QLabel;
  imageLabel->setPixmap(QPixmap::fromImage(qimage));

  // Use a layout to display the image within the QMessageBox
  QVBoxLayout * layout = new QVBoxLayout();
  layout->addWidget(imageLabel);
  msgBox.setLayout(layout);

  msgBox.exec();
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::InteractiveMarker, rviz_common::Display)
