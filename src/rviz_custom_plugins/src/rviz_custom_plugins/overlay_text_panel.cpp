#include <rviz_custom_plugins/overlay_text_panel.hpp>

namespace rviz_custom_plugins
{

OverlayTextPanel::OverlayTextPanel(QWidget * parent)
: rviz_common::Panel(parent), topic_name_("/your_custom_topic")
{
  // Create GUI elements
  topic_info_label_ = new QLabel("Topic:");
  topic_input_ = new QLineEdit(topic_name_);
  label_ = new QLabel("Waiting for message...");

  // Layouts
  QHBoxLayout * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(topic_info_label_);
  topic_layout->addWidget(topic_input_);

  QHBoxLayout * layout = new QHBoxLayout;
  layout->addWidget(label_);
  setLayout(layout);

  // Connect signals and slots
  connect(topic_input_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));
}

void OverlayTextPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();

  // Get the ROS node associated with RViz
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Subscribe to the initial topic
  subscribe();
}

void OverlayTextPanel::subscribe()
{
  if (!topic_name_.isEmpty()) {
    // Create a new subscription
    subscription_ = node_->create_subscription<std_msgs::msg::String>(
      topic_name_.toStdString(), 10,
      std::bind(&OverlayTextPanel::callback, this, std::placeholders::_1));

    label_->setText("Subscribed to " + topic_name_);
  }
}

void OverlayTextPanel::unsubscribe()
{
  // Reset the subscription
  subscription_.reset();
}

void OverlayTextPanel::updateTopic()
{
  // Get the new topic name from the input field
  QString new_topic = topic_input_->text();

  if (new_topic != topic_name_) {
    // Unsubscribe from the current topic
    unsubscribe();

    // Update the topic name
    topic_name_ = new_topic;

    // Subscribe to the new topic
    subscribe();
  }
}

void OverlayTextPanel::callback(const std_msgs::msg::String::SharedPtr msg)
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
  label_->setText(text);
}

void OverlayTextPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_name_);
}

void OverlayTextPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_input_->setText(topic);
    updateTopic();
  }
}

}  // namespace rviz_custom_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_custom_plugins::OverlayTextPanel, rviz_common::Panel)
