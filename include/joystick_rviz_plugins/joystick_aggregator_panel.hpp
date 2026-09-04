#ifndef JOYSTICK_RVIZ_PLUGINS__JOYSTICK_AGGREGATOR_PANEL_HPP_
#define JOYSTICK_RVIZ_PLUGINS__JOYSTICK_AGGREGATOR_PANEL_HPP_

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/subscription.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/properties/property_tree_widget.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <std_msgs/msg/string.hpp>

namespace joystick_rviz_plugins
{

class JoystickAggregatorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  JoystickAggregatorPanel();
  ~JoystickAggregatorPanel() override = default;

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateNode();

private:
  void topicsAvailableCallback(std_msgs::msg::String::ConstSharedPtr message);
  void topicsSubscribedCallback(std_msgs::msg::String::ConstSharedPtr message);
  void parametersCallback(
    std::shared_future<std::vector<rclcpp::Parameter>> result);
  void populateNamespaceOptions();
  void rebuildTopicListProperties();
  void refreshTopicOptions();
  void refreshParamFileOptions();
  void addTopic(const std::string & topic);
  void removeTopic(size_t index);
  void topicSelectionChanged(rviz_common::properties::EditableEnumProperty * property);
  void parameterChanged(rviz_common::properties::StringProperty * property, const char * name);
  std::vector<std::string> parseTopicList(const std::string & list_text);

  rviz_common::properties::Property * property_topics_;
  rviz_common::properties::EditableEnumProperty * property_add_topic_;
  rviz_common::properties::EditableEnumProperty * property_node_;
  rviz_common::properties::FilePickerProperty * property_param_path_;
  rviz_common::properties::EditableEnumProperty * property_param_file_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_available_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_subscribed_subscription_;
  rviz_common::properties::Property * property_root_;
  rviz_common::properties::PropertyTreeModel * property_model_;
  rviz_common::properties::PropertyTreeWidget * property_tree_;

  std::mutex mutex_;
  std::vector<std::string> topic_available_;
  std::vector<std::string> topic_subscribed_;
  std::string param_path_;
  std::string param_file_;
  std::optional<std::vector<std::string>> pending_topics_;
  std::vector<rviz_common::properties::EditableEnumProperty *> property_subscribed_topics_;
  std::vector<rviz_common::properties::BoolProperty *> property_remove_topics_;
  bool updating_properties_{false};
};

}  // namespace joystick_rviz_plugins

#endif  // JOYSTICK_RVIZ_PLUGINS__JOYSTICK_AGGREGATOR_PANEL_HPP_
