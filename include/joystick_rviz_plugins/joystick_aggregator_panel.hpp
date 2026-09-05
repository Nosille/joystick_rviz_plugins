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
#include <rclcpp/parameter_event_handler.hpp>
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
#include <std_srvs/srv/trigger.hpp>

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
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void updateNode();

private:
  void topicsAvailableCallback(std_msgs::msg::String::ConstSharedPtr message);
  void topicsSubscribedCallback(std_msgs::msg::String::ConstSharedPtr message);
  void joysticksAvailableCallback(std_msgs::msg::String::ConstSharedPtr message);
  void parametersCallback(
    std::shared_future<std::vector<rclcpp::Parameter>> result);
  void axisParametersCallback(
    std::shared_future<std::vector<rclcpp::Parameter>> result);
  void requestAxisParameters(const std::vector<std::string> & axes);
  void buttonParametersCallback(
    std::shared_future<std::vector<rclcpp::Parameter>> result);
  void requestButtonParameters(const std::vector<std::string> & buttons);
  void populateNamespaceOptions();
  void rebuildAxisProperties();
  void rebuildButtonProperties();
  void rebuildTopicListProperties();
  void refreshTopicOptions();
  void refreshParamFileOptions();
  void addAxis(const std::string & axis);
  void removeAxis(size_t index);
  void addButton(const std::string & button);
  void removeButton(size_t index);
  void addTopic(const std::string & topic);
  void removeTopic(size_t index);
  void topicSelectionChanged(rviz_common::properties::EditableEnumProperty * property);
  void parameterChanged(rviz_common::properties::StringProperty * property, const char * name);
  void loadParameters();
  void saveParameters();
  void requestNodeParameters();
  std::vector<std::string> parseTopicList(const std::string & list_text);

  rviz_common::properties::Property * property_root_;
  rviz_common::properties::PropertyTreeModel * property_model_;
  rviz_common::properties::PropertyTreeWidget * property_tree_;
  rviz_common::properties::EditableEnumProperty * property_node_;
  rviz_common::properties::FilePickerProperty * property_param_path_;
  rviz_common::properties::EditableEnumProperty * property_param_file_;

  rviz_common::properties::Property * property_topics_;
  rviz_common::properties::EditableEnumProperty * property_add_topic_;
  rviz_common::properties::Property * property_axes_;
  rviz_common::properties::StringProperty * property_add_axis_;
  rviz_common::properties::Property * property_buttons_;
  rviz_common::properties::StringProperty * property_add_button_;
  rviz_common::properties::BoolProperty * property_load_parameters_;
  rviz_common::properties::BoolProperty * property_save_parameters_;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_event_callback_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_topic_available_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_topic_subscribed_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_joysticks_available_;

  std::mutex mutex_;
  std::vector<std::string> topics_selected_;
  std::vector<std::string> topics_available_;
  std::vector<std::string> topics_subscribed_;
  std::string joysticks_available_message_;
  std::string param_path_;
  std::string param_file_;
  std::vector<std::string> axis_names_;
  std::unordered_map<std::string, std::string> axis_parameters_;
  std::unordered_map<rviz_common::properties::StringProperty *, std::string> axis_property_names_;
  std::vector<std::string> button_names_;
  std::unordered_map<std::string, std::string> button_parameters_;
  std::unordered_map<rviz_common::properties::StringProperty *, std::string> button_property_names_;
  std::optional<std::vector<std::string>> pending_topics_;
  std::vector<rviz_common::properties::EditableEnumProperty *> property_subscribed_topics_;
  std::vector<rviz_common::properties::BoolProperty *> property_remove_topics_;
  bool updating_properties_{false};
};

}  // namespace joystick_rviz_plugins

#endif  // JOYSTICK_RVIZ_PLUGINS__JOYSTICK_AGGREGATOR_PANEL_HPP_
