#include "joystick_rviz_plugins/joystick_aggregator_panel.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <QDir>
#include <QFileDialog>
#include <QMetaObject>
#include <QSignalBlocker>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace joystick_rviz_plugins
{

class DirectoryPickerProperty : public rviz_common::properties::FilePickerProperty
{
public:
  using FilePickerProperty::FilePickerProperty;

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override
  {
    Q_UNUSED(option);
    const QString initial_path = getString();
    auto * dialog = new QFileDialog(parent, "Select parameter folder", initial_path);
    dialog->setFileMode(QFileDialog::Directory);
    dialog->setOption(QFileDialog::ShowDirsOnly, true);
    if (!initial_path.isEmpty()) {
      dialog->setDirectory(initial_path);
    }
    QObject::connect(dialog, &QFileDialog::fileSelected, this, [this, dialog](const QString & path) {
      setString(path);
      QMetaObject::invokeMethod(dialog, "accept", Qt::QueuedConnection);
    });
    QObject::connect(dialog, &QFileDialog::accepted, this, [this, dialog]() {
      const QStringList selected = dialog->selectedFiles();
      if (!selected.isEmpty()) {
        setString(selected.front());
      }
    });
    QObject::connect(dialog, &QFileDialog::rejected, this, [this, initial_path]() {
      const QSignalBlocker blocker(this);
      setString(initial_path);
    });
    return dialog;
  }
};

JoystickAggregatorPanel::JoystickAggregatorPanel()
: property_root_(new rviz_common::properties::Property(
    "Joystick Aggregator", QVariant(), "Currently subscribed joystick topics")),
  property_node_(nullptr), property_topics_(nullptr),
  property_model_(new rviz_common::properties::PropertyTreeModel(property_root_, this)),
  property_tree_(new rviz_common::properties::PropertyTreeWidget(this))
{
  property_node_ = new rviz_common::properties::EditableEnumProperty(
    "Node", "/teleop/aggregator_node",
    "joystick_tools_erdc aggregator node.", property_root_,
    SLOT(updateNode()), this);
  populateNamespaceOptions();
  QObject::connect(
    property_node_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    [this]() { populateNamespaceOptions(); });
  QObject::connect(
    property_node_, &rviz_common::properties::StringProperty::changed, this,
    [this]() { updateNode(); });
  property_param_path_ = new DirectoryPickerProperty(
    "Param path", "", "Folder used to save and load aggregator parameters", property_root_,
    nullptr, nullptr, QFileDialog::Directory);
  property_param_file_ = new rviz_common::properties::EditableEnumProperty(
    "Param file", "joystick_aggregator.yaml", "File name used to save and load aggregator parameters", property_root_);
  QObject::connect(
    property_param_path_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      refreshParamFileOptions();
      parameterChanged(property_param_path_, "param_path");
    });
  QObject::connect(
    property_param_file_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    [this]() { refreshParamFileOptions(); });
  QObject::connect(
    property_param_file_, &rviz_common::properties::StringProperty::changed, this,
    [this]() { parameterChanged(property_param_file_, "param_file"); });
  property_topics_ = new rviz_common::properties::Property(
      "Subscribed topics", QVariant(), "Editable list of currently subscribed joystick topics", property_root_);
  property_add_topic_ = new rviz_common::properties::EditableEnumProperty(
      "Add topic", QString(), "Add an available joystick topic", property_topics_);
  QObject::connect(
    property_add_topic_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    [this]() { refreshTopicOptions(); });
  QObject::connect(
    property_add_topic_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      const std::string topic = property_add_topic_->getStdString();
      if (!topic.empty()) {
        addTopic(topic);
        property_add_topic_->setString("");
      }
    });
  property_tree_->setModel(property_model_);
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(property_tree_);
}

void JoystickAggregatorPanel::onInitialize()
{
  updateNode();
  rebuildTopicListProperties();
}

void JoystickAggregatorPanel::parametersCallback(
  std::shared_future<std::vector<rclcpp::Parameter>> result)
{
  const auto parameters = result.get();
  std::vector<std::string> topics;
  std::string param_path;
  std::string param_file;
  bool has_topics = false;
  bool has_param_path = false;
  bool has_param_file = false;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "topics" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      topics = parameter.as_string_array();
      has_topics = true;
    } else if (parameter.get_name() == "param_path" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      param_path = parameter.as_string();
      has_param_path = true;
    } else if (parameter.get_name() == "param_file" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      param_file = parameter.as_string();
      has_param_file = true;
    }
  }

  bool topics_changed = false;
  bool param_path_changed = false;
  bool param_file_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_topics) {
      if (!pending_topics_ || *pending_topics_ == topics) {
        pending_topics_.reset();
        topics_changed = topic_subscribed_ != topics;
        topic_subscribed_ = topics;
      } else {
        has_topics = false;
      }
    }
    if (has_param_path) {
      param_path_changed = param_path_ != param_path;
      param_path_ = param_path;
    }
    if (has_param_file) {
      param_file_changed = param_file_ != param_file;
      param_file_ = param_file;
    }
  }

  if (!topics_changed && !param_path_changed && !param_file_changed) {
    return;
  }

  QMetaObject::invokeMethod(this, [this, topics_changed, param_path_changed, param_file_changed,
    param_path, param_file]() {
    updating_properties_ = true;
    if (topics_changed) {
      rebuildTopicListProperties();
    }
    if (param_path_changed && property_param_path_->getStdString() != param_path) {
      property_param_path_->setString(QString::fromStdString(param_path));
    }
    if (param_file_changed && property_param_file_->getStdString() != param_file) {
      property_param_file_->setString(QString::fromStdString(param_file));
    }
    updating_properties_ = false;
  }, Qt::QueuedConnection);
}

void JoystickAggregatorPanel::parameterChanged(
  rviz_common::properties::StringProperty * property, const char * name)
{
  if (!property || !parameters_client_) {
    return;
  }

  if (updating_properties_) {
    return;
  }

  parameters_client_->set_parameters({rclcpp::Parameter(name, property->getStdString())});
}

void JoystickAggregatorPanel::refreshParamFileOptions()
{
  if (!property_param_file_ || !property_param_path_) {
    return;
  }

  property_param_file_->clearOptions();
  const QDir directory(QString::fromStdString(property_param_path_->getStdString()));
  const QStringList filters{"*.yaml", "*.yml"};
  const QFileInfoList files = directory.entryInfoList(
    filters, QDir::Files | QDir::Readable, QDir::Name);
  const QString current_file = property_param_file_->getString();
  bool current_file_found = false;
  for (const auto & file : files) {
    property_param_file_->addOption(file.fileName());
    current_file_found = current_file_found || file.fileName() == current_file;
  }

  if (!current_file.isEmpty() && !current_file_found) {
    property_param_file_->addOption(current_file);
  }
  property_param_file_->sortOptions();
}

void JoystickAggregatorPanel::topicsAvailableCallback(std_msgs::msg::String::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_available_ = parseTopicList(message->data);
  }
}

void JoystickAggregatorPanel::topicsSubscribedCallback(std_msgs::msg::String::ConstSharedPtr message)
{
  (void)message;
}

void JoystickAggregatorPanel::updateNode()
{
  parameters_client_.reset();
  topic_available_subscription_.reset();
  topic_subscribed_subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_topics_.reset();
  }

  if (!getDisplayContext()) {
    return;
  }

  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  std::string namespace_value = property_node_ ? property_node_->getStdString() : "/teleop/aggregator_node";
  while (!namespace_value.empty() && namespace_value.back() == '/') {
    namespace_value.pop_back();
  }
  if (namespace_value.empty()) {
    namespace_value = "/teleop/aggregator_node";
  }

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, namespace_value);
  parameters_client_->get_parameters(
    {"topics", "param_path", "param_file"},
    std::bind(&JoystickAggregatorPanel::parametersCallback, this, std::placeholders::_1));

  const std::string available_topic = namespace_value + "/topics_available";
  const std::string subscribed_topic = namespace_value + "/topics_subscribed";

  topic_available_subscription_ = node->create_subscription<std_msgs::msg::String>(
    available_topic, rclcpp::SensorDataQoS(),
    std::bind(&JoystickAggregatorPanel::topicsAvailableCallback, this, std::placeholders::_1));
  topic_subscribed_subscription_ = node->create_subscription<std_msgs::msg::String>(
    subscribed_topic, rclcpp::SensorDataQoS(),
    std::bind(&JoystickAggregatorPanel::topicsSubscribedCallback, this, std::placeholders::_1));
}

void JoystickAggregatorPanel::rebuildTopicListProperties()
{
  std::lock_guard<std::mutex> lock(mutex_);
  property_topics_->removeChildren(1);
  property_subscribed_topics_.clear();
  property_remove_topics_.clear();

  for (size_t index = 0; index < topic_subscribed_.size(); ++index) {
    const auto & topic = topic_subscribed_[index];
    auto * property = new rviz_common::properties::EditableEnumProperty(
      QString::fromStdString("Topic " + std::to_string(index)),
      QString::fromStdString(topic),
      "Currently subscribed topic",
      property_topics_);

    for (const auto & available_topic : topic_available_) {
      property->addOptionStd(available_topic);
    }
    if (std::find(topic_available_.begin(), topic_available_.end(), topic) == topic_available_.end()) {
      property->addOptionStd(topic);
    }
    property->sortOptions();
    property_subscribed_topics_.push_back(property);
    auto * remove_property = new rviz_common::properties::BoolProperty(
      "Remove", false, "Remove this subscribed topic", property);
    property_remove_topics_.push_back(remove_property);
    QObject::connect(
      remove_property, &rviz_common::properties::BoolProperty::changed, this,
      [this, index]() { removeTopic(index); });
    QObject::connect(
      property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
      [this, property]() {
        property->clearOptions();
        for (const auto & available_topic : topic_available_) {
          property->addOptionStd(available_topic);
        }
        for (const auto & topic : topic_subscribed_) {
          if (std::find(topic_available_.begin(), topic_available_.end(), topic) == topic_available_.end()) {
            property->addOptionStd(topic);
          }
        }
        property->sortOptions();
      });
    QObject::connect(
      property, &rviz_common::properties::StringProperty::changed, this,
      [this, property]() { topicSelectionChanged(property); });
  }
}

void JoystickAggregatorPanel::refreshTopicOptions()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!property_add_topic_) {
    return;
  }

  property_add_topic_->clearOptions();
  for (const auto & topic : topic_available_) {
    if (std::find(topic_subscribed_.begin(), topic_subscribed_.end(), topic) == topic_subscribed_.end()) {
      property_add_topic_->addOptionStd(topic);
    }
  }
  property_add_topic_->sortOptions();
}

void JoystickAggregatorPanel::addTopic(const std::string & topic)
{
  std::vector<std::string> topics;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::find(topic_subscribed_.begin(), topic_subscribed_.end(), topic) != topic_subscribed_.end()) {
      return;
    }
    topic_subscribed_.push_back(topic);
    topics = topic_subscribed_;
    pending_topics_ = topics;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("topics", topics)});
  }
  rebuildTopicListProperties();
  refreshTopicOptions();
}

void JoystickAggregatorPanel::removeTopic(size_t index)
{
  std::vector<std::string> topics;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= topic_subscribed_.size()) {
      return;
    }
    topic_subscribed_.erase(topic_subscribed_.begin() + index);
    topics = topic_subscribed_;
    pending_topics_ = topics;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("topics", topics)});
  }
  rebuildTopicListProperties();
  refreshTopicOptions();
}




void JoystickAggregatorPanel::populateNamespaceOptions()
{
  if (!property_node_) {
    return;
  }

  property_node_->clearOptions();
  std::unordered_set<std::string> node_options;
  const auto add_node_option = [this, &node_options](const std::string & node_name) {
      if (!node_name.empty() && node_options.insert(node_name).second) {
        property_node_->addOptionStd(node_name);
      }
    };

  if (getDisplayContext()) {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    if (node) {
      for (const auto & node_entry : node->get_node_graph_interface()->get_node_names_and_namespaces()) {
        const std::string & node_name = node_entry.first;
        const std::string & node_namespace = node_entry.second;

        std::string full_name = node_namespace;
        if (full_name.empty() || full_name == "/") {
          full_name = "/" + node_name;
        } else {
          if (full_name.back() != '/') {
            full_name += "/";
          }
          full_name += node_name;
        }

        const std::string available_topic = full_name + "/topics_available";
        const std::string subscribed_topic = full_name + "/topics_subscribed";
        const auto available_publishers = node->get_publishers_info_by_topic(available_topic);
        const auto subscribed_publishers = node->get_publishers_info_by_topic(subscribed_topic);
        const auto is_topic_list_publisher = [](const auto & publisher_info) {
            return publisher_info.topic_type() == "std_msgs/msg/String";
          };
        const bool publishes_available = std::any_of(
          available_publishers.begin(), available_publishers.end(), is_topic_list_publisher);
        const bool publishes_subscribed = std::any_of(
          subscribed_publishers.begin(), subscribed_publishers.end(), is_topic_list_publisher);

        if (publishes_available && publishes_subscribed) {
          add_node_option(full_name);
        }
      }
    }
  }

  const std::string current_value = property_node_->getStdString();
  add_node_option(current_value);

  const std::string default_value = "/teleop/aggregator_node";
  add_node_option(default_value);
  property_node_->sortOptions();
}

void JoystickAggregatorPanel::topicSelectionChanged(
  rviz_common::properties::EditableEnumProperty * property)
{
  if (!property) {
    return;
  }

  const std::string value = property->getStdString();
  if (value.empty()) {
    return;
  }

  std::vector<std::string> topics;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t index = 0; index < property_subscribed_topics_.size(); ++index) {
      if (property_subscribed_topics_[index] == property) {
        if (index < topic_subscribed_.size()) {
          topic_subscribed_[index] = value;
        }
        break;
      }
    }
    topics = topic_subscribed_;
    pending_topics_ = topics;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("topics", topics)});
  }
}

std::vector<std::string> JoystickAggregatorPanel::parseTopicList(const std::string & list_text)
{
  std::vector<std::string> topics;
  if (list_text.empty()) {
    return topics;
  }

  std::string text = list_text;
  while (!text.empty() && (text.front() == ' ' || text.front() == '\t' || text.front() == '\n')) {
    text.erase(text.begin());
  }
  while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\n')) {
    text.pop_back();
  }

  if (text.size() < 2 || text.front() != '[' || text.back() != ']') {
    return topics;
  }

  std::stringstream stream(text.substr(1, text.size() - 2));
  std::string item;
  while (std::getline(stream, item, ',')) {
    while (!item.empty() && (item.front() == ' ' || item.front() == '\t' || item.front() == '\n')) {
      item.erase(item.begin());
    }
    while (!item.empty() && (item.back() == ' ' || item.back() == '\t' || item.back() == '\n')) {
      item.pop_back();
    }
    if (!item.empty()) {
      topics.push_back(item);
    }
  }
  return topics;
}

}  // namespace joystick_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  joystick_rviz_plugins::JoystickAggregatorPanel,
  rviz_common::Panel)
