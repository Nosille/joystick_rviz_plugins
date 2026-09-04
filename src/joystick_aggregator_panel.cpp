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
namespace
{
std::vector<std::string> parseJoystickValues(
  const std::string & text,
  const std::string & property_key,
  const std::string & topic_filter,
  const std::string & list_key,
  const std::string & host_filter = "",
  const std::string & index_filter = "",
  const std::string & name_filter = "",
  const std::string & serial_filter = "",
  const std::string & guid_filter = "")
{
  std::vector<std::string> values;
  if (topic_filter.empty()) {
    return values;
  }

  const std::string key = property_key == "joy_host" ? "host" : property_key;
  const std::string topic_key = "topic:";
  size_t record_position = 0;
  while ((record_position = text.find(topic_key, record_position)) != std::string::npos) {
    const size_t record_end = text.find(topic_key, record_position + topic_key.size());
    const std::string record = text.substr(
      record_position, record_end == std::string::npos ? std::string::npos : record_end - record_position);

    const auto parse_value = [&record](const std::string & field_key) {
        const size_t field_position = record.find(field_key + ":");
        if (field_position == std::string::npos) {
          return std::string();
        }
        size_t value_position = field_position + field_key.size() + 1;
        while (value_position < record.size() &&
          (record[value_position] == ' ' || record[value_position] == '\t')) {
          ++value_position;
        }
        size_t value_end = value_position;
        while (value_end < record.size() && record[value_end] != ',' &&
          record[value_end] != ']' && record[value_end] != '\n') {
          ++value_end;
        }
        std::string value = record.substr(value_position, value_end - value_position);
        while (!value.empty() && (value.front() == ' ' || value.front() == '\t' || value.front() == '\n')) {
          value.erase(value.begin());
        }
        while (!value.empty() && (value.back() == ' ' || value.back() == '\t' || value.back() == '\n')) {
          value.pop_back();
        }
        return value;
      };

    const auto matches_filter = [&parse_value](
      const std::string & field, const std::string & filter) {
        return filter.empty() || filter == "any" || parse_value(field) == filter;
      };

    if (parse_value("topic") == topic_filter &&
      matches_filter("host", host_filter) &&
      matches_filter("joy_index", index_filter) &&
      matches_filter("joy_name", name_filter) &&
      matches_filter("joy_serial", serial_filter) &&
      matches_filter("joy_guid", guid_filter)) {
      if (property_key == "joy_label") {
        const size_t list_position = record.find(list_key + ":");
        if (list_position != std::string::npos) {
          const size_t values_start = record.find('[', list_position);
          const size_t values_end = record.find(']', values_start);
          if (values_start != std::string::npos && values_end != std::string::npos) {
            std::stringstream axes(record.substr(values_start + 1, values_end - values_start - 1));
            std::string value;
            while (std::getline(axes, value, ',')) {
              while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.erase(value.begin());
              }
              while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.pop_back();
              }
              if (!value.empty()) {
                values.push_back(value);
              }
            }
          }
        }
      } else {
        const std::string value = parse_value(key);
        if (!value.empty()) {
          values.push_back(value);
        }
      }
    }
    if (record_end == std::string::npos) {
      break;
    }
    record_position = record_end;
  }

  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

}  // namespace

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
  property_axes_(nullptr), property_buttons_(nullptr),
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
    [this]() {populateNamespaceOptions();});
  QObject::connect(
    property_node_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {updateNode();});
  property_param_path_ = new DirectoryPickerProperty(
    "Param path", "", "Folder used to save and load aggregator parameters", property_node_,
    nullptr, nullptr, QFileDialog::Directory);
  property_param_file_ = new rviz_common::properties::EditableEnumProperty(
    "Param file", "joystick_aggregator.yaml", "File name used to save and load aggregator parameters", property_node_);
  QObject::connect(
    property_param_path_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      refreshParamFileOptions();
      parameterChanged(property_param_path_, "param_path");
    });
  QObject::connect(
    property_param_file_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    [this]() {refreshParamFileOptions();});
  QObject::connect(
    property_param_file_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {parameterChanged(property_param_file_, "param_file");});
  property_topics_ = new rviz_common::properties::Property(
      "Subscribed topics", QVariant(), "Editable list of currently subscribed joystick topics", property_root_);
  property_add_topic_ = new rviz_common::properties::EditableEnumProperty(
      "Add topic", QString(), "Add an available joystick topic", property_topics_);
  QObject::connect(
    property_add_topic_, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
    [this]() {refreshTopicOptions();});
  QObject::connect(
    property_add_topic_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      const std::string topic = property_add_topic_->getStdString();
      if (!topic.empty()) {
        addTopic(topic);
        property_add_topic_->setString("");
      }
    });
  property_axes_ = new rviz_common::properties::Property(
    "Axes", QVariant(), "Editable joystick axis mappings", property_root_);
  property_add_axis_ = new rviz_common::properties::StringProperty(
    "Add axis", QString(), "Add a joystick axis mapping", property_axes_);
  QObject::connect(
    property_add_axis_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      const std::string axis = property_add_axis_->getStdString();
      if (!axis.empty()) {
        addAxis(axis);
        property_add_axis_->setString("");
      }
    });
  property_buttons_ = new rviz_common::properties::Property(
    "Buttons", QVariant(), "Editable joystick button mappings", property_root_);
  property_add_button_ = new rviz_common::properties::StringProperty(
    "Add button", QString(), "Add a joystick button mapping", property_buttons_);
  QObject::connect(
    property_add_button_, &rviz_common::properties::StringProperty::changed, this,
    [this]() {
      const std::string button = property_add_button_->getStdString();
      if (!button.empty()) {
        addButton(button);
        property_add_button_->setString("");
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
  std::vector<std::string> axes;
  std::vector<std::string> buttons;
  bool has_topics = false;
  bool has_param_path = false;
  bool has_param_file = false;
  bool has_axes = false;
  bool has_buttons = false;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "topics" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      topics = parameter.as_string_array();
      has_topics = true;
    } else if (parameter.get_name() == "param_path" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      param_path = parameter.as_string();
      has_param_path = true;
    } else if (parameter.get_name() == "param_file" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      param_file = parameter.as_string();
      has_param_file = true;
    } else if (parameter.get_name() == "axes" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      axes = parameter.as_string_array();
      has_axes = true;
    } else if (parameter.get_name() == "buttons" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      buttons = parameter.as_string_array();
      has_buttons = true;
    }
  }

  if (has_axes) {
    requestAxisParameters(axes);
  }
  if (has_buttons) {
    requestButtonParameters(buttons);
  }

  bool topics_changed = false;
  bool param_path_changed = false;
  bool param_file_changed = false;
  bool axes_changed = false;
  bool buttons_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_topics) {
      if (!pending_topics_ || *pending_topics_ == topics) {
        pending_topics_.reset();
        topics_changed = topics_selected_ != topics;
        topics_selected_ = topics;
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
    if (has_axes) {
      axes_changed = axis_names_ != axes;
      axis_names_ = axes;
      if (axes_changed) {
        axis_parameters_.clear();
      }
    }
    if (has_buttons) {
      buttons_changed = button_names_ != buttons;
      button_names_ = buttons;
      if (buttons_changed) {
        button_parameters_.clear();
      }
    }
  }

  if (!topics_changed && !param_path_changed && !param_file_changed && !axes_changed && !buttons_changed) {
    return;
  }

  QMetaObject::invokeMethod(this, [this, topics_changed, param_path_changed, param_file_changed,
    axes_changed, buttons_changed, param_path, param_file]() {
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
      if (axes_changed && axis_parameters_.empty()) {
        rebuildAxisProperties();
      }
      if (buttons_changed && button_parameters_.empty()) {
        rebuildButtonProperties();
      }
      updating_properties_ = false;
  }, Qt::QueuedConnection);
}

void JoystickAggregatorPanel::axisParametersCallback(
  std::shared_future<std::vector<rclcpp::Parameter>> result)
{
  const auto parameters = result.get();
  std::unordered_map<std::string, std::string> values;
  for (const auto & parameter : parameters) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      values[parameter.get_name()] = parameter.as_string();
    }
  }

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = axis_parameters_ != values;
    axis_parameters_ = std::move(values);
  }
  if (changed) {
    QMetaObject::invokeMethod(this, [this]() {rebuildAxisProperties();}, Qt::QueuedConnection);
  }
}

void JoystickAggregatorPanel::requestAxisParameters(const std::vector<std::string> & axes)
{
  if (!parameters_client_) {
    return;
  }

  std::vector<std::string> axis_parameter_names;
  const std::vector<std::string> fields{
    "topic", "joy_host", "joy_index", "joy_name", "joy_serial", "joy_guid", "joy_label"};
  for (const auto & axis : axes) {
    for (const auto & field : fields) {
      axis_parameter_names.push_back(axis + "." + field);
    }
  }
  if (!axis_parameter_names.empty()) {
    parameters_client_->get_parameters(
      axis_parameter_names,
      std::bind(&JoystickAggregatorPanel::axisParametersCallback, this, std::placeholders::_1));
  }
}

void JoystickAggregatorPanel::buttonParametersCallback(
  std::shared_future<std::vector<rclcpp::Parameter>> result)
{
  const auto parameters = result.get();
  std::unordered_map<std::string, std::string> values;
  for (const auto & parameter : parameters) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      values[parameter.get_name()] = parameter.as_string();
    }
  }

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = button_parameters_ != values;
    button_parameters_ = std::move(values);
  }
  if (changed) {
    QMetaObject::invokeMethod(this, [this]() {rebuildButtonProperties();}, Qt::QueuedConnection);
  }
}

void JoystickAggregatorPanel::requestButtonParameters(const std::vector<std::string> & buttons)
{
  if (!parameters_client_) {
    return;
  }

  std::vector<std::string> parameter_names;
  const std::vector<std::string> fields{
    "topic", "joy_host", "joy_index", "joy_name", "joy_serial", "joy_guid", "joy_label", "joy_min"};
  for (const auto & button : buttons) {
    for (const auto & field : fields) {
      parameter_names.push_back(button + "." + field);
    }
  }
  if (!parameter_names.empty()) {
    parameters_client_->get_parameters(
      parameter_names,
      std::bind(&JoystickAggregatorPanel::buttonParametersCallback, this, std::placeholders::_1));
  }
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

void JoystickAggregatorPanel::rebuildAxisProperties()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!property_axes_ || !property_add_axis_) {
    return;
  }

  property_axes_->removeChildren(1);
  axis_property_names_.clear();

  const std::vector<std::string> fields{
    "topic", "joy_host", "joy_index", "joy_name", "joy_serial", "joy_guid", "joy_label"};
  for (size_t index = 0; index < axis_names_.size(); ++index) {
    const auto & axis_name = axis_names_[index];
    auto * axis_property = new rviz_common::properties::Property(
      QString::fromStdString(axis_name), QVariant(), "Axis mapping", property_axes_);
    rviz_common::properties::EditableEnumProperty * topic_property = nullptr;
    for (const auto & field : fields) {
      const std::string parameter_name = axis_name + "." + field;
      const auto value = axis_parameters_.find(parameter_name);
      auto * field_property = field == "topic" || field == "joy_host" ||
        field == "joy_index" || field == "joy_name" || field == "joy_serial" ||
        field == "joy_guid" || field == "joy_label" ?
        new rviz_common::properties::EditableEnumProperty(
          QString::fromStdString(field),
          value == axis_parameters_.end() ? QString() : QString::fromStdString(value->second),
          "Axis parameter", axis_property) :
        new rviz_common::properties::StringProperty(
          QString::fromStdString(field),
          value == axis_parameters_.end() ? QString() : QString::fromStdString(value->second),
          "Axis parameter", axis_property);
      if (field == "topic") {
        topic_property = static_cast<rviz_common::properties::EditableEnumProperty *>(field_property);
        topic_property->clearOptions();
        for (const auto & topic : topics_subscribed_) {
          topic_property->addOptionStd(topic);
        }
        const std::string current_value = value == axis_parameters_.end() ? std::string() : value->second;
        if (!current_value.empty() && std::find(topics_subscribed_.begin(), topics_subscribed_.end(),
            current_value) == topics_subscribed_.end()) {
          topic_property->addOptionStd(current_value);
        }
        topic_property->sortOptions();
        QObject::connect(
          topic_property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
          [this, topic_property]() {
            topic_property->clearOptions();
            for (const auto & topic : topics_subscribed_) {
              topic_property->addOptionStd(topic);
            }
            const std::string current_value = topic_property->getStdString();
            if (!current_value.empty() && std::find(topics_subscribed_.begin(), topics_subscribed_.end(),
              current_value) == topics_subscribed_.end()) {
              topic_property->addOptionStd(current_value);
            }
            topic_property->sortOptions();
          });
      } else if (field == "joy_host" || field == "joy_index" || field == "joy_name" ||
        field == "joy_serial" || field == "joy_guid" || field == "joy_label") {
        auto * joystick_property = static_cast<rviz_common::properties::EditableEnumProperty *>(field_property);
        QObject::connect(
          joystick_property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
          [this, joystick_property, topic_property, field, axis_name]() {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto parameter_value = [this, &axis_name](const std::string & name) {
              const auto value = axis_parameters_.find(axis_name + "." + name);
              return value == axis_parameters_.end() ? std::string("any") : value->second;
            };
            const auto values = parseJoystickValues(
              joysticks_available_message_, field, topic_property->getStdString(), "axes",
              field == "joy_label" ? parameter_value("joy_host") : "",
              field == "joy_label" ? parameter_value("joy_index") : "",
              field == "joy_label" ? parameter_value("joy_name") : "",
              field == "joy_label" ? parameter_value("joy_serial") : "",
              field == "joy_label" ? parameter_value("joy_guid") : "");
            joystick_property->clearOptions();
            if (field != "joy_label") {
              joystick_property->addOptionStd("any");
            }
            for (const auto & value : values) {
              joystick_property->addOptionStd(value);
            }
            const std::string current_value = joystick_property->getStdString();
            if (!current_value.empty() && current_value != "any" &&
            std::find(values.begin(), values.end(), current_value) == values.end()) {
              joystick_property->addOptionStd(current_value);
            }
            joystick_property->sortOptions();
          });
      }
      axis_property_names_[field_property] = parameter_name;
      QObject::connect(
        field_property, &rviz_common::properties::StringProperty::changed, this,
        [this, field_property]() {
          const auto parameter = axis_property_names_.find(field_property);
          if (parameter != axis_property_names_.end()) {
            parameterChanged(field_property, parameter->second.c_str());
          }
        });
    }
    auto * remove_property = new rviz_common::properties::BoolProperty(
      "Remove", false, "Remove this axis mapping", axis_property);
    QObject::connect(
      remove_property, &rviz_common::properties::BoolProperty::changed, this,
      [this, index]() {removeAxis(index);});
  }
}

void JoystickAggregatorPanel::rebuildButtonProperties()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!property_buttons_ || !property_add_button_) {
    return;
  }

  property_buttons_->removeChildren(1);
  button_property_names_.clear();
  const std::vector<std::string> fields{
    "topic", "joy_host", "joy_index", "joy_name", "joy_serial", "joy_guid", "joy_label", "joy_min"};
  for (size_t index = 0; index < button_names_.size(); ++index) {
    const auto & button_name = button_names_[index];
    auto * button_property = new rviz_common::properties::Property(
      QString::fromStdString(button_name), QVariant(), "Button mapping", property_buttons_);
    rviz_common::properties::EditableEnumProperty * topic_property = nullptr;
    for (const auto & field : fields) {
      const std::string parameter_name = button_name + "." + field;
      const auto value = button_parameters_.find(parameter_name);
      const bool enum_field = field == "topic" || field == "joy_host" || field == "joy_index" ||
        field == "joy_name" || field == "joy_serial" || field == "joy_guid" || field == "joy_label";
      auto * field_property = enum_field ?
        static_cast<rviz_common::properties::StringProperty *>(new rviz_common::properties::EditableEnumProperty(
          QString::fromStdString(field),
          value == button_parameters_.end() ? QString() : QString::fromStdString(value->second),
          "Button parameter", button_property)) :
        new rviz_common::properties::StringProperty(
          QString::fromStdString(field),
          value == button_parameters_.end() ? QString() : QString::fromStdString(value->second),
          "Button parameter", button_property);
      if (field == "topic") {
        topic_property = static_cast<rviz_common::properties::EditableEnumProperty *>(field_property);
        topic_property->clearOptions();
        for (const auto & topic : topics_subscribed_) {
          topic_property->addOptionStd(topic);
        }
        topic_property->sortOptions();
        QObject::connect(
          topic_property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
          [this, topic_property]() {
            std::lock_guard<std::mutex> lock(mutex_);
            topic_property->clearOptions();
            for (const auto & topic : topics_subscribed_) {
              topic_property->addOptionStd(topic);
            }
            topic_property->sortOptions();
          });
      } else if (enum_field) {
        auto * enum_property = static_cast<rviz_common::properties::EditableEnumProperty *>(field_property);
        QObject::connect(
          enum_property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
          [this, enum_property, topic_property, field, button_name]() {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto parameter_value = [this, &button_name](const std::string & name) {
              const auto value = button_parameters_.find(button_name + "." + name);
              return value == button_parameters_.end() ? std::string("any") : value->second;
            };
            const auto values = parseJoystickValues(
              joysticks_available_message_, field, topic_property->getStdString(),
              field == "joy_label" ? "buttons" : "axes",
              field == "joy_label" ? parameter_value("joy_host") : "",
              field == "joy_label" ? parameter_value("joy_index") : "",
              field == "joy_label" ? parameter_value("joy_name") : "",
              field == "joy_label" ? parameter_value("joy_serial") : "",
              field == "joy_label" ? parameter_value("joy_guid") : "");
            enum_property->clearOptions();
            if (field != "joy_label") {
              enum_property->addOptionStd("any");
            }
            for (const auto & option : values) {
              enum_property->addOptionStd(option);
            }
            const std::string current_value = enum_property->getStdString();
            if (!current_value.empty() && current_value != "any" &&
            std::find(values.begin(), values.end(), current_value) == values.end()) {
              enum_property->addOptionStd(current_value);
            }
            enum_property->sortOptions();
          });
      }
      button_property_names_[field_property] = parameter_name;
      QObject::connect(
        field_property, &rviz_common::properties::StringProperty::changed, this,
        [this, field_property]() {
          const auto parameter = button_property_names_.find(field_property);
          if (parameter != button_property_names_.end()) {
            parameterChanged(field_property, parameter->second.c_str());
          }
        });
    }
    auto * remove_property = new rviz_common::properties::BoolProperty(
      "Remove", false, "Remove this button mapping", button_property);
    QObject::connect(
      remove_property, &rviz_common::properties::BoolProperty::changed, this,
      [this, index]() {
        QMetaObject::invokeMethod(this, [this, index]() {removeButton(index);}, Qt::QueuedConnection);
      });
  }
}

void JoystickAggregatorPanel::topicsAvailableCallback(std_msgs::msg::String::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    topics_available_ = parseTopicList(message->data);
  }
}

void JoystickAggregatorPanel::topicsSubscribedCallback(std_msgs::msg::String::ConstSharedPtr message)
{
  const auto topics = parseTopicList(message->data);
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = topics_subscribed_ != topics;
    if (changed) {
      topics_subscribed_ = topics;
      topics_selected_ = topics;
      pending_topics_ = topics;
    }
  }
  if (changed) {
    QMetaObject::invokeMethod(this, [this]() {
        rebuildTopicListProperties();
        refreshTopicOptions();
        rebuildAxisProperties();
        rebuildButtonProperties();
    }, Qt::QueuedConnection);
  }
}

void JoystickAggregatorPanel::joysticksAvailableCallback(
  std_msgs::msg::String::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joysticks_available_message_ = message->data;
  }
}

void JoystickAggregatorPanel::updateNode()
{
  parameters_client_.reset();
  sub_topic_available_.reset();
  sub_topic_subscribed_.reset();
  sub_joysticks_available_.reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_topics_.reset();
    axis_names_.clear();
    axis_parameters_.clear();
    button_names_.clear();
    button_parameters_.clear();
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
    {"topics", "param_path", "param_file", "axes", "buttons"},
    std::bind(&JoystickAggregatorPanel::parametersCallback, this, std::placeholders::_1));

  const std::string available_topic = namespace_value + "/topics_available";
  const std::string subscribed_topic = namespace_value + "/topics_subscribed";
  const std::string joysticks_available_topic = namespace_value + "/joysticks_available";

  sub_topic_available_ = node->create_subscription<std_msgs::msg::String>(
    available_topic, rclcpp::SensorDataQoS(),
    std::bind(&JoystickAggregatorPanel::topicsAvailableCallback, this, std::placeholders::_1));
  sub_topic_subscribed_ = node->create_subscription<std_msgs::msg::String>(
    subscribed_topic, rclcpp::SensorDataQoS(),
    std::bind(&JoystickAggregatorPanel::topicsSubscribedCallback, this, std::placeholders::_1));
  sub_joysticks_available_ = node->create_subscription<std_msgs::msg::String>(
    joysticks_available_topic, rclcpp::SensorDataQoS(),
    std::bind(&JoystickAggregatorPanel::joysticksAvailableCallback, this, std::placeholders::_1));
}

void JoystickAggregatorPanel::rebuildTopicListProperties()
{
  std::lock_guard<std::mutex> lock(mutex_);
  property_topics_->removeChildren(1);
  property_subscribed_topics_.clear();
  property_remove_topics_.clear();

  for (size_t index = 0; index < topics_selected_.size(); ++index) {
    const auto & topic = topics_selected_[index];
    auto * property = new rviz_common::properties::EditableEnumProperty(
      QString::fromStdString("Topic " + std::to_string(index)),
      QString::fromStdString(topic),
      "Currently subscribed topic",
      property_topics_);

    for (const auto & available_topic : topics_available_) {
      property->addOptionStd(available_topic);
    }
    property->addOptionStd("any");
    if (std::find(topics_available_.begin(), topics_available_.end(), topic) == topics_available_.end()) {
      property->addOptionStd(topic);
    }
    property->sortOptions();
    property_subscribed_topics_.push_back(property);
    auto * remove_property = new rviz_common::properties::BoolProperty(
      "Remove", false, "Remove this subscribed topic", property);
    property_remove_topics_.push_back(remove_property);
    QObject::connect(
      remove_property, &rviz_common::properties::BoolProperty::changed, this,
      [this, index]() {removeTopic(index);});
    QObject::connect(
      property, &rviz_common::properties::EditableEnumProperty::requestOptions, this,
      [this, property]() {
        property->clearOptions();
        for (const auto & available_topic : topics_available_) {
          property->addOptionStd(available_topic);
        }
        property->addOptionStd("any");
        for (const auto & topic : topics_selected_) {
          if (std::find(topics_available_.begin(), topics_available_.end(), topic) == topics_available_.end()) {
            property->addOptionStd(topic);
          }
        }
        property->sortOptions();
      });
    QObject::connect(
      property, &rviz_common::properties::StringProperty::changed, this,
      [this, property]() {topicSelectionChanged(property);});
  }
}

void JoystickAggregatorPanel::refreshTopicOptions()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!property_add_topic_) {
    return;
  }

  property_add_topic_->clearOptions();
  property_add_topic_->addOptionStd("any");
  for (const auto & topic : topics_available_) {
    if (std::find(topics_selected_.begin(), topics_selected_.end(), topic) == topics_selected_.end()) {
      property_add_topic_->addOptionStd(topic);
    }
  }
  property_add_topic_->sortOptions();
}

void JoystickAggregatorPanel::addAxis(const std::string & axis)
{
  std::vector<std::string> axes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (axis.empty() || std::find(axis_names_.begin(), axis_names_.end(), axis) != axis_names_.end()) {
      return;
    }
    axis_names_.push_back(axis);
    axes = axis_names_;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters(
      {rclcpp::Parameter("axes", axes)},
      [this, axes](const auto & result) {
        const auto results = result.get();
        const bool successful = std::all_of(
          results.begin(), results.end(), [](const auto & item) {return item.successful;});
        if (successful) {
          QTimer::singleShot(100, this, [this, axes]() {requestAxisParameters(axes);});
        }
      });
  }
  rebuildAxisProperties();
}

void JoystickAggregatorPanel::removeAxis(size_t index)
{
  std::vector<std::string> axes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= axis_names_.size()) {
      return;
    }
    const std::string axis_name = axis_names_[index];
    axis_names_.erase(axis_names_.begin() + index);
    axes = axis_names_;
    for (auto it = axis_parameters_.begin(); it != axis_parameters_.end(); ) {
      if (it->first.rfind(axis_name + ".", 0) == 0) {
        it = axis_parameters_.erase(it);
      } else {
        ++it;
      }
    }
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("axes", axes)});
  }
  rebuildAxisProperties();
}

void JoystickAggregatorPanel::addButton(const std::string & button)
{
  std::vector<std::string> buttons;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (button.empty() || std::find(button_names_.begin(), button_names_.end(), button) != button_names_.end()) {
      return;
    }
    button_names_.push_back(button);
    buttons = button_names_;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters(
      {rclcpp::Parameter("buttons", buttons)},
      [this, buttons](const auto & result) {
        const auto results = result.get();
        const bool successful = std::all_of(
          results.begin(), results.end(), [](const auto & item) {return item.successful;});
        if (successful) {
          QTimer::singleShot(100, this, [this, buttons]() {requestButtonParameters(buttons);});
        }
      });
  }
  rebuildButtonProperties();
}

void JoystickAggregatorPanel::removeButton(size_t index)
{
  std::vector<std::string> buttons;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= button_names_.size()) {
      return;
    }
    const std::string button_name = button_names_[index];
    button_names_.erase(button_names_.begin() + index);
    buttons = button_names_;
    for (auto it = button_parameters_.begin(); it != button_parameters_.end(); ) {
      if (it->first.rfind(button_name + ".", 0) == 0) {
        it = button_parameters_.erase(it);
      } else {
        ++it;
      }
    }
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("buttons", buttons)});
  }
  rebuildButtonProperties();
}

void JoystickAggregatorPanel::addTopic(const std::string & topic)
{
  std::vector<std::string> topics;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::find(topics_selected_.begin(), topics_selected_.end(), topic) != topics_selected_.end()) {
      return;
    }
    topics_selected_.push_back(topic);
    topics = topics_selected_;
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
    if (index >= topics_selected_.size()) {
      return;
    }
    topics_selected_.erase(topics_selected_.begin() + index);
    topics = topics_selected_;
    pending_topics_ = topics;
  }
  if (parameters_client_) {
    parameters_client_->set_parameters({rclcpp::Parameter("topics", topics)});
  }
  QMetaObject::invokeMethod(this, [this]() {
      rebuildTopicListProperties();
      refreshTopicOptions();
  }, Qt::QueuedConnection);
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
        if (index < topics_selected_.size()) {
          topics_selected_[index] = value;
        }
        break;
      }
    }
    topics = topics_selected_;
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
