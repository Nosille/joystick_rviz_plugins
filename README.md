# joystick_rviz_plugins

RViz plugins for visualizing joystick input.

## Build

From the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select joystick_rviz_plugins --symlink-install
source install/setup.bash
```

## Use

1. Start RViz2.
2. Add the `JoystickAggregatorPanel` panel.
3. Set `Joystick topic` to the `sensor_msgs/msg/Joy` topic published by the joystick driver.
4. Set `Aggregator node` to the fully qualified `joystick_aggregator` node name.

The display reports the number of axes and buttons in the latest joystick message. The `Axes`
and `Buttons` property groups contain each configured mapping. Expand a mapping to edit its
topic, joystick identity fields, label expression, and button minimum value where applicable.
Changes are sent directly to the node's ROS 2 parameters.
