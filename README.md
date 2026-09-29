# fieldfriend_driver

ROS 2 hardware driver for FieldFriend robots. The `fieldfriend_driver_node` communicates with the robot's ESP over a serial connection, loads a Lizard startup script, and exposes the base, battery, emergency-stop, and end-effector axis interfaces as ROS topics.

## Requirements

- ROS 2 and `colcon` (the project container currently uses ROS 2 Jazzy)
- A FieldFriend controller with its ESP connected at `/dev/esp`
- The Lizard tools installed at `/root/.lizard/espresso.py`; the driver calls this tool at startup to enable the ESP

When running outside the project container, make sure the connected serial device is available as `/dev/esp` and the process has permission to access it. The ESP connection uses 115200 baud.

## Build

From the root of the `app_fieldfriend` workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fieldfriend_driver
source install/setup.bash
```

Use the setup file for the ROS 2 distribution installed on your system if it is not Jazzy.

## Run

Start the node with the package's default ROS parameters and Lizard script:

```bash
ros2 run fieldfriend_driver fieldfriend_driver_node --ros-args \
	--params-file "$(ros2 pkg prefix --share fieldfriend_driver)/config/default.yaml" \
	-p "lizard_file:=$(ros2 pkg prefix --share fieldfriend_driver)/config/startup.liz"
```

The node expects the hardware connection and Lizard tools to be available at startup. `flashing_arguments` is passed to `espresso.py enable`; `sleep_after_flash` adds a delay after that command. Configure these in the ROS parameter file for the target vehicle.

## ROS interfaces

The default configuration loads the modules listed in `modules.module_list`. Their principal interfaces are:

| Interface | Type | Direction | Description |
| --- | --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | Base velocity command. The driver uses `linear.x` and `angular.z`; if commands stop arriving for `twist_timeout`, it sends a zero velocity. |
| `odom` | `nav_msgs/msg/Odometry` | Publish | Integrated planar odometry with `odom` and `base_link` frames. The default publish rate is 20 Hz. |
| `battery_state` | `sensor_msgs/msg/BatteryState` | Publish | Battery data received through the controller's BMS connection. |
| `yaxis/target_speed` | `std_msgs/msg/Float32` | Subscribe | Y-axis target speed in m/s, clamped to +/- 0.02 m/s. |
| `zaxis/target_speed` | `std_msgs/msg/Float32` | Subscribe | Z-axis target speed in m/s, clamped to +/- 0.02 m/s. |
| `configure` | `std_msgs/msg/String` | Subscribe | Reloads and sends the configured Lizard script to the ESP, then restarts the controller. The message contents are ignored. |
| `/emergency_stop/button_front` | `std_msgs/msg/Bool` | Publish and subscribe | Default front hardware-stop state. The button module publishes the active state; the estop module consumes it. |
| `/emergency_stop/global` | `std_msgs/msg/Bool` | Publish | Aggregate emergency-stop state. |
| `/emergency_stop/message` | `std_msgs/msg/String` | Publish | Text describing the active emergency-stop state. |

Emergency-stop button input levels are inverted by the button handler: a low hardware input is published as an active stop (`true`). The default configuration lists the front button as an emergency stop; adjust `modules.estop_handler.estop_list` and its per-stop settings to match the robot wiring.

## Configuration

The installed files are in the package's `config` directory:

- `default.yaml` sets ROS parameters, enabled modules, and the fields expected in ESP core-data messages.
- `startup.liz` configures the ESP hardware and declares the core-data fields sent back to the driver.

The ROS `read_data.list` and the fields emitted by the Lizard script must agree. Module handlers look up values by the names in this list, so changing one side without the other can prevent the corresponding data from working. Modules can be enabled or disabled through `modules.module_list`; each entry must have a matching module parameter block and supported `type`.

Odometry TF publication is disabled by default. See `default.yaml` for the configured module-specific settings, including velocity timeout and command send frequency.