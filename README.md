# Functionality
The configuration manager of the bot that receives the power consumption of a list of robot configurations and maintain the current configuration of the robot. When the configuration of the robot will change, it will reflect the change to power consumption by changing the power load of the [robot battery plugin](https://github.com/cmu-mars/brass_gazebo_battery).

# Support
This plugin is tested for ROS kinetic and Gazebo 7.8.1.

# Build
Create the build directory:
```bash
mkdir ~/catkin_ws/src/brass_gazebo_config_manager/build
cd ~/catkin_ws/src/brass_gazebo_config_manager/build
```

Make sure you have sourced ROS before compiling the code:
```bash
source /opt/ros/<DISTRO>/setup.bash
```

Compile the code:
```bash
cmake ../
make    
```

Compiling will result in a shared library, `~/catkin_ws/src/brass_gazebo_config_manager/build/devel/lib/libconfig_manager.so`, that can be inserted in a Gazebo simulation.

Lastly, add your library path to the `GAZEBO_PLUGIN_PATH`:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/brass_gazebo_config_manager/build/devel/lib
```

# Build by catkin
Build the plugin by going to the base of your work space and running catkin:
```bash
cd ~/catkin_ws
catkin_make
```

# Installing the Plugin
```bash
cd ~/catkin_ws/src/brass_gazebo_config_manager/build
cmake ../
make
sudo make install
```

# Usage

In the brass.world file, `libconfig_manager.so` is mentioned as a plugin. 
This implied that plugin is initialized and loaded when `p2-cp1.world` is opened in Gazebo. 
The xml code could be linked to any model in a new `.world` file.
```xml
<plugin filename="libconfig_manager.so" name="configuration_manager">
  <config_list_path>/cp1/config_list.json</config_list_path>
  <default_config>0</default_config>
  <ros_node>cp1_node</ros_node>
</plugin>
```

# Run the Plugin
```bash
cd ~/catkin_ws/src/brass_gazebo_config_manager
gazebo test/worlds/p2-cp1.world --verbose
```


# Exposed ROS services and topics

This Gazebo plugin expose several services that can be accessed via ROS:

```
/battery_monitor_client/battery_demo_model/get_config
/battery_monitor_client/battery_demo_model/set_config
```

# Extending ROS Services

First create the service description file `.srv` and put it in the `srv` folder. Then declare it in the `CMakeList.txt` in the
`add_service_files()` section. Also, add the following to the `CMakeList.txt`:
```cmake
generate_messages(
DEPENDENCIES
std_msgs  # Or other packages containing msgs
)
```

For setting and getting configuration of the robot we use ROS services,
so here we explain how to add new services to the code if needed:

```bash
cd ~/catkin_ws
catkin_make
```
The header files associated to the service can be found here:

```bash
cd ~/catkin_ws/devel/include/brass_gazebo_config_manager
```
The add the following header into the code that want to use the services:

```cpp
#include "brass_gazebo_config_manager/SetLoad.h"
```
And then add the following declaration:

```cpp
public: bool GetConfiguration(brass_gazebo_config_manager::GetConfig::Request &req,
                                brass_gazebo_config_manager::GetConfig::Response &res)
```
The service can then be advertised as follows:

```cpp
this->rosNode->advertiseService(this->model->GetName() + "/api", &Plugin::ServiceName, this);
```


# Maintainer

If you need a new feature to be added, please contact [Pooyan Jamshidi](https://pooyanjamshidi.github.io)