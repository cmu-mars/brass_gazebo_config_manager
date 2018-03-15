#include <gazebo/common/Plugin.hh>
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "ROS_debugging.h"
#include "std_msgs/String.h"
#include "jsoncpp/json/json.h"
#include "brass_gazebo_battery/SetLoad.h"


#include <fstream>
#include <iostream>
#define CONFIGURATION_DEBUG

namespace gazebo{
class GAZEBO_VISIBLE ConfigurationPlugin : public ModelPlugin
{
protected: double power_load;
protected: int default_config;
protected: physics::ModelPtr model;
protected: std::unique_ptr<ros::NodeHandle> rosNode;
protected: ros::ServiceClient power_load_client;

    public: ConfigurationPlugin()
    {
        this->power_load = 0;
        this->default_config = 0;

    }

    public: ~ConfigurationPlugin()
    {

    }

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

        ROS_GREEN_STREAM("Loading the configuration manager plugin");

        // check if the ros is up!
        if (!ros::isInitialized()){
            ROS_RED_STREAM("Initializing ROS.");
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
        }

        this->model = _model;

        this->power_load_client = this->rosNode->serviceClient<brass_gazebo_battery::SetLoad>(this->model->GetName() + "/set_power_load");

        // Create ros node and publish stuff there!
        this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
        if (this->rosNode->ok())
        {
            ROS_GREEN_STREAM("ROS node is up");
        }

        Json::Value config_list;
        Json::Reader json_reader;

        std::string config_path = _sdf->Get<std::string>("config_list_path");
        this->default_config = _sdf->Get<int>("default_config");

        std::ifstream config_file(config_path, std::ifstream::binary);
        json_reader.parse(config_file, config_list, false);
        std::string load = config_list[std::to_string(this->default_config)]["power_load"].asString();
        this->power_load = std::stoi(load.c_str());
        SetPowerLoad(this->power_load);

    }

    public: virtual void Init()
    {

    }

public: bool SetPowerLoad(int power_load)
    {
        brass_gazebo_battery::SetLoad srv;
        srv.request.power_load = power_load;
        bool success = this->power_load_client.call(srv);
        if (success)
        {
            ROS_GREEN_STREAM("A new load has been set to the battery of the robot");
            return true;
        }
        else{
            ROS_RED_STREAM("An error happened calling power load service set");
            return false;
        }

    }


};
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ConfigurationPlugin)
}