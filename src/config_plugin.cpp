#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"
#include <ros/ros.h>
#include "ROS_debugging.h"
#include "std_msgs/String.h"
#include "jsoncpp/json/json.h"
#include "brass_gazebo_battery/SetLoad.h"
#include "brass_gazebo_config_manager/SetConfig.h"

namespace gazebo{
class GAZEBO_VISIBLE ConfigurationPlugin : public ModelPlugin
{
protected: double power_load;
protected: int default_config;
protected: int current_config;
protected: physics::ModelPtr model;
protected: std::unique_ptr<ros::NodeHandle> rosNode;
protected: ros::ServiceClient power_load_client;
protected: ros::ServiceServer set_bot_configuration;
protected: boost::mutex lock;
protected: Json::Value config_list;

    public: ConfigurationPlugin()
    {
        ROS_GREEN_STREAM("Creating the configuration manager plugin");
        this->power_load = 0;
        this->default_config = 0;
        this->current_config = this->default_config;
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

        // Create ros node and publish stuff there!
        this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("ros_node")));
        if (this->rosNode->ok())
        {
            ROS_GREEN_STREAM("ROS node is up");
        }

        this->model = _model;

        this->power_load_client = this->rosNode->serviceClient<brass_gazebo_battery::SetLoad>(this->model->GetName() + "/set_power_load");
        this->set_bot_configuration = this->rosNode->advertiseService(this->model->GetName() + "/set_robot_configuration", &ConfigurationPlugin::SetConfiguration, this);

        Json::Reader json_reader;
        std::string config_path = _sdf->Get<std::string>("config_list_path");
        this->default_config = _sdf->Get<int>("default_config");
        this->current_config = this->default_config;

        std::ifstream config_file(config_path, std::ifstream::binary);
        json_reader.parse(config_file, this->config_list, false);

    }

    public: virtual void Init()
    {
        std::string load = config_list[std::to_string(this->default_config)]["power_load"].asString();
        this->power_load = std::stod(load.c_str());
        SetPowerLoad(this->power_load);
        ROS_GREEN_STREAM("Default configuration has been set");
    }

public: bool SetPowerLoad(double power_load)
    {
        brass_gazebo_battery::SetLoad srv;
        srv.request.power_load = power_load;
        bool success = power_load_client.call(srv);
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

public: bool SetConfiguration(brass_gazebo_config_manager::SetConfig::Request &req,
                                brass_gazebo_config_manager::SetConfig::Response &res)
    {
        lock.lock();
        this->current_config = req.current_config;
        ROS_GREEN_STREAM("New configuration of the robot: " << this->current_config);
        std::string load = config_list[std::to_string(this->current_config)]["power_load"].asString();
        this->power_load = std::stod(load.c_str());
        SetPowerLoad(this->power_load);
        lock.unlock();
        res.result = true;
        return true;
    }

};
    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(ConfigurationPlugin)
}