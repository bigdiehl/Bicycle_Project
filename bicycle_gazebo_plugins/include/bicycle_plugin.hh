#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H


#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros/PhysicsConfig.h>

#include <sdf/sdf.hh>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <string>
#include <map>

#include "bicycle_msgs/bicycle_cmd.h"


namespace gazebo  
{

    class Bicycle : public ModelPlugin {
    public:
        // Constructor/Destructor
        //Bicycle();
        //~Bicycle();
        // Called once when model is loaded
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void Reset();
        
    protected:
        void OnUpdate(const common::UpdateInfo &_info);
        
    private:
        void get_model_info();
        void get_params();
        
        void cmdTorqueCallback(const bicycle::bicycle_cmd::ConstPtr& msg);
        
        // Custom Callback Queue
        void QueueThread();
        ros::CallbackQueue queue;
        boost::thread callback_queue_thread;


        // a pointer to the model and sdf this plugin was loaded with
        physics::ModelPtr model;
        sdf::ElementPtr sdf;
        
        // object for callback connection
        event::ConnectionPtr updateConnection;
        // number of updates received
        int update_num = 0;
        
        //boost::shared_ptr<GazeboRos> gazebo_ros_;
        GazeboRosPtr gazebo_ros;
        ros::NodeHandle* rosnode;
        
        ros::Subscriber bicycle_cmd_subscriber;
        common::Time last_update_time;


        boost::mutex lock;

        bool alive = true;

        std::vector<physics::JointPtr> joints;

        //Vectors to hold joint and link names
        std::vector<std::string> model_joints;
        std::vector<std::string> model_links;

        //Parameters
        std::map<std::string, double> initial_conditions;
        std::map<std::string, double> back_wheel_motor_params;
        std::map<std::string, double> steering_motor_params;
        std::string command_topic;
        double wheel_diameter;
        bool back_wheel_vel_control;
        bool steering_vel_control;

        //Bicycle commands
        double back_tau = 0;
        double steering_tau = 0;
        double back_vel = 0;
        double steering_vel = 0;

        std::string back_wheel_joint;
        std::string steering_joint;

    };
        
} // namespace gazebo

#endif //TORQUE_CONTROLLER_H