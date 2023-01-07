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
#include <gazebo/physics/JointWrench.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros/PhysicsConfig.h>

#include <sdf/sdf.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
//#include <bicycle_msgs/joint_torque.h>
//#include <bicycle_msgs/joint_vel.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <vector>
#include <string>
#include <map>


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
            void GetModelInfo();
            void GetParams();
            void SetInitialStates();

            void CmdTorqueCallback(const sensor_msgs::JointState::ConstPtr& state_cmd);
            void CmdVelCallback(const sensor_msgs::JointState::ConstPtr& state_cmd);
            
            void PublishBaseLinkTF();
            void PublishJointStates();

            void PrintJointStateMsg(const sensor_msgs::JointState::ConstPtr& cmd, std::string msg);

            // Custom Callback Queue
            void QueueThread();

            ros::CallbackQueue queue;
            boost::thread callback_queue_thread;

            // a pointer to the model and sdf this plugin was loaded with
            physics::ModelPtr model;
            sdf::ElementPtr sdf;
            
            // object for callback connection
            event::ConnectionPtr updateConnection;

            int update_num = 0;
            bool init = true;;
            
            //boost::shared_ptr<GazeboRos> gazebo_ros_;
            GazeboRosPtr gazebo_ros;
            ros::NodeHandle* rosnode;
            
            common::Time last_update_time;
            boost::mutex lock;

            bool alive = true;

            std::map<std::string, physics::JointPtr> joints;
            std::map<std::string, physics::LinkPtr> links;

            physics::JointControllerPtr jointController;

            // ROS STUFF
            ros::Subscriber cmd_torque_subscriber;
            ros::Subscriber cmd_vel_subscriber;
            ros::Publisher joint_state_publisher;
            sensor_msgs::JointState joint_state;
            boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster;
            std::string tf_prefix;

            sensor_msgs::JointState::ConstPtr vel_cmds;
            sensor_msgs::JointState::ConstPtr torque_cmds;

            std::string vel_command_topic;
            std::string torque_command_topic;

            
            
            std::string back_wheel_joint;
            std::string steering_joint;

            //Parameters
            std::map<std::string, double> initial_conditions;
            std::map<std::string, double> back_wheel_motor_params;
            std::map<std::string, double> steering_motor_params;

            double publish_update_period;            
            double wheel_radius;
            bool publishJointStates = true;
            
    };
        
} // namespace gazebo

#endif //TORQUE_CONTROLLER_H