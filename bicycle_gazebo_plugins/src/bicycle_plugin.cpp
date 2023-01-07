#include "bicycle_plugin.hh"

/* DESCRIPTION: Plugin to support interfacing Gazebo bicycle simulation with ROS. 
This plugin will: 
    1. Publish the bicycle joint states, and the tf2 transform from the world frame 
    to the base line (the back_frame with origin at rear wheel contact point) 

    2. Listen for torque and velocity commands, and apply these to the appropriate 
    joint. For velocity commands, Gazebo PID controllers are used. 

    3. Set the initial conditions. Initial conditions are read from the ROS parameter 
    server. If initial conditions cannot be found, default values will be used. 

    4. TODO - Publishes info for debugging? (applied torque?)
*/


// URDF should follow these naming conventions
// #define BACK_WHEEL_JOINT  "back_wheel_joint"
// #define FRONT_WHEEL_JOINT "front_wheel_joint"
// #define STEERING_JOINT    "steering_joint"
// #define GIMBAL_JOINT      "gimbal_joint"
// #define FLYWHEEL_JOINT    "flywheel_joint"
// #define BASE_LINK         "back_frame"

const double DEG2RAD = 0.0174533;

const std::string BACK_WHEEL_JOINT("back_wheel_joint");
const std::string FRONT_WHEEL_JOINT("front_wheel_joint");
const std::string STEERING_JOINT("steering_joint");
const std::string GIMBAL_JOINT("gimbal_joint");
const std::string FLYWHEEL_JOINT("flywheel_joint");
const std::string BASE_LINK("back_frame");


namespace gazebo {
    

void Bicycle::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

    model = _model;
    sdf = _sdf;

    gazebo_ros = GazeboRosPtr ( new GazeboRos (model, sdf, "bicycle_plugin" ) );

    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros->isInitialized();

    // Create node handle and get parameters from param server
    // TODO - alternatively, should we get parameters from gazebo_ros-getParameter<>? Allows 
    // us to specify a default value. 
    rosnode = new ros::NodeHandle();
    GetParams();

    //Get Joint/Link information from model and print summary to console
    this->GetModelInfo();

    // Set up our JointController for any joints we want PID control on
    //this->jointController.reset(new physics::JointController(this->model));
    jointController = model->GetJointController();
    jointController->Reset();    
    jointController->AddJoint(model->GetJoint(BACK_WHEEL_JOINT));

    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time = model->GetWorld()->SimTime();
    #else
        last_update_time = model->GetWorld()->GetSimTime();
    #endif


    if(publishJointStates)
    {
        joint_state_publisher = rosnode->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("bicycle_plugin", "%s: Advertising to joint_states", gazebo_ros->info());
    }

    transform_broadcaster = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    //Subscribe to the joint velocity command topic
    ROS_INFO_NAMED("bicycle_plugin", "%s: Trying to subscribe to %s", gazebo_ros->info(), vel_command_topic.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<sensor_msgs::JointState>(
            vel_command_topic, 1,
            boost::bind(&Bicycle::CmdVelCallback, this, _1),
            ros::VoidPtr(), &queue);

    cmd_vel_subscriber = rosnode->subscribe(so);
    ROS_INFO_NAMED("bicycle_plugin", "%s: Subscribed to %s", gazebo_ros->info(), vel_command_topic.c_str());

    //Subscribe to the joint torque command topic
    ROS_INFO_NAMED("bicycle_plugin", "%s: Trying to subscribe to %s", gazebo_ros->info(), torque_command_topic.c_str());

    so = ros::SubscribeOptions::create<sensor_msgs::JointState>(
            torque_command_topic, 1,
            boost::bind(&Bicycle::CmdTorqueCallback, this, _1),
            ros::VoidPtr(), &queue);

    cmd_torque_subscriber = rosnode->subscribe(so);
    ROS_INFO_NAMED("bicycle_plugin", "%s: Subscribed to %s", gazebo_ros->info(), torque_command_topic.c_str());

    // TODO - publish roll, heading, etc? In addition to tf transformation for base link?

    // start custom queue 
    callback_queue_thread = boost::thread ( boost::bind ( &Bicycle::QueueThread, this ) );

    // listen to the update event (broadcast every Gazebo simulation iteration)
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Bicycle::OnUpdate, this, std::placeholders::_1));
}

void Bicycle::SetInitialStates() {

    back_wheel_joint = model->GetJoint(BACK_WHEEL_JOINT)->GetScopedName();
    jointController->SetVelocityPID(back_wheel_joint, common::PID(
        back_wheel_motor_params["kp"],
        back_wheel_motor_params["ki"],
        back_wheel_motor_params["kd"]));
        // back_wheel_motor_params["i_max"],
        // back_wheel_motor_params["i_min"],
        // back_wheel_motor_params["u_max"],
        // back_wheel_motor_params["u_min"]));

    // TODO - calculate pitch angle from initial roll and steering angle


    //Set initial conditions on startup and reset
    //Set initial delta steering angle
    //model->SetJointPosition(STEERING_JOINT, initial_conditions["delta"]);
    joints[STEERING_JOINT]->SetPosition(0, initial_conditions["delta"]);

    //Set pose of main frame link in xyz rpy coordinates
    // TODO - possible issue is order of rpy. Not clear on the order. Ideally should be ypr, not rpy
    ignition::math::Pose3d initial_pose(
        initial_conditions["x"], initial_conditions["y"],initial_conditions["z"],
        initial_conditions["tilt"], initial_conditions["pitch"], initial_conditions["heading"]);
    model->SetLinkWorldPose(initial_pose, links[BASE_LINK]);
    
    //Set individual link velocities to set entire bicycle velocity
    //The velocity must be expressed in the world frame in m/s
    //*********** TODO - Rotate velocity vector in accordance with rpy initial conditions******
    // Does that mean just rotating to match desired heading? Roll and pitch don't have effect here?
    // Actually, with initial steer angle, front wheel will have different velocity vector? Not sure. 
    //ignition::math::Vector3d vel()
    for(auto i : links) 
        i.second->SetLinearVel({initial_conditions["vel"], 0, 0});
    
    //Set wheel joint velocities to match link velocities
    joints[BACK_WHEEL_JOINT]->SetVelocity(0, initial_conditions["vel"]/(wheel_radius));
    joints[FRONT_WHEEL_JOINT]->SetVelocity(0, initial_conditions["vel"]/(wheel_radius));
    
}


void Bicycle::GetParams() {
    //Get parameters from ROS parameter server
    rosnode->getParam("/bicycle_plugin/vel_command_topic", vel_command_topic);
    rosnode->getParam("/bicycle_plugin/torque_command_topic", torque_command_topic);

    double publish_update_rate;
    rosnode->getParam("/bicycle_plugin/publish_update_rate", publish_update_rate);
    publish_update_period = 1.0 / publish_update_rate;
    gzdbg << "Update period = " << publish_update_period << std::endl;

    rosnode->getParam("/bicycle/initial_conditions", initial_conditions);
    if(!rosnode->getParam("/bicycle/back_wheel_motor", back_wheel_motor_params)) {
        gzdbg << "ERROR: Could not retrieve back_wheel_motor_params\n";
    } else {
        // gzdbg << "back_wheel_motor_params: \n";
        // for(auto i : back_wheel_motor_params) {
        //     gzdbg << i.first << " = " << i.second << std::endl;
        // }
    }
    rosnode->getParam("/bicycle/steering_motor", steering_motor_params);
    rosnode->getParam("/bicycle/wheel_radius", wheel_radius);

    //Reporting
    gzdbg << "Initial roll (deg) = " << initial_conditions["roll"] << std::endl;
    gzdbg << "Initial steering angle (deg) = " << initial_conditions["delta"] << std::endl;
    gzdbg << "Initial heading (deg) = " << initial_conditions["heading"] << std::endl;

    // Convert to radians
    initial_conditions["delta"]   *= DEG2RAD;
    initial_conditions["roll"]    *= DEG2RAD;
    initial_conditions["heading"] *= DEG2RAD;
}


void Bicycle::GetModelInfo() {

    //Save and print map of model links
    physics::Link_V link_list = model->GetLinks();
    unsigned int num_links = link_list.size();
    gzdbg << "The model has " << num_links << " link(s):" << std::endl;

    for(auto link : link_list) {
        std::string link_name = link->GetName();
        links[link_name] = link;
        gzdbg << "\tLink name: " << link_name << std::endl;
        
    }

    //Save and print map of model joints
    unsigned int num_joints = model->GetJointCount();
    gzdbg << "The model has " << num_joints << " joint(s):" << std::endl;
    
    physics::Joint_V joint_list = model->GetJoints();
    for(auto joint : joint_list) {
        std::string joint_name = joint->GetName();
        joints[joint_name] = joint;
        gzdbg << "\tJoint name: " << joint_name << std::endl;
    }

    //gzdbg << BASE_LINK << " parent: " << joints[BASE_LINK]->GetParent()->GetName() << std::endl;
}

void Bicycle::Reset() {
    update_num = 0;
    init = true;
    gzdbg << "WORLD RESET\n";
    GetParams();

    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time = model->GetWorld()->SimTime();
    #else
        last_update_time = model->GetWorld()->GetSimTime();
    #endif

    jointController->Reset();

    gzdbg << "\n";
}

#include <sstream>

// For debugging
void Bicycle::PrintJointStateMsg(const sensor_msgs::JointState::ConstPtr& cmd, std::string msg) {
    std::stringstream out;
    out << msg << " - JointState msg: (";

    for(int i=0; i<cmd->name.size(); i++) {
        out << cmd->name[i] << ",";
    }
    out<< "), (";
    for(int i=0; i<cmd->velocity.size(); i++) {
        out << cmd->velocity[i] << ",";
    }
    out << "), (";
    for(int i=0; i<cmd->effort.size(); i++) {
        out << cmd->effort[i] << ",";
    }
    out << ")\n";
    gzdbg << out.str();
}

void Bicycle::CmdTorqueCallback(const sensor_msgs::JointState::ConstPtr& state_cmd) {
//void Bicycle::CmdTorqueCallback(boost::shared_ptr<sensor_msgs::JointState
    boost::mutex::scoped_lock scoped_lock ( lock );

    torque_cmds = state_cmd;
    PrintJointStateMsg(state_cmd, "CmdTorqueCallback");
}

void Bicycle::CmdVelCallback(const sensor_msgs::JointState::ConstPtr& state_cmd) {
    boost::mutex::scoped_lock scoped_lock ( lock );
    
    vel_cmds = state_cmd;
    PrintJointStateMsg(state_cmd, "CmdVelCallback");
}

void Bicycle::QueueThread() {
    static const double timeout = 0.01;
    while (alive && rosnode->ok() ) {
        queue.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void Bicycle::PublishJointStates() {
    ros::Time current_time = ros::Time::now();

    joint_state.header.stamp = current_time;

    // TODO - can probably just do this once during init. 
    joint_state.name.resize(joints.size());
    joint_state.position.resize(joints.size());
    joint_state.effort.resize(joints.size());
    joint_state.velocity.resize(joints.size());

    std::map<std::string, double> forces = jointController->GetForces();

    int idx = 0;
    for(auto i: joints) {
        //i.first = Joint name
        //i.second = Physics::JointPtr

        joint_state.name[idx] = i.first;
        #if GAZEBO_MAJOR_VERSION >= 8
            joint_state.position[idx] = i.second->Position ( 0 );
        #else
            joint_state.position[idx] = i.second->GetAngle ( 0 ).Radian();
        #endif

        // physics::JointWrench wrench = i.second->GetForceTorque(0);
        // ignition::math::Vector3d torque = wrench.body2Torque;

        joint_state.effort[idx] = forces[i.first];
        joint_state.velocity[idx] = i.second->GetVelocity(0);
                
        idx++;
    }
    joint_state_publisher.publish(joint_state);
}


void Bicycle::PublishBaseLinkTF() {
    ros::Time current_time = ros::Time::now();

    std::string world = "world";
    //std::string world_frame = gazebo_ros_->resolveTF(joints[BASE_LINK].GetParent()->GetName());
    std::string world_frame = gazebo_ros->resolveTF(world);
    std::string base_link_frame = gazebo_ros->resolveTF(BASE_LINK);

    ignition::math::Pose3d pose = model->WorldPose();
    
    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform tf_base_frame(qt, vt);

    transform_broadcaster->sendTransform (
            tf::StampedTransform (tf_base_frame, current_time, world_frame, base_link_frame) );
}



void Bicycle::OnUpdate(const common::UpdateInfo &_info) {
    boost::mutex::scoped_lock scoped_lock ( lock );
    static int twice = 0;

    if(init) {
        SetInitialStates();
        init = false;
        gzdbg << "OnUpdate - init completed\n";
    }
    
    //back_wheel_joint
    std::string joint_name;
    if(vel_cmds) {
        for(int i=0; i<vel_cmds->name.size(); i++) {
            if(vel_cmds->name.size() != vel_cmds->velocity.size()) {
                gzdbg << "Unequal number of joint names to joint velocities\n";
                continue;
            }

            if(!joints.count(vel_cmds->name[i])) {
                gzdbg << "Invalid joint: " << vel_cmds->name[i] << std::endl;
                continue;
            }
            joint_name = model->GetJoint(vel_cmds->name[i])->GetScopedName();
            gzdbg << "Here: " << joint_name << " " << vel_cmds->velocity[i] << std::endl;
            jointController->SetVelocityTarget(joint_name, vel_cmds->velocity[i]);
            //jointController->SetVelocityTarget(vel_cmds->name[i], vel_cmds->velocity[i]);

            
        }
        //vel_cmds.reset();
    }
    jointController->Update();

    std::map<std::string, double> mymap;
    std::map<std::string, common::PID> pids;
    common::Time t;

    mymap = jointController->GetForces();
    for(auto i : mymap) {
        gzdbg << "Forces: " << i.first << " " << i.second << std::endl;
    }

    mymap = jointController->GetVelocities();
    for(auto i : mymap) {
        gzdbg << "VelTarget: " << i.first << " " << i.second << std::endl;
    }

    pids = jointController->GetVelocityPIDs();
    for(auto i : pids) {
        gzdbg << "PID: " << i.first << " " << i.second.GetPGain() << std::endl;
    }

    // std::map<std::string, common::PID> GetVelocityPIDs
    // std::map<std::string, double> GetForces 	( 		) 	const
    // common::Time GetLastUpdateTime 	( 		) 	const
    // std::map<std::string, double> GetVelocities 	( 		) 	const


    if(torque_cmds) {
        for(int i=0; i<torque_cmds->name.size(); i++) {
            if(torque_cmds->name.size() != torque_cmds->effort.size()) {
                gzdbg << "Unequal number of joint names to joint efforts\n";
                continue;
            }

            if(!joints.count(torque_cmds->name[i])) {
                gzdbg << "Invalid joint: " << torque_cmds->name[i] << std::endl;
                continue;
            }

            joints[torque_cmds->name[i]]->SetForce(0, torque_cmds->effort[i]);
        }
    }
    //joints[BACK_WHEEL_JOINT]->SetForce(0, 10);


    //Compute elapsed time since last control
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif

    double seconds_since_last_update = ( current_time - last_update_time ).Double();
    
    if(seconds_since_last_update > publish_update_period) {
    
        PublishJointStates();
        PublishBaseLinkTF();
        //gzdbg << "Update num = " << update_num << std::endl;

        last_update_time += common::Time ( publish_update_period );
    }
    
    ++update_num;
}



GZ_REGISTER_MODEL_PLUGIN(Bicycle)
} // namespace gazebo
