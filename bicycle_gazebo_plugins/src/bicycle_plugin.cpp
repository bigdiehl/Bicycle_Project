#include "bicycle_plugin.hh"


//Global constants
const int NUM_JOINTS = 3;
const int NUM_LINKS = 4;
const double IN2M = 0.0254;
const double DEG2RAD = 0.0174533;

/* TODO:
    - Plugin publishes info for debugging if desired (applied torques,
     joint velocities, etc). Plugin parameter sets publishing status
    - Should PID input be rad/s or m/s? (currently rad/s)
    - Finish initial conditions section
    - Add some damping to bicycle joints (in URDF)
    - Move visualization to Rviz? (figure out how to reset in Rviz)
*/


// URDF should follow these naming conventions
#define BACK_WHEEL_JOINT "back_wheel_joint"
#define STEERING_JOINT   "steering_joint"
#define GIMBAL_JOINT     "gimbal_joint"
#define FLYWHEEL_JOINT   "flywheel_joint"


//We assume model is defined with joints/links in these orders
enum {
    BACK_WHEEL,
    STEERING,
    FRONT_WHEEL
};

enum {
    BACK_WHEEL_LINK,
    FRAME_LINK,
    STEERING_LINK,
    FRONT_WHEEL_LINK
};



namespace gazebo {
    
//Bicycle() : ModelPlugin() - Do we need to call base constructor?
// Bicycle::Bicycle() : ModelPlugin() {
//     gzdbg << "In the Constructor" << std::endl;
// }

void Bicycle::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {


    model = _model;
    sdf = _sdf;

    //Create new GazeboRos object
    gazebo_ros = GazeboRosPtr ( new GazeboRos (model, sdf, "bicycle_plugin" ) );

    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros->isInitialized();

    // Create an update connection (So that OnUpdate function is called with Gazebo update)
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Bicycle::OnUpdate, this, std::placeholders::_1));

    //Get Joint/Link information from model and print summary to console
    this->get_model_info();

    // Set up our JointController for any joints we want PID control on
    this->jointController.reset(new physics::JointController(this->model));
    
    this->jointController->AddJoint(model->GetJoint(BACK_WHEEL_JOINT));
    back_wheel_joint = model->GetJoint(BACK_WHEEL_JOINT)->GetScopedName();
    this->jointController->SetVelocityPID(name, common::PID(
        _p = back_wheel_motor_params['ko'],
        _i = back_wheel_motor_params['ki'],
        _d = back_wheel_motor_params['kp'],
        _cmdMax = back_wheel_motor_params['u_max'],
        _cmdMin = back_wheel_motor_params['u_min'],));

    //Create node handle and get parameters from param server
    rosnode = new ros::NodeHandle();
    get_params();

    //Initialize time variable
    last_update_time = model->GetWorld()->GetSimTime();
    
    //Subscribe to the bicycle command topic
    rosnode->getParam("/bicycle_plugin/command_topic", command_topic);

    ROS_INFO_NAMED("bicycle_plugin", "%s: Try to subscribe to %s", gazebo_ros->info(), command_topic.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<bicycle::bicycle_cmd>(
            command_topic, 1,
            boost::bind(&Bicycle::cmdTorqueCallback, this, _1),
            ros::VoidPtr(), &queue);

    bicycle_cmd_subscriber = gazebo_ros->node()->subscribe(so);
    //bicycle_cmd_subscriber = rosnode->subscribe(so);

    ROS_INFO_NAMED("bicycle_plugin", "%s: Subscribe to %s", gazebo_ros->info(), command_topic.c_str());

    // start custom queue (WHY?)
    callback_queue_thread = boost::thread ( boost::bind ( &Bicycle::QueueThread, this ) );
}

void Bicycle::get_params() {
    //Get parameters from ROS parameter server
    rosnode->getParam("/bicycle/initial_conditions", initial_conditions);
    rosnode->getParam("/bicycle/back_wheel_vel_PID", back_wheel_motor_params);
    rosnode->getParam("/bicycle/steering_motor", steering_motor_params);
    rosnode->getParam("/bicycle/wheel_diameter", wheel_diameter);
    rosnode->getParam("/bicycle_plugin/back_wheel_vel_control", back_wheel_vel_control);
    rosnode->getParam("/bicycle_plugin/steering_vel_control", steering_vel_control);

    //Convert units to metric 
    initial_conditions["delta"] *= DEG2RAD;
    initial_conditions["tilt"] *= DEG2RAD;
    initial_conditions["heading"] *= DEG2RAD;
    wheel_diameter *= IN2M; //Since yaml file defines diameter in inches

    //Reporting
    gzdbg << "Back wheel velocity control: " << back_wheel_vel_control << std::endl;
    gzdbg << "Steering velocity control: " << steering_vel_control << std::endl;
}

void Bicycle::Reset() {
    update_num = 0;
    gzdbg << "WORLD RESET\n";
    get_params();

    back_tau     = 0;
    steering_tau = 0;
    back_vel     = 0;
    steering_vel = 0;

    gzdbg << "\n";
}


void Bicycle::cmdTorqueCallback(const bicycle::bicycle_cmd::ConstPtr& msg) {
    boost::mutex::scoped_lock scoped_lock ( lock );
    back_tau     = msg->tau_back;
    steering_tau = msg->tau_steering;
    back_vel     = msg->vel_back;
    steering_vel = msg->vel_steering;
}


void Bicycle::QueueThread() {
    static const double timeout = 0.01;
    while (alive && rosnode->ok() ) {
        queue.callAvailable ( ros::WallDuration ( timeout ) );
    }
}


void Bicycle::OnUpdate(const common::UpdateInfo &_info) {
    //Set initial conditions on startup and reset
    if(update_num == 0) {    
        //Set initial delta steering angle
        //model->SetJointPosition(model_joints[STEERING], initial_conditions["delta"]);

        //Set pose of main frame link in xyz rpy coordinates
        math::Pose initial_pose(
            initial_conditions["x"], initial_conditions["y"],initial_conditions["z"],
            initial_conditions["tilt"], initial_conditions["pitch"], initial_conditions["heading"]);
        model->SetLinkWorldPose(initial_pose, model_links[FRAME_LINK]);
        
        //Set individual link velocities to set entire bicycle velocity
        //The velocity must be expressed in the world frame in m/s
        //*********** TODO - Rotate velocity vector in accordance with rpy initial conditions******
        for(int i = 0; i < NUM_LINKS; ++i) 
            model->GetLink(model_links[i])->SetLinearVel({initial_conditions["vel"], 0, 0});
        
        //Set wheel joint velocities to match link velocities
        joints[BACK_WHEEL]->SetVelocity(0, initial_conditions["vel"]/(wheel_diameter/2));
        joints[FRONT_WHEEL]->SetVelocity(0, initial_conditions["vel"]/(wheel_diameter/2));
        
        update_num++;
    }
    else {
        boost::mutex::scoped_lock scoped_lock ( lock );

        //Compute elapsed time since last control
        common::Time current_time = model->GetWorld()->GetSimTime();
        double seconds_since_last_update = ( current_time - last_update_time ).Double();
        last_update_time = current_time;

        //Back wheel PID velocity controller
        if(back_wheel_vel_control) {
   
            this->jointController->SetVelocityTarget(back_wheel_joint, back_vel);
            this->jointController->Update();

        }
        
        //Front Fork PID velocity controller
        if(steering_vel_control) {
            steering_tau = steering_controller->control(joints[STEERING]->GetVelocity(0), 
                steering_vel, seconds_since_last_update);
        }

        joints[BACK_WHEEL]->SetForce(0, back_tau);
        joints[STEERING]->SetForce(0, steering_tau);


        // TODO - publish any info like applied torques
    }
}

void Bicycle::get_model_info() {
    //Save and print list of model links
    gzdbg << "Model Links: "<< std::endl;
    physics::Link_V link_list = model->GetLinks();
    for(auto link : link_list) {
        std::string link_name = link->GetName();
        gzdbg << "Link name: " << link_name << std::endl;
        model_links.push_back(link_name);
    }

    //Save and print list of model joints
    unsigned int num_joints = model->GetJointCount();
    gzdbg << "The model has " << num_joints << " joint(s)." << std::endl;
    
    joints = model->GetJoints();
    for(auto joint : joints) {
        std::string joint_name = joint->GetName();
        model_joints.push_back(joint_name);
        gzdbg << "Joint name: " << joint_name << std::endl;
    }
}

GZ_REGISTER_MODEL_PLUGIN(Bicycle)
} // namespace gazebo
