
//Global constants
const int NUM_JOINTS = 3;
const int NUM_LINKS = 4;
const double IN2M = 0.0254;
const double DEG2RAD = 0.0174533;

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


//Bicycle() : ModelPlugin() - Do we need to call base constructor?
// Bicycle::Bicycle() : ModelPlugin() {
//     gzdbg << "In the Constructor" << std::endl;
// }


       enum joints_enum {
            BACK_WHEEL_JOINT,
            STEERING_JOINT,
            FRONT_WHEEL_JOINT,
            GIMBAL_JOINT,
            FLYWHEEL_JOINT,
            N_MAX_JOINTS
        };


ros::SubscribeOptions so =
        ros::SubscribeOptions::create<bicycle::bicycle_cmd>(
            command_topic, 1,
            boost::bind(&Bicycle::CmdTorqueCallback, this, _1),
            ros::VoidPtr(), &queue);


void Bicycle::CmdTorqueCallback(const bicycle::bicycle_cmd::ConstPtr& msg) {
    boost::mutex::scoped_lock scoped_lock ( lock );
    back_tau     = msg->tau_back;
    steering_tau = msg->tau_steering;
    back_vel     = msg->vel_back;
    steering_vel = msg->vel_steering;
}



gzdbg << "Here OnUpdate, if(joint_torques)\n";
        if (twice < 2) {
            gzdbg << joint_torques->name.size() << std::endl;
            for ( auto i : joint_torques->name) {
                gzdbg << i << std::endl;
            }
            for ( auto i : joint_torques->effort) {
                gzdbg << i << std::endl;
            }
            ++twice;
        }