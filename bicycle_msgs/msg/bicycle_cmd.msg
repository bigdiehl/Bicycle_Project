#Custom message for bicycle control. Send torque or velocity commands
#for the back wheel and steering joints. Subscriber has the option to 
#listen to either command (tau/vel) for each joint.
time stamp
float64 tau_back
float64 vel_back
float64 tau_steering
float64 vel_steering