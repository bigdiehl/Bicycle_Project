#include "pid_controller.hh"
#include <cmath>
#include <ctime>


PIDController::PIDController() {
}

PIDController::PIDController(std::map<std::string, double> params) {
    set_params(params);
}

void PIDController::set_params(std::map<std::string, double> params) {
    kp = params["kp"];
    kd = params["kd"];
    ki = params["ki"];
    u_max = params["u_max"];
    u_min = params["u_min"];
    sigma = params["sigma"];
}

double PIDController::control(double y, double y_des, double Ts) { 
    //Compute error   
    double error = y_des - y;

    //Update integrator and differentiator
    integrate(error, Ts);
    dirty_derivative(error, Ts);

    //Compute PID control, and add equilibrium offset
    double u_unsat = kp*error + kd*differentiator + ki*integrator + u_eq;

    //Saturate control and perform anti-windup
    double u = saturate(u_unsat);
    if(ki != 0) anti_windup(u, u_unsat);

    //Update variables and return
    error_d1 = error;
    y_d1 = y;
    return u;
}

void PIDController::reset() {
    integrator = 0;
    differentiator = 0;
    error_d1 = 0;
    y_d1 = 0;
}

void PIDController::integrate(double error, double Ts) {
    integrator += (Ts/2)*(error+error_d1);
}

void PIDController::dirty_derivative(double error, double Ts) {
    differentiator = (2.0*sigma-Ts)/(2.0*sigma+Ts)*differentiator +
        (2.0/(2.0*sigma+Ts))*(error - error_d1);
}

void PIDController::anti_windup(double u, double u_unsat) {
    integrator += 1.0/ki*(u-u_unsat);
}

double PIDController::saturate(double u) {
    if(u > u_max) 
        u = u_max;
    else if(u < u_min)
        u = u_min;
    return u;
}
