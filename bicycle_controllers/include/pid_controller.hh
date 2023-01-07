#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <map>
#include <string>

/* DESCRIPTION: Generic SISO PID controller.
 * GENERAL SYMBOLS:
 *  - u is control output (the effort)
 *  - y is the control input (the plant/sensor output)
 *  - e is the error between y and y_des
 * PARAMETERS:
 *  - Set all parameters in a configuration file (Or give as JSON object?)
 *  - kp, kd, ki = PID gains
 * OPTIONS:
 *  - Can choose to differentiate the error or the input directly
 * TODO
 *  - Configure so that either y or error is used in differentiator
 *  - Perhaps make more efficient (smaller types, inlining, etc)
 */

//HOW TO SET UP THE LARGE NUMBER OF PARAMETERS? PREFERABLY FROM A PARAMETER/
//CONFIGURATION FILE

class PIDController{
public:
    PIDController();
    PIDController(std::map<std::string, double> params);

    void set_params(std::map<std::string, double> params);
    double control(double y, double y_des, double Ts);
    void reset();
private:
    //---Member Variables---
    double kp;
    double kd;
    double ki;

    double u_max;
    double u_min;

    double integrator = 0;
    double differentiator = 0;
    double error_d1 = 0;
    double y_d1 = 0;

    double u_eq;
    double sigma; 

    //---Member Functions---
    //
    void dirty_derivative(double error, double Ts);

    //Integrate error using trapezoidal rule
    void integrate(double error, double Ts);
    void anti_windup(double u, double u_unsat);
    double saturate(double u);
};

#endif //PID_CONTROLLER_H