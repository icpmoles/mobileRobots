#include "turtlebot_traj_ctrl_PI.h"

PID::PID() {
    u_act   = 0.0;
    uI_prev = 0.0;
    y_act   = 0.0;
    ysp_act = 0.0;
}

void PID::initialize(double Kc, double Ti, double Ts) {
    a = Kc*Ts/Ti;
    b = Kc;
}

void PID::setReference(double ysp) {
    ysp_act = ysp;
}

void PID::setMeasurement(double y) {
    y_act = y;
}

double PID::getControl() {
    return u_act;
}

void PID::execute() {
    double uI_act = uI_prev+a*(ysp_act-y_act);
    double uP_act = b*(ysp_act-y_act);
    
    u_act = uP_act+uI_act;

    // Update the state
    uI_prev = uI_act;
}
