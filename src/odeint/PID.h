#ifndef CONTROL_SYSTEM_PID_H
#define CONTROL_SYSTEM_PID_H


class PID {
public:
    PID(double Kc, double Ti, double Ts);

    void initialize();
    void setMeasurement(double y);
    void setReference(double ysp);
    double getControl();
    void execute();

private:
    double u_act, uI_prev, y_act, ysp_act;
    double a, b;
};


#endif //CONTROL_SYSTEM_PID_H
