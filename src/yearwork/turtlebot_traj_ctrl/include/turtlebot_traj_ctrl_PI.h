#ifndef PID4
#define PID4


class PID {
public:
    PID(void); //double Kc, double Ti, double Ts

    void initialize(double Kc, double Ti, double Ts);
    void setMeasurement(double y);
    void setReference(double ysp);
    double getControl();
    void execute();

private:
    double u_act, uI_prev, y_act, ysp_act,e_prev;
    double a, b;
};


#endif //PID4
