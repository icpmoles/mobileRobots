#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;
// state type definition for odeint library


class simulator_odefun
{
    public:

        simulator_odefun(double deltaT); 
        // constructor: accepts deltaT as the SAMPLING TIME

        void setInitialState(double x1, double x2);
        void setModelParams(double m, double l, double d);
        // mass, length, friction coefficent

        void integrate();
    
        void setInputValues(double u);
    
        void getState(double &x1, double &x2);
        void getTime(double &time); //get time after integration step

private:
    // Simulator and integrator variables
    double t, dt;
    double m, l, d;
    double u;

    bool modelParams_set; // false at initialization
    // true after you set the parameters.

    state_type state; // state of unspecified size.
    boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper; // integration algorithm choice

    // ODE function
    void simulator_ode(const state_type &state, state_type &dstate, double t);
};
