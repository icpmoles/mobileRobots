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


simulator_odefun::simulator_odefun(double deltaT)
{
    // Initialize time and dt
    t = 0.0;
    dt = deltaT;

    // Initialize flags
    modelParams_set = false;

    // Initial state values
    state.resize(2); // specify size of state
    state[0] = 0.0;
    state[1] = 0.0;
    // dummy initialization
}

void simulator_odefun::setInitialState(double x1, double x2)
{
    // Initial state values
    state[0] = x1;
    state[1] = x2;
}

void simulator_odefun::setModelParams(double m, double l, double d)
{
    // Initialize model parameters
    this->m = m;
    this->l = l;
    this->d = d;

    modelParams_set = true;
}

void simulator_odefun::setInputValues(double u)
{
    this->u = u;
}

void simulator_odefun::integrate()
{
    // Check model parameters are set
    if (!modelParams_set) {
        throw std::invalid_argument( "Model parameters not set!" );
    }

    // Integrate for one step ahead using Boost library
    // stepper.do_step method from Boost library
    // std::bind() is a callback method defined by boost
    // do step with this state, at this time, and dt as the time step (in this case the freq of the controller)
    // placeholders are stuff to pass more arguments to the callback function
    stepper.do_step(std::bind(&simulator_odefun::simulator_ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), state, t, dt);

    // Update time
    t += dt;
}


// requested by the ODE lib to be like this. You can only change the name but the arguments and the types
// are forced
void simulator_odefun::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double x1 = state[0];
    const double x2 = state[1];

    // Model equations of a nonlinear pendulum with friction
    // state[0] = theta
    // state[1] = thetadot
    dstate[0] = x2;
    dstate[1] = -9.81/l*std::sin(x1)-d/(m*std::pow(l,2.0))*x2+u/(m*std::pow(l,2.0));
}

void simulator_odefun::getState(double &x1, double &x2)
{
    x1 = state[0];
    x2 = state[1];
}

void simulator_odefun::getTime(double &time)
{
    time = t;
}