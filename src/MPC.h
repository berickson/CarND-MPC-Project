#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const unsigned int n_states = 4;
const unsigned int state_x = 0;
const unsigned int state_y = 1;
const unsigned int state_psi = 2;
const unsigned int state_v = 3;

const unsigned int n_actuators = 2;
const unsigned int actuators_delta = 0;
const unsigned int actuators_a = 1;


Eigen::VectorXd get_next_state(Eigen::VectorXd state,
                               Eigen::VectorXd actuators, double dt);
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

class MPC {
public:
    int n_time_steps;  // number of time steps total
    double dt;         // time step granularity in seconds
    double v_set;      // desired velocity
    MPC(int n_time_steps, double dt, double v_set){
        this->n_time_steps = n_time_steps;
        this->dt = dt;
        this->v_set = v_set;
    }

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return all actuations
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd route_coeffs);
};

#endif /* MPC_H */
