#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/utility/poly.hpp>

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// returns the next state using the global kinematic equations
// from quiz at
// https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/eaad060f-cd6d-4e60-9386-995f586126be
Eigen::VectorXd get_next_state(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());
  double x = state(state_x);
  double y = state(state_y);
  double psi = state(state_psi);
  double v = state(state_v);

  double delta = actuators(actuators_delta);
  double a = actuators(actuators_a);
  x += v*cos(psi) * dt;
  y += v*sin(psi) * dt;
  psi += v/Lf * delta * dt;
  v += a * dt;

  next_state << x,y,psi,v;

  return next_state;
}



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd route_coefs;
  double v0;
  double n_time_steps;
  double dt;
  double v_set;

  FG_eval(double v0, double dt, double n_time_steps, Eigen::VectorXd route_coefs, double v_set) {
      this->route_coefs = route_coefs;
      this->v0 = v0;
      this->dt = dt;
      this->n_time_steps = n_time_steps;
      this->v_set = v_set;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // returns a cost in fg[0] given based on actuations
  void operator()(ADvector& fg, const ADvector& actuations) {
      auto & cost = fg[0];

      AD<double> x = 0;
      AD<double> y = 0;
      AD<double> psi = 0;
      AD<double> v = v0;

      // play through the route and update cost at each point
      for (int i=0; i<n_time_steps; ++i) {
          // get actuations (unflatten)
          auto & a = actuations[actuators_a + i * n_actuators];     // acceleration
          auto & delta = actuations[actuators_delta + i * n_actuators]; // wheel angle

          // update state at time step
          x += v * CppAD::cos(psi) * dt;
          y += v * CppAD::sin(psi) * dt;
          psi += v/Lf * delta * dt;
          v += a * dt;

          // calculate desired position
          AD<double> y_desired = 0;
          for (int i = 0; i < route_coefs.size(); i++) {
              y_desired += route_coefs[i] * CppAD::pow(x, i);
          }

          // calculate desired yaw (using derivative)
          AD<double> dy_desired = 0;
          for (int i = 1; i < route_coefs.size(); i++) {
              dy_desired += i * route_coefs[i] * CppAD::pow(x, i-1);
          }
          AD<double> psi_desired = CppAD::atan(dy_desired);

          // add cost for heading error
          cost += CppAD::pow(psi_desired-psi,2);

          // follow speed limit
          cost += CppAD::pow(v-v_set,2);


          // add cost for cross-track error
          cost += CppAD::pow(y-y_desired,2);

          // punish choppy actuations
          for (int i = 1; i < n_time_steps; ++i) {
            auto &a1 = actuations[actuators_a + n_actuators * (i - 1)];
            auto &delta1 = actuations[actuators_delta + n_actuators * (i - 1)];

            auto &a2 = actuations[actuators_a + n_actuators * i];
            auto &delta2 = actuations[actuators_delta + n_actuators * i];

            cost += 10 * CppAD::pow(a2 - a1, 2);
            cost += 30 * CppAD::pow(delta2 - delta1, 2);
          }
      }
  }
};

//
// MPC class definition implementation.
//
MPC::~MPC()
{

}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd route_coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = n_actuators * n_time_steps;

  size_t n_constraints = 0;

  Dvector vars(n_vars);
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(0);
  Dvector constraints_upperbound(0);
  for (int i = 0; i < n_time_steps; i++) {
      int offset = 2 * i;
      int index_a = offset + actuators_a;
      int index_delta = offset + actuators_delta;
      vars[index_a] = 0;
      vars[index_delta] = 0;
      vars_lowerbound[index_a] = -1;
      vars_upperbound[index_a] = 1;
      vars_lowerbound[index_delta] = -0.45;
      vars_upperbound[index_delta] = +0.45;
  }

  // object that computes objective and constraints

  FG_eval fg_eval(state[state_v], dt, n_time_steps, route_coeffs, v_set);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
              options,
              vars,
              vars_lowerbound,
              vars_upperbound,
              constraints_lowerbound,
              constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // auto cost = solution.obj_value;

  vector<double> rv(n_vars);
  for(int i=0; i<n_vars; i++) {
      rv[i]=solution.x[i];
  }
  return rv;
}

double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}
