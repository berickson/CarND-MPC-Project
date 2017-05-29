#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const unsigned int state_x = 0;
const unsigned int state_y = 1;
const unsigned int state_psi = 2;
const unsigned int state_v = 3;

const unsigned int actuators_delta = 0;
const unsigned int actuators_a = 1;


Eigen::VectorXd get_next_state(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt);

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
