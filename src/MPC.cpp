#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;      // suggested by walkthrough video
double dt = 0.1;    // dictated by simulator

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

// this is the desired speed
double ref_v = 100;


// Vector index offsets for all of the state variables and actuator variables.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    /********** OBJECTIVE/COST FUNCTION ************/
    // the objective/cost function will be loaded into fg[0]
    // initialize it to 0 so we can += all of the cost function terms below
    fg[0] = 0;

    // We want to minimize our car's distance from center of lane,
    // minimize car's orientation away from center line,
    // and minimize how far we deviate from target velocity.
    for (int t = 0; t < N; t++) {
      fg[0] += 3000*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // We want to minimize the use of steering wheel and accelerator.
    // That is we do not want to steer severely or accelerate/brake too hard.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
    }

    // We do not want to change our steering angle or acceleration/braking
    // too much from one moment to the next - so minimize these too.
    // NOTE:  this is particularly interesting for acceleration which is already
    // a second order change, so the "change" in acceleration is a third order change!
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /********** CONSTRAINT FUNCTIONS ************/
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.

    // First load in the initial state values as constants
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Now load in all of the constraining functions,
    // each one being dependent upon the previous state
    // which is itself dependent on its previous state, etc.
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // get y value, f0, for given x value, x0, of 3rd order polynomial we are fitting
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      // calcualte psi desired as atan of slope of polynomial at x0
      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);

      // These are the constraining functions based on the MPC model
      // Basically we want the difference between the actual value at t+1
      // and the calculated value for t+1 (based on values at t) to be
      // as close to zero as possible
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      // in simulation, delta is positive for right turn so modify equation below
      //fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of constraints for our optimization problem.
  // This is the number of points we are projecting out, N, times the
  // number of equations, 6.
  size_t n_constraints = N * 6;
  // Set the number of variables which is the number of
  // state variables, 6, over the N steps, or N * 6
  // plus the number of transitions from one state to the next, N-1,
  // times the number of actuator variables, 2.
  size_t n_vars = N * 6 + (N - 1) * 2;

  // capture values of current state
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // bounds for variables are +/- inf
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians) from simulator.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.436332*Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraint functions
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

   // result format from walkthrough video
  vector<double> result;
  // capture delta and a for first transition from current state to next state
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  // capture calculated path with x's in even indexes, y's in odd indexes
  for(int i=0;i<N-1;i++)
  {
    result.push_back(solution.x[x_start + i]);  // removed +1 from walkthrough video
    result.push_back(solution.x[y_start + i]);
  }
  return result;

}
