#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;


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
const double Lf = 2.67;  // in meters 


//reference value for speed
double ref_v = 60;  //miles for hour  


// the start indice for each component in the vars vector:
// vars contains all variables used by the cost function and model
//[x,y,ψ,v,cte,eψ] and [δ,a]

size_t x_start = 0;   //states x
size_t y_start = x_start + N; //states y
size_t psi_start = y_start + N;  //states orientation
size_t v_start = psi_start + N;  //states velocity
size_t cte_start = v_start + N;   // cte error 
size_t epsi_start = cte_start + N; // orientation error 
size_t delta_start = epsi_start + N; // actuators for steering
size_t a_start = delta_start + N - 1;  //actuators for acceleration


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // weights for each cost function part, adjusted manually 
    const double cte_weight = 500;  //cte error
    const double epsi_weight = 500;  // orientation error
    const double v_weight   = 1;     //velocity
    const double delta_weight = 5;   // control angle
    const double a_weight = 5;       //acceleration 
    const double delta_smooth_weight = 50000;  //steering smoothing
    const double a_smooth_weight = 5;   //acceleration smoothing 
    
	//the cost fg[0].
    fg[0] = 0 ;

    //cost on cte (cross track error), epsi (orientation), velocity (to avoid stopping) based on reference state 
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += cte_weight * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
   
    //cost for steering and acceleration magnitude to minimize the use of actuators  
    for (unsigned  int t = 0; t < N-1; t++) {
      fg[0] += delta_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_weight * CppAD::pow(vars[a_start + t], 2);
    }
   
   //cost for change rate of actuators (steering and acceleration) to make the control smoother 
    for (unsigned  int t = 0; t < N-2; t++) {
      fg[0] += delta_smooth_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += a_smooth_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
   
    // Initialization & constraints    
    // Initial constraints, t = 0
    // Add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
	//constraints based on vehicle model 
    for (unsigned int t = 1; t < N; t++) {
      
	  // State at time t + 1 
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // State at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // Only need to consider the actuation at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
	  //fitted trajectory reference line  - 3rd order polynomial
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));
	  
	 // The idea here is to constraint this value to be 0.
	 // Recall the equations for the model:
	 // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
	 // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
	 // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
	 // v_[t] = v[t-1] + a[t-1] * dt
 	 // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
	 // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
	  // two items below including the current cross track error (reference and current value) +  the change in error caused by the vehicle's movement
      fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));  //
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
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
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // the state is a 6 element vector, and the actuator is a 2 element vector
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6 ;

  // Initial value of all the independent variables.
  // SHOULD BE 0 besides initial state (at the start indices).
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  //initial state //not necessary here? 
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // to the max negative and positive values.
  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e10;
    vars_upperbound[i] = 1.0e10;
  }

  // The upper and lower limits of delta: -25 and 25 degrees (values in radians).
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -25 * M_PI / 180;
    vars_upperbound[i] = 25 * M_PI / 180;
  }

  // The upper and lower limits of Acceleration
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  //constraints for the initial state 
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

  // object that computes objective and constraints
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

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  // Return the first actuator values, along with predicted x and y values to plot in the simulator.
  vector<double> result;
  result.push_back(solution.x[delta_start]);  //steering 
  result.push_back(solution.x[a_start]);      //acceleration
  for (unsigned int i = 0; i < N; ++i) {      //predicated x and y values 
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }
  
  return result;
}



