#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;



// The reference velocity set
double ref_v = 80;
double steering_factor = 0.436332; // 25 deg = 0.436332 rad

// Setting up the starting points of the variables in the vars vector

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



class FG_eval
{
  public:
  
   //Fitted polynomial coefficients
   Eigen::VectorXd coeffs;
   FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.
		
		// Cost stored in fg[0]

  void operator()(ADvector& fg, const ADvector& vars)
  {
    // fg[0] contains the total cost value
    // fg[1] contains the initial states
    // fg[1+t] contains the values based on the predicted model
    const double f_e = 120; //factor multiplying heading error
    const double f_xy = 10; // factor multiplying the position error on the map
	const double f_d = 600; // factor multiplying usage of actuators ( only used for steering)
    const double f_ddiff = 50000; // factor multiplying actuation changes between consecutive steps ( only used for steering)
	
    
    // Cost based on the reference states: cte, epsi, and velocity.
    for(unsigned int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      fg[0] += f_xy * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += f_e  * CppAD::pow(vars[epsi_start + t], 2);
    }

    // Cost caused by use of actuators
    for(unsigned int t = 0; t < N - 1; t++) {
      fg[0] += f_d* CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Cost caused by actuation gradients
    for(unsigned int t = 0; t < N - 2; t++) {
      fg[0] += f_ddiff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
   // Constraints for the point t fg[1...1+n_states+n_actuators] 
   
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Constraints for the points t+1 => t+N fg[1+n_states+n_actuators+1...n_states*N+n_actuators*(N-1)]
	
	
	
    for(unsigned int t = 1; t < N; t++) {
       // The state at time t.
		  AD<double> x0 = vars[x_start + t - 1];
		  AD<double> y0 = vars[y_start + t - 1];
		  AD<double> psi0 = vars[psi_start + t - 1];
		  AD<double> v0 = vars[v_start + t - 1];
		  AD<double> cte0 = vars[cte_start + t - 1];
		  AD<double> epsi0 = vars[epsi_start + t - 1];
		  
		   // The state at time t+1 .
		  AD<double> x1 = vars[x_start + t];
		  AD<double> y1 = vars[y_start + t];
		  AD<double> psi1 = vars[psi_start + t];
		  AD<double> v1 = vars[v_start + t];
		  AD<double> cte1 = vars[cte_start + t];
		  AD<double> epsi1 = vars[epsi_start + t];

		  // Only consider the actuation at time t.
		  AD<double> delta0 = vars[delta_start + t - 1];
		  AD<double> a0 = vars[a_start + t - 1];

		  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
		  AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));

		  // Kinematic bicycle model equations:
		  
		  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		  // v_[t+1] = v[t] + a[t] * dt
		  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
		  
		  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
		  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
		  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	}		  
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(){}

MPC::~MPC(){}

 MPC::MPC_Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs){
  bool ok = true;

  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 6 * N + 2 * (N-1)
  size_t n_vars = N * 6 + (N - 1) * 2; // 6 from 4 states+2 actuators
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

 
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for(unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  
  for(unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -1 * steering_factor; 
    vars_upperbound[i] = steering_factor; 
  }

  // Acceleration/decceleration upper and lower limits.
  for(unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for(unsigned int i = 0; i < n_constraints; i++) {
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
  options += "Numeric max_cpu_time          0.05\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  // std::cout << "vars " << vars << std::endl;
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Solution: " << solution.x << std::endl;

 
  MPC::MPC_Solution solved;
  
  for(unsigned int i = 0; i < N - 1; i++) {
    solved.xpts.push_back(solution.x[x_start + i]);
    solved.ypts.push_back(solution.x[y_start + i]);
    solved.cte.push_back(solution.x[cte_start + i]);
    solved.epsi.push_back(solution.x[epsi_start + i]);
  }
  
  solved.delta = solution.x[delta_start];
  solved.a = solution.x[a_start];
  solved.cost = cost;
  return solved;
}
