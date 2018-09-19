#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

using namespace std;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.

//Parameters
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;



// TODO: Set the timestep length and duration



const size_t N = 7;
const double dt = 0.1;



class MPC {
 
 public:
 
  MPC();

  ~MPC();
  
  
  struct MPC_Solution {
	vector<double> xpts;
	vector<double> ypts;
	vector<double> cte;
	vector<double> epsi;
	double delta;
	double a;
	double cost; 
  } ;
    
  MPC_Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  };

  
 
  
#endif /* MPC_H */
