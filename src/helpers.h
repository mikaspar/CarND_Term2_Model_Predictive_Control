#ifndef HELPERS_H
#define HELPERS_H


#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "MPC.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>





//constexpr double pi(){return M_PI;}
//double deg2rad(double x){return x * pi() / 180;}
//double rad2deg(double x){return x * 180 / pi();}



// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for(int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}



// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for(int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for(int j = 0; j < xvals.size(); j++) {
    for(int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}





// Create a transformation Matrix to transform given points to vehicle CS
void map2veh(const double x_m, const double y_m, const double psi,const vector<double> ptsx_m, const vector<double> ptsy_m, Eigen::VectorXd& ptsx_v, Eigen::VectorXd& ptsy_v) {
  
  
  for (unsigned int i = 0; i < ptsx_m.size(); i++) {
            double x = ptsx_m[i] - x_m;
            double y = ptsy_m[i] - y_m;
            ptsx_v[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_v[i] = x * sin(-psi) + y * cos(-psi);
	}
  
}




// Delay is considered in the model
void include_delay(Eigen::VectorXd& state, const double delta, const double acc, unsigned int delay)
{
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  state[0] = x + v * cos(psi) * delay/1000;
  state[1] = y + v * sin(psi) * delay/1000;
  state[2] = psi + v / Lf * delta * delay/1000;
  state[3] = v + acc * delay/1000;
  state[4] = cte + v * sin(epsi) * delay/1000;
  state[5] = epsi + v / Lf * delta * delay/1000;
}

#endif /* HELPERS_H_H */