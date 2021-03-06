#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"

#include "json.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if(found_null != string::npos) {
    return "";
  } else if(b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;


  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if(sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if(s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if(event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double m_px = j[1]["x"];
          double m_py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double delta = j[1]["steering_angle"];
          delta *= -1; //conversion of sense (psi left positive  -> steering left negative
		  double a = j[1]["throttle"];
          

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          
          // Transformation map - > vehicle coordinates
          Eigen::VectorXd ptsx_veh(ptsx.size());
          Eigen::VectorXd ptsy_veh(ptsy.size());

          map2veh(m_px, m_py, psi, ptsx, ptsy, ptsx_veh, ptsy_veh);

          
          // Fit the above points of the waypoints as a polynomial line
          auto coeffs = polyfit(ptsx_veh, ptsy_veh, 3); // using third order to deal with the curve

          // Calculate the crosstrack error
          double cte = polyeval(coeffs, 0.);
          // Calculate the orientation error
          double epsi = -atan(coeffs[1]);
          
		  // Set the state vector + velocity + errors
          Eigen::VectorXd state(6);
          
		  state << 0, 0, 0, v, cte, epsi; // x,y,psi =0 because they are in the vehicle coordinates now

          
          // Include latency
		  unsigned int delay = 100;
          predict_include_delay(state, delta, a, delay); 
			
		  MPC::MPC_Solution solved_mpc;
         
          solved_mpc = mpc.Solve(state, coeffs);

          
		  double steer_value;
          double throttle_value;
		  double steering_factor = 0.436332; // 25 deg = 0.436332 rad
		  
          steer_value = -1* solved_mpc.delta / steering_factor;
          throttle_value = solved_mpc.a;
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = solved_mpc.xpts; // mpc_x_vals;
          msgJson["mpc_y"] = solved_mpc.ypts; // mpc_y_vals;

          // Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

		  
		   // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          
       
          for (unsigned int i = 1; i < 2*N; i++) {
            next_x_vals.push_back(dt *50* i);
            next_y_vals.push_back(polyeval(coeffs, dt*50* i));
			}
		  
		  
		  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          
          this_thread::sleep_for(chrono::milliseconds(delay));
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if(req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection(
      [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if(h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
