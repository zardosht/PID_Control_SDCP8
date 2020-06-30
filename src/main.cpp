#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::cout;
using std::endl;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steer;
  // pid_steer.Init(0.07, 0, 0);
  // pid_steer.Init(0.1, 0, 0);
  // pid_steer.Init(0.1, 0, 3.0);
  // pid_steer.Init(0.1, 0, 2.0);
  // pid_steer.Init(0.1, 0.001, 2.0);
  pid_steer.Init(0.15, 0.001, 1.5);


  PID pid_speed;
  // pid_speed.Init(1.0, 0, 0);
  //pid_speed.Init(10.0, 0, 0);
  pid_speed.Init(5.0, 0, 3.0);


  PID pid_throttle;
  pid_throttle.Init(0.07, 0, 0);

  h.onMessage([&pid_steer, &pid_speed, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */


          cout << "\n----------------------- Input " << endl;
          cout << "cte = " << cte << endl;
          cout << "speed = " << speed << endl;
          cout << "steering_angle = " << angle << endl;


          

          // Speed depends on throttle. The Throttle PID must have speed as error. 

          // CTE depends on steering. The steering PID must have CTE as error. 

          // Apply Ziegler-Nichols method, or Stackexchange method. 

          // First try to understand the steering: 
          // First give high throttle till reach speed 20. Then reduce throttle to 0.1. And observe the steering. 

          // The target velue for speed should depend also on errror

          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          
          if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          if (steer_value < -1.0) {
            steer_value = -1.0;
          }

          double target_speed = 25;
          pid_speed.UpdateError(abs(cte));
          target_speed = target_speed + pid_speed.TotalError();
          
          double speed_error = target_speed - speed;
          pid_throttle.UpdateError(speed_error);
          double target_throttle = 0.5;
          double throttle = target_throttle;
          throttle = throttle - pid_throttle.TotalError();

          // double throttle = 0.8;
          // if (speed > 20) {
          //   throttle = 0;
          // } 

          // pid_throttle.UpdateError(cte);
          // double throttle = 0.2; 
          // if (speed > 20) {
          //   throttle += throttle * pid_throttle.TotalError();
          // }
          
          // DEBUG
          cout << "----------------------- Output" << endl;
          cout << "total_error_steer = " << pid_steer.TotalError() << endl;
          cout << "steer_value = " << steer_value << endl;
          cout << "target_speed = " << target_speed << endl;
          cout << "throttle = " << throttle << endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}