#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

PID Twiddle(PID pid);

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

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  double init_Kp = -0.1109;
  double init_Ki = -0.0003;
  double init_Kd = -1.54;
  pid.Init(init_Kp, init_Ki, init_Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          PID a = pid;
          PID copy_pid = PID(pid);
          std::vector<double> dp(3, 1);
          std::vector<double> p(3, 0);
          double threshold = 0.001;
          double best_error = copy_pid.TotalError();
          double err = 0;

          double sum_of_elems = 0;
          std::for_each(dp.begin(), dp.end(), [&] (double n) {sum_of_elems += n;});

          while (sum_of_elems > threshold) {
            for(int i = 0; i < dp.size(); i++ ) {
              p[i] += dp[i];
              copy_pid.ModifyK(i, p[i]);
              err = copy_pid.TotalError();

              if (err < best_error) {
                best_error = err;
                dp[i] *= 1.1;
              } else {
                p[i] -= 2*dp[i];
                copy_pid.ModifyK(i, p[i]);
                err = copy_pid.TotalError();

                if (err < best_error) {
                  best_error = err;
                  dp[i] *= 1.05;
                } else {
                  p[i] += dp[i];
                  copy_pid.ModifyK(i, p[i]);
                  dp[i] *= 0.95;
                }
              }
            }

            sum_of_elems = 0;
            std::for_each(dp.begin(), dp.end(), [&] (double n) {sum_of_elems += n;});
          }

          pid.setKd(p[0]);
          pid.setKd(p[1]);
          pid.setKd(p[2]);

          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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