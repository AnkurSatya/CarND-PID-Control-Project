#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

bool is_initialized = false;
int it = 0;//iterator for twiddle.
int factor;
int idx = 1;
int flag = 0;
int flag_twiddle = 0;
int no_of_it = 0;

double run_robot(PID pid)
{
    uWS::Hub h;

    h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode){
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
      // cout<<"Check1"<<endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          
          // * TODO: Calcuate steering value here, remember the steering value is
          // * [-1, 1].
          // * NOTE: Feel free to play around with the throttle and speed. Maybe use
          // * another PID controller to control the speed!
          
          if(!is_initialized)
          {
            pid.cte_previous = cte;
            is_initialized = true;
            // pid.param_update(idx);
          }

          pid.cte_t = cte;
          pid.UpdateError();
          steer_value = pid.TotalError();
          if(steer_value>1) steer_value = 1;
          if(steer_value<-1) steer_value = -1;
          // steer_value  = deg2rad(steer_value);
          cout<<"Iteration number "<<it<<endl;

          no_of_it = 350;

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // factor = it/50;// number represents the number iterations to let the controller stabilize.
          // if((factor%2) != 0)//Error is being evaluated for the other 50 iterations before running the twiddle.
          // {
          if(it%50)
            pid.error+= cte*cte;
          
          if(cte>2.2)
          {
            no_of_it = it - ((it/350) * 350);
            it = (it/350) * 350;//To reset the no. of iterations to the value at the beginning of the error evaluation after the parameter update. 
            cout<<"Iteration number new "<<it<<endl;
            pid.error-=cte*cte;
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            // if(flag == 0)
            // {
            // idx--;
            // idx = idx%2;
            // }
          }

          if(it == 350 || cte>2.2) flag_twiddle = 1;

          if(it%350 == 0 && (pid.dp[0] + pid.dp[1])>0.01 && flag_twiddle == 1) 
          {
            pid.error/=no_of_it;

            //Twiddle Implementation.
            if(flag == 0)
            {

              if(pid.error<=pid.best_error)
              {
                pid.best_error = pid.error;
                cout<<"Check 1"<<endl;
                // cout<<"Index "<<idx<<endl;
                pid.dp[idx]*=1.1;//If the direction seems good, then increasing this value will help to converge faster.
                // pid.params[idx]+=pid.dp[idx];//Updating the parameter for the next time when this parameter will be used.
                pid.error = 0.0;
              }
              else
              {
                pid.params[idx]-= 2*pid.dp[idx];
                pid.error = 0.0;
                flag = 1;
                cout<<"Decreasing the parameters.."<<endl;
                // cout<<"Reseting the sim."<<endl;
                // cout<<"Index "<<idx<<endl;
                // std::string reset_msg = "42[\"reset\",{}]";
                // ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }

            else if(flag == 1)
            {
              if(pid.error<pid.best_error)
                {
                  cout<<"Check 2"<<endl;
                  // cout<<"Index "<<idx<<endl;
                  pid.best_error = pid.error;
                  pid.dp[idx]*=1.1;
                }
                else
                {
                  cout<<"Check 3"<<endl;
                  // cout<<"Index "<<idx<<endl;
                  pid.params[idx]+=pid.dp[idx];
                  pid.dp[idx]*=0.9;
                }
                // pid.param_update(idx);//For the next time.
                pid.error = 0.0;
                flag = 0;
            }
            // cout<<"Before index change"<<endl;  
            if(flag != 1) 
            { 
              // idx++;
              // idx = idx%2;
              pid.param_update(idx);
            }
          }
          it++;

          // cout<<"Iterunation number "<<it<<endl;
          cout<<"Kp "<<pid.params[0]<<" "<<"Kd "<<pid.params[1]<<endl;
          cout<<"dp1 "<<pid.dp[0]<<" "<<"dp2 "<<pid.dp[1]<<endl;
          cout<<"Sum of DPs "<<pid.dp[0] + pid.dp[1]<<endl;
          cout<<"Best error "<<pid.best_error<<endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.10;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
  }
  });

h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

int main()
{
  uWS::Hub h;
  PID pid;

  // TODO: Initialize the pid variable.
  double kp = 1.1;//0.2
  // double ki = 0.0;//0.04
  double kd = 1.25;//3.0
  
  pid.Init(kp,kd);
  pid.set_drift(0.0);

  run_robot(pid);
}