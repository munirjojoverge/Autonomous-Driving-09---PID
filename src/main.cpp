#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

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

int main()
{
  uWS::Hub h;

  PID pid_Steer, pid_Brk;
  // TODO: Initialize the pid variable.
  // FORMAT: Init(double Kp, double Ki, double Kd, double max, double min, bool AutoTune);
  double MaxAbsSteerIni = 1.0;
  // 0.134611, 0.000270736, 3.05349
  // LAST BEST PARAMS : Kp 0.145111 Ki 0.0207707 Kd 3.16663
  // LAST BEST PARAMS : Kp 0.195537 Ki 0.1791 Kd 3.17663
  // LAST BEST PARAMS : Kp 0.195537 Ki 0.1891 Kd 3.17663
  // LAST BEST PARAMS : Kp 0.195537 Ki 0.1991 Kd 3.17663
  // LAST BEST PARAMS : Kp 0.145111 Ki 0.0502707 Kd 3.16663
  //pid_Steer.Init(4.2611, 1.702707, 5.0663, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });
  //pid_Steer.Init(3.9, 1.15, 5.1, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });
  //pid_Steer.Init(3.0, 0.9, 4.8, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });
  //pid_Steer.Init(5.13, 0.123, 20.9, MaxAbsSteerIni, -MaxAbsSteerIni, true, { false, false, false });
  //Last Succesful by Twiddle: Testing PID : Kp = 5.26403 Ki = 0.162475 Kd = 21.0102
  //Testing PID : Kp = 5.33293 Ki = 0.20345 Kd = 21.0202
  //Testing PID : Kp = 5.43607 Ki = 0.21345 Kd = 21.0397
  //pid_Steer.Init(5.43607, 0.21345, 21.0397, MaxAbsSteerIni, -MaxAbsSteerIni, true, { false, false, false });
  //pid_Steer.Init(4.0023607, 0.21345, 5.00397, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });
  //pid_Steer.Init(3.0123607, 0.21345, 4.7397, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });
  pid_Steer.Init(3.000123607, 0.1345, 3.0397, MaxAbsSteerIni, -MaxAbsSteerIni, false, { false, false, false });


  
  
  double MaxBrk = 2.0;
  double MinBrk = 0;
  //0.4731, 0.0000, 1.026185
  // LAST BEST PARAMS : Kp 0.506201 Ki 0 Kd 1.13747
  // LAST BEST PARAMS : Kp 0.672447 Ki 0 Kd 1.1979
  // LAST BEST PARAMS : Kp 0.741352 Ki 0 Kd 1.2079
  // LAST BEST PARAMS : Kp 0.758907 Ki 0 Kd 1.2079
  // LAST BEST PARAMS : Kp 0.4731 Ki 0 Kd 1.05568
  //pid_Brk.Init(0.70031, 0.0000, 3.15568, MaxBrk, MinBrk, false, { false, true, false });
  //pid_Brk.Init(7.95, 0.0000, 9.95, MaxBrk, MinBrk, false, { false, true, false });
  //pid_Brk.Init(0.00007, 0.0000, 0.0018, MaxBrk, MinBrk, false, { false, true, false });
  //pid_Brk.Init(1.13, 0.0000, 4.8, MaxBrk, MinBrk, true, { false, true, false });
  //Testing PID: Kp = 1.17302 Ki = 0 Kd = 4.8789
  //Testing PID : Kp = 1.20399 Ki = 0 Kd = 4.99204
  pid_Brk.Init(1.02399, 0.0000, 4.0204, MaxBrk, MinBrk, false, { false, true, false });

  static double SteerError, MaxCTE, MaxSpeed = 0.0;
  

  h.onMessage([&pid_Steer, &pid_Brk](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          double steer_cmd;
		  double thrBrk_cmd;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
		  /**********************************
						STEERING
		  ***********************************/
		  double maxAbsSteer = 1.0;
		  
		  if (fabs(speed) > 2) {
			  SteerError = cte * fabs(cte) / speed;
		  }
		  else {
			  SteerError = cte * fabs(cte);
		  }
		  steer_cmd = -pid_Steer.Run(SteerError, maxAbsSteer, -maxAbsSteer);
		  
		  /**********************************
					THROTTLE AND BRAKE
		  ***********************************/
		  double MaxBrk, MinBrk;
		  MinBrk = 0.0;
		  if (( (speed < 50.0) && (fabs(cte) < 1.2) ) or (speed < 30))  {
			  MaxBrk = 0.0;
		  }
		  else
		  {
			  MaxBrk = 2.0;
		  }
		 
		  double BrkError, SafetyDrivingFactor;
		  SafetyDrivingFactor = 1.0;
		 
		  if (fabs(speed) > 1) {
			  BrkError = fabs(cte*cte / speed) * SafetyDrivingFactor;
		  }
		  else {
			  BrkError = fabs(cte);
		  }
		  
		  double BrkCompFactor = 0.0;
		  thrBrk_cmd = (1 + BrkCompFactor) - pid_Brk.Run(BrkError, MaxBrk, MinBrk);
		

          // DEBUG
		  MaxCTE = (MaxCTE > cte) ? MaxCTE : cte;
		  MaxSpeed = (MaxSpeed > speed) ? MaxSpeed : speed;
		  
		  /*
		  std::cout << "CTE: " << cte << " SPEED: " << speed << " cteAvg:" << cteAvg << std::endl;
		  MaxCTE = (MaxCTE > cte) ? MaxCTE : cte;
		  MaxSpeed = (MaxSpeed > speed) ? MaxSpeed : speed;
	      std::cout << "MaxCTE: " << MaxCTE << " MaxSPEED: " << MaxSpeed << std::endl;
			//minThr = !(maxThr < minThrottle) ? minThrottle : maxThr;
		  std::cout << " Steering Cmd: " << steer_cmd << " Thr Cmd: " << thrBrk_cmd << std::endl;
		  */

          json msgJson;
          msgJson["steering_angle"] = steer_cmd;
          msgJson["throttle"] = thrBrk_cmd;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // When you press ESC
  std::ofstream myfile;
  myfile.open("Run_Report.txt");
  myfile << "MaxCTE: " << MaxCTE << " MaxSPEED: " << MaxSpeed << std::endl;
  myfile << "Steer PID Params: Kp " << pid_Steer.Kp << " " << "Ki " << pid_Steer.Ki << " " << "Kd " << pid_Steer.Kd << std::endl;
  myfile << "Brake PID Params: Kp " << pid_Brk.Kp << " " << "Ki " << pid_Brk.Ki << " " << "Kd " << pid_Brk.Kd << std::endl;
  myfile.close();
  

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
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


