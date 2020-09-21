#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

bool inc_p = true;
bool inc_check = true;
bool dec_p = false;
bool dec_check = false;
bool twiddle_on = false;

int twiddle_id=0;
int n = 0;
int sub_move = 0;
int ext_str_value=0;

double p[] = {0.2,0.0004,10};
double dp[] = {p[0]/10.0,p[1]/10.0,p[2]/10.0};

//double p[] = {0.2,0.0004,4};
//double dp[] = {1.0,1.0,1.0};

double total_cte = 0.0;
double tol = 0.002;
double avg_prev_err=0.0;
int n_prev_err=1;
double best_err = 100000.0;


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
			
			if (n==0) {
				pid.Init(p[0],p[1],p[2]);
				//twiddle_on=true;
			}
			
			if (n>1) { twiddle_on=true; }
			
			double err;
			if (twiddle_on==true) {
				
				total_cte+=cte*cte;
				err=total_cte/(n-1);
				std::cout <<"best err : " << best_err<<"\t"<<err<<"\n";
				
				if ((dp[0]+dp[1]+dp[2])>tol) {
					
					if (inc_p==true){
						p[twiddle_id]+=dp[twiddle_id];
						inc_p = false;
						std::cout<<"inc_p"<<"\n";
					} else {
						if (inc_check==true && err<best_err) {
                        std::cout <<"best err : "<< best_err<<"\t"<<err<<"\n";
						best_err=err;
                        std::cout <<"best err : "<< best_err<<"\t"<<err<<"\n\n";
						dp[twiddle_id]*=1.1;
						//twiddle_id+=1;
						std::cout<<"inc_check"<<"\n";
						//inc_p=true;
						inc_check=false;
						sub_move+=1;
						//dec_p=true;
						//dec_check=true;
						} else {
							if (inc_check==true){
								p[twiddle_id]-=2*dp[twiddle_id];
								inc_check = false;
								//dec_check = true;
								//dec_p=false;
								std::cout<<"dec_p"<<"\n";
							} else {
								if (err<best_err) {
									best_err=err;
									dp[twiddle_id]*=1.1;
									sub_move+=1;
									//twiddle_id+=1;
									std::cout<<"dec_check"<<"\n";
									//inc_p=true;
								} else {
									p[twiddle_id]+=dp[twiddle_id];
									dp[twiddle_id]*=0.9;
									sub_move+=1;
									//twiddle_id+=1;
									std::cout<<"None"<<"\n";
									//inc_p=true
								}		
							}
						}	
					}	
				}
			}				
			
				pid.Init(p[0],p[1],p[2]);
				pid.UpdateError(cte);
			//pid.Twiddle();
				steer_value=-pid.TotalError();
				
				if (sub_move>0) {
					twiddle_id+=1;
					twiddle_id=twiddle_id%3;
					inc_p=true;
					inc_check=true;
					sub_move=0;
					if (best_err<err && twiddle_on==true){
						if (avg_prev_err<err){
							avg_prev_err=((avg_prev_err*n_prev_err)+err)/(n_prev_err+1);
							n_prev_err+=1;
						} else {
							avg_prev_err=0.0;
							n_prev_err=1.0;
						}
					}	
				}
								
				
				if (n_prev_err>10)//(abs(steer_value-angle)>0.5)
				{ 
					total_cte=0.0;
					best_err=100000;
					pid.Init(0.2,0.0004,10);
					dp[0]=0.02; dp[1]=0.00004; dp[2]=1;
					n=100;
					avg_prev_err=0.0;
					n_prev_err=1.0;
				}
				
				n+=1;
				
			
			if (steer_value>1) {
				steer_value=1;
				ext_str_value+=1;
			}
			else if (steer_value<-1) {
				steer_value=-1;
				ext_str_value+=1;
			} else {
				ext_str_value=0;
			}
		  
          // DEBUG
		  std::cout <<"best err : " << best_err<<"\t"<<err<<"\n";
		  std::cout<<inc_p<<"\t"<<inc_check<<"\t"<<dec_p<<"\t"<<dec_check<<"\n";
		  std::cout<<dp[0]<<"\t"<<dp[1]<<"\t"<<dp[2]<<"\n";
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;
		  std::cout<<"\n";

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