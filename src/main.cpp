#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // start in lane 1
  int lane = 1;

  // Reference velocity (mph) to target
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          double mph2mps = 0.44704; // 1 mph is 0.44704 m/s

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          double car_speed_mps = mph2mps * car_speed;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          // min distance b/w us and the car next to us to consider a lange change
          double min_dist = 7.0;
          // min distance (one sec into the future) b/w us and the car next to us
          // to consider a lange change
          double clear_dist = 14.0;
          // safe distance b/w us and the car ahead
          double safe_dist = 30.0;
          // max distance range of considered cars
          double range_dist = 60.0;
          // "infinity" distance (for default cost calculation)
          double inf_dist = 100.0;
          // small cost of a lane shift to avoid going back and forth
          double lane_shift_cost = 1.0;
          double cost_center = 0.0;
          double cost_left = (lane == 0) ? std::numeric_limits<double>::infinity() : lane_shift_cost;
          double cost_right = (lane == 2) ? std::numeric_limits<double>::infinity() : lane_shift_cost;
          
          // min speeds for the left/center/right cars ahead of us
          double speed_center = std::numeric_limits<double>::infinity();
          double speed_left = std::numeric_limits<double>::infinity();
          double speed_right = std::numeric_limits<double>::infinity();
          double speed_left_left = std::numeric_limits<double>::infinity();
          double speed_right_right = std::numeric_limits<double>::infinity();
          
          // distance to the closest car ahead of us
          double space_center = inf_dist;
          double space_left = inf_dist;
          double space_right = inf_dist;
          double space_left_left = inf_dist;
          double space_right_right = inf_dist;

          // find ref_v to use
          for (int i=0; i<sensor_fusion.size(); i++) {
            // car is in my lane
            float d = sensor_fusion[i][6];
            double lane_d = 2 + 4*lane; // d coordinate for our lane's center
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            // car speed in m/s
            double check_speed = sqrt(vx*vx + vy*vy);
            double speed_diff = check_speed - car_speed_mps;
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*0.02*check_speed);
            double s_diff = check_car_s - car_s;
            // distance b/w us and the car 1 second into the future assuming no changes
            double future_s_diff = s_diff + speed_diff;
            if (lane_d-2 < d && d < lane_d+2) {
              // car is in our (center) lane
              if (s_diff > 0) {
                // car is ahead of us
                space_center = std::min(space_center, s_diff);
                if (s_diff < range_dist) {
                  // min speed of cars ahead of us within the considered distance range
                  speed_center = std::min(speed_center, speed_diff);
                }
                if (future_s_diff < safe_dist) {
                  too_close = true;
                }
              }
            } else if (lane >= 1 && lane_d-6 < d && d < lane_d-2) {
              // car is in the left lane relative to us
              if (fabs(s_diff) < min_dist || fabs(future_s_diff) < clear_dist) {
                // can't shift to the left lane due to traffic
                space_left = 0.0;
              } else if (s_diff > 0) {
                // car is ahead of us
                space_left = std::min(space_left, s_diff);
                if (s_diff < range_dist) {
                  // min speed of cars ahead of us within the considered distance range
                  speed_left = std::min(speed_left, speed_diff);
                }
              }
            } else if (lane <= 1 && lane_d+2 < d && d < lane_d+6) {
              // car is in the right lane relative to us
              if (fabs(s_diff) < min_dist || fabs(future_s_diff) < clear_dist) {
                // can't shift to the right lane due to traffic
                space_right = 0.0;
              } else if (s_diff > 0) {
                // car is ahead of us
                space_right = std::min(space_right, s_diff);
                if (s_diff < range_dist) {
                  // min speed of cars ahead of us within the considered distance range
                  speed_right = std::min(speed_right, speed_diff);
                }
              }
            } else if (lane == 2 && lane_d-10 < d && d < lane_d-6) {
              // car is two lanes to the left of us
              if (fabs(s_diff) < min_dist || fabs(future_s_diff) < clear_dist) {
                // can't shift two lanes to the left
                space_left_left = 0.0;
              } else if (s_diff > 0) {
                // car is ahead of us
                space_left_left = std::min(space_left_left, s_diff);
                if (s_diff < range_dist) {
                  // min speed of cars ahead of us within the considered distance range
                  speed_left_left = std::min(speed_left_left, speed_diff);
                }
              }
            } else if (lane == 0 && lane_d+6 < d && d < lane_d+10) {
              // car is two lanes to the right of us
              if (fabs(s_diff) < min_dist || fabs(future_s_diff) < clear_dist) {
                // can't shift two lanes to the right
                space_right_right = 0.0;
              } else if (s_diff > 0) {
                // car is ahead of us
                space_right_right = std::min(space_right_right, check_car_s - car_s);
                if (s_diff < range_dist) {
                  // min speed of cars ahead of us within the considered distance range
                  speed_right_right = std::min(speed_right_right, speed_diff);
                }
              }
            }
          }
          
          if (space_left == 0.0) {
            cost_left = std::numeric_limits<double>::infinity();
          } else {
            cost_left += (safe_dist - space_left);
            if (lane == 2) {
              cost_left += std::min(0.0, space_left - space_left_left);
            }
          }
          if (space_right == 0.0) {
            cost_right = std::numeric_limits<double>::infinity();
          } else {
            cost_right += (safe_dist - space_right);
            if (lane == 0) {
              cost_right += std::min(0.0, space_right - space_right_right);
            }
          }
          cost_center += (safe_dist - space_center);
          double speed_diff_mph = speed_center / mph2mps;
          
          //std::cout << "car_speed_mps = " << car_speed_mps << ", speed_left = " << speed_left;
          //std::cout << ", speed_center = " << speed_center << ", speed_right = " << speed_right << std::endl;
          //std::cout << "cost_center = " << cost_center << ", cost_left = " << cost_left << ", cost_right = " << cost_right << std::endl;
          double max_acc = 0.448;
          if (cost_center <= cost_left && cost_center <= cost_right) {
            // stay in the current lane: changing lanes isn't cost optimal
            if (too_close) {
              // the car ahead is too close - slow down
              // deceleration due to lack of space ahead of us
              double acc_space = max_acc * std::max(0.0, safe_dist - std::max(clear_dist, space_center)) / (safe_dist - clear_dist);
              // deceleration due to speed difference between us and the car ahead
              double acc_speed = std::min(max_acc, std::max(0.0, -speed_diff_mph));
              ref_vel -= std::max(acc_space, acc_speed);
            } else if (ref_vel < 49.5) {
              // acceleration due to available space ahead of us
              double acc_space = max_acc * std::min(1.0, std::max(0.0, space_center - safe_dist) / safe_dist);
              // acceleartion due to speed difference between us and the car ahead
              double acc_speed = std::min(max_acc, std::max(0.0, speed_diff_mph));
              ref_vel += std::max(acc_space, acc_speed);
            }
          } else if (cost_left <= cost_right) {
            // change lane to the left
            lane--;
          } else {
            // change lane to the right
            lane++;
          }

          // Vector of widely spaced (x, y) waypoints, evenly spaced at 30m
          // These waypoints are later interpolated with a spline
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y,yaw states
          // we will either reference the starting point using the car position
          // or the previous path's end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            // use the car as a starting reference

            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // use the previous path's end point as a starting reference

            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          double d = 2 + 4*lane;
          double spacing = 35.0;
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + spacing, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 2*spacing, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 3*spacing, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (int i=0; i<previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at the desired ref velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

          double x_add_on = 0;

          // Fill up the rest of the path planner after filling it with prev points
          for (int i=1; i<=50-previous_path_x.size(); i++) {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
