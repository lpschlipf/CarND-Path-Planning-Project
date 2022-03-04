#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          bool car_proximity = false;
          bool car_left = false;
          bool car_right = false;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // Assign lane to the object.
            float d = sensor_fusion[i][6];
            int car_lane = -1;
            if (d > 0 && d <= 4)
            {
              car_lane = 0;
            }
            else if (d > 4 && d <= 8)
            {
              car_lane = 1;
            }
            else if (d > 8 && d <= 12)
            {
              car_lane = 2;
            }
            if (car_lane < 0)
            {
              continue;
            }

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            // Predict car s position.
            check_car_s += ((double)speed * 0.02 * prev_size);
            if (car_lane == lane)
            {
              // vehicle in our lane.
              too_close |= check_car_s > car_s && check_car_s - car_s < 30;
            }
            else if (car_lane == lane - 1)
            {
              // vehicle left
              car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
            else if (car_lane == lane + 1)
            {
              // vehicle right
              car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
          }

          // Decide what to do based on positions of other cars, with the aim to get into the rightmost lane.
          double speed_diff = 0;
          if (too_close)
          {
            if (!car_left && lane > 0)
            {
              // Can safely change to left
              lane--;
            }
            else if (!car_right && lane < 2)
            {
              // We can safely change to the right lane.
              lane++;
            }
            else
            {
              // decelerate, there is a car in front of us!
              ref_vel -= 0.2;
            }
          }
          else
          {
            if (lane != 0)
            {
              // Let's try to get back to the rightmost lane!
              if (lane == 1 && !car_right)
              {
                lane = 0; 
              }
              else if (lane == 2 && !car_left)
              {
                lane = 1;
              }
            }
            if (ref_vel < 49.5)
            {
              // Accelerate, the road is clear!
              ref_vel += 0.2;
            }
          }

          // Generate the trajectory that executes action.
          // Gerneate a coarse path and interpolate using splines.
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_raw = deg2rad(car_yaw);

          if (prev_size < 2)
          {
            // Use the car's last position as a starting point when we have too few previous path points.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // We have enopugh previous path points to use them for starting the next path.
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_raw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's end points.
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          // Add Frenet waypoints.
          vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Shift the points to local car coordinates, i.e. point at origin and angle = 0!
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_raw) - shift_y * sin(0 - ref_raw);
            ptsy[i] = shift_x * sin(0 - ref_raw) + shift_y * cos(0 - ref_raw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          // We re-use points from the last valid path.
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          // Finally interpolate up to 50 points on the previous path points, so that the car will
          // travel at the desired reference speed.
          for (int i = 0; i <= 50 - prev_size; ++i)
          {
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Transform back to global map coordinate system
            x_point = x_ref * cos(ref_raw) - y_ref * sin(ref_raw);
            y_point = x_ref * sin(ref_raw) + y_ref * cos(ref_raw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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