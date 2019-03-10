#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
// 多点拟合曲线工具
#include "spline.h"

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

  // en: Start in lane 1
  // zh: 从最中间的车道开始
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 0.0;  // mph

  h.onMessage(
      [&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
      &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
          uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {

          auto s = hasData(data);

          if (s != "")
          {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry")
            {
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

              /**
               * TODO: define a path made up of (x,y) points that the car will visit
               *   sequentially every .02 seconds
               */
              // en: The previous path vector's size.
              // zh: 之前的路径点集合的大小
              int prev_size = previous_path_x.size();
              // en: Sensor fusion is below.
              // zh: 下面是传感器融合的内容
              if (prev_size > 0)
              {
                car_s = end_path_s;
              }

              // en: Too close to the car in front of us
              // zh: 是否与前车过于接近
              bool too_close = false;
              // en: There is a car on left of us
              // zh: 左边有车
              bool car_in_left_lane = false;
              // en: There is a car on right of us
              // zh: 右边有车
              bool car_in_right_lane = false;

              // Find ref_v to use
              for (int i = 0; i < sensor_fusion.size(); i++)
              {

                float d = sensor_fusion[i][6];
                // en: Detect the car around us is at which lane
                // zh: 判断周围的车在哪条车道
                int other_car_lane;
                if (d >= 0 && d < 4)
                {
                  other_car_lane = 0;
                }
                else if (d >= 4 && d < 8)
                {
                  other_car_lane = 1;
                }
                else if (d >= 8 && d <= 12)
                {
                  other_car_lane = 2;
                }
                else
                {
                  continue;
                }

                // 备注: 车道的宽度为4米
                // Check width of lane, in case cars are merging into our lane
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5];

                // If using previous points can project an s value outwards in time
                check_car_s += ((double)prev_size * .02 * check_speed);

                // Check s values greater than ours and s gap
                double s_gap = 30;
                // en: Car in front of us.
                // zh: 车辆在我们正前方
                if (other_car_lane == lane)
                {
                  if (((car_s - s_gap) < check_car_s) && ((car_s + s_gap) > check_car_s))
                  {
                    too_close = true;
                  }
                }
                // en: Car is on left of us.
                // zh: 车辆在我们左方车道
                else if (lane - other_car_lane == 1)
                {
                  if (((car_s - s_gap) < check_car_s) && ((car_s + s_gap) > check_car_s))
                  {
                    car_in_left_lane = true;
                  }
                }
                else if (other_car_lane - lane == 1)
                {
                  if (((car_s - s_gap) < check_car_s) && ((car_s + s_gap) > check_car_s))
                  {
                    car_in_right_lane = true;
                  }
                }
              }

              if (too_close)
              {
                // en: Left lane is high priority
                // zh: 优先走超车道超车
                if (!car_in_left_lane && lane > 0)
                {
                  lane--;
                }
                // en: Right lane is mid priority
                // zh: 试着从慢车道超车
                else if (!car_in_right_lane && lane < 2)
                {
                  lane++;
                }
                // en: If we can not overtake, then slow down the speed
                // zh: 实在不行就减速
                else
                {
                  ref_vel -= .224;
                }
              }
              // en: In order to avoid exceeding the speed limit (50 mph), it is better to use 49.5.
              // zh: 给定最大的车速。为了避免超过限速（50mph），所以使用49.5比较好。
              else if (ref_vel < 49.5)
              {
                ref_vel += .224;
              }
              // en: Here is the code for the path planning, how to make the car stable and not overspeed
              // zh: 以下是路径规划的代码，如何让汽车行驶的稳定并且不超速
              vector<double> ptsx;
              vector<double> ptsy;

              // en: Reference x,y, yaw status
              // en: either we will reference the starting point as where the car is or at the previous paths end point
              // zh: 我们将把上一个位置作为当前的起始点
              double ref_x = car_x;
              double ref_y = car_y;
              double ref_yaw = deg2rad(car_yaw);

              // en: If previous size is almost empty, use the car as starting reference
              // zh: 检查之前路径点集合的大小，如果小于2（接近为空） 时
              if (prev_size < 2)
              {
                // en: Use two points that make the path tangent to the car
                // zh: 使用这两个点来生成一条与当前车辆位置相切的曲线
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
              }
              // en: Use the previous path's endpoint as starting reference
              // zh: 使用上一个路径的终点作为开始
              else
              {
                // en: Redefine reference state as previous path end point
                // zh: 重新定义当前状态
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                // en: 2nd-to-last point
                // zh: 倒数第二个点
                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                // en: Use two points that make the path tangent to the path's previous endpoint
                // zh: 使用这两个点来生成一条与上一个终点相切的曲线
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
              }

              // en: In frenet add evenly 30m spaced points ahead of the starting reference
              // zh: 在Frenet坐标系中在起始点前每隔30米添加一个点，只创建几个点，剩下的由spline来拟合。备注: 车道的宽度为4米
              vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

              ptsx.push_back(next_wp0[0]);
              ptsx.push_back(next_wp1[0]);
              ptsx.push_back(next_wp2[0]);

              ptsy.push_back(next_wp0[1]);
              ptsy.push_back(next_wp1[1]);
              ptsy.push_back(next_wp2[1]);

              for (int i = 0; i < ptsx.size(); i++)
              {
                // en: Shift car reference angle to 0 degrees
                // zh: 将汽车参考角度转换为0度
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
              }

              // en: Create a spline
              // zh: 创建spline实例
              tk::spline s;

              // en: Set (x,y) points to the spline
              // zh: 传入点
              s.set_points(ptsx, ptsy);

              vector<double> next_x_vals;
              vector<double> next_y_vals;
              // en: Start with all the previous path points from last time
              // zh: 将上次的路径点添加进去（上一次汽车行驶过的点会被消费，所以没有行驶过的点就会剩下。）
              for (int i = 0; i < previous_path_x.size(); i++)
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // en: Calculate how to break up spline points so that we travel at our desired reference velocity
              // zh: 计算如何分解样条点，保持我们不超过限速行驶
              double target_x = 30.0;
              double target_y = s(target_x);
              double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
              double x_add_on = 0;

              // en: Fill up the rest of the path planner after filling it with previous points, here we will always output 50 points
              // zh: 因为上一次汽车行驶过的点会被消费，所以这次只需要生成消费过数量的点即可满足要求（50个点）
              for (int i = 1; i <= 50 - previous_path_x.size(); i++)
              {
                double N = (target_dist / (.02 * ref_vel / 2.24));
                double x_point = x_add_on + (target_x) / N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // Rotate back to normal after rotating it earlier
                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }

              msgJson["next_x"] = next_x_vals;
              msgJson["next_y"] = next_y_vals;

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
          }
          else
          {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }  // end websocket if
      });  // end h.onMessage

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
