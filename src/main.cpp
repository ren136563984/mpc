#include <math.h>
#include <uWS/uWS.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "nlohmann/json.hpp"

const int PORT = 4567;

// for convenience
using json = nlohmann::json;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {  //拟合函数
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  //
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          //**************************************************************
          //* 获取当前状态
          //**************************************************************

          // j[1]数据体对象
          std::vector<double> points_xs = j[1]["ptsx"];  // x点序列
          std::vector<double> points_ys = j[1]["ptsy"];  // y点序列
          //接收到的车当前状态和控制量
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double delta = j[1]["steering_angle"];
          const double a = j[1]["throttle"];

          //**************************************************************
          //* 全局空间转换为车当前空间
          //**************************************************************
          const int NUMBER_OF_WAYPOINTS = points_xs.size();
          Eigen::VectorXd waypoints_xs(NUMBER_OF_WAYPOINTS);
          Eigen::VectorXd waypoints_ys(NUMBER_OF_WAYPOINTS);

          for (int i = 0; i < NUMBER_OF_WAYPOINTS; ++i) {
            const double dx = points_xs[i] - px;
            const double dy = points_ys[i] - py;

            waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
            waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
          }

          //**************************************************************
          //* 拟合多项式
          //**************************************************************
          const int ORDER = 3;
          auto K = polyfit(waypoints_xs, waypoints_ys, ORDER);  //曲率系数

          //**************************************************************
          //* 从拟合多项式（道路曲线）获取要显示的点
          //**************************************************************
          std::vector<double> next_xs(N);
          std::vector<double> next_ys(N);
          const double D = 5.0;

          for (int i = 0; i < N; ++i) {
            const double dx = D * i;
            const double dy =
                K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];

            next_xs[i] = dx;
            next_ys[i] = dy;
          }

          //**************************************************************
          //* 生成当前错误估计 (cte, epsi)
          //**************************************************************

          // 当前位置误差CTE是拟合多项式（道路曲线），px=0.0
          // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
          const double cte = K[0];

          // 当前航向角误差epsi是px=0.0时与道路曲线的切线
          // epsi = arctan(f') where f' is the derivative of the fitted
          // polynomial f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
          const double epsi = -atan(K[1]);

          //**************************************************************
          //* 获取当前延迟状态
          //**************************************************************

          const double dt = 0.1;
          const double Lf = 2.67;

          // 当前状态必须是车辆坐标系，px, py, psi = 0.0, 0.0, 0.0
          // 延迟造成车辆向前直线行驶，只改变x，不改变y和psi
          // 在转换过程中，我们把所有的路径点旋转了-psi，因此打角是负值
          const double current_px = 0.0 + v * dt;
          const double current_py = 0.0;
          const double current_psi = 0.0 + v * (-delta) / Lf * dt;
          const double current_v = v + a * dt;
          const double current_cte = cte + v * sin(epsi) * dt;
          const double current_epsi = epsi + v * (-delta) / Lf * dt;

          const int NUMBER_OF_STATES = 6;
          Eigen::VectorXd state(NUMBER_OF_STATES);
          state << current_px, current_py, current_psi, current_v, current_cte,
              current_epsi;

          //**************************************************************
          //* 使用MPC确定下一步控制方案和预测状态
          //**************************************************************
          mpc.solve(state, K);
          // cout << setw(20) << mpc.steer << setw(20) << mpc.throttle << endl;
          //准备数据发送
          json msgJson;
          msgJson["steering_angle"] = mpc.steer;
          msgJson["throttle"] = mpc.throttle;

          msgJson["mpc_x"] = mpc.future_xs;
          msgJson["mpc_y"] = mpc.future_ys;

          msgJson["next_x"] = next_xs;
          msgJson["next_y"] = next_ys;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          // 延迟仿真
          this_thread::sleep_for(chrono::milliseconds(100));

          // sent do simulator
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  if (h.listen(PORT)) {
    std::cout << "Listening to port " << PORT << std::endl;
  } else {
    std::cerr << "Failed to listen to port " << PORT << std::endl;
    return -1;
  }
  h.run();
}
