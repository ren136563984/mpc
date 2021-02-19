#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>

#include "Eigen/Core"

typedef CPPAD_TESTVECTOR(double) Dvector;

const int N = 10;       // 预测步数
const double dt = 0.1;  // 每步时间

const double Lf = 2.67;             // 舵轮到重心的距离
const double VELOCITY_MAX = 100.0;  // 最大速度

const int NUMBER_OF_STATES = 6;      // px, py, psi, v, cte, epsi
const int NUMBER_OF_ACTUATIONS = 2;  // steering angle, acceleration

const int NX = N * NUMBER_OF_STATES +
               (N - 1) * NUMBER_OF_ACTUATIONS;  // 状态和控制变量总数量
const int NG = N * NUMBER_OF_STATES;            // 约束数量

// where the first element of each state variable is stored in the vector to be
// feeded the optimization algorithm
const int ID_FIRST_px = 0;
const int ID_FIRST_py = ID_FIRST_px + N;
const int ID_FIRST_psi = ID_FIRST_py + N;
const int ID_FIRST_v = ID_FIRST_psi + N;
const int ID_FIRST_cte = ID_FIRST_v + N;
const int ID_FIRST_epsi = ID_FIRST_cte + N;
const int ID_FIRST_delta = ID_FIRST_epsi + N;
const int ID_FIRST_a = ID_FIRST_delta + N - 1;

// 损失权重
const double W_cte = 1500.0;    //位置误差
const double W_epsi = 1500.0;   //航向角误差
const double W_v = 1.0;         //速度
const double W_delta = 10.0;    //打角权重
const double W_a = 10.0;        //加速度权重
const double W_ddelta = 150.0;  //连续打角之间的高差的权重
const double W_da = 15.0;       //连续加速驱动之间的高差的权重

class MPC {
 public:
  double steer;     //声明舵轮变量
  double throttle;  //申明加速度变量

  Dvector x;             //储存状态变量与控制变量
  Dvector x_lowerbound;  // x约束下界
  Dvector x_upperbound;  // x约束上界
  Dvector g_lowerbound;  //每个对应约束表达式的值约束
  Dvector g_upperbound;  //每个对应约束表达式的值约束

  std::vector<double> future_xs;
  std::vector<double> future_ys;

  MPC();
  virtual ~MPC();

  // 该函数求解给定当前状态和道路曲线系数的模型。
  void solve(Eigen::VectorXd state, Eigen::VectorXd K);
};

#endif /* MPC_H */
