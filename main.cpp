#include <iostream>
#include "mission_solver.h"

const int kInf = 1000000000;

int main() {
  MissionSolver::Request example{
      .count_nodes = 5,
      .start_node = 0,
      .edges =
          {
              {0, 1, 5},
              {1, 2, 20},
              {2, 0, 10, kInf, 1 << 1},
              {0, 4, 0, kInf, 1 << 1},
              {0, 3, 20},
              {3, 1, 10},
              {3, 2, 5},
          },
      .artifacts =
          {
              {3, 2, {0, 10}},
              {1, 2, {30, 40}},
              {4, 8, {0, 80}},
          },
      .finish_nodes = {3, 4},
      .t_lim = 120,
      .t_cost = 10,
  };

  MissionSolver mission_solver(&example);

  auto response = mission_solver.CheckMission();
  std::cout << response.is_feasible << " " << response.min_time << " " << response.max_profit << "'\n'";
  return 0;
}
