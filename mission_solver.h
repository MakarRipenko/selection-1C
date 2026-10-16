#pragma once
#include <vector>

class MissionSolver {
  public:
  constexpr static int kInf = 1000000000;

  struct Edge {
    int from;
    int to;
    int time;
    int deadline = kInf;
    int mask_artifacts = 0;
  };

  struct Artifact {
    int node;
    int cost;
    std::pair<int, int> time_available = {-kInf, kInf};
  };

  struct Request {
    int count_nodes;
    int start_node;
    std::vector<Edge> edges;
    std::vector<Artifact> artifacts;
    std::vector<int> finish_nodes;
    int t_lim;
    double t_cost;
  };

  struct Response {
    bool is_feasible = false;
    int min_time = kInf;
    double max_profit = -kInf;
  };

private:
  struct Edge2 {
    int to;
    int time;
    std::pair<int, int> time_available = {-kInf, kInf};
  };

public:
  explicit MissionSolver(const Request* start_graph);
  MissionSolver() = default;

  auto ChangeRequest(const Request* start_graph) -> void;

  auto CheckMission() const -> Response;

private:
  auto CheckRequest() const -> bool;

  auto CountArtifacts() const -> int;

  auto TransformFromMask(int mask) const -> std::pair<int, int>;

  auto TransformToMask(int number_node, int mask_artifacts) const -> int;

  auto Dijkstra(const std::vector<std::vector<Edge2>>& graph, int start_state, int start_time) const
      -> std::vector<int>;

  auto CalculatePossibleStartTimes(const std::vector<std::vector<Edge2>>& graph) const -> std::vector<int>;

  void RecalculateResponse(Response& response, const std::vector<int>& dist, int cur_start_time) const;

private:
  const Request* request_{};
};
