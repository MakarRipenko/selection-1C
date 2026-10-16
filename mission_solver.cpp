#include <queue>
#include <vector>
#include <unordered_set>
#include "mission_solver.h"

namespace {
auto IsSubmask(int mask1, int mask2) -> bool {
  return (mask1 & mask2) == mask1;
}
}  // namespace

MissionSolver::MissionSolver(const Request* start_graph)
    : request_(start_graph) {
  bool check = CheckRequest();
  if (!check) {
    throw std::runtime_error("Invalid request");
  }
}

auto MissionSolver::ChangeRequest(const Request* start_graph) -> void {
  auto old_request = request_;
  request_ = start_graph;
  bool check = CheckRequest();
  if (!check) {
    request_ = old_request;
    throw std::runtime_error("Invalid request");
  }
}
auto MissionSolver::CheckRequest() const -> bool {
  if (request_ == nullptr) {
    return true;
  }

  const int k_max_artifacts = 20;
  const int k_max_states = 1 << 24;
  const int k_max_time = kInf / 4;

  const int count_nodes = request_->count_nodes;
  const int count_artifacts = static_cast<int>(request_->artifacts.size());
  auto is_node = [count_nodes](int node) { return 0 <= node && node < count_nodes; };
  auto is_time = [k_max_time](int time) { return -k_max_time <= time && time <= k_max_time; };

  if (count_nodes <= 0 || !is_node(request_->start_node)) {
    return false;
  }
  if (count_artifacts > k_max_artifacts || count_nodes > k_max_states / (1 << count_artifacts)) {
    return false;
  }
  if (!is_time(request_->t_lim) || request_->t_cost <= 0) {
    return false;
  }

  for (auto node : request_->finish_nodes) {
    if (!is_node(node)) {
      return false;
    }
  }

  for (const auto& edge : request_->edges) {
    if (!is_node(edge.from) || !is_node(edge.to)) {
      return false;
    }
    if (edge.time < 0 || edge.time > k_max_time) {
      return false;
    }
    if (edge.deadline != kInf && !is_time(edge.deadline)) {
      return false;
    }
    if (edge.mask_artifacts < 0 || edge.mask_artifacts >= (1 << count_artifacts)) {
      return false;
    }
  }

  for (const auto& artifact : request_->artifacts) {
    const auto [from, to] = artifact.time_available;
    if (!is_node(artifact.node)) {
      return false;
    }
    if ((from != -kInf && !is_time(from)) || (to != kInf && !is_time(to)) || from > to) {
      return false;
    }
  }

  return true;
}

auto MissionSolver::CountArtifacts() const -> int {
  return request_->artifacts.size();
}

auto MissionSolver::TransformFromMask(int mask) const -> std::pair<int, int> {
  const int C = 1 << CountArtifacts();
  int number_node = mask / C;
  int mask_artifacts = mask % C;
  return {number_node, mask_artifacts};
}

auto MissionSolver::TransformToMask(int number_node, int mask_artifacts) const -> int {
  return number_node * (1 << CountArtifacts()) + mask_artifacts;
}

auto MissionSolver::Dijkstra(const std::vector<std::vector<Edge2>>& graph, int start_state, int start_time) const
    -> std::vector<int> {
  std::vector<int> dist(graph.size(), kInf);
  dist[start_state] = start_time;
  std::priority_queue<std::pair<int, int>> q;
  q.push({-dist[start_state], start_state});

  while (!q.empty()) {
    auto [old_dist, v] = q.top();
    q.pop();
    old_dist *= -1;
    if (dist[v] != old_dist) {
      continue;
    }

    for (const auto& edge : graph[v]) {
      int u = edge.to;

      if (dist[v] + edge.time > edge.time_available.second) {
        continue;
      }

      int time_after_transition = std::max(dist[v] + edge.time, edge.time_available.first);
      if (dist[u] > time_after_transition) {
        dist[u] = time_after_transition;
        q.push({-dist[u], u});
      }
    }
  }

  return dist;
}

auto MissionSolver::CalculatePossibleStartTimes(const std::vector<std::vector<Edge2>>& graph) const
    -> std::vector<int> {

  std::vector<std::vector<Edge2>> reverse_graph(graph.size());
  for (size_t i = 0; i < graph.size(); i++) {
    for (const auto& edge : graph[i]) {
      int first = edge.time_available.second == kInf ? -kInf : edge.time - edge.time_available.second;
      int second = edge.time_available.first == -kInf ? kInf : edge.time - edge.time_available.first;
      std::pair<int, int> new_available = {first, second};

      reverse_graph[edge.to].push_back({static_cast<int>(i), edge.time, new_available});
    }
  }

  std::unordered_set<int> start_times;

  for (size_t i = 0; i < graph.size(); i++) {
    for (auto& edge : graph[i]) {
      if (edge.time_available.second == kInf) {
        continue;
      }
      auto dist = Dijkstra(reverse_graph, i, edge.time - edge.time_available.second);

      int res = dist[TransformToMask(request_->start_node, 0)];
      if (res > 0 && res != kInf) {
        start_times.insert(-res);
      }
    }
  }

  for (int finish_node : request_->finish_nodes) {
    for (int mask = 0; mask < (1 << CountArtifacts()); ++mask) {
      int finish_state = TransformToMask(finish_node, mask);
      auto dist = Dijkstra(reverse_graph, finish_state, -request_->t_lim);

      int res = dist[TransformToMask(request_->start_node, 0)];
      if (res > 0 && res != kInf) {
        start_times.insert(-res);
      }
    }
  }

  start_times.insert(0);
  return std::vector<int>(start_times.begin(), start_times.end());
}

void MissionSolver::RecalculateResponse(Response& response, const std::vector<int>& dist, int cur_start_time) const {
  int min_time = kInf;
  double max_profit = -kInf;
  std::unordered_set<int> finish_nodes(request_->finish_nodes.begin(), request_->finish_nodes.end());
  for (size_t i = 0; i < dist.size(); i++) {
    if (dist[i] == kInf) {
      continue;
    }
    int real_dist = dist[i] + ((cur_start_time < 0) ? -2 * cur_start_time : 0);
    if (dist[i] > request_->t_lim) {
      continue;
    }
    auto [node, mask_artifacts] = TransformFromMask(i);
    if (!finish_nodes.contains(node)) {
      continue;
    }
    min_time = std::min(min_time, real_dist);

    int cost = 0;
    for (int artifact = 0; artifact < static_cast<int>(request_->artifacts.size()); ++artifact) {
      if (mask_artifacts & (1 << artifact)) {
        cost += request_->artifacts[artifact].cost;
      }
    }
    max_profit = std::max(max_profit, cost - real_dist / request_->t_cost);
  }

  if (min_time != kInf) {
    response.is_feasible = true;
    response.min_time = std::min(response.min_time, min_time);
    response.max_profit = std::max(response.max_profit, max_profit);
  }
}

auto MissionSolver::CheckMission() const -> Response {
  if (request_ == nullptr) {
    throw std::runtime_error("Request is nullptr");
  }

  std::vector<std::vector<Edge2>> graph(request_->count_nodes * (1 << CountArtifacts()));
  for (int state = 0; state < (1 << CountArtifacts()); ++state) {
    for (const auto& edge : request_->edges) {
      if (!IsSubmask(edge.mask_artifacts, state)) {
        continue;
      }
      graph[TransformToMask(edge.from, state)].push_back({
          TransformToMask(edge.to, state),
          edge.time,
          {-kInf, edge.deadline},
      });
    }
    for (size_t i = 0; i < request_->artifacts.size(); i++) {
      const auto& artifact = request_->artifacts[i];
      if (IsSubmask(1 << i, state)) {
        continue;
      }
      graph[TransformToMask(artifact.node, state)].push_back({
          TransformToMask(artifact.node, state | (1 << i)),
          0,
          artifact.time_available,
      });
    }
  }

  Response response;
  for (const auto& start_time : CalculatePossibleStartTimes(graph)) {
    auto dist = Dijkstra(graph, TransformToMask(request_->start_node, 0), start_time);

    RecalculateResponse(response, dist, start_time);
  }

  return response;
}
