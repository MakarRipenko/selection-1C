#include <iostream>
#include <queue>
#include <vector>
#include <unordered_set>

const int kInf = 1e9;

struct Edge {
  int from;
  int to;
  int time;
  int dedline = kInf;
  int mask_artifacts = 0;
};

struct Artifact {
  int id;
  int node;
  int cost;
  std::pair<int, int> time_available = {-kInf, kInf};
};

struct State {
  int count_nodes;
  int start_node;
  std::vector<Edge> edges;
  std::vector<Artifact> artifacts;
  std::vector<int> finish_nodes;
  int t_lim;
  int t_cost;
};

struct Response {
  bool is_feasible = false;
  int min_time = kInf;
  double max_profit = -kInf;
};

struct Edge2 {
  int to;
  int time;
  std::pair<int, int> time_available = {-kInf, kInf};
};

auto TransformFromMask(int mask, int count_artifacts) -> std::pair<int, int> {
  const int C = 1 << count_artifacts;
  int number_node = mask / C;
  int mask_artifacts = mask % C;
  return {number_node, mask_artifacts};
}

auto TransformToMask(int number_node, int mask_artifacts, int count_artifacts) -> int {
  return number_node * (1 << count_artifacts) + mask_artifacts;
}

auto IsSubmask(int mask1, int mask2) -> bool {
  return (mask1 & mask2) == mask1;
}

auto Dijkstra(const std::vector<std::vector<Edge2>>& graph, int start_state, int start_time, const State& start_graph)
    -> std::vector<int> {
  int count_artifacts = start_graph.artifacts.size();
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

auto CalculatePosibleStartTimes(const std::vector<std::vector<Edge2>>& graph, const State& start_graph)
    -> std::vector<int> {

  int count_artifacts = start_graph.artifacts.size();
  std::vector<std::vector<Edge2>> reverse_graph(graph.size());
  for (int i = 0; i < graph.size(); i++) {
    for (const auto& edge : graph[i]) {
      int first = edge.time_available.second == kInf ? -kInf : edge.time - edge.time_available.second;
      int second = edge.time_available.first == -kInf ? kInf : edge.time - edge.time_available.first;
      std::pair<int, int> new_avalible = {first, second};

      reverse_graph[edge.to].push_back({i, edge.time, new_avalible});
    }
  }

  std::unordered_set<int> start_times;

  for (int i = 0; i < graph.size(); i++) {
    for (auto& edge : graph[i]) {
      if (edge.time_available.second == kInf) {
        continue;
      }
      auto dist = Dijkstra(reverse_graph, i, edge.time - edge.time_available.second, start_graph);

      int res = dist[TransformToMask(start_graph.start_node, 0, count_artifacts)];
      if (res > 0 && res != kInf) {
        start_times.insert(-res);
      }
    }
  }

  for (int finish_node : start_graph.finish_nodes) {
    for (int mask = 0; mask < (1 << count_artifacts); ++mask) {
      int finish_state = TransformToMask(finish_node, mask, count_artifacts);
      auto dist = Dijkstra(reverse_graph, finish_state, -start_graph.t_lim, start_graph);

      int res = dist[TransformToMask(start_graph.start_node, 0, count_artifacts)];
      if (res > 0 && res != kInf) {
        start_times.insert(-res);
      }
    }
  }

  start_times.insert(0);
  return std::vector<int>(start_times.begin(), start_times.end());
}

void RecalculateResponse(Response& response, const std::vector<int>& dist, int cur_start_time,
                         const State& start_graph) {
  int min_time = kInf;
  double max_profit = -kInf;
  std::unordered_set<int> finish_nodes(start_graph.finish_nodes.begin(), start_graph.finish_nodes.end());
  for (int i = 0; i < dist.size(); i++) {
    if (dist[i] == kInf) {
      continue;
    }
    int real_dist = dist[i] + ((cur_start_time < 0) ? -2 * cur_start_time : 0);
    if (dist[i] > start_graph.t_lim) {
      continue;
    }
    auto [node, mask_artifacts] = TransformFromMask(i, start_graph.artifacts.size());
    if (!finish_nodes.contains(node)) {
      continue;
    }
    min_time = std::min(min_time, real_dist);

    int cost = 0;
    for (int artifact = 0; artifact < start_graph.artifacts.size(); ++artifact) {
      if (mask_artifacts & (1 << artifact)) {
        cost += start_graph.artifacts[artifact].cost;
      }
    }
    max_profit = std::max(max_profit, cost - 1.0 * real_dist / start_graph.t_cost);
  }

  if (min_time != kInf) {
    response.is_feasible = true;
    response.min_time = std::min(response.min_time, min_time);
    response.max_profit = std::max(response.max_profit, max_profit);
  }
}

auto CheckMission(const State& start_graph) -> Response {
  int count_artifacts = start_graph.artifacts.size();
  std::vector<std::vector<Edge2>> graph(start_graph.count_nodes * (1 << count_artifacts));
  for (int state = 0; state < (1 << count_artifacts); ++state) {
    for (const auto& edge : start_graph.edges) {
      if (!IsSubmask(edge.mask_artifacts, state)) {
        continue;
      }
      graph[TransformToMask(edge.from, state, count_artifacts)].push_back({
          TransformToMask(edge.to, state, count_artifacts),
          edge.time,
          {-kInf, edge.dedline},
      });
    }
    for (const auto& artifact : start_graph.artifacts) {
      if (IsSubmask(1 << artifact.id, state)) {
        continue;
      }
      graph[TransformToMask(artifact.node, state, count_artifacts)].push_back({
          TransformToMask(artifact.node, state | (1 << artifact.id), count_artifacts),
          0,
          artifact.time_available,
      });
    }
  }

  Response response;
  for (const auto& start_time : CalculatePosibleStartTimes(graph, start_graph)) {
    auto dist = Dijkstra(graph, TransformToMask(start_graph.start_node, 0, count_artifacts), start_time, start_graph);

    RecalculateResponse(response, dist, start_time, start_graph);
  }

  return response;
}

int main() {
  State example{
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
              {0, 3, 2, {0, 10}},
              {1, 1, 2, {30, 40}},
              {2, 4, 8, {0, 80}},
          },
      .finish_nodes = {3, 4},
      .t_lim = 120,
      .t_cost = 10,
  };

  auto response = CheckMission(example);
  std::cout << response.is_feasible << " " << response.min_time << " " << response.max_profit << std::endl;
  return 0;
}
