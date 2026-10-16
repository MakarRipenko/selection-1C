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
  std::pair<int, int> time_available = {-kInf, -kInf};
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
  std::pair<int, int> time_available = {-kInf, -kInf};
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

auto Dijkstra(const std::vector<std::vector<Edge2>>& graph, int start_time, const State& start_graph)
    -> std::pair<std::vector<int>, int> {
  int count_artifacts = start_graph.artifacts.size();
  std::vector<int> dist(graph.size(), kInf);
  int min_delta = kInf;
  int start_state = TransformToMask(start_graph.start_node, 0, count_artifacts);
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
        min_delta = std::min(min_delta, dist[v] + edge.time - edge.time_available.second);
        continue;
      }
      if (dist[v] + edge.time > start_graph.t_lim) {
        min_delta = std::min(min_delta, dist[v] + edge.time - start_graph.t_lim);
        continue;
      }

      int time_after_transition = std::max(dist[v] + edge.time, edge.time_available.first);
      if (dist[u] > time_after_transition) {
        dist[u] = time_after_transition;
        q.push({-dist[u], u});
      }
    }
  }

  for (auto& d : dist) {
    if (d > start_graph.t_lim) {
      d = kInf;
    }
  }

  return {dist, min_delta};
}

void RecalculateResponse(Response& response, const std::vector<int>& dist, int cur_start_time,
                         const State& start_graph) {
  int min_time = kInf;
  double max_profit = -kInf;
  std::unordered_set<int> finish_nodes(start_graph.finish_nodes.begin(), start_graph.finish_nodes.end());
  for (int i = 0; i < dist.size(); i++) {
    int real_dist = dist[i] + ((cur_start_time < 0) ? -cur_start_time : 0);
    auto [node, mask_artifacts] = TransformFromMask(i, start_graph.artifacts.size());
    if (!finish_nodes.contains(node) || dist[i] == kInf) {
      continue;
    }
    min_time = std::min(min_time, dist[i]);

    int cost = 0;
    for (int artifact = 0; artifact < start_graph.artifacts.size(); ++artifact) {
      if (mask_artifacts & (1 << artifact)) {
        cost += start_graph.artifacts[artifact - 1].cost;
      }
    }
    max_profit = std::max(max_profit, cost - 1.0 * dist[i] / start_graph.t_cost);
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

  int cur_start_time = 0;
  int cur_delta = 0;
  while (cur_delta != kInf && -cur_start_time <= start_graph.t_lim) {
    auto [dist, delta] = Dijkstra(graph, cur_start_time, start_graph);
    cur_start_time -= delta;
    cur_delta = delta;

    RecalculateResponse(response, dist, cur_start_time, start_graph);
  }

  return response;
}

int main() {
}
