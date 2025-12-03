#include "pathfinding.h"
#include <queue>
#include <set>
#include <limits>
#include <iostream>
#include <random> // Added
#include <cmath> // Added for std::sqrt, std::abs

using pii = std::pair<float, int>; // dist, node

std::vector<int> dijkstra(const Graph& g, int start, int goal, Metrics& m) {
    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<float> dist(g.numVertices, std::numeric_limits<float>::infinity());
    std::vector<int> prev(g.numVertices, -1);
    dist[start] = 0;
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq;
    pq.push({0, start});
    std::set<int> visited;
    m.max_fringe = 1;
    m.fill = 0;

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (visited.count(u)) continue;
        visited.insert(u);
        m.fill++;
        if (u == goal) break;

        for (auto [v, w] : g.adj[u]) {
            if (d + w < dist[v]) {
                dist[v] = d + w;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
        m.max_fringe = std::max(m.max_fringe, (int)pq.size());
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    m.runtime_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    // Reconstruct path
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return (path.front() == start) ? path : std::vector<int>{};
}

std::vector<int> aStar(const Graph& g, int start, int goal, Heuristic h, Metrics& m) {
    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<float> gscore(g.numVertices, std::numeric_limits<float>::infinity());
    std::vector<float> fscore(g.numVertices, std::numeric_limits<float>::infinity());
    std::vector<int> prev(g.numVertices, -1);
    gscore[start] = 0;
    fscore[start] = h(start, goal, g);
    std::priority_queue<pii, std::vector<pii>, std::greater<pii>> pq; // f, node
    pq.push({fscore[start], start});
    std::set<int> visited;
    m.max_fringe = 1;
    m.fill = 0;

    while (!pq.empty()) {
        auto [f, u] = pq.top(); pq.pop();
        if (visited.count(u)) continue;
        visited.insert(u);
        m.fill++;
        if (u == goal) break;

        for (auto [v, w] : g.adj[u]) {
            float tent_g = gscore[u] + w;
            if (tent_g < gscore[v]) {
                prev[v] = u;
                gscore[v] = tent_g;
                fscore[v] = tent_g + h(v, goal, g);
                pq.push({fscore[v], v});
            }
        }
        m.max_fringe = std::max(m.max_fringe, (int)pq.size());
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    m.runtime_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());
    return (path.front() == start) ? path : std::vector<int>{};
}

float euclideanHeur(int u, int v, const Graph& g) {
    auto d = g.positions[u] - g.positions[v];
    return std::sqrt(d.x * d.x + d.y * d.y);
}

float overEstHeur(int u, int v, const Graph& g) {
    return 2.f * euclideanHeur(u, v, g); // inadmissible
}

// Clusters for large (simple: ID / (n/clusters))
std::vector<int> cluster; // global for simplicity
std::vector<float> clusterCenter; // avg ID or something
void initClusters(const Graph& g, int numClusters) {
    cluster.resize(g.numVertices);
    for (int i = 0; i < g.numVertices; ++i) cluster[i] = i % numClusters;
    clusterCenter.resize(numClusters, 0.f);
    // Fake centers: cluster ID * (g.numVertices / numClusters)
    for (int c = 0; c < numClusters; ++c) clusterCenter[c] = c * 10.f; // arbitrary
}

float clusterHeur(int u, int v, const Graph& g) {
    (void)g; // Suppress unused warning
    int cu = cluster[u], cv = cluster[v];
    if (cu == cv) return 0.f;
    return std::abs(clusterCenter[cu] - clusterCenter[cv]); // reasonable dist
}

void analyzeHeur(const Graph& g, Heuristic h, bool admissible) {
    int overCount = 0;
    float totalOver = 0.f;
    int samples = 100;
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<int> dis(0, g.numVertices - 1);
    for (int i = 0; i < samples; ++i) {
        int s = dis(gen), goal = dis(gen);
        Metrics m;
        auto path = dijkstra(g, s, goal, m); // true dist
        if (path.empty()) continue;
        float trueDist = 0.f;
        for (size_t j = 1; j < path.size(); ++j) {
            for (auto [v, w] : g.adj[path[j-1]]) if (v == path[j]) trueDist += w;
        }
        float heurVal = h(s, goal, g);
        if (heurVal > trueDist) {
            overCount++;
            totalOver += heurVal - trueDist;
        }
    }
    std::cout << "Heuristic " << (admissible ? "admissible" : "inadmissible") << ": Overestimates " << overCount << "/" << samples << " times, avg over " << (overCount ? totalOver / overCount : 0.f) << std::endl;
}