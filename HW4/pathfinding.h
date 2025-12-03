#pragma once
#include "graph.h"
#include <vector>
#include <functional>
#include <chrono>

struct Metrics {
    double runtime_ms = 0.0;
    int max_fringe = 0;
    int fill = 0;
};

using Heuristic = std::function<float(int, int, const Graph&)>;

// Standard Heuristics
float euclideanHeur(int u, int v, const Graph& g);
float clusterHeur(int u, int v, const Graph& g);
void initClusters(const Graph& g, int numClusters);

// Algorithms
std::vector<int> dijkstra(const Graph& g, int start, int goal, Metrics& m);
std::vector<int> aStar(const Graph& g, int start, int goal, Heuristic h, Metrics& m);