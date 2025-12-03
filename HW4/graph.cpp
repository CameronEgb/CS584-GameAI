#include "graph.h"
#include <random>
#include <cmath>

Graph::Graph(int n, bool spatial) : numVertices(n), adj(n) {
    if (spatial) positions.resize(n);
}

void Graph::addEdge(int u, int v, float w) {
    adj[u].emplace_back(v, w);
}

void Graph::generateRandomLarge(int n, int avgDegree) {
    numVertices = n;
    adj.resize(n);
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis(0.f, 1.f);
    float p = static_cast<float>(avgDegree) / (n - 1); // Erdos-Renyi for avg degree
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (u != v && dis(gen) < p) {
                float w = dis(gen) * 100.f + 1.f; // positive >0
                addEdge(u, v, w);
                addEdge(v, u, w); // undirected for simplicity
            }
        }
    }
}

Graph createSmallCampusGraph() {
    Graph g(40, true); // spatial
    // Positions (scaled 0-1000)
    g.positions = {
        {300, 400}, {500, 600}, {450, 550}, {550, 500}, {600, 450},
        {700, 700}, {750, 750}, {650, 650}, {400, 500}, {550, 450},
        {250, 350}, {200, 300}, {600, 600}, {700, 650}, {450, 550},
        {500, 650}, {650, 550}, {500, 500}, {550, 600}, {720, 720},
        {400, 500}, {600, 700}, {450, 600}, {650, 700}, {350, 500},
        {500, 400}, {550, 450}, {750, 700}, {700, 680}, {750, 750},
        {620, 620}, {480, 580}, {580, 480}, {600, 580}, {620, 560},
        {740, 720}, {520, 420}, {680, 620}, {520, 520}, {580, 460}
    };

    auto dist = [&](int i, int j) {
        auto d = g.positions[i] - g.positions[j];
        return std::sqrt(d.x * d.x + d.y * d.y) + 1.f; // >0
    };

    // Initial edges
    std::vector<std::pair<int, int>> edgePairs = {
        {0,1}, {1,0}, {1,2}, {2,1}, {2,3}, {3,2}, {1,4}, {4,3},
        {5,6}, {6,5}, {6,7}, {7,6}, {1,8}, {8,9}, {0,11}, {11,10},
        {6,29}, {29,30}, {1,14}, {14,15}, {16,17}, {17,18}, {5,19},
        {20,21}, {21,22}, {23,24}, {25,26}, {27,28}, {31,32}, {33,34},
        {35,36}, {36,37}, {37,38}, {38,39}, {4,39}
    };
    for (auto [u,v] : edgePairs) {
        float w = dist(u, v);
        g.addEdge(u, v, w);
    }

    // Add more edges: connect if dist < 200, due to walking accessibile connections, to make ~120 total
    for (int u = 0; u < 40; ++u) {
        for (int v = u+1; v < 40; ++v) {
            float d = dist(u, v);
            if (d < 200.f) {
                g.addEdge(u, v, d);
                g.addEdge(v, u, d); // bidirectional
            }
        }
    }

    return g;
}