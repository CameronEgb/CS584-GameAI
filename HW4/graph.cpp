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
    float p = static_cast<float>(avgDegree) / (n - 1); 
    for (int u = 0; u < n; ++u) {
        for (int v = 0; v < n; ++v) {
            if (u != v && dis(gen) < p) {
                float w = dis(gen) * 100.f + 1.f; 
                addEdge(u, v, w);
                addEdge(v, u, w); 
            }
        }
    }
}

Graph createSmallCampusGraph() {
    Graph g(40, true);
    // Simple grid-like positions for testing
    int cols = 8;
    for(int i=0; i<40; ++i) {
        g.positions[i] = sf::Vector2f((i % cols) * 100.f + 50.f, (i / cols) * 100.f + 50.f);
    }
    // Connect neighbors
    for(int i=0; i<40; ++i) {
        if((i+1)%cols != 0 && i+1 < 40) { // Right
            float d = std::hypot(g.positions[i].x - g.positions[i+1].x, g.positions[i].y - g.positions[i+1].y);
            g.addEdge(i, i+1, d); g.addEdge(i+1, i, d);
        }
        if(i+cols < 40) { // Down
            float d = std::hypot(g.positions[i].x - g.positions[i+cols].x, g.positions[i].y - g.positions[i+cols].y);
            g.addEdge(i, i+cols, d); g.addEdge(i+cols, i, d);
        }
    }
    return g;
}