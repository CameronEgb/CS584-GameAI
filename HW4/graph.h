#pragma once
#include <vector>
#include <SFML/System/Vector2.hpp>

struct Edge {
    int to;
    float weight;
    Edge(int t, float w) : to(t), weight(w) {}
};

struct Graph {
    int numVertices;
    std::vector<std::vector<Edge>> adj;
    std::vector<sf::Vector2f> positions;

    Graph(int n, bool spatial = false);
    void addEdge(int u, int v, float w);
    void generateRandomLarge(int n, int avgDegree);
};

// Global factory function
Graph createSmallCampusGraph();