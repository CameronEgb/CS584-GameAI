#pragma once
#include <vector>
#include <SFML/Graphics.hpp>

struct Edge {
    int to;
    float weight;
    Edge(int t, float w) : to(t), weight(w) {}
};

struct Graph {
    int numVertices;
    std::vector<std::vector<Edge>> adj;
    std::vector<sf::Vector2f> positions;

    // Helper to map grid coordinates to node ID (-1 if invalid/wall)
    std::vector<int> gridMap; 
    int cols, rows;

    Graph(int n = 0, bool spatial = false);
    void addEdge(int u, int v, float w);
    int getNodeAt(float x, float y, float cellSize);
};

// Returns the graph and fills the obstacles vector for rendering
Graph createFourRoomGraph(std::vector<sf::FloatRect>& walls);