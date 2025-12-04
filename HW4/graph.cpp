#include "graph.h"
#include <cmath>
#include <iostream>

Graph::Graph(int n, bool spatial) : numVertices(n), adj(n), cols(0), rows(0) {
    if (spatial) positions.resize(n);
}

void Graph::addEdge(int u, int v, float w) {
    if (u >= 0 && u < numVertices && v >= 0 && v < numVertices) {
        adj[u].emplace_back(v, w);
    }
}

int Graph::getNodeAt(float x, float y, float cellSize) {
    int gx = static_cast<int>(x / cellSize);
    int gy = static_cast<int>(y / cellSize);
    if (gx < 0 || gx >= cols || gy < 0 || gy >= rows) return -1;
    return gridMap[gy * cols + gx];
}

Graph createFourRoomGraph(std::vector<sf::FloatRect>& walls) {
    // Configuration
    const int W = 1200;
    const int H = 900;
    const float CELL_SIZE = 30.f; 
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    walls.clear();

    // --- 1. Define Geometry (Explicit Rectangles) ---
    float midX = 600.f;
    float midY = 450.f;
    float thick = 40.f; // Thicker walls for better visibility

    // Vertical Wall (Top Segment)
    walls.emplace_back(sf::FloatRect({midX - thick/2, 0.f}, {thick, 150.f}));
    walls.emplace_back(sf::FloatRect({midX - thick/2, 250.f}, {thick, midY - 250.f}));

    // Vertical Wall (Bottom Segment)
    walls.emplace_back(sf::FloatRect({midX - thick/2, midY}, {thick, 200.f})); 
    walls.emplace_back(sf::FloatRect({midX - thick/2, 750.f}, {thick, 900.f - 750.f}));

    // Horizontal Wall (Left Segment)
    walls.emplace_back(sf::FloatRect({0.f, midY - thick/2}, {150.f, thick}));
    walls.emplace_back(sf::FloatRect({250.f, midY - thick/2}, {midX - 250.f, thick}));

    // Horizontal Wall (Right Segment)
    walls.emplace_back(sf::FloatRect({midX, midY - thick/2}, {350.f, thick})); 
    walls.emplace_back(sf::FloatRect({1050.f, midY - thick/2}, {1200.f - 1050.f, thick}));

    // --- 2. Obstacles ---
    walls.emplace_back(sf::FloatRect({100.f, 100.f}, {80.f, 80.f}));
    walls.emplace_back(sf::FloatRect({900.f, 100.f}, {40.f, 250.f}));
    walls.emplace_back(sf::FloatRect({150.f, 700.f}, {200.f, 40.f}));

    // --- 3. Build Navigation Mesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // Intersection test logic
            sf::FloatRect cellRect(
                {x * CELL_SIZE + 5.f, y * CELL_SIZE + 5.f}, 
                {CELL_SIZE - 10.f, CELL_SIZE - 10.f}
            );

            bool blocked = false;
            for (const auto& w : walls) {
                if (w.findIntersection(cellRect)) {
                    blocked = true;
                    break;
                }
            }

            if (!blocked) {
                g.gridMap[y * COLS + x] = nodeCounter++;
                g.positions.push_back(pos);
            }
        }
    }

    g.numVertices = nodeCounter;
    g.adj.resize(nodeCounter);

    // Connect Neighbors
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            int u = g.gridMap[y * COLS + x];
            if (u == -1) continue;

            int dirs[8][2] = {
                {0,1}, {0,-1}, {1,0}, {-1,0}, 
                {1,1}, {1,-1}, {-1,1}, {-1,-1} 
            };

            for (auto& d : dirs) {
                int nx = x + d[0];
                int ny = y + d[1];

                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
                    int v = g.gridMap[ny * COLS + nx];
                    if (v != -1) {
                        bool isDiag = (d[0] != 0 && d[1] != 0);
                        if (isDiag) { // Don't cut corners through walls
                            if (g.gridMap[y * COLS + nx] == -1 || g.gridMap[ny * COLS + x] == -1)
                                continue;
                        }
                        g.addEdge(u, v, isDiag ? CELL_SIZE * 1.414f : CELL_SIZE);
                    }
                }
            }
        }
    }
    return g;
}