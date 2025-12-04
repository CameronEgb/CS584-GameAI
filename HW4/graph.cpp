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
    const float CELL_SIZE = 30.f; // Granularity of the navmesh
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    // Define Environment Geometry
    walls.clear();

    // 1. Cross Walls (The 4 Rooms)
    // Vertical split
    // SFML 3.0: Rect constructor takes (Position, Size) as vectors
    walls.emplace_back(sf::FloatRect({585.f, 0.f}, {30.f, 900.f})); 
    // Horizontal split
    walls.emplace_back(sf::FloatRect({0.f, 435.f}, {1200.f, 30.f}));

    // 2. The 3 Obstacles
    walls.emplace_back(sf::FloatRect({200.f, 200.f}, {100.f, 100.f})); // Top-Left room obstacle
    walls.emplace_back(sf::FloatRect({900.f, 200.f}, {60.f, 200.f}));  // Top-Right room obstacle
    walls.emplace_back(sf::FloatRect({250.f, 650.f}, {150.f, 50.f}));  // Bottom-Left room obstacle

    // Define "Doorways" (negative obstacles, or simply areas we allow)
    std::vector<sf::FloatRect> doorways;
    doorways.emplace_back(sf::FloatRect({585.f, 200.f}, {30.f, 100.f})); // Gap in vertical wall (top)
    doorways.emplace_back(sf::FloatRect({585.f, 600.f}, {30.f, 100.f})); // Gap in vertical wall (bottom)
    doorways.emplace_back(sf::FloatRect({200.f, 435.f}, {100.f, 30.f})); // Gap in horizontal wall (left)
    doorways.emplace_back(sf::FloatRect({900.f, 435.f}, {100.f, 30.f})); // Gap in horizontal wall (right)

    // Build the Grid Graph
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    // First pass: Create Nodes
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // SFML 3.0: Rect constructor takes (Position, Size)
            sf::FloatRect cellRect(
                {x * CELL_SIZE + 2.f, y * CELL_SIZE + 2.f}, 
                {CELL_SIZE - 4.f, CELL_SIZE - 4.f}
            );

            bool isWall = false;
            for (const auto& w : walls) {
                if (w.findIntersection(cellRect)) {
                    // Check if it's actually a doorway
                    bool isDoor = false;
                    for (const auto& d : doorways) {
                        if (d.findIntersection(cellRect)) {
                            isDoor = true;
                            break;
                        }
                    }
                    if (!isDoor) {
                        isWall = true;
                        break;
                    }
                }
            }

            if (!isWall) {
                g.gridMap[y * COLS + x] = nodeCounter++;
                g.positions.push_back(pos);
            }
        }
    }

    g.numVertices = nodeCounter;
    g.adj.resize(nodeCounter);

    // Second pass: Connect Neighbors
    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            int u = g.gridMap[y * COLS + x];
            if (u == -1) continue;

            // Check 4 neighbors
            int neighbors[4][2] = {{0,1}, {0,-1}, {1,0}, {-1,0}};
            for (auto& nb : neighbors) {
                int nx = x + nb[0];
                int ny = y + nb[1];

                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
                    int v = g.gridMap[ny * COLS + nx];
                    if (v != -1) {
                        float dist = CELL_SIZE; // Orthogonal
                        g.addEdge(u, v, dist);
                    }
                }
            }
            
            // Optional: Diagonals
            int diag[4][2] = {{1,1}, {1,-1}, {-1,1}, {-1,-1}};
             for (auto& nb : diag) {
                int nx = x + nb[0];
                int ny = y + nb[1];

                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
                    int v = g.gridMap[ny * COLS + nx];
                    // Check orthogonal blockers to prevent cutting corners
                    int v1 = g.gridMap[y * COLS + nx];
                    int v2 = g.gridMap[ny * COLS + x];
                    
                    if (v != -1 && v1 != -1 && v2 != -1) {
                        float dist = CELL_SIZE * 1.414f;
                        g.addEdge(u, v, dist);
                    }
                }
            }
        }
    }

    return g;
}