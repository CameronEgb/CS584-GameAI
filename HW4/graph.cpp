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
    // Window: 1200x900
    // Arena: 1160x860 (20px padding on all sides)
    // Bounds: X[20, 1180], Y[20, 880]
    
    const int W = 1200;
    const int H = 900;
    const float CELL_SIZE = 30.f; 
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    walls.clear();

    float startX = 20.f;
    float startY = 20.f;
    float roomW = 1160.f;
    float roomH = 860.f;
    float midX = 600.f; // Center of screen
    float midY = 450.f; // Center of screen
    float thick = 20.f;

    // --- 1. Outer Arena Walls ---
    walls.emplace_back(sf::FloatRect({startX - thick, startY - thick}, {roomW + 2*thick, thick})); // Top
    walls.emplace_back(sf::FloatRect({startX - thick, startY + roomH}, {roomW + 2*thick, thick})); // Bottom
    walls.emplace_back(sf::FloatRect({startX - thick, startY}, {thick, roomH})); // Left
    walls.emplace_back(sf::FloatRect({startX + roomW, startY}, {thick, roomH})); // Right

    // --- 2. Inner Cross Walls (Aligned to Center) ---
    // Vertical Split
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY}, {thick, 300.f})); // Top segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY + 450.f}, {thick, 410.f})); // Bottom segment

    // Horizontal Split
    walls.emplace_back(sf::FloatRect({startX, midY - thick/2}, {400.f, thick})); // Left segment
    walls.emplace_back(sf::FloatRect({startX + 550.f, midY - thick/2}, {610.f, thick})); // Right segment

    // --- 3. Obstacles ---
    walls.emplace_back(sf::FloatRect({150.f, 150.f}, {100.f, 100.f})); // TL
    walls.emplace_back(sf::FloatRect({900.f, 150.f}, {50.f, 300.f}));  // TR
    walls.emplace_back(sf::FloatRect({200.f, 700.f}, {300.f, 50.f}));  // BL

    // --- 4. Build Navmesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // Buffer to keep nodes away from walls
            sf::FloatRect bufferRect(
                {pos.x - 20.f, pos.y - 20.f}, 
                {40.f, 40.f}
            );

            bool blocked = false;
            for (const auto& w : walls) {
                if (w.findIntersection(bufferRect)) {
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

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            int u = g.gridMap[y * COLS + x];
            if (u == -1) continue;

            int dirs[8][2] = {{0,1}, {0,-1}, {1,0}, {-1,0}, {1,1}, {1,-1}, {-1,1}, {-1,-1}};

            for (auto& d : dirs) {
                int nx = x + d[0];
                int ny = y + d[1];

                if (nx >= 0 && nx < COLS && ny >= 0 && ny < ROWS) {
                    int v = g.gridMap[ny * COLS + nx];
                    if (v != -1) {
                        bool isDiag = (d[0] != 0 && d[1] != 0);
                        if (isDiag) {
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