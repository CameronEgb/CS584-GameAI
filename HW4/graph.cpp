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
    // PREVIOUS Arena: 1160x860
    // NEW Arena (Scaled 0.5): 580x430
    
    const int W = 1200;
    const int H = 900;
    const float CELL_SIZE = 20.f; // Reduced cell size for smaller map
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    walls.clear();

    // Center calculations
    float roomW = 580.f;
    float roomH = 430.f;
    float startX = (W - roomW) / 2.f; // 310
    float startY = (H - roomH) / 2.f; // 235
    float midX = W / 2.f; // 600
    float midY = H / 2.f; // 450
    float thick = 20.f;

    // --- 1. Outer Arena Walls ---
    walls.emplace_back(sf::FloatRect({startX - thick, startY - thick}, {roomW + 2*thick, thick})); // Top
    walls.emplace_back(sf::FloatRect({startX - thick, startY + roomH}, {roomW + 2*thick, thick})); // Bottom
    walls.emplace_back(sf::FloatRect({startX - thick, startY}, {thick, roomH})); // Left
    walls.emplace_back(sf::FloatRect({startX + roomW, startY}, {thick, roomH})); // Right

    // --- 2. Inner Cross Walls (Scaled lengths) ---
    // Vertical Split (x = 600)
    // Top Segment: Length 150 (was 300)
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY}, {thick, 150.f}));
    // Bottom Segment: Length 205 (was 410) -> Start at 450 (midY)
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY + 225.f}, {thick, 205.f}));

    // Horizontal Split (y = 450)
    // Left Segment: Length 200 (was 400)
    walls.emplace_back(sf::FloatRect({startX, midY - thick/2}, {200.f, thick}));
    // Right Segment: Length 305 (was 610) -> Start at 600+75 = 675?
    // Gap needs to be ~75px. 
    // StartX + 200 + 75 = 310 + 275 = 585. 
    // Let's place it from (midX + 37.5) to end.
    walls.emplace_back(sf::FloatRect({midX + 37.5f, midY - thick/2}, {roomW - (midX - startX) - 37.5f, thick}));

    // --- 3. Obstacles (Scaled 0.5) ---
    // TL: (375, 300) size (50, 50)
    walls.emplace_back(sf::FloatRect({startX + 65.f, startY + 65.f}, {50.f, 50.f})); 
    // TR: (750, 300) size (25, 150)
    walls.emplace_back(sf::FloatRect({startX + 440.f, startY + 65.f}, {25.f, 150.f})); 
    // BL: (400, 575) size (150, 25)
    walls.emplace_back(sf::FloatRect({startX + 90.f, startY + 340.f}, {150.f, 25.f})); 

    // --- 4. Build Navmesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // Buffer to keep nodes away from walls (scaled down buffer too)
            sf::FloatRect bufferRect(
                {pos.x - 12.f, pos.y - 12.f}, 
                {24.f, 24.f}
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