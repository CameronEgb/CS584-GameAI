#include "graph.h"
#include "steering.h" // Needed for WINDOW_WIDTH / WINDOW_HEIGHT
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
    const float W = static_cast<float>(WINDOW_WIDTH);
    const float H = static_cast<float>(WINDOW_HEIGHT);
    
    // Use a slightly smaller cell size to navigate tighter spaces if needed
    const float CELL_SIZE = 20.f; 
    const int COLS = WINDOW_WIDTH / (int)CELL_SIZE;
    const int ROWS = WINDOW_HEIGHT / (int)CELL_SIZE;

    walls.clear();

    // Define a padding so walls are VISIBLE and INSIDE the window
    float padding = 40.f; 
    float thick = 20.f;

    // The playable area is now [padding, W-padding] x [padding, H-padding]
    float left = padding;
    float right = W - padding;
    float top = padding;
    float bottom = H - padding;

    float midX = W / 2.f;
    float midY = H / 2.f;

    // --- 1. Boundary Walls (Inset from screen edges) ---
    // Top Wall
    walls.emplace_back(sf::FloatRect({left, top}, {right - left, thick}));
    // Bottom Wall
    walls.emplace_back(sf::FloatRect({left, bottom - thick}, {right - left, thick}));
    // Left Wall
    walls.emplace_back(sf::FloatRect({left, top}, {thick, bottom - top}));
    // Right Wall
    walls.emplace_back(sf::FloatRect({right - thick, top}, {thick, bottom - top}));

    // --- 2. Room Dividers (Cross with Doorways) ---
    float doorSize = 100.f; // Gap size

    // Vertical Divider (Mid X)
    // Top Segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, top}, {thick, (midY - doorSize/2) - top}));
    // Bottom Segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, midY + doorSize/2}, {thick, bottom - (midY + doorSize/2)}));

    // Horizontal Divider (Mid Y)
    // Left Segment
    walls.emplace_back(sf::FloatRect({left, midY - thick/2}, {(midX - doorSize/2) - left, thick}));
    // Right Segment
    walls.emplace_back(sf::FloatRect({midX + doorSize/2, midY - thick/2}, {right - (midX + doorSize/2), thick}));

    // --- 3. Obstacles (Centered in quadrants) ---
    float obsSize = 50.f;
    float qW = (right - left) / 2.f;
    float qH = (bottom - top) / 2.f;

    // Calculate centers of the 4 rooms
    float c1x = left + qW/2.f; float c1y = top + qH/2.f;
    float c2x = midX + qW/2.f; float c2y = top + qH/2.f;
    float c3x = left + qW/2.f; float c3y = midY + qH/2.f;
    float c4x = midX + qW/2.f; float c4y = midY + qH/2.f;

    walls.emplace_back(sf::FloatRect({c1x - obsSize/2, c1y - obsSize/2}, {obsSize, obsSize}));
    walls.emplace_back(sf::FloatRect({c2x - obsSize/2, c2y - obsSize/2}, {obsSize, obsSize}));
    walls.emplace_back(sf::FloatRect({c3x - obsSize/2, c3y - obsSize/2}, {obsSize, obsSize}));
    walls.emplace_back(sf::FloatRect({c4x - obsSize/2, c4y - obsSize/2}, {obsSize, obsSize}));

    // --- 4. Build Navigation Mesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // Check collision with inflated rect
            sf::FloatRect bufferRect(
                {pos.x - 10.f, pos.y - 10.f}, 
                {20.f, 20.f}
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

    // Connectivity
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