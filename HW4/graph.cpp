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
    const int W = 1200;
    const int H = 900;
    const float CELL_SIZE = 25.f; 
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    walls.clear();

    float thick = 20.f;
    float midX = W / 2.f; // 600
    float midY = H / 2.f; // 450

    // --- 1. Boundary Walls (Edges of Screen) ---
    // Top
    walls.emplace_back(sf::FloatRect({0.f, 0.f}, {1200.f, thick}));
    // Bottom
    walls.emplace_back(sf::FloatRect({0.f, 900.f - thick}, {1200.f, thick}));
    // Left
    walls.emplace_back(sf::FloatRect({0.f, thick}, {thick, 900.f - 2*thick}));
    // Right
    walls.emplace_back(sf::FloatRect({1200.f - thick, thick}, {thick, 900.f - 2*thick}));

    // --- 2. Room Dividers (Cross with Doorways) ---
    
    // Vertical Wall (x=590)
    // Top segment (0 to 350) -> Gap (350-550) -> Bottom segment (550 to 900)
    walls.emplace_back(sf::FloatRect({midX - thick/2, 0.f}, {thick, 350.f}));
    walls.emplace_back(sf::FloatRect({midX - thick/2, 550.f}, {thick, 350.f}));

    // Horizontal Wall (y=440)
    // Left segment (0 to 500) -> Gap (500-700) -> Right segment (700 to 1200)
    walls.emplace_back(sf::FloatRect({0.f, midY - thick/2}, {500.f, thick}));
    walls.emplace_back(sf::FloatRect({700.f, midY - thick/2}, {500.f, thick}));

    // --- 3. Obstacles (1 per room) ---
    float obsSize = 80.f;
    
    // Room 1 (Top-Left): Center roughly (300, 225)
    walls.emplace_back(sf::FloatRect({300.f - obsSize/2, 225.f - obsSize/2}, {obsSize, obsSize}));

    // Room 2 (Top-Right): Center roughly (900, 225)
    walls.emplace_back(sf::FloatRect({900.f - obsSize/2, 225.f - obsSize/2}, {obsSize, obsSize}));

    // Room 3 (Bottom-Left): Center roughly (300, 675)
    walls.emplace_back(sf::FloatRect({300.f - obsSize/2, 675.f - obsSize/2}, {obsSize, obsSize}));

    // Room 4 (Bottom-Right): Center roughly (900, 675)
    walls.emplace_back(sf::FloatRect({900.f - obsSize/2, 675.f - obsSize/2}, {obsSize, obsSize}));


    // --- 4. Build Navigation Mesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // Check collision with slightly inflated rect to buffer nodes from walls
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

    // 8-way connectivity
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
                        // Prevent cutting corners through walls
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