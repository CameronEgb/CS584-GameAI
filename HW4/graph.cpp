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
    // Window is 1200x900. 
    // We will define an ARENA of 800x600 centered in the screen.
    // Arena Bounds: X[200, 1000], Y[150, 750]
    
    const int W = 1200;
    const int H = 900;
    const float CELL_SIZE = 25.f; // Smaller cells for finer navmesh
    const int COLS = W / (int)CELL_SIZE;
    const int ROWS = H / (int)CELL_SIZE;

    walls.clear();

    // Offsets to center the map
    float startX = 200.f;
    float startY = 150.f;
    float roomW = 800.f;
    float roomH = 600.f;
    float midX = startX + roomW / 2.f; // 600
    float midY = startY + roomH / 2.f; // 450
    float thick = 20.f;

    // --- 1. Outer Arena Walls (So agent can't leave) ---
    walls.emplace_back(sf::FloatRect({startX - thick, startY - thick}, {roomW + 2*thick, thick})); // Top
    walls.emplace_back(sf::FloatRect({startX - thick, startY + roomH}, {roomW + 2*thick, thick})); // Bottom
    walls.emplace_back(sf::FloatRect({startX - thick, startY}, {thick, roomH})); // Left
    walls.emplace_back(sf::FloatRect({startX + roomW, startY}, {thick, roomH})); // Right

    // --- 2. Inner Cross Walls (The 4 Rooms) ---
    // Vertical Split (Gap in middle-ish)
    // Top segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY}, {thick, 200.f}));
    // Bottom segment (Gap is 100px tall)
    walls.emplace_back(sf::FloatRect({midX - thick/2, startY + 300.f}, {thick, 300.f}));

    // Horizontal Split
    // Left segment
    walls.emplace_back(sf::FloatRect({startX, midY - thick/2}, {300.f, thick}));
    // Right segment (Gap is 100px wide)
    walls.emplace_back(sf::FloatRect({startX + 400.f, midY - thick/2}, {400.f, thick}));

    // --- 3. Obstacles (Scaled down) ---
    walls.emplace_back(sf::FloatRect({300.f, 250.f}, {60.f, 60.f})); // Top-Left
    walls.emplace_back(sf::FloatRect({800.f, 250.f}, {40.f, 150.f})); // Top-Right
    walls.emplace_back(sf::FloatRect({350.f, 650.f}, {150.f, 30.f})); // Bottom-Left

    // --- 4. Build Navigation Mesh ---
    Graph g(0, true);
    g.cols = COLS;
    g.rows = ROWS;
    g.gridMap.resize(COLS * ROWS, -1);

    int nodeCounter = 0;

    for (int y = 0; y < ROWS; ++y) {
        for (int x = 0; x < COLS; ++x) {
            sf::Vector2f pos(x * CELL_SIZE + CELL_SIZE/2.f, y * CELL_SIZE + CELL_SIZE/2.f);
            
            // PADDING CHECK: Check a larger box than the cell to keep nodes away from walls
            // Cell is 25x25, we check 35x35 to force a buffer
            sf::FloatRect bufferRect(
                {pos.x - 18.f, pos.y - 18.f}, 
                {36.f, 36.f}
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
                        // Prevent cutting corners
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