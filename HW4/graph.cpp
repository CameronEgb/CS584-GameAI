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
    // Dynamic dimensions based on constants
    const float W = static_cast<float>(WINDOW_WIDTH);
    const float H = static_cast<float>(WINDOW_HEIGHT);
    
    const float CELL_SIZE = 20.f; 
    const int COLS = WINDOW_WIDTH / (int)CELL_SIZE;
    const int ROWS = WINDOW_HEIGHT / (int)CELL_SIZE;

    walls.clear();

    float thick = 20.f;
    float midX = W / 2.f;
    float midY = H / 2.f;

    // --- 1. Boundary Walls (Edges of Screen) ---
    walls.emplace_back(sf::FloatRect({0.f, 0.f}, {W, thick}));            // Top
    walls.emplace_back(sf::FloatRect({0.f, H - thick}, {W, thick}));      // Bottom
    walls.emplace_back(sf::FloatRect({0.f, thick}, {thick, H - 2*thick})); // Left
    walls.emplace_back(sf::FloatRect({W - thick, thick}, {thick, H - 2*thick})); // Right

    // --- 2. Room Dividers (Cross with Doorways) ---
    // Doorway size
    float doorSize = 100.f;

    // Vertical Wall (Center X)
    // Top segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, 0.f}, {thick, midY - doorSize/2}));
    // Bottom segment
    walls.emplace_back(sf::FloatRect({midX - thick/2, midY + doorSize/2}, {thick, H - (midY + doorSize/2)}));

    // Horizontal Wall (Center Y)
    // Left segment
    walls.emplace_back(sf::FloatRect({0.f, midY - thick/2}, {midX - doorSize/2, thick}));
    // Right segment
    walls.emplace_back(sf::FloatRect({midX + doorSize/2, midY - thick/2}, {W - (midX + doorSize/2), thick}));

    // --- 3. Obstacles (1 per room, centered in quadrant) ---
    float obsSize = 60.f;
    float qW = W / 2.f; // Quadrant Width
    float qH = H / 2.f; // Quadrant Height

    // Room 1 (TL)
    walls.emplace_back(sf::FloatRect({qW/2 - obsSize/2, qH/2 - obsSize/2}, {obsSize, obsSize}));
    // Room 2 (TR)
    walls.emplace_back(sf::FloatRect({midX + qW/2 - obsSize/2, qH/2 - obsSize/2}, {obsSize, obsSize}));
    // Room 3 (BL)
    walls.emplace_back(sf::FloatRect({qW/2 - obsSize/2, midY + qH/2 - obsSize/2}, {obsSize, obsSize}));
    // Room 4 (BR)
    walls.emplace_back(sf::FloatRect({midX + qW/2 - obsSize/2, midY + qH/2 - obsSize/2}, {obsSize, obsSize}));

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