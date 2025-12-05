#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include "ai.h"
#include "recorder.h"
#include "dt_learner.h"
#include "bt.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <optional>
#include <cmath>

// --- CONSTANTS ---
const sf::Vector2f AGENT_START_POS(200.f, 150.f);
const sf::Vector2f ENEMY_START_POS(600.f, 450.f);
const sf::Vector2f CENTER_SCREEN(WINDOW_WIDTH / 2.f, WINDOW_HEIGHT / 2.f);

// --- FORWARD DECLARATIONS ---
void planPath(Character& chara, const Graph& graph, sf::Vector2f target);
void moveEnemyChase(Character& enemy, const sf::Vector2f& targetPos, const Graph& graph, const std::vector<sf::FloatRect>& walls, float dt);
void moveEnemySearch(Character& enemy, const Graph& graph, float dt);

// --- GEOMETRY HELPERS ---
bool lineSegmentsIntersect(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, sf::Vector2f p4) {
    float det = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
    if (std::abs(det) < 0.001f) {
        return false; // Parallel or collinear
    }
    float t = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / det;
    float u = -((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / det;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

bool hasLineOfSight(sf::Vector2f start, sf::Vector2f end, const std::vector<sf::FloatRect>& walls) {
    for (const auto& wall : walls) {
        sf::Vector2f p1 = wall.position;
        sf::Vector2f p2 = {wall.position.x + wall.size.x, wall.position.y};
        sf::Vector2f p3 = {wall.position.x + wall.size.x, wall.position.y + wall.size.y};
        sf::Vector2f p4 = {wall.position.x, wall.position.y + wall.size.y};
        if (lineSegmentsIntersect(start, end, p1, p2) ||
            lineSegmentsIntersect(start, end, p2, p3) ||
            lineSegmentsIntersect(start, end, p3, p4) ||
            lineSegmentsIntersect(start, end, p4, p1)) {
            return false;
        }
    }
    return true;
}

sf::Vector2f findHidingSpot(const sf::Vector2f& seekerPos, const sf::Vector2f& threatPos, const std::vector<sf::FloatRect>& walls) {
    sf::Vector2f bestSpot = seekerPos;
    float minDist = -1.f;
    bool found = false;

    for (const auto& wall : walls) {
        float offset = 40.f;
        std::vector<sf::Vector2f> candidates = {
            {wall.position.x - offset, wall.position.y - offset},
            {wall.position.x + wall.size.x + offset, wall.position.y - offset},
            {wall.position.x + wall.size.x + offset, wall.position.y + wall.size.y + offset},
            {wall.position.x - offset, wall.position.y + wall.size.y + offset}
        };

        for (const auto& p : candidates) {
            if (p.x < 20.f || p.x > WINDOW_WIDTH - 20.f || p.y < 20.f || p.y > WINDOW_HEIGHT - 20.f) continue;
            
            if (!hasLineOfSight(p, threatPos, walls)) {
                float d = std::hypot(p.x - seekerPos.x, p.y - seekerPos.y);
                if (!found || d < minDist) {
                    minDist = d;
                    bestSpot = p;
                    found = true;
                }
            }
        }
    }
    return found ? bestSpot : sf::Vector2f(-1.f, -1.f);
}

bool isNearAnyWall(sf::Vector2f pos, const std::vector<sf::FloatRect>& walls, float threshold) {
    // Check screen borders
    if (pos.x < threshold || pos.x > WINDOW_WIDTH - threshold ||
        pos.y < threshold || pos.y > WINDOW_HEIGHT - threshold) return true;

    // Check internal walls
    for (const auto& w : walls) {
        float closestX = std::fmax(w.position.x, std::fmin(pos.x, w.position.x + w.size.x));
        float closestY = std::fmax(w.position.y, std::fmin(pos.y, w.position.y + w.size.y));
        
        float dx = pos.x - closestX;
        float dy = pos.y - closestY;
        
        if ((dx * dx + dy * dy) < (threshold * threshold)) return true;
    }
    return false;
}

// --- PHYSICS HELPER ---
void resolveKinematicCollisions(Kinematic& k, const std::vector<sf::FloatRect>& walls) {
    float r = 10.f; 
    sf::FloatRect bounds({k.position.x - r, k.position.y - r}, {r * 2.f, r * 2.f});

    for (const auto& w : walls) {
        std::optional<sf::FloatRect> intersection = w.findIntersection(bounds);
        if (intersection) {
            if (intersection->size.x < intersection->size.y) {
                // Horizontal collision (hit vertical side of wall)
                if (k.position.x < w.position.x) k.position.x -= intersection->size.x;
                else k.position.x += intersection->size.x;
                k.velocity.x = 0.f; // Kill x velocity
            } else {
                // Vertical collision (hit horizontal side of wall)
                if (k.position.y < w.position.y) k.position.y -= intersection->size.y;
                else k.position.y += intersection->size.y;
                k.velocity.y = 0.f; // Kill y velocity
            }
        }
    }
}

// Decision Tree for the PLAYER
std::unique_ptr<DTNode> buildPlayerDT() {
    auto wander = std::make_unique<DTAction>(ActionType::WANDER);
    auto flee = std::make_unique<DTAction>(ActionType::FLEE_ENEMY);
    auto seek_center = std::make_unique<DTAction>(ActionType::SEEK_CENTER);

    // If near a wall, seek center. Otherwise, wander.
    auto checkNearWall = std::make_unique<DTDecision>("isNearWall", std::move(seek_center), std::move(wander));

    // Root: Enemy Near?
    auto root = std::make_unique<DTDecision>("enemyNear", std::move(flee), std::move(checkNearWall));

    return root;
}

// Behavior Tree for the ENEMY
std::unique_ptr<BTNode> buildEnemyBT(DataRecorder& recorder) {
    auto root = std::make_unique<BTSelector>();
    
    // --- 1. CHASE SEQUENCE ---
    auto chaseSeq = std::make_unique<BTSequence>();
    
    // Condition: Can See Player
    chaseSeq->addChild(std::make_unique<BTCondition>([](EnemyContext& ctx) {
        return hasLineOfSight(ctx.enemy.getKinematic().position, ctx.player.position, ctx.walls);
    }));
    
    // Action: Chase
    chaseSeq->addChild(std::make_unique<BTAction>([&recorder](EnemyContext& ctx) {
        ctx.danceTimer = 0.f; // Stop dancing if we see the player
        
        WorldState state;
        state.canSeeEnemy = true; 
        state.enemyNear = (std::hypot(ctx.player.position.x - ctx.enemy.getKinematic().position.x, ctx.player.position.y - ctx.enemy.getKinematic().position.y) < 200.f);
        state.isNearWall = false; 
        state.canHide = false;
        recorder.record(state, ActionType::CHASE);

        moveEnemyChase(ctx.enemy, ctx.player.position, ctx.graph, ctx.walls, ctx.dt);
        return BTStatus::SUCCESS;
    }));
    
    root->addChild(std::move(chaseSeq));
    
    // --- 2. DANCE SEQUENCE ---
    auto danceSeq = std::make_unique<BTSequence>();

    // Condition: Should Dance? (Active timer OR random chance)
    danceSeq->addChild(std::make_unique<BTCondition>([](EnemyContext& ctx) {
        if (ctx.danceTimer > 0.f) return true; // Already dancing
        
        // 0.5% chance per tick to start dancing if not already
        if ((rand() % 1000) < 5) {
            ctx.danceTimer = 1.5f; // Dance for 1.5 seconds
            return true;
        }
        return false;
    }));

    // Action: Dance (Spin)
    danceSeq->addChild(std::make_unique<BTAction>([&recorder](EnemyContext& ctx) {
        ctx.danceTimer -= ctx.dt;
        
        WorldState state;
        state.canSeeEnemy = false; // Can't see player while dancing (simplified)
        state.enemyNear = false;
        state.isNearWall = false;
        state.canHide = false;
        recorder.record(state, ActionType::DANCE);
        
        // Spin behavior
        Kinematic& k = ctx.enemy.getKinematicRef();
        k.velocity = {0.f, 0.f}; // Stop moving
        k.rotation = 15.f; // Fast spin
        k.orientation += k.rotation * ctx.dt; // Apply manually since we might have stopped physics updates for velocity

        // Record (optional, mapping to NONE or a specific state)
        // recorder.record(..., ActionType::NONE); 
        
        return BTStatus::SUCCESS;
    }));

    root->addChild(std::move(danceSeq));

    // --- 3. WANDER ACTION (Default) ---
    root->addChild(std::make_unique<BTAction>([&recorder](EnemyContext& ctx) {
        WorldState state;
        state.canSeeEnemy = false; 
        state.enemyNear = false;
        state.isNearWall = false;
        state.canHide = false;
        recorder.record(state, ActionType::WANDER);

        // Use the same wander logic as player, or graph wander
        // Let's use Graph Wander (Search) for the enemy
        moveEnemySearch(ctx.enemy, ctx.graph, ctx.dt);
        
        return BTStatus::SUCCESS;
    }));
    
    return root;
}

void planPath(Character& chara, const Graph& graph, sf::Vector2f target) {
    Metrics m;
    sf::Vector2f pos = chara.getKinematic().position;
    int startNode = graph.getNodeAt(pos.x, pos.y, 20.f);
    int endNode = graph.getNodeAt(target.x, target.y, 20.f);
    
    if (startNode != -1 && endNode != -1) {
        std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
        if (!pathIndices.empty()) {
            std::vector<sf::Vector2f> points;
            for (int idx : pathIndices) points.push_back(graph.positions[idx]);
            points.push_back(target);
            chara.setPath(points);
        } else {
            chara.setPath({target});
        }
    } else {
        chara.setPath({target});
    }
}

void moveEnemyChase(Character& enemy, const sf::Vector2f& targetPos, const Graph& graph, const std::vector<sf::FloatRect>& walls, float dt) {
    if (hasLineOfSight(enemy.getKinematic().position, targetPos, walls)) {
        enemy.setPath({});
        enemy.seek(targetPos, dt);
    } else {
        if (enemy.isPathComplete()) {
             planPath(enemy, graph, targetPos);
        }
    }
}

void moveEnemySearch(Character& enemy, const Graph& graph, float dt) {
    (void)dt;
    if (enemy.isPathComplete()) {
        if (graph.numVertices > 0) {
             int r = std::rand() % graph.numVertices;
             planPath(enemy, graph, graph.positions[r]);
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode({(unsigned int)WINDOW_WIDTH, (unsigned int)WINDOW_HEIGHT}), "HW4: Player Decision Tree");
    window.setFramerateLimit(60);

    DataRecorder recorder("training_data.csv");

    // --- ENVIRONMENT ---
    std::vector<sf::FloatRect> walls;
    Graph graph = createFourRoomGraph(walls); 
    
    // --- SETUP ENTITIES ---
    Character chara; // This is the player
    chara.teleport(AGENT_START_POS.x, AGENT_START_POS.y); 

    Character enemy; // This is the monster
    enemy.teleport(ENEMY_START_POS.x, ENEMY_START_POS.y);
    enemy.setColor(sf::Color::Red);
    
    Breadcrumb enemyTrail(150, 5, sf::Color::Red);

    // --- AI STATE ---
    const float THREAT_DIST = 200.0f;
    const float WALL_PROXIMITY = 25.0f; // Further decreased for less aggressive wall avoidance
    const float NORMAL_SPEED = 150.f;
    const float FLEE_SPEED = 250.f;

    auto playerDT = buildPlayerDT();
    auto enemyBT = buildEnemyBT(recorder);
    std::unique_ptr<DTNode> enemyDT = nullptr;

    sf::Clock clock;
    enum Mode { WARMUP, ACTING };
    Mode mode = WARMUP; 
    float stateTimer = 0.f;
    float pathUpdateTimer = 0.f;

    // --- ENEMY PATHFINDING STATE (simple chase) ---
    float enemyRepathTimer = 0.f;
    float enemyDanceTimer = 0.f; // Timer for the dance behavior

    std::cout << "--- STARTING ---" << std::endl;
    std::cout << "Player is AI-controlled." << std::endl;

    auto planPathTo = [&](sf::Vector2f target) {
        Metrics m;
        int startNode = graph.getNodeAt(chara.getKinematic().position.x, chara.getKinematic().position.y, 20.f);
        int endNode = graph.getNodeAt(target.x, target.y, 20.f);
        
        if (startNode != -1 && endNode != -1) {
            std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
            if (!pathIndices.empty()) {
                std::vector<sf::Vector2f> points;
                for (int idx : pathIndices) points.push_back(graph.positions[idx]);
                points.push_back(target);
                chara.setPath(points);
            }
        } else {
            chara.seek(target, 0.016f); 
        }
    };

    // Helper to Reset Game State
    auto resetGame = [&]() {
        std::cout << ">>> CAUGHT! Resetting positions... <<<" << std::endl;
        chara.teleport(AGENT_START_POS.x, AGENT_START_POS.y);
        enemy.teleport(ENEMY_START_POS.x, ENEMY_START_POS.y);
        mode = WARMUP;
        stateTimer = 0.f;
        std::cout << "Player is AI-controlled." << std::endl;
    };

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.1f) dt = 0.1f; 

        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            
            if (const auto* keyPress = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPress->code == sf::Keyboard::Key::L) {
                    std::cout << "Learning ENEMY DT from data..." << std::endl;
                    enemyDT = learnDT("training_data.csv");
                    if (enemyDT) {
                        std::cout << "--- Learned Decision Tree ---" << std::endl;
                        enemyDT->print();
                        std::cout << "-----------------------------" << std::endl;
                    }
                }
            }
        }

        // --- 1. GAME LOGIC & PLAYER AI ---
        if (mode == WARMUP) {
            stateTimer += dt;
            if (stateTimer > 1.0f) {
                mode = ACTING;
                stateTimer = 0.f;
                std::cout << "--- PLAYER AI ACTIVE ---" << std::endl;
            }
        }
        else {
            float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.getKinematic().position.x, 
                                      chara.getKinematic().position.y - enemy.getKinematic().position.y);

            // *** GAME OVER CHECK ***
            if (dEnemy < 30.f) {
                resetGame();
                continue; 
            }

            if (mode == ACTING) {
                WorldState state;
                state.canSeeEnemy = hasLineOfSight(chara.getKinematic().position, enemy.getKinematic().position, walls);
                state.enemyNear = (dEnemy < THREAT_DIST) && state.canSeeEnemy;
                
                sf::Vector2f hidingSpot = findHidingSpot(chara.getKinematic().position, enemy.getKinematic().position, walls);
                state.canHide = (hidingSpot.x != -1.f);
                
                state.isNearWall = isNearAnyWall(chara.getKinematic().position, walls, WALL_PROXIMITY);

                // Make decisions for the player character
                ActionType action = playerDT->makeDecision(state);
                
                // Adjust speed based on threat
                if (state.enemyNear) {
                    chara.setMaxSpeed(FLEE_SPEED);
                } else {
                    chara.setMaxSpeed(NORMAL_SPEED);
                }

                pathUpdateTimer -= dt;
                switch(action) {
                    case ActionType::FLEE_ENEMY:
                        chara.setPath({}); // Clear any path and flee directly
                        chara.flee(enemy.getKinematic().position, dt);
                        break;
                    case ActionType::SEEK_CENTER: // Maps to "Avoid Wall" in the DT
                        {
                            // Find closest wall point to steer away from
                            sf::Vector2f charPos = chara.getKinematic().position;
                            sf::Vector2f repulsionVector(0.f, 0.f);
                            bool foundWall = false;

                            // Check borders
                            if (charPos.x < WALL_PROXIMITY) { repulsionVector.x += 1.f; foundWall = true; }
                            if (charPos.x > WINDOW_WIDTH - WALL_PROXIMITY) { repulsionVector.x -= 1.f; foundWall = true; }
                            if (charPos.y < WALL_PROXIMITY) { repulsionVector.y += 1.f; foundWall = true; }
                            if (charPos.y > WINDOW_HEIGHT - WALL_PROXIMITY) { repulsionVector.y -= 1.f; foundWall = true; }

                            // Check internal walls (simple AABB proximity)
                            for (const auto& w : walls) {
                                float closestX = std::fmax(w.position.x, std::fmin(charPos.x, w.position.x + w.size.x));
                                float closestY = std::fmax(w.position.y, std::fmin(charPos.y, w.position.y + w.size.y));
                                
                                float dx = charPos.x - closestX;
                                float dy = charPos.y - closestY;
                                float distSq = dx*dx + dy*dy;
                                
                                if (distSq < WALL_PROXIMITY * WALL_PROXIMITY && distSq > 0.001f) {
                                    float dist = std::sqrt(distSq);
                                    repulsionVector.x += (dx / dist);
                                    repulsionVector.y += (dy / dist);
                                    foundWall = true;
                                }
                            }

                            if (foundWall) {
                                // normalize repulsion
                                float len = std::hypot(repulsionVector.x, repulsionVector.y);
                                if (len > 0.001f) {
                                    repulsionVector /= len;
                                } else {
                                    // Forces cancelled out (e.g. doorway or narrow hall)
                                    // Break deadlock by moving forward or random
                                    if (chara.getKinematic().getSpeed() > 10.f) {
                                        repulsionVector = chara.getKinematic().velocity / chara.getKinematic().getSpeed();
                                    } else {
                                        // Totally stuck, pick random
                                        float angle = (float)(rand() % 360) * 3.14159f / 180.f;
                                        repulsionVector = sf::Vector2f(std::cos(angle), std::sin(angle));
                                    }
                                }

                                // Steer in the direction of the repulsion vector
                                // Increased distance to 100.f for less dramatic bounce
                                sf::Vector2f target = charPos + repulsionVector * 100.f;
                                chara.setPath({});
                                chara.seek(target, dt);
                            } else {
                                // Fallback
                                chara.wander(dt);
                            }
                        }
                        break;
                    case ActionType::ATTACK:
                        chara.setPath({});
                        chara.attack(enemy.getKinematic().position, dt);
                        break;
                    case ActionType::HIDE:
                        if (state.canHide) {
                             planPathTo(hidingSpot);
                        } else {
                             chara.flee(enemy.getKinematic().position, dt);
                        }
                        break;
                    case ActionType::WANDER:
                        // Graph-Based Wander: Pick a random node and pathfind
                        // This avoids local optima (getting stuck in corners/bouncing)
                        if (chara.isPathComplete()) {
                            if (graph.numVertices > 0) {
                                // Pick a random target node that isn't the current one
                                int r = std::rand() % graph.numVertices;
                                sf::Vector2f target = graph.positions[r];
                                
                                // Plan path using A*
                                planPathTo(target);
                            }
                        }
                        // If path exists, Character::update automatically follows it.
                        break;
                    default: 
                        chara.stop();
                        break;
                }
            }
        }

        // --- 2. PLAYER PHYSICS ---
        Kinematic dummy; // Dummy target, not used by player update logic
        chara.update(dt, dummy);
        
        Kinematic kChar = chara.getKinematic();
        resolveKinematicCollisions(kChar, walls);
        chara.setPosition(kChar.position.x, kChar.position.y);

        // --- 3. ENEMY INTELLIGENCE (Behavior Tree) ---
        if (mode != WARMUP) {
            if (enemyDT) {
                 WorldState state;
                 state.canSeeEnemy = hasLineOfSight(enemy.getKinematic().position, chara.getKinematic().position, walls);
                 state.enemyNear = false; state.isNearWall = false; state.canHide = false;

                 ActionType act = enemyDT->makeDecision(state);
                 if (act == ActionType::CHASE) {
                     moveEnemyChase(enemy, chara.getKinematic().position, graph, walls, dt);
                 } else if (act == ActionType::DANCE) {
                     // Replicate Dance spin behavior
                     Kinematic& k = enemy.getKinematicRef();
                     k.velocity = {0.f, 0.f}; 
                     k.rotation = 15.f; 
                     k.orientation += k.rotation * dt;
                 } else {
                     moveEnemySearch(enemy, graph, dt);
                 }
            } else {
                // Execute Behavior Tree
                EnemyContext ctx { enemy, chara.getKinematic(), walls, graph, dt, enemyDanceTimer };
                enemyBT->tick(ctx);
            }

            Kinematic dummy;
            enemy.update(dt, dummy);
            
            Kinematic& kEnemy = enemy.getKinematicRef();
            resolveKinematicCollisions(kEnemy, walls);
            enemy.setPosition(kEnemy.position.x, kEnemy.position.y);

            enemyTrail.update(kEnemy.position);
        }

        // --- DRAW ---
        window.clear(sf::Color(20, 20, 25));
        
        for (const auto& w : walls) {
            sf::RectangleShape r({w.size.x, w.size.y});
            r.setPosition(w.position);
            r.setFillColor(sf::Color(100, 100, 110));
            r.setOutlineColor(sf::Color(200, 200, 200));
            r.setOutlineThickness(1);
            window.draw(r);
        }

        enemyTrail.draw(window);

        // Draw threat ring around enemy, not player
        sf::CircleShape ring(THREAT_DIST);
        ring.setOrigin({THREAT_DIST, THREAT_DIST});
        ring.setPosition(enemy.getKinematic().position);
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineColor(sf::Color(255, 50, 50, 80));
        ring.setOutlineThickness(1);
        window.draw(ring);

        enemy.draw(window);
        chara.draw(window);

        window.display();
    }
    return 0;
}