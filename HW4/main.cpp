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

// --- PHYSICS HELPER ---
void resolveKinematicCollisions(Kinematic& k, const std::vector<sf::FloatRect>& walls) {
    float r = 10.f; 
    sf::FloatRect bounds({k.position.x - r, k.position.y - r}, {r * 2.f, r * 2.f});

    for (const auto& w : walls) {
        std::optional<sf::FloatRect> intersection = w.findIntersection(bounds);
        if (intersection) {
            if (intersection->size.x < intersection->size.y) {
                if (k.position.x < w.position.x) k.position.x -= intersection->size.x;
                else k.position.x += intersection->size.x;
            } else {
                if (k.position.y < w.position.y) k.position.y -= intersection->size.y;
                else k.position.y += intersection->size.y;
            }
        }
    }
}

// Decision Tree for the PLAYER
std::unique_ptr<DTNode> buildPlayerDT() {
    auto wander = std::make_unique<DTAction>(ActionType::WANDER);
    auto flee = std::make_unique<DTAction>(ActionType::FLEE_ENEMY);
    auto seek_center = std::make_unique<DTAction>(ActionType::SEEK_CENTER);
    auto attack = std::make_unique<DTAction>(ActionType::ATTACK);
    auto hide = std::make_unique<DTAction>(ActionType::HIDE);

    // If can hide, hide. Else flee.
    auto checkCanHide = std::make_unique<DTDecision>("canHide", std::move(hide), std::move(flee));

    // If can see enemy (and enemy is near), try to hide. Else (hidden but near), attack.
    auto checkVisibility = std::make_unique<DTDecision>("canSeeEnemy", std::move(checkCanHide), std::move(attack));

    // If near wall, seek center. Else wander.
    auto checkNearWall = std::make_unique<DTDecision>("isNearWall", std::move(seek_center), std::move(wander));

    // Root: Enemy Near?
    auto root = std::make_unique<DTDecision>("enemyNear", std::move(checkVisibility), std::move(checkNearWall));

    return root;
}

// Behavior Tree for the ENEMY
std::unique_ptr<BTNode> buildEnemyBT(DataRecorder& recorder) {
    auto root = std::make_unique<BTSelector>();
    
    // Sequence: See Player -> Chase
    auto chaseSeq = std::make_unique<BTSequence>();
    
    // Condition: Can See Player
    chaseSeq->addChild(std::make_unique<BTCondition>([](EnemyContext& ctx) {
        return hasLineOfSight(ctx.enemy.position, ctx.player.position, ctx.walls);
    }));
    
    // Action: Chase
    chaseSeq->addChild(std::make_unique<BTAction>([&recorder](EnemyContext& ctx) {
        WorldState state;
        state.canSeeEnemy = true; 
        state.enemyNear = (std::hypot(ctx.player.position.x - ctx.enemy.position.x, ctx.player.position.y - ctx.enemy.position.y) < 200.f);
        state.isNearWall = false; 
        state.canHide = false; // Enemy doesn't hide
        recorder.record(state, ActionType::CHASE);

        sf::Vector2f steering = ctx.player.position - ctx.enemy.position;
        float dist = std::hypot(steering.x, steering.y);
        if (dist > 0.1f) steering /= dist;
        
        float enemySpeed = 110.f;
        sf::Vector2f accel = (steering * enemySpeed - ctx.enemy.velocity) * 4.0f;
        ctx.enemy.velocity += accel * ctx.dt;
        
        float s = std::hypot(ctx.enemy.velocity.x, ctx.enemy.velocity.y);
        if (s > enemySpeed) ctx.enemy.velocity = (ctx.enemy.velocity / s) * enemySpeed;
        
        return BTStatus::SUCCESS;
    }));
    
    root->addChild(std::move(chaseSeq));
    
    // Action: Search (Seek Center)
    root->addChild(std::make_unique<BTAction>([&recorder](EnemyContext& ctx) {
        WorldState state;
        state.canSeeEnemy = false; 
        state.enemyNear = (std::hypot(ctx.player.position.x - ctx.enemy.position.x, ctx.player.position.y - ctx.enemy.position.y) < 200.f);
        state.isNearWall = false;
        state.canHide = false;
        recorder.record(state, ActionType::SEEK_CENTER);

        sf::Vector2f center(400.f, 300.f);
        sf::Vector2f steering = center - ctx.enemy.position;
        float dist = std::hypot(steering.x, steering.y);
        if (dist > 0.1f) steering /= dist;
        
        float enemySpeed = 60.f; 
        sf::Vector2f accel = (steering * enemySpeed - ctx.enemy.velocity) * 2.0f;
        ctx.enemy.velocity += accel * ctx.dt;
        
        return BTStatus::SUCCESS;
    }));
    
    return root;
}

void moveEnemyChase(Kinematic& enemy, const sf::Vector2f& targetPos, float dt) {
    sf::Vector2f steering = targetPos - enemy.position;
    float dist = std::hypot(steering.x, steering.y);
    if (dist > 0.1f) steering /= dist;
    
    float enemySpeed = 110.f;
    sf::Vector2f accel = (steering * enemySpeed - enemy.velocity) * 4.0f;
    enemy.velocity += accel * dt;
    
    float s = std::hypot(enemy.velocity.x, enemy.velocity.y);
    if (s > enemySpeed) enemy.velocity = (enemy.velocity / s) * enemySpeed;
}

void moveEnemySearch(Kinematic& enemy, float dt) {
    sf::Vector2f center(400.f, 300.f);
    sf::Vector2f steering = center - enemy.position;
    float dist = std::hypot(steering.x, steering.y);
    if (dist > 0.1f) steering /= dist;
    
    float enemySpeed = 60.f; 
    sf::Vector2f accel = (steering * enemySpeed - enemy.velocity) * 2.0f;
    enemy.velocity += accel * dt;
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

    Kinematic enemy; // This is the monster
    enemy.position = ENEMY_START_POS;
    enemy.velocity = {0.f, 0.f};
    
    Breadcrumb enemyTrail(150, 5, sf::Color::Red);

    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    // --- AI STATE ---
    const float THREAT_DIST = 150.0f;
    const float WALL_PROXIMITY = 60.0f; // Increased for better wall avoidance
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
        enemy.position = ENEMY_START_POS;
        enemy.velocity = {0.f, 0.f};
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
            float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                      chara.getKinematic().position.y - enemy.position.y);

            // *** GAME OVER CHECK ***
            if (dEnemy < 30.f) {
                resetGame();
                continue; 
            }

            if (mode == ACTING) {
                WorldState state;
                state.enemyNear = (dEnemy < THREAT_DIST);
                state.canSeeEnemy = hasLineOfSight(chara.getKinematic().position, enemy.position, walls);
                
                sf::Vector2f hidingSpot = findHidingSpot(chara.getKinematic().position, enemy.position, walls);
                state.canHide = (hidingSpot.x != -1.f);
                
                const auto& pos = chara.getKinematic().position;
                state.isNearWall = pos.x < WALL_PROXIMITY || pos.x > WINDOW_WIDTH - WALL_PROXIMITY ||
                                   pos.y < WALL_PROXIMITY || pos.y > WINDOW_HEIGHT - WALL_PROXIMITY;

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
                        chara.flee(enemy.position, dt);
                        break;
                    case ActionType::SEEK_CENTER:
                        // Only plan a new path every so often to avoid constant recalculation
                        if (pathUpdateTimer <= 0.f) { 
                            planPathTo(CENTER_SCREEN); 
                            pathUpdateTimer = 1.5f; 
                        }
                        break;
                    case ActionType::ATTACK:
                        chara.setPath({});
                        chara.attack(enemy.position, dt);
                        break;
                    case ActionType::HIDE:
                        if (state.canHide) {
                             planPathTo(hidingSpot);
                        } else {
                             chara.flee(enemy.position, dt);
                        }
                        break;
                    case ActionType::WANDER:
                        // Smart Wander: Pick a random node and pathfind
                        if (chara.isPathComplete()) {
                            if (graph.numVertices > 0) {
                                int r = std::rand() % graph.numVertices;
                                planPathTo(graph.positions[r]);
                            }
                        }
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
                 state.canSeeEnemy = hasLineOfSight(enemy.position, chara.getKinematic().position, walls);
                 state.enemyNear = false; state.isNearWall = false; state.canHide = false;

                 ActionType act = enemyDT->makeDecision(state);
                 if (act == ActionType::CHASE) moveEnemyChase(enemy, chara.getKinematic().position, dt);
                 else moveEnemySearch(enemy, dt);
            } else {
                // Execute Behavior Tree
                EnemyContext ctx { enemy, chara.getKinematic(), walls, dt };
                enemyBT->tick(ctx);
            }

            enemy.position += enemy.velocity * dt;
            resolveKinematicCollisions(enemy, walls);
            enemyShape.setPosition(enemy.position);
            enemyTrail.update(enemy.position);
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
        ring.setPosition(enemy.position);
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineColor(sf::Color(255, 50, 50, 80));
        ring.setOutlineThickness(1);
        window.draw(ring);

        window.draw(enemyShape);
        chara.draw(window);

        window.display();
    }
    return 0;
}