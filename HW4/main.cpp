#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include "ai.h"
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

    // If near a wall, seek center. Otherwise, wander.
    auto checkNearWall = std::make_unique<DTDecision>("isNearWall", std::move(seek_center), std::move(wander));
    // If enemy is near, flee. Otherwise, check for walls.
    auto root = std::make_unique<DTDecision>("enemyNear", std::move(flee), std::move(checkNearWall));

    return root;
}

int main() {
    sf::RenderWindow window(sf::VideoMode({(unsigned int)WINDOW_WIDTH, (unsigned int)WINDOW_HEIGHT}), "HW4: Player Decision Tree");
    window.setFramerateLimit(60);

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
    const float THREAT_DIST = 200.0f;
    const float WALL_PROXIMITY = 60.0f; // Increased for better wall avoidance
    const float NORMAL_SPEED = 150.f;
    const float FLEE_SPEED = 250.f;

    auto playerDT = buildPlayerDT();

    sf::Clock clock;
    enum Mode { WARMUP, ACTING, MANUAL };
    Mode mode = WARMUP; 
    float stateTimer = 0.f;
    float pathUpdateTimer = 0.f;

    // --- ENEMY PATHFINDING STATE (simple chase) ---
    float enemyRepathTimer = 0.f;

    std::cout << "--- STARTING ---" << std::endl;
    std::cout << "Player is AI-controlled. Click to take manual control." << std::endl;

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
        std::cout << "Player is AI-controlled. Click to take manual control." << std::endl;
    };

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.1f) dt = 0.1f; 

        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            
            if (const auto* mousePress = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mousePress->button == sf::Mouse::Button::Left) {
                    sf::Vector2f clickPos(static_cast<float>(mousePress->position.x), 
                                          static_cast<float>(mousePress->position.y));
                    planPathTo(clickPos);
                    mode = MANUAL; 
                    std::cout << "--- MANUAL CONTROL ---" << std::endl;
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
                    case ActionType::WANDER:
                        chara.setPath({}); // Clear any path and just wander
                        chara.wander(dt);
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

        // --- 3. ENEMY INTELLIGENCE (Simple Chase) ---
        if (mode != WARMUP) {
            enemyRepathTimer -= dt;
            // Simple deterministic chase
            if (enemyRepathTimer <= 0.f) {
                // Enemy just seeks the player's current position
                sf::Vector2f steering = chara.getKinematic().position - enemy.position;
                float dist = std::hypot(steering.x, steering.y);
                if (dist > 0.1f) steering /= dist;

                float enemySpeed = 110.f;
                sf::Vector2f accel = (steering * enemySpeed - enemy.velocity) * 4.0f;
                enemy.velocity += accel * dt;

                float s = std::hypot(enemy.velocity.x, enemy.velocity.y);
                if (s > enemySpeed) enemy.velocity = (enemy.velocity / s) * enemySpeed;

                enemyRepathTimer = 0.2f; // Re-evaluate direction frequently
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