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

// --- PHYSICS HELPER ---
void resolveKinematicCollisions(Kinematic& k, const std::vector<sf::FloatRect>& walls) {
    float r = 10.f; // Radius
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

std::unique_ptr<DTNode> buildHardcodedDT() {
    auto wander = std::make_unique<DTAction>(ActionType::WANDER);
    auto seek = std::make_unique<DTAction>(ActionType::SEEK_GOAL);
    auto recharge = std::make_unique<DTAction>(ActionType::RECHARGE);
    auto flee = std::make_unique<DTAction>(ActionType::FLEE_ENEMY);

    auto checkGoal = std::make_unique<DTDecision>("goalVisible", std::move(seek), std::move(wander));
    auto checkEnergy = std::make_unique<DTDecision>("energyLow", std::move(recharge), std::move(checkGoal));
    auto root = std::make_unique<DTDecision>("enemyNear", std::move(flee), std::move(checkEnergy));
    return root;
}

int main() {
    sf::RenderWindow window(sf::VideoMode({(unsigned int)WINDOW_WIDTH, (unsigned int)WINDOW_HEIGHT}), "HW4: Intelligent Enemy");
    window.setFramerateLimit(60);

    // --- ENVIRONMENT ---
    std::vector<sf::FloatRect> walls;
    Graph graph = createFourRoomGraph(walls); 
    
    // --- SETUP ENTITIES ---
    Character chara;
    chara.teleport(200.f, 150.f); 

    Kinematic enemy;
    enemy.position = {600.f, 450.f}; // Room 4
    enemy.velocity = {0.f, 0.f};
    
    // NEW: Enemy Breadcrumbs (Red, fade slightly slower than agent)
    Breadcrumb enemyTrail(150, 5, sf::Color::Red);

    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(600.f, 150.f); // Room 2
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);
    goalShape.setOrigin({15.f, 15.f});

    sf::Vector2f stationPos(200.f, 450.f); // Room 3
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);
    stationShape.setOrigin({20.f, 20.f});

    // --- AI STATE ---
    float energy = 100.0f;
    const float THREAT_DIST = 150.0f;
    const float GOAL_DIST = 300.0f;

    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { WARMUP, RECORDING, LEARNING, ACTING, MANUAL };
    Mode mode = WARMUP; 
    float stateTimer = 0.f;
    float pathUpdateTimer = 0.f;

    // --- ENEMY PATHFINDING STATE ---
    std::vector<sf::Vector2f> enemyPath;
    int enemyWaypoint = 0;
    float enemyRepathTimer = 0.f;

    std::cout << "--- STARTING ---" << std::endl;
    std::cout << "Warmup (1s)..." << std::endl;

    auto planPathTo = [&](sf::Vector2f target) {
        Metrics m;
        if (target.x < 40 || target.x > WINDOW_WIDTH-40 || target.y < 40 || target.y > WINDOW_HEIGHT-40) return;

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

    auto planEnemyPath = [&](sf::Vector2f target) {
        Metrics m;
        int startNode = graph.getNodeAt(enemy.position.x, enemy.position.y, 20.f);
        int endNode = graph.getNodeAt(target.x, target.y, 20.f);

        if (startNode != -1 && endNode != -1) {
            std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
            if (!pathIndices.empty()) {
                enemyPath.clear();
                for (int idx : pathIndices) enemyPath.push_back(graph.positions[idx]);
                enemyPath.push_back(target);
                enemyWaypoint = 0;
            }
        }
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
                }
            }
        }

        // --- 1. GAME LOGIC ---
        if (mode == WARMUP) {
            stateTimer += dt;
            if (stateTimer > 1.0f) {
                mode = RECORDING;
                stateTimer = 0.f;
                std::cout << "Phase 1: Recording..." << std::endl;
            }
        }
        else {
            WorldState state;
            float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                      chara.getKinematic().position.y - enemy.position.y);
            float dGoal = std::hypot(chara.getKinematic().position.x - goalPos.x,
                                     chara.getKinematic().position.y - goalPos.y);

            state.enemyNear = (dEnemy < THREAT_DIST);
            state.energyLow = (energy < 30.0f);
            state.goalVisible = (dGoal < GOAL_DIST);

            ActionType action = ActionType::NONE;
            
            if (mode != MANUAL) {
                if (mode == RECORDING) {
                    action = hardcodedDT->makeDecision(state);
                    if ((int)(stateTimer * 10) % 5 == 0) trainingData.push_back({state, action});
                    stateTimer += dt;
                    if (stateTimer > 10.0f) {
                        mode = LEARNING;
                        std::cout << "--- LEARNING COMPLETE ---" << std::endl;
                    }
                } 
                else if (mode == LEARNING) {
                    std::vector<std::string> attrs = {"enemyNear", "energyLow", "goalVisible"};
                    learnedDT = ID3Learner::learn(trainingData, attrs);
                    learnedDT->print();
                    mode = ACTING;
                }
                else if (mode == ACTING && learnedDT) {
                    action = learnedDT->makeDecision(state);
                }

                pathUpdateTimer -= dt;
                switch(action) {
                    case ActionType::FLEE_ENEMY:
                        chara.setPath({});
                        chara.flee(enemy.position, dt);
                        energy -= 5.f * dt;
                        break;
                    case ActionType::SEEK_GOAL:
                        if (pathUpdateTimer <= 0.f) { planPathTo(goalPos); pathUpdateTimer = 1.0f; }
                        energy -= 2.f * dt;
                        break;
                    case ActionType::RECHARGE:
                        if (pathUpdateTimer <= 0.f) { planPathTo(stationPos); pathUpdateTimer = 1.0f; }
                        if (std::hypot(chara.getKinematic().position.x - stationPos.x, 
                                       chara.getKinematic().position.y - stationPos.y) < 50) 
                            energy += 20.f * dt;
                        break;
                    case ActionType::WANDER:
                        chara.setPath({});
                        chara.wander(dt);
                        energy -= 1.f * dt;
                        break;
                    default: break;
                }
            }
        }

        // --- 2. AGENT PHYSICS ---
        Kinematic dummy; 
        chara.update(dt, dummy);
        energy = std::max(0.f, std::min(100.f, energy));
        
        Kinematic kChar = chara.getKinematic();
        resolveKinematicCollisions(kChar, walls);
        chara.setPosition(kChar.position.x, kChar.position.y);

        // --- 3. ENEMY INTELLIGENCE ---
        enemyRepathTimer -= dt;
        bool chase = (std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                 chara.getKinematic().position.y - enemy.position.y) < 300.f);

        if (chase) {
            if (enemyRepathTimer <= 0.f) {
                planEnemyPath(chara.getKinematic().position);
                enemyRepathTimer = 0.5f;
            }
        } else {
            if (enemyPath.empty() || enemyWaypoint >= (int)enemyPath.size()) {
                int rNode = rand() % graph.positions.size();
                planEnemyPath(graph.positions[rNode]);
            }
        }

        sf::Vector2f steering(0,0);
        float enemySpeed = 110.f; // Slower Enemy

        if (!enemyPath.empty() && enemyWaypoint < (int)enemyPath.size()) {
            sf::Vector2f target = enemyPath[enemyWaypoint];
            sf::Vector2f dir = target - enemy.position;
            float dist = std::hypot(dir.x, dir.y);

            if (dist < 20.f) {
                enemyWaypoint++;
            } else {
                dir /= dist;
                steering = dir * enemySpeed;
            }
        }

        sf::Vector2f accel = (steering - enemy.velocity) * 4.0f;
        enemy.velocity += accel * dt;
        
        float s = std::hypot(enemy.velocity.x, enemy.velocity.y);
        if (s > enemySpeed) enemy.velocity = (enemy.velocity / s) * enemySpeed;

        enemy.position += enemy.velocity * dt;
        resolveKinematicCollisions(enemy, walls);
        enemyShape.setPosition(enemy.position);
        
        // UPDATE ENEMY BREADCRUMBS
        enemyTrail.update(enemy.position);

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

        // Draw Enemy Trail
        enemyTrail.draw(window);

        sf::CircleShape ring(THREAT_DIST);
        ring.setOrigin({THREAT_DIST, THREAT_DIST});
        ring.setPosition(chara.getKinematic().position);
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineColor(sf::Color(255, 50, 50, 80));
        ring.setOutlineThickness(1);
        window.draw(ring);

        window.draw(enemyShape);
        window.draw(goalShape);
        window.draw(stationShape);
        chara.draw(window);

        window.display();
    }
    return 0;
}