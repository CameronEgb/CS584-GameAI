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
// Forcefully keep the character inside the screen and outside of walls.
void resolveCollisions(Character& chara, const std::vector<sf::FloatRect>& walls) {
    Kinematic k = chara.getKinematic();
    float r = 10.f; // Approximate radius of character for collision
    
    // 1. Screen Boundaries
    if (k.position.x < r) k.position.x = r;
    if (k.position.x > 1200.f - r) k.position.x = 1200.f - r;
    if (k.position.y < r) k.position.y = r;
    if (k.position.y > 900.f - r) k.position.y = 900.f - r;

    // 2. Wall Collisions (AABB)
    // We check a small box around the character against all walls
    sf::FloatRect agentBounds({k.position.x - r, k.position.y - r}, {r * 2.f, r * 2.f});

    for (const auto& w : walls) {
        std::optional<sf::FloatRect> intersection = w.findIntersection(agentBounds);
        if (intersection) {
            // Push out in the direction of least overlap
            if (intersection->size.x < intersection->size.y) {
                if (k.position.x < w.position.x) k.position.x -= intersection->size.x;
                else k.position.x += intersection->size.x;
            } else {
                if (k.position.y < w.position.y) k.position.y -= intersection->size.y;
                else k.position.y += intersection->size.y;
            }
        }
    }
    
    // Apply resolved position back to character
    chara.setPosition(k.position.x, k.position.y);
}

std::unique_ptr<DTNode> buildHardcodedDT() {
    auto wander = std::make_unique<DTAction>(ActionType::WANDER);
    auto seek = std::make_unique<DTAction>(ActionType::SEEK_GOAL);
    auto recharge = std::make_unique<DTAction>(ActionType::RECHARGE);
    auto flee = std::make_unique<DTAction>(ActionType::FLEE_ENEMY);

    // If goal is visible -> Seek Goal
    // Else if Energy Low -> Recharge
    // Else -> Wander
    // (Enemy check is usually top priority, but for testing mouse click we might want to override)
    
    auto checkGoal = std::make_unique<DTDecision>("goalVisible", std::move(seek), std::move(wander));
    auto checkEnergy = std::make_unique<DTDecision>("energyLow", std::move(recharge), std::move(checkGoal));
    auto root = std::make_unique<DTDecision>("enemyNear", std::move(flee), std::move(checkEnergy));
    return root;
}

int main() {
    sf::RenderWindow window(sf::VideoMode({1200, 900}), "HW4: Intelligent Navigation");
    window.setFramerateLimit(60);

    // --- ENVIRONMENT ---
    std::vector<sf::FloatRect> walls;
    Graph graph = createFourRoomGraph(walls); 
    
    // --- SETUP ENTITIES ---
    Character chara;
    chara.setPosition(100.f, 100.f); // Top-Left Room

    Kinematic enemy;
    enemy.position = {1100.f, 800.f}; // Bottom-Right Room
    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(1100.f, 100.f); // Top-Right Room
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);
    goalShape.setOrigin({15.f, 15.f});

    sf::Vector2f stationPos(100.f, 800.f); // Bottom-Left Room
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);
    stationShape.setOrigin({20.f, 20.f});

    // --- AI / LOGIC STATE ---
    float energy = 100.0f;
    const float THREAT_DIST = 200.0f;
    const float GOAL_DIST = 400.0f;

    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING, MANUAL_OVERRIDE };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;
    float pathUpdateTimer = 0.f;

    std::cout << "--- STARTING SIMULATION ---\n";
    std::cout << "Phase 1: Recording Data (10s). You can click to force move, but let it run for data.\n";

    // --- HELPER LAMBDA FOR PATHFINDING ---
    auto planPathTo = [&](sf::Vector2f target) {
        Metrics m;
        // Map world positions to graph nodes (using 30.f cell size from graph.cpp)
        int startNode = graph.getNodeAt(chara.getKinematic().position.x, chara.getKinematic().position.y, 30.f);
        int endNode = graph.getNodeAt(target.x, target.y, 30.f);
        
        if (startNode != -1 && endNode != -1) {
            std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
            if (!pathIndices.empty()) {
                std::vector<sf::Vector2f> points;
                for (int idx : pathIndices) points.push_back(graph.positions[idx]);
                // Append exact target as final point
                points.push_back(target);
                chara.setPath(points);
            }
        } else {
            // Fallback: direct seek if off-mesh
            chara.seek(target, 0.016f); // dt approximation
        }
    };

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.1f) dt = 0.1f; 

        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
            
            // --- MOUSE CLICK HANDLING ---
            if (const auto* mousePress = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mousePress->button == sf::Mouse::Button::Left) {
                    sf::Vector2f clickPos(static_cast<float>(mousePress->position.x), 
                                          static_cast<float>(mousePress->position.y));
                    
                    std::cout << "User clicked at: " << clickPos.x << ", " << clickPos.y << "\n";
                    planPathTo(clickPos);
                    mode = MANUAL_OVERRIDE; // Temporarily disable AI to let user drive
                }
            }
        }

        // 1. Perception
        WorldState state;
        float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                  chara.getKinematic().position.y - enemy.position.y);
        float dGoal = std::hypot(chara.getKinematic().position.x - goalPos.x,
                                 chara.getKinematic().position.y - goalPos.y);

        state.enemyNear = (dEnemy < THREAT_DIST);
        state.energyLow = (energy < 30.0f);
        state.goalVisible = (dGoal < GOAL_DIST);

        // 2. Decision Making (Skipped if Manual Override until path done)
        ActionType action = ActionType::NONE;
        
        // If we are in manual override, check if we reached destination (velocity ~ 0)
        if (mode == MANUAL_OVERRIDE) {
            if (std::hypot(chara.getKinematic().velocity.x, chara.getKinematic().velocity.y) < 5.f) {
                // Determine logic to resume AI? For now, stay in override unless specific key pressed
                // Or just switch back to ACTING if we were acting
            }
        } 
        else {
            // AI Logic
            if (mode == RECORDING) {
                action = hardcodedDT->makeDecision(state);
                // Only log valid actions
                if ((int)(recordingTimer * 10) % 5 == 0) trainingData.push_back({state, action});
                
                recordingTimer += dt;
                if (recordingTimer > 10.0f) {
                    mode = LEARNING;
                    std::cout << "--- LEARNING COMPLETE (Tree Generated) ---\n";
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
        }

        // 3. Actuation based on AI Decision
        // Note: Manual Override doesn't use 'action', the path is already set in 'chara'
        if (mode != MANUAL_OVERRIDE) {
            pathUpdateTimer -= dt;
            
            switch(action) {
                case ActionType::FLEE_ENEMY:
                    chara.setPath({}); // Cancel paths
                    chara.flee(enemy.position, dt);
                    energy -= 5.f * dt;
                    break;
                case ActionType::SEEK_GOAL:
                    if (pathUpdateTimer <= 0.f) { planPathTo(goalPos); pathUpdateTimer = 1.0f; }
                    // Update called below
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

        // Always update character physics/movement integration
        // Pass dummy target because path following handles its own target internally
        Kinematic dummy; 
        chara.update(dt, dummy);

        energy = std::max(0.f, std::min(100.f, energy));

        // 4. PHYSICS RESOLUTION (Prevents walking through walls)
        resolveCollisions(chara, walls);

        // Enemy Logic (Simple bounce/wander)
        if (dEnemy < 300) {
            sf::Vector2f dir = chara.getKinematic().position - enemy.position;
            float len = std::hypot(dir.x, dir.y);
            if (len > 0) dir /= len;
            enemy.position += dir * 75.f * dt;
        } else {
            enemy.position.x += (rand()%3 - 1)*1.5f; 
            enemy.position.y += (rand()%3 - 1)*1.5f;
        }
        // Clamp enemy
        if(enemy.position.x < 10) enemy.position.x = 10;
        if(enemy.position.x > 1190) enemy.position.x = 1190;
        if(enemy.position.y < 10) enemy.position.y = 10;
        if(enemy.position.y > 890) enemy.position.y = 890;
        enemyShape.setPosition(enemy.position);

        // --- DRAW ---
        window.clear(sf::Color(20, 20, 25));
        
        // Draw Walls
        for (const auto& w : walls) {
            sf::RectangleShape r({w.size.x, w.size.y});
            r.setPosition(w.position);
            r.setFillColor(sf::Color(100, 100, 110));
            r.setOutlineColor(sf::Color(200, 200, 200));
            r.setOutlineThickness(1);
            window.draw(r);
        }

        // Draw Nodes (Debug - uncomment to see graph)
        /*
        for(const auto& p : graph.positions) {
            sf::RectangleShape n({2,2});
            n.setPosition(p);
            n.setFillColor(sf::Color(50,50,50));
            window.draw(n);
        }
        */

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