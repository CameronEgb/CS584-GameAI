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

// Simple collision resolution
void resolveCollisions(Character& chara, const std::vector<sf::FloatRect>& walls) {
    Kinematic k = chara.getKinematic();
    sf::FloatRect bounds({k.position.x - 10.f, k.position.y - 10.f}, {20.f, 20.f}); // Approx character size

    for (const auto& w : walls) {
        std::optional<sf::FloatRect> intersection = w.findIntersection(bounds);
        if (intersection) {
            // Simple AABB resolution: push out the shortest way
            if (intersection->size.x < intersection->size.y) {
                // Horizontal push
                if (k.position.x < w.position.x) k.position.x -= intersection->size.x;
                else k.position.x += intersection->size.x;
            } else {
                // Vertical push
                if (k.position.y < w.position.y) k.position.y -= intersection->size.y;
                else k.position.y += intersection->size.y;
            }
        }
    }
    // Update chara
    chara.setPosition(k.position.x, k.position.y);
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
    sf::RenderWindow window(sf::VideoMode({1200, 900}), "HW4: 4 Rooms & Learning");
    window.setFramerateLimit(60);

    // --- ENVIRONMENT ---
    std::vector<sf::FloatRect> walls;
    Graph graph = createFourRoomGraph(walls); 
    
    // --- SETUP ---
    Character chara;
    chara.setPosition(80.f, 80.f); // Safe spawn Room 1

    Kinematic enemy;
    enemy.position = {1100.f, 800.f}; // Room 4
    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(1100.f, 80.f); // Room 2
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);
    goalShape.setOrigin({15.f, 15.f});

    sf::Vector2f stationPos(80.f, 800.f); // Room 3
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);
    stationShape.setOrigin({20.f, 20.f});

    float energy = 100.0f;
    const float THREAT_DIST = 200.0f;
    const float GOAL_DIST = 500.0f; // Sight range

    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;
    float pathUpdateTimer = 0.f;

    std::cout << "--- STARTING SIMULATION ---\n";
    std::cout << "Phase 1: Recording (10s)...\n";

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.1f) dt = 0.1f; // Cap dt for physics stability

        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        // 1. Perception
        WorldState state;
        float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                  chara.getKinematic().position.y - enemy.position.y);
        float dGoal = std::hypot(chara.getKinematic().position.x - goalPos.x,
                                 chara.getKinematic().position.y - goalPos.y);

        state.enemyNear = (dEnemy < THREAT_DIST);
        state.energyLow = (energy < 30.0f);
        state.goalVisible = (dGoal < GOAL_DIST); // Simplified Line-of-Sight

        // 2. Decision
        ActionType action = ActionType::WANDER;
        
        if (mode == RECORDING) {
            action = hardcodedDT->makeDecision(state);
            if ((int)(recordingTimer * 10) % 5 == 0) trainingData.push_back({state, action});
            recordingTimer += dt;
            if (recordingTimer > 10.0f) {
                mode = LEARNING;
                std::cout << "--- LEARNING COMPLETE ---\n";
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

        // 3. Actuation
        pathUpdateTimer -= dt;

        // Pathfinding helper
        auto goTo = [&](sf::Vector2f target) {
            // Recalculate path occasionally
            if (pathUpdateTimer <= 0.f) {
                Metrics m;
                int startNode = graph.getNodeAt(chara.getKinematic().position.x, chara.getKinematic().position.y, 30.f);
                int endNode = graph.getNodeAt(target.x, target.y, 30.f);
                
                if (startNode != -1 && endNode != -1) {
                    // Use A* (reusing pathfinding.cpp logic)
                    std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
                    std::vector<sf::Vector2f> pathPoints;
                    for (int idx : pathIndices) pathPoints.push_back(graph.positions[idx]);
                    chara.setPath(pathPoints);
                    pathUpdateTimer = 1.0f; 
                } else {
                    // Fallback: Seek directly (if start/end invalid)
                    chara.seek(target, dt);
                }
            }
            // Execute movement
            Kinematic dummy; dummy.position = target;
            chara.update(dt, dummy);
        };

        switch(action) {
            case ActionType::FLEE_ENEMY: 
                chara.flee(enemy.position, dt); 
                energy -= 5.f * dt; 
                chara.setPath({}); // Clear path logic
                break;
            case ActionType::SEEK_GOAL: 
                goTo(goalPos); 
                energy -= 2.f * dt; 
                break;
            case ActionType::RECHARGE: 
                goTo(stationPos);
                if (std::hypot(chara.getKinematic().position.x - stationPos.x, chara.getKinematic().position.y - stationPos.y) < 50) 
                    energy += 20.f * dt;
                break;
            case ActionType::WANDER: 
                chara.wander(dt); 
                energy -= 1.f * dt; 
                chara.setPath({});
                break;
            default: break;
        }
        energy = std::max(0.f, std::min(100.f, energy));

        // 4. PHYSICS & COLLISION RESOLUTION (The Fix)
        // This ensures the character does not walk through walls regardless of Behavior/Pathfinding
        resolveCollisions(chara, walls);

        // Simple Enemy Logic (Ghost-like patrol)
        enemy.position.x += (rand()%3 - 1)*1.5f; 
        enemy.position.y += (rand()%3 - 1)*1.5f;
        enemyShape.setPosition(enemy.position);

        // --- DRAW ---
        window.clear(sf::Color(20, 20, 25));
        
        // Draw Walls
        for (const auto& w : walls) {
            sf::RectangleShape r({w.size.x, w.size.y});
            r.setPosition(w.position);
            r.setFillColor(sf::Color(80, 80, 90));
            r.setOutlineColor(sf::Color::White);
            r.setOutlineThickness(1);
            window.draw(r);
        }

        window.draw(enemyShape);
        window.draw(goalShape);
        window.draw(stationShape);
        chara.draw(window);
        window.display();
    }
    return 0;
}