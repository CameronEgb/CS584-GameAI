#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include "ai.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <optional>

// Helper to build the HW4 trees
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

    // --- ENVIRONMENT GENERATION ---
    std::vector<sf::FloatRect> walls; // To store wall geometry for drawing
    Graph graph = createFourRoomGraph(walls); // Generates the 4-room graph
    
    // --- SETUP ENTITIES ---
    Character chara;
    // Spawn chara in Top-Left room, safe spot
    chara.setPosition(100, 100); 

    Kinematic enemy;
    enemy.position = {800, 600}; // Bottom-Right room
    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(1100, 100); // Top-Right room
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);
    goalShape.setOrigin({15.f, 15.f});

    sf::Vector2f stationPos(100, 800); // Bottom-Left room
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);
    stationShape.setOrigin({20.f, 20.f});

    // Agent Stats
    float energy = 100.0f;
    const float THREAT_DIST = 200.0f;
    const float GOAL_DIST = 400.0f; // Increased for complex environment

    // AI Components
    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;

    std::cout << "--- STARTING SIMULATION (4 ROOMS) ---\n";
    std::cout << "Phase 1: Recording (10s)...\n";

    // Pathfinding helper state
    std::vector<sf::Vector2f> currentPath;
    float pathUpdateTimer = 0.f;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();
        }

        // 1. Perception
        WorldState state;
        float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                  chara.getKinematic().position.y - enemy.position.y);
        float dGoal = std::hypot(chara.getKinematic().position.x - goalPos.x,
                                 chara.getKinematic().position.y - goalPos.y);

        // Simple line of sight check (raycast) could be added here, 
        // but distance is sufficient for the basic HW requirements.
        state.enemyNear = (dEnemy < THREAT_DIST);
        state.energyLow = (energy < 30.0f);
        state.goalVisible = (dGoal < GOAL_DIST);

        // 2. Decision Making
        ActionType action = ActionType::WANDER;
        
        if (mode == RECORDING) {
            action = hardcodedDT->makeDecision(state);
            if ((int)(recordingTimer * 10) % 5 == 0) // Log less frequently
                trainingData.push_back({state, action});
            
            recordingTimer += dt;
            if (recordingTimer > 10.0f) {
                mode = LEARNING;
                std::cout << "--- LEARNING PHASE ---\n";
            }
        } 
        else if (mode == LEARNING) {
            std::vector<std::string> attrs = {"enemyNear", "energyLow", "goalVisible"};
            learnedDT = ID3Learner::learn(trainingData, attrs);
            std::cout << "Learned Tree:\n";
            learnedDT->print();
            mode = ACTING;
        }
        else if (mode == ACTING && learnedDT) {
            action = learnedDT->makeDecision(state);
        }

        // 3. Actuation (Pathfinding Integration)
        pathUpdateTimer -= dt;
        
        // Helper lambda to calculate path if needed
        auto goTo = [&](sf::Vector2f target) {
            if (pathUpdateTimer <= 0.f) {
                Metrics m;
                // Map positions to nodes
                int startNode = graph.getNodeAt(chara.getKinematic().position.x, chara.getKinematic().position.y, 30.f);
                int endNode = graph.getNodeAt(target.x, target.y, 30.f);
                
                if (startNode != -1 && endNode != -1) {
                    std::vector<int> pathIndices = aStar(graph, startNode, endNode, euclideanHeur, m);
                    std::vector<sf::Vector2f> pathPoints;
                    for (int idx : pathIndices) pathPoints.push_back(graph.positions[idx]);
                    chara.setPath(pathPoints);
                    pathUpdateTimer = 1.0f; // Re-plan every second
                } else {
                    // Fallback if off-mesh: direct seek
                    chara.seek(target, dt);
                }
            }
            // Character handles path following internally if path is set
            // We just need to trigger update
            Kinematic dummyTarget; dummyTarget.position = target;
            chara.update(dt, dummyTarget); 
        };

        switch(action) {
            case ActionType::FLEE_ENEMY: 
                // Simple flee (direct steering away) usually works okay even in rooms
                // unless cornered. Pathfinding away is complex, so we stick to steering.
                chara.flee(enemy.position, dt); 
                energy -= 5.f * dt; 
                chara.setPath({}); // Clear path
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
                chara.setPath({}); // Clear path
                break;
                
            default: break;
        }
        energy = std::max(0.f, std::min(100.f, energy));

        // Enemy AI (Ghost-like, passes through walls for simplicity, or simple bounce)
        // Let's make the enemy respect bounds at least
        if (dEnemy < 300) {
             sf::Vector2f dir = chara.getKinematic().position - enemy.position;
             float len = std::hypot(dir.x, dir.y);
             if (len > 0) dir /= len;
             enemy.position += dir * 70.f * dt;
        } else {
            // Patrol jitter
            enemy.position.x += (rand() % 3 - 1) * 2;
            enemy.position.y += (rand() % 3 - 1) * 2;
        }
        // Clamp enemy to screen
        enemy.position.x = std::max(10.f, std::min(1190.f, enemy.position.x));
        enemy.position.y = std::max(10.f, std::min(890.f, enemy.position.y));
        enemyShape.setPosition(enemy.position);

        // --- DRAW ---
        window.clear(sf::Color(20, 20, 25)); // Dark background
        
        // Draw Walls
        for (const auto& wall : walls) {
            sf::RectangleShape r;
            r.setPosition({wall.position.x, wall.position.y});
            r.setSize({wall.size.x, wall.size.y});
            r.setFillColor(sf::Color(100, 100, 100));
            r.setOutlineColor(sf::Color(150, 150, 150));
            r.setOutlineThickness(1);
            window.draw(r);
        }

        sf::CircleShape ring(THREAT_DIST);
        ring.setOrigin({THREAT_DIST, THREAT_DIST});
        ring.setPosition(chara.getKinematic().position);
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineColor(sf::Color(255, 50, 50, 100));
        ring.setOutlineThickness(2);
        window.draw(ring);

        window.draw(enemyShape);
        window.draw(goalShape);
        window.draw(stationShape);
        chara.draw(window);

        window.display();
    }
    return 0;
}