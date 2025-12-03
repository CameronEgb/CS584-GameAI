#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include "ai.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

// Simulation Constants
const int GRID_SIZE = 40;
const int CELL_W = WINDOW_WIDTH / GRID_SIZE;
const int CELL_H = WINDOW_HEIGHT / GRID_SIZE;

// --- Helper to build specific trees ---

// 1. Hardcoded Decision Tree (for testing/generating data)
// Rules:
// IF EnemyNear -> FLEE
// ELSE IF EnergyLow -> RECHARGE
// ELSE IF GoalVisible -> SEEK_GOAL
// ELSE -> WANDER
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

// 2. Behavior Tree (equivalent logic)
std::unique_ptr<BTNode> buildBehaviorTree() {
    auto root = std::make_unique<BTSelector>();

    // Priority 1: Flee Enemy
    auto fleeSeq = std::make_unique<BTSequence>();
    fleeSeq->addChild(std::make_unique<BTCondition>("enemyNear", true));
    fleeSeq->addChild(std::make_unique<BTAction>(ActionType::FLEE_ENEMY));
    root->addChild(std::move(fleeSeq));

    // Priority 2: Recharge
    auto rechargeSeq = std::make_unique<BTSequence>();
    rechargeSeq->addChild(std::make_unique<BTCondition>("energyLow", true));
    rechargeSeq->addChild(std::make_unique<BTAction>(ActionType::RECHARGE));
    root->addChild(std::move(rechargeSeq));

    // Priority 3: Seek Goal
    auto seekSeq = std::make_unique<BTSequence>();
    seekSeq->addChild(std::make_unique<BTCondition>("goalVisible", true));
    seekSeq->addChild(std::make_unique<BTAction>(ActionType::SEEK_GOAL));
    root->addChild(std::move(seekSeq));

    // Default: Wander
    root->addChild(std::make_unique<BTAction>(ActionType::WANDER));
    return root;
}

int main() {
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "HW4: Learning & Behavior");
    window.setFramerateLimit(60);

    // --- SETUP ENVIRONMENT ---
    Character chara;
    chara.setPosition(100, 100);

    Kinematic enemy;
    enemy.position = {800, 600};
    sf::RectangleShape enemyShape(sf::Vector2f(30, 30));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin(15, 15);

    sf::Vector2f goalPos(1100, 800);
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);

    sf::Vector2f stationPos(100, 800);
    sf::RectangleShape stationShape(sf::Vector2f(40, 40));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);

    // Agent Stats
    float energy = 100.0f;
    const float THREAT_DIST = 200.0f;
    const float GOAL_DIST = 300.0f;

    // AI Components
    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;

    std::cout << "--- STARTING SIMULATION ---\n";
    std::cout << "Phase 1: Recording data using Hardcoded Rules (10 seconds)...\n";

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        // 1. Perception (Build WorldState)
        WorldState state;
        float dEnemy = std::hypot(chara.getKinematic().position.x - enemy.position.x, 
                                  chara.getKinematic().position.y - enemy.position.y);
        float dGoal = std::hypot(chara.getKinematic().position.x - goalPos.x,
                                 chara.getKinematic().position.y - goalPos.y);

        state.enemyNear = (dEnemy < THREAT_DIST);
        state.energyLow = (energy < 30.0f);
        state.goalVisible = (dGoal < GOAL_DIST); // Simplified Line-of-sight

        // 2. Decision Making
        ActionType action = ActionType::WANDER;

        if (mode == RECORDING) {
            action = hardcodedDT->makeDecision(state);
            
            // Log data
            if ((int)(recordingTimer * 10) % 2 == 0) { // Log occasionally to avoid spam
                trainingData.push_back({state, action});
            }
            
            recordingTimer += dt;
            if (recordingTimer > 10.0f) {
                mode = LEARNING;
                std::cout << "--- RECORDING COMPLETE ---\n";
                std::cout << "Collected " << trainingData.size() << " examples.\n";
            }
        } 
        else if (mode == LEARNING) {
            std::cout << "--- LEARNING PHASE ---\n";
            std::vector<std::string> attrs = {"enemyNear", "energyLow", "goalVisible"};
            learnedDT = ID3Learner::learn(trainingData, attrs);
            
            std::cout << "Learned Tree Structure:\n";
            learnedDT->print();
            
            mode = ACTING;
            std::cout << "--- SWITCHING TO LEARNED TREE ---\n";
        }
        else if (mode == ACTING) {
            if (learnedDT) action = learnedDT->makeDecision(state);
        }

        // 3. Actuation (Apply Action)
        switch(action) {
            case ActionType::FLEE_ENEMY:
                chara.flee(enemy.position, dt);
                energy -= 5.0f * dt;
                break;
            case ActionType::SEEK_GOAL:
                chara.seek(goalPos, dt);
                energy -= 2.0f * dt;
                break;
            case ActionType::RECHARGE:
                chara.seek(stationPos, dt); // Move to station
                if (std::hypot(chara.getKinematic().position.x - stationPos.x, 
                               chara.getKinematic().position.y - stationPos.y) < 50) {
                    energy += 20.0f * dt;
                }
                break;
            case ActionType::WANDER:
                chara.wander(dt);
                energy -= 1.0f * dt;
                break;
            default: break;
        }

        // Update Stats
        if (energy > 100) energy = 100;
        if (energy < 0) energy = 0;

        // Simple Enemy Movement (Wander + Chase if close)
        if (dEnemy < 300) {
             // Chase player
             sf::Vector2f dir = chara.getKinematic().position - enemy.position;
             float len = std::hypot(dir.x, dir.y);
             if (len > 0) dir /= len;
             enemy.position += dir * 80.0f * dt;
        } else {
             // Jitter
             enemy.position.x += (rand() % 3 - 1) * 2;
             enemy.position.y += (rand() % 3 - 1) * 2;
        }

        // Handling (keep chara in bounds is handled in class, but update enemy)
        if (enemy.position.x < 0) enemy.position.x = 0;
        if (enemy.position.x > WINDOW_WIDTH) enemy.position.x = WINDOW_WIDTH;
        if (enemy.position.y < 0) enemy.position.y = 0;
        if (enemy.position.y > WINDOW_HEIGHT) enemy.position.y = WINDOW_HEIGHT;
        enemyShape.setPosition(enemy.position);

        // --- DRAW ---
        window.clear(sf::Color(30, 30, 30));
        
        // Draw Zones
        sf::CircleShape perceptionRing(THREAT_DIST);
        perceptionRing.setOrigin(THREAT_DIST, THREAT_DIST);
        perceptionRing.setPosition(chara.getKinematic().position);
        perceptionRing.setFillColor(sf::Color::Transparent);
        perceptionRing.setOutlineColor(sf::Color(255, 0, 0, 50));
        perceptionRing.setOutlineThickness(2);
        window.draw(perceptionRing);

        window.draw(enemyShape);
        window.draw(goalShape);
        window.draw(stationShape);
        chara.draw(window);

        // GUI Text (Energy)
        // (Simple Console Output for status)
        // static float printTimer = 0;
        // printTimer += dt;
        // if (printTimer > 1.0f) {
        //     std::cout << "Energy: " << (int)energy << " Action: " << (int)action << "\n";
        //     printTimer = 0;
        // }

        window.display();
    }
    return 0;
}