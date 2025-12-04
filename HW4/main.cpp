#include "graph.h"
#include "pathfinding.h"
#include "steering.h"
#include "ai.h"
#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>
#include <optional>

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
    sf::RenderWindow window(sf::VideoMode({1200, 900}), "HW4: Learning & Behavior");
    window.setFramerateLimit(60);

    // --- SETUP ---
    Character chara;
    chara.setPosition(100, 100);

    Kinematic enemy;
    enemy.position = {800, 600};
    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(1100, 800);
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);

    sf::Vector2f stationPos(100, 800);
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);

    float energy = 100.0f;
    const float THREAT_DIST = 200.0f;
    const float GOAL_DIST = 300.0f;

    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;

    std::cout << "--- STARTING SIMULATION ---\n";
    std::cout << "Phase 1: Recording (10s)...\n";

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

        state.enemyNear = (dEnemy < THREAT_DIST);
        state.energyLow = (energy < 30.0f);
        state.goalVisible = (dGoal < GOAL_DIST);

        // 2. Decision
        ActionType action = ActionType::WANDER;
        if (mode == RECORDING) {
            action = hardcodedDT->makeDecision(state);
            if ((int)(recordingTimer * 10) % 2 == 0) trainingData.push_back({state, action});
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

        // 3. Act
        switch(action) {
            case ActionType::FLEE_ENEMY: chara.flee(enemy.position, dt); energy -= 5.f * dt; break;
            case ActionType::SEEK_GOAL: chara.seek(goalPos, dt); energy -= 2.f * dt; break;
            case ActionType::RECHARGE: 
                chara.seek(stationPos, dt); 
                if (std::hypot(chara.getKinematic().position.x - stationPos.x, chara.getKinematic().position.y - stationPos.y) < 50) energy += 20.f * dt;
                break;
            case ActionType::WANDER: chara.wander(dt); energy -= 1.f * dt; break;
            default: break;
        }
        energy = std::max(0.f, std::min(100.f, energy));

        // Enemy AI
        if (dEnemy < 300) {
             sf::Vector2f dir = chara.getKinematic().position - enemy.position;
             float len = std::hypot(dir.x, dir.y);
             if (len > 0) dir /= len;
             enemy.position += dir * 80.f * dt;
        }
        enemyShape.setPosition(enemy.position);

        window.clear(sf::Color(30, 30, 30));
        
        sf::CircleShape ring(THREAT_DIST);
        ring.setOrigin({THREAT_DIST, THREAT_DIST});
        ring.setPosition(chara.getKinematic().position);
        ring.setFillColor(sf::Color::Transparent);
        ring.setOutlineColor(sf::Color(255, 0, 0, 50));
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