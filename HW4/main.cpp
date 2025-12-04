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
void resolveCollisions(Character& chara, const std::vector<sf::FloatRect>& walls) {
    Kinematic k = chara.getKinematic();
    float r = 10.f; 
    sf::FloatRect agentBounds({k.position.x - r, k.position.y - r}, {r * 2.f, r * 2.f});

    for (const auto& w : walls) {
        std::optional<sf::FloatRect> intersection = w.findIntersection(agentBounds);
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
    sf::RenderWindow window(sf::VideoMode({1200, 900}), "HW4: Scaled Arena");
    window.setFramerateLimit(60);

    // --- ENVIRONMENT ---
    std::vector<sf::FloatRect> walls;
    Graph graph = createFourRoomGraph(walls); 
    
    // --- SETUP ENTITIES (Within 800x600 Arena) ---
    // Arena range: X[200, 1000], Y[150, 750]
    Character chara;
    chara.setPosition(250.f, 200.f); // Top-Left Room

    Kinematic enemy;
    enemy.position = {900.f, 650.f}; // Bottom-Right Room
    sf::RectangleShape enemyShape(sf::Vector2f(30.f, 30.f));
    enemyShape.setFillColor(sf::Color::Red);
    enemyShape.setOrigin({15.f, 15.f});

    sf::Vector2f goalPos(900.f, 200.f); // Top-Right Room
    sf::CircleShape goalShape(15);
    goalShape.setFillColor(sf::Color::Green);
    goalShape.setPosition(goalPos);
    goalShape.setOrigin({15.f, 15.f});

    sf::Vector2f stationPos(250.f, 650.f); // Bottom-Left Room
    sf::RectangleShape stationShape(sf::Vector2f(40.f, 40.f));
    stationShape.setFillColor(sf::Color::Blue);
    stationShape.setPosition(stationPos);
    stationShape.setOrigin({20.f, 20.f});

    float energy = 100.0f;
    const float THREAT_DIST = 150.0f;
    const float GOAL_DIST = 300.0f;

    auto hardcodedDT = buildHardcodedDT();
    std::unique_ptr<DTNode> learnedDT = nullptr;
    std::vector<TrainingExample> trainingData;

    sf::Clock clock;
    enum Mode { RECORDING, LEARNING, ACTING, MANUAL };
    Mode mode = RECORDING;
    float recordingTimer = 0.f;
    float pathUpdateTimer = 0.f;

    std::cout << "--- STARTING ---" << std::endl;

    auto planPathTo = [&](sf::Vector2f target) {
        Metrics m;
        // Check if click is inside arena bounds first
        if (target.x < 200 || target.x > 1000 || target.y < 150 || target.y > 750) return;

        int startNode = graph.getNodeAt(chara.getKinematic().position.x, chara.getKinematic().position.y, 25.f);
        int endNode = graph.getNodeAt(target.x, target.y, 25.f);
        
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
                if ((int)(recordingTimer * 10) % 5 == 0) trainingData.push_back({state, action});
                recordingTimer += dt;
                if (recordingTimer > 10.0f) {
                    mode = LEARNING;
                    std::cout << "--- LEARNING ---" << std::endl;
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

        Kinematic dummy; 
        chara.update(dt, dummy);
        energy = std::max(0.f, std::min(100.f, energy));
        resolveCollisions(chara, walls);

        // Enemy within Arena
        if (dEnemy < 300) {
            sf::Vector2f dir = chara.getKinematic().position - enemy.position;
            float len = std::hypot(dir.x, dir.y);
            if (len > 0) dir /= len;
            enemy.position += dir * 75.f * dt;
        } else {
            enemy.position.x += (rand()%3 - 1)*1.5f; 
            enemy.position.y += (rand()%3 - 1)*1.5f;
        }
        // Clamp Enemy to Arena
        if(enemy.position.x < 210) enemy.position.x = 210;
        if(enemy.position.x > 990) enemy.position.x = 990;
        if(enemy.position.y < 160) enemy.position.y = 160;
        if(enemy.position.y > 740) enemy.position.y = 740;
        enemyShape.setPosition(enemy.position);

        window.clear(sf::Color(20, 20, 25));
        
        for (const auto& w : walls) {
            sf::RectangleShape r({w.size.x, w.size.y});
            r.setPosition(w.position);
            r.setFillColor(sf::Color(100, 100, 110));
            r.setOutlineColor(sf::Color(200, 200, 200));
            r.setOutlineThickness(1);
            window.draw(r);
        }

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