#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <random>

const float PI = 3.14159265f;
const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 900;

struct Kinematic {
    sf::Vector2f position;
    float orientation;
    sf::Vector2f velocity;
    float rotation;
    
    Kinematic() : position(0,0), orientation(0), velocity(0,0), rotation(0) {}
};

struct SteeringOutput {
    sf::Vector2f linear;
    float angular;
    SteeringOutput() : linear(0,0), angular(0) {}
};

class Breadcrumb {
    int maxCrumbs;
    int dropInterval;
    int counter;
    sf::Color color;
    std::queue<sf::Vector2f> q;
public:
    Breadcrumb(int maxCrumbs_ = 100, int dropInterval_ = 10, sf::Color c = sf::Color::White);
    void update(const sf::Vector2f& pos);
    void draw(sf::RenderWindow& win);
    void clear(); // Added to satisfy potential calls
};

class Character {
private:
    Kinematic kinematic;
    sf::ConvexShape shape;
    Breadcrumb breadcrumbs;
    std::vector<sf::Vector2f> path;
    int currentWaypoint;
    float maxSpeed;

public:
    Character();
    void setPosition(float x, float y);
    Kinematic getKinematic() const { return kinematic; }
    void setPath(const std::vector<sf::Vector2f>& p);
    
    // Core updates
    void update(float dt, const Kinematic& target);
    void draw(sf::RenderWindow& win);
    
    // Explicit movement commands for AI
    void wander(float dt);
    void seek(sf::Vector2f targetPos, float dt);
    void flee(sf::Vector2f targetPos, float dt);
    void stop();
};

float mapToRange(float rotation);
float randomBinomial();
float randomFloat(float a, float b);