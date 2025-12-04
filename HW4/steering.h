#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <random>

const float PI = 3.14159265f;

// Centralized Window Dimensions
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

struct Kinematic {
    sf::Vector2f position;
    float orientation;
    sf::Vector2f velocity;
    float rotation;
    
    Kinematic() : position(0,0), orientation(0), velocity(0,0), rotation(0) {}

    float getSpeed() const {
        return std::hypot(velocity.x, velocity.y);
    }
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
    Breadcrumb(int maxCrumbs_ = 200, int dropInterval_ = 5, sf::Color c = sf::Color::White);
    void update(const sf::Vector2f& pos);
    void draw(sf::RenderWindow& win);
    void clear();
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
    // Updates physics position only (preserves history)
    void setPosition(float x, float y);
    // Fully resets character state (clears history & velocity)
    void teleport(float x, float y);
    
    Kinematic getKinematic() const { return kinematic; }
    void setPath(const std::vector<sf::Vector2f>& p);
    
    void update(float dt, const Kinematic& target);
    void draw(sf::RenderWindow& win);
    
    void wander(float dt);
    void seek(sf::Vector2f targetPos, float dt);
    void flee(sf::Vector2f targetPos, float dt);
    void stop();
};

float mapToRange(float rotation);
float randomBinomial();
float randomFloat(float a, float b);