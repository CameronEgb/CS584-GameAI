#include "steering.h"
#include <iostream>
#include <algorithm>
#include <cstdint> 
#include <cmath>

// --- Utilities ---
float mapToRange(float rotation) {
    while (rotation > PI) rotation -= 2.f * PI;
    while (rotation < -PI) rotation += 2.f * PI;
    return rotation;
}

float randomBinomial() {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis(0.f, 1.f);
    return dis(gen) - dis(gen);
}

// --- Breadcrumb ---
// Drop interval reduced to 3 for smoother trails
Breadcrumb::Breadcrumb(int maxCrumbs_, int dropInterval_, sf::Color c)
    : maxCrumbs(maxCrumbs_), dropInterval(3), counter(0), color(c) {}

void Breadcrumb::update(const sf::Vector2f &pos) {
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    std::queue<sf::Vector2f> temp = q;
    // Start brighter and larger
    float alpha = 50.f; 
    float inc = 205.f / std::max(1, (int)q.size()); 
    
    while (!temp.empty()) {
        sf::Vector2f p = temp.front(); temp.pop();
        
        sf::CircleShape dot(3.f); 
        dot.setOrigin({1.5f, 1.5f});
        dot.setPosition(p);
        
        std::uint8_t a = static_cast<std::uint8_t>(std::min(255.f, alpha));
        dot.setFillColor(sf::Color(color.r, color.g, color.b, a));
        
        win.draw(dot);
        alpha += inc;
    }
}

void Breadcrumb::clear() {
    std::queue<sf::Vector2f> empty;
    std::swap(q, empty);
}

// --- Character ---
// Max speed increased 1.5x (150 -> 225)
Character::Character() 
    : breadcrumbs(500, 3, sf::Color::Magenta), currentWaypoint(0), maxSpeed(225.f) 
{
    shape.setPointCount(3);
    shape.setPoint(0, sf::Vector2f(20, 0));
    shape.setPoint(1, sf::Vector2f(-10, 10));
    shape.setPoint(2, sf::Vector2f(-10, -10));
    shape.setFillColor(sf::Color::Cyan);
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1);
    shape.setOrigin({0, 0}); 
}

void Character::setPosition(float x, float y) {
    kinematic.position = {x, y};
    shape.setPosition(kinematic.position);
    breadcrumbs.clear();
}

void Character::setPath(const std::vector<sf::Vector2f>& p) {
    path = p;
    currentWaypoint = 0;
}

// "Arrive" behavior - SPEED TUNED 1.5x
void Character::seek(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = targetPos - kinematic.position;
    float dist = std::hypot(dir.x, dir.y);
    
    const float slowRadius = 150.f; // Start slowing down earlier due to higher speed
    const float stopRadius = 2.f;

    float targetSpeed = maxSpeed;

    if (dist < stopRadius) {
        targetSpeed = 0.f;
        dir = {0,0};
    } else if (dist < slowRadius) {
        targetSpeed = maxSpeed * (dist / slowRadius);
        dir /= dist;
    } else {
        dir /= dist;
    }

    sf::Vector2f targetVelocity = dir * targetSpeed;

    // Linear Acceleration 1.5x (2.0 -> 3.0)
    sf::Vector2f linearAccel = (targetVelocity - kinematic.velocity) * 3.0f; 
    
    kinematic.velocity += linearAccel * dt;
    
    float currentSpeed = std::hypot(kinematic.velocity.x, kinematic.velocity.y);
    if (currentSpeed > maxSpeed) {
        kinematic.velocity = (kinematic.velocity / currentSpeed) * maxSpeed;
    }

    // Align 1.5x (3.0 -> 4.5)
    if (currentSpeed > 10.f) {
        float targetOrient = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        float diff = mapToRange(targetOrient - kinematic.orientation);
        float rotationSpeed = 4.5f; 
        
        if (std::abs(diff) > 0.05f) {
            kinematic.rotation = (diff > 0 ? 1.f : -1.f) * rotationSpeed;
        } else {
            kinematic.rotation = 0.f;
            kinematic.orientation = targetOrient;
        }
    } else {
        kinematic.rotation = 0.f;
    }
}

void Character::flee(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = kinematic.position - targetPos; 
    float dist = std::hypot(dir.x, dir.y);
    if (dist > 0.1f) dir /= dist;

    kinematic.velocity = dir * maxSpeed;
    if (dist > 1.0f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
}

void Character::wander(float dt) {
    kinematic.orientation += randomBinomial() * 6.0f * dt; // Faster wander turn
    kinematic.velocity.x = std::cos(kinematic.orientation) * (maxSpeed * 0.6f);
    kinematic.velocity.y = std::sin(kinematic.orientation) * (maxSpeed * 0.6f);
    kinematic.rotation = 0; 
}

void Character::update(float dt, const Kinematic& /*target*/) {
    // Path Following
    if (!path.empty() && currentWaypoint < path.size()) {
        sf::Vector2f target = path[currentWaypoint];
        sf::Vector2f dir = target - kinematic.position;
        float dist = std::hypot(dir.x, dir.y);
        
        float switchRadius = 30.f; // Larger radius for higher speed
        if (currentWaypoint == path.size() - 1) switchRadius = 2.f;

        if (dist < switchRadius) {
            currentWaypoint++; 
        } else {
            seek(target, dt);
        }
    } else if (!path.empty() && currentWaypoint >= path.size()) {
        kinematic.velocity = {0,0};
        kinematic.rotation = 0;
        path.clear();
    }

    // Physics
    kinematic.position += kinematic.velocity * dt;
    kinematic.orientation += kinematic.rotation * dt;
    kinematic.orientation = mapToRange(kinematic.orientation);

    // Visuals
    breadcrumbs.update(kinematic.position);
    shape.setPosition(kinematic.position);
    shape.setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(shape);
}