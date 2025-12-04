#include "steering.h"
#include <iostream>
#include <algorithm>
#include <cstdint> // Required for std::uint8_t
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

float randomFloat(float a, float b) {
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dis(a, b);
    return dis(gen);
}

// --- Breadcrumb ---
Breadcrumb::Breadcrumb(int maxCrumbs_, int dropInterval_, sf::Color c)
    : maxCrumbs(maxCrumbs_), dropInterval(dropInterval_), counter(0), color(c) {}

void Breadcrumb::update(const sf::Vector2f &pos) {
    // Drop a crumb based on interval
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    // Copy queue to iterate (std::queue doesn't support iteration)
    std::queue<sf::Vector2f> temp = q;
    
    // Alpha ramp logic: Oldest crumbs are faint, newest are bright
    float alpha = 20.f; 
    float inc = 235.f / std::max(1, (int)q.size()); 
    
    while (!temp.empty()) {
        sf::Vector2f p = temp.front(); temp.pop();
        
        sf::CircleShape dot(3.f); // Size 3 for visibility
        dot.setOrigin({1.5f, 1.5f});
        dot.setPosition(p);
        
        // SFML 3.0 requires std::uint8_t for color components
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
// Initialize with visible Magenta crumbs, dropping every 3 frames
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
    // We do NOT clear breadcrumbs here so you can see the trail of where it came from
}

// "Arrive" behavior - Tuned for smoother stops
void Character::seek(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = targetPos - kinematic.position;
    float dist = std::hypot(dir.x, dir.y);
    
    const float slowRadius = 150.f; // Start slowing down
    const float stopRadius = 2.f;   // Snap to target

    float targetSpeed = maxSpeed;

    if (dist < stopRadius) {
        targetSpeed = 0.f;
        dir = {0,0};
    } else if (dist < slowRadius) {
        // Linearly ramp down speed
        targetSpeed = maxSpeed * (dist / slowRadius);
        dir /= dist;
    } else {
        dir /= dist;
    }

    sf::Vector2f targetVelocity = dir * targetSpeed;

    // Linear Acceleration (3.0f is the "snappiness" factor)
    sf::Vector2f linearAccel = (targetVelocity - kinematic.velocity) * 3.0f; 
    
    kinematic.velocity += linearAccel * dt;
    
    // Clamp velocity
    float currentSpeed = std::hypot(kinematic.velocity.x, kinematic.velocity.y);
    if (currentSpeed > maxSpeed) {
        kinematic.velocity = (kinematic.velocity / currentSpeed) * maxSpeed;
    }

    // Smooth Align (Look where you are going)
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

    // Flee is usually instant panic (max speed immediately)
    kinematic.velocity = dir * maxSpeed;
    
    if (dist > 1.0f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
    
    // Reset rotation as we set orientation directly
    kinematic.rotation = 0.f;
}

void Character::wander(float dt) {
    // Random binomial gives a value between -1 and 1 biased towards 0
    kinematic.orientation += randomBinomial() * 6.0f * dt; 
    
    kinematic.velocity.x = std::cos(kinematic.orientation) * (maxSpeed * 0.6f);
    kinematic.velocity.y = std::sin(kinematic.orientation) * (maxSpeed * 0.6f);
    kinematic.rotation = 0.f; 
}

void Character::stop() {
    kinematic.velocity = {0.f, 0.f};
    kinematic.rotation = 0.f;
}

void Character::update(float dt, const Kinematic& /*target*/) {
    // 0. Handle Path Following Logic
    if (!path.empty() && currentWaypoint < (int)path.size()) {
        sf::Vector2f target = path[currentWaypoint];
        sf::Vector2f dir = target - kinematic.position;
        float dist = std::hypot(dir.x, dir.y);
        
        // Switching radius
        float switchRadius = 30.f; 
        if (currentWaypoint == (int)path.size() - 1) switchRadius = 2.f;

        if (dist < switchRadius) {
            currentWaypoint++; 
        } else {
            seek(target, dt);
        }
    } else if (!path.empty() && currentWaypoint >= (int)path.size()) {
        // Path complete
        stop();
        path.clear();
    }

    // 1. Update Physics
    kinematic.position += kinematic.velocity * dt;
    kinematic.orientation += kinematic.rotation * dt;
    kinematic.orientation = mapToRange(kinematic.orientation);

    // 2. Update Visuals
    // IMPORTANT: Breadcrumbs update here
    breadcrumbs.update(kinematic.position);
    
    shape.setPosition(kinematic.position);
    // SFML 3.0 uses sf::radians/sf::degrees
    shape.setRotation(sf::radians(kinematic.orientation));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(shape);
}