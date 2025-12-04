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

// --- Breadcrumb ---
Breadcrumb::Breadcrumb(int maxCrumbs_, int dropInterval_, sf::Color c)
    : maxCrumbs(maxCrumbs_), dropInterval(dropInterval_), counter(0), color(c) {}

void Breadcrumb::update(const sf::Vector2f &pos) {
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    std::queue<sf::Vector2f> temp = q;
    float alpha = 20.f; 
    float inc = 235.f / std::max(1, (int)q.size()); 
    
    while (!temp.empty()) {
        sf::Vector2f p = temp.front(); temp.pop();
        
        sf::CircleShape dot(3.f); 
        dot.setOrigin({1.5f, 1.5f});
        dot.setPosition(p);
        
        // FIX: Use std::uint8_t instead of sf::Uint8
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
Character::Character() 
    : breadcrumbs(300, 3, sf::Color::Magenta), currentWaypoint(0), maxSpeed(150.f) 
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

void Character::seek(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = targetPos - kinematic.position;
    float dist = std::hypot(dir.x, dir.y);
    if (dist > 5.f) dir /= dist; 
    else dir = {0,0}; 

    kinematic.velocity = dir * maxSpeed;
    
    if (dist > 5.f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
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
    kinematic.orientation += randomBinomial() * 4.0f * dt; 
    kinematic.velocity.x = std::cos(kinematic.orientation) * (maxSpeed * 0.5f);
    kinematic.velocity.y = std::sin(kinematic.orientation) * (maxSpeed * 0.5f);
}

void Character::update(float dt, const Kinematic& /*target*/) {
    // 0. Path Following
    if (!path.empty() && currentWaypoint < path.size()) {
        sf::Vector2f target = path[currentWaypoint];
        sf::Vector2f dir = target - kinematic.position;
        float dist = std::hypot(dir.x, dir.y);
        
        if (dist < 15.f) {
            currentWaypoint++; 
        } else {
            if (dist > 0.01f) dir /= dist;
            kinematic.velocity = dir * maxSpeed;
            kinematic.orientation = std::atan2(dir.y, dir.x);
        }
    } else if (!path.empty() && currentWaypoint >= path.size()) {
        kinematic.velocity = {0,0};
        path.clear();
    }

    // 1. Update Physics
    kinematic.position += kinematic.velocity * dt;
    kinematic.orientation += kinematic.rotation * dt;
    kinematic.orientation = mapToRange(kinematic.orientation);

    // 2. Update Visuals
    breadcrumbs.update(kinematic.position);
    
    shape.setPosition(kinematic.position);
    // FIX: Use sf::radians for SFML 3.0 rotation
    shape.setRotation(sf::radians(kinematic.orientation));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(shape);
}