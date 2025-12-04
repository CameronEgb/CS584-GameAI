#include "steering.h"
#include <iostream>
#include <algorithm> // For std::max, std::min
#include <cstdint>   // For std::uint8_t

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
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    std::queue<sf::Vector2f> temp = q;
    float alpha = 50.f;
    float inc = 200.f / std::max(1, maxCrumbs);
    while (!temp.empty()) {
        sf::Vector2f p = temp.front(); temp.pop();
        sf::CircleShape dot(2);
        dot.setPosition(p);
        
        // Fix: Use std::uint8_t instead of sf::Uint8 for SFML 3.0
        dot.setFillColor(sf::Color(color.r, color.g, color.b, static_cast<std::uint8_t>(alpha)));
        
        win.draw(dot);
        alpha = std::min(255.f, alpha + inc);
    }
}

void Breadcrumb::clear() {
    std::queue<sf::Vector2f> empty;
    std::swap(q, empty);
}

// --- Character ---
Character::Character() : breadcrumbs(200, 5, sf::Color::Cyan), currentWaypoint(0), maxSpeed(100.f) {
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
}

void Character::setPath(const std::vector<sf::Vector2f>& p) {
    path = p;
    currentWaypoint = 0;
    breadcrumbs.clear();
}

void Character::stop() {
    kinematic.velocity = {0, 0};
    kinematic.rotation = 0;
}

void Character::seek(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = targetPos - kinematic.position;
    float dist = std::hypot(dir.x, dir.y);
    if (dist > 0.1f) dir /= dist; // Normalize

    kinematic.velocity = dir * maxSpeed;
    
    // Face direction of movement
    if (dist > 1.0f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
    
    update(dt, kinematic); // Apply movement
}

void Character::flee(sf::Vector2f targetPos, float dt) {
    sf::Vector2f dir = kinematic.position - targetPos; // Away from target
    float dist = std::hypot(dir.x, dir.y);
    if (dist > 0.1f) dir /= dist;

    kinematic.velocity = dir * maxSpeed;

    if (dist > 1.0f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
    
    update(dt, kinematic);
}

void Character::wander(float dt) {
    // Simple wander: tweak orientation randomly
    kinematic.orientation += randomBinomial() * 2.0f * dt; 
    
    // Move forward in that direction
    kinematic.velocity.x = std::cos(kinematic.orientation) * (maxSpeed * 0.6f);
    kinematic.velocity.y = std::sin(kinematic.orientation) * (maxSpeed * 0.6f);

    update(dt, kinematic);
}

// Fix: Commented out unused target parameter
void Character::update(float dt, const Kinematic& /*target*/) {
    // 1. Update Position
    kinematic.position += kinematic.velocity * dt;
    kinematic.orientation += kinematic.rotation * dt;

    // 2. Wrap Orientation
    kinematic.orientation = mapToRange(kinematic.orientation);

    // 3. Screen Boundary Handling (Bounce)
    bool hit = false;
    if (kinematic.position.x < 10) { kinematic.position.x = 10; kinematic.velocity.x *= -1; hit=true; }
    if (kinematic.position.x > WINDOW_WIDTH - 10) { kinematic.position.x = WINDOW_WIDTH - 10; kinematic.velocity.x *= -1; hit=true; }
    if (kinematic.position.y < 10) { kinematic.position.y = 10; kinematic.velocity.y *= -1; hit=true; }
    if (kinematic.position.y > WINDOW_HEIGHT - 10) { kinematic.position.y = WINDOW_HEIGHT - 10; kinematic.velocity.y *= -1; hit=true; }
    
    if (hit) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }

    // 4. Update Visuals
    breadcrumbs.update(kinematic.position);
    shape.setPosition(kinematic.position);
    
    // Fix: Use sf::radians to create an sf::Angle for SFML 3.0
    // kinematic.orientation is already in radians
    shape.setRotation(sf::radians(kinematic.orientation));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(shape);
}