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
    if (++counter >= dropInterval) {
        counter = 0;
        q.push(pos);
        if ((int)q.size() > maxCrumbs) q.pop();
    }
}

void Breadcrumb::draw(sf::RenderWindow &win) {
    std::queue<sf::Vector2f> temp = q;
    float alpha = 20.f; 
    // Calculate increment to reach ~255 at the head of the trail
    float inc = 235.f / std::max(1, (int)q.size()); 
    
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
// TUNING: Reduced maxCrumbs (100) for faster fade. Increased maxSpeed (300).
Character::Character() 
    : breadcrumbs(100, 4, sf::Color::Magenta), currentWaypoint(0), maxSpeed(150.f), // Adjusted default speed
      wanderOrientation(0.f), wanderOffset(100.f), wanderRadius(50.f), isAttacking(false), attackTimer(0.f)
{
    shape.setPointCount(3);
    shape.setPoint(0, sf::Vector2f(20, 0));
    shape.setPoint(1, sf::Vector2f(-10, 10));
    shape.setPoint(2, sf::Vector2f(-10, -10));
    shape.setFillColor(sf::Color::Cyan);
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1);
    shape.setOrigin({0, 0}); 
    
    // Explicitly zero out velocity
    kinematic.velocity = {0.f, 0.f};
    kinematic.rotation = 0.f;
}

void Character::setPosition(float x, float y) {
    kinematic.position = {x, y};
    shape.setPosition(kinematic.position);
}

void Character::teleport(float x, float y) {
    kinematic.position = {x, y};
    kinematic.velocity = {0.f, 0.f};
    kinematic.rotation = 0.f;
    shape.setPosition(kinematic.position);
    breadcrumbs.clear();
    path.clear();
    isAttacking = false;
    attackTimer = 0.f;
}

void Character::setPath(const std::vector<sf::Vector2f>& p) {
    path = p;
    currentWaypoint = 0;
}

void Character::setMaxSpeed(float speed) {
    maxSpeed = speed;
}

void Character::setColor(sf::Color c) {
    shape.setFillColor(c);
}

void Character::seek(sf::Vector2f targetPos, float dt) {
    if (isAttacking) return;
    sf::Vector2f dir = targetPos - kinematic.position;
    float dist = std::hypot(dir.x, dir.y);
    
    const float slowRadius = 150.f; 
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
    sf::Vector2f linearAccel = (targetVelocity - kinematic.velocity) * 4.0f; // Snappier accel for higher speed
    
    kinematic.velocity += linearAccel * dt;
    
    float currentSpeed = std::hypot(kinematic.velocity.x, kinematic.velocity.y);
    if (currentSpeed > maxSpeed) {
        kinematic.velocity = (kinematic.velocity / currentSpeed) * maxSpeed;
    }

    if (currentSpeed > 10.f) {
        float targetOrient = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        float diff = mapToRange(targetOrient - kinematic.orientation);
        float rotationSpeed = 6.0f; // Faster rotation for higher speed
        
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
    if (isAttacking) return;
    sf::Vector2f dir = kinematic.position - targetPos; 
    float dist = std::hypot(dir.x, dir.y);
    if (dist > 0.1f) dir /= dist;

    // Use seek logic but with max speed to get away
    sf::Vector2f targetVelocity = dir * maxSpeed;
    sf::Vector2f linearAccel = (targetVelocity - kinematic.velocity) * 8.0f; // More responsive flee
    
    kinematic.velocity += linearAccel * dt;
    
    float currentSpeed = std::hypot(kinematic.velocity.x, kinematic.velocity.y);
    if (currentSpeed > maxSpeed) {
        kinematic.velocity = (kinematic.velocity / currentSpeed) * maxSpeed;
    }

    if (dist > 1.0f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
    kinematic.rotation = 0.f;
}

void Character::wander(float dt) {
    if (isAttacking) return;
    // 1. Update the wander orientation
    wanderOrientation += randomBinomial() * 2.0f * dt; // wanderRate

    // 2. Calculate the center of the wander circle
    sf::Vector2f circleCenter = kinematic.position;
    circleCenter.x += wanderOffset * std::cos(kinematic.orientation);
    circleCenter.y += wanderOffset * std::sin(kinematic.orientation);

    // 3. Calculate the target point on the circle
    sf::Vector2f targetPos = circleCenter;
    targetPos.x += wanderRadius * std::cos(wanderOrientation);
    targetPos.y += wanderRadius * std::sin(wanderOrientation);

    // 4. Seek the target
    seek(targetPos, dt);
}

void Character::attack(sf::Vector2f targetPos, float dt) {
    (void)dt;
    if (isAttacking) return;

    stop();
    isAttacking = true;
    attackTimer = 0.5f;

    sf::Vector2f dir = targetPos - kinematic.position;
    if (std::hypot(dir.x, dir.y) > 0.1f) {
        kinematic.orientation = std::atan2(dir.y, dir.x);
    }
}

void Character::stop() {
    kinematic.velocity = {0.f, 0.f};
    kinematic.rotation = 0.f;
}

void Character::update(float dt, const Kinematic& /*target*/) {
    // 0. Path Following
    if (!isAttacking && !path.empty() && currentWaypoint < (int)path.size()) {
        sf::Vector2f target = path[currentWaypoint];
        sf::Vector2f dir = target - kinematic.position;
        float dist = std::hypot(dir.x, dir.y);
        
        float switchRadius = 30.f; 
        if (currentWaypoint == (int)path.size() - 1) switchRadius = 2.f;

        if (dist < switchRadius) {
            currentWaypoint++; 
        } else {
            seek(target, dt);
        }
    } else if (!path.empty() && currentWaypoint >= (int)path.size()) {
        stop();
        path.clear();
    }

    // Handle attack state transition
    if (isAttacking) {
        attackTimer -= dt;
        shape.setFillColor(sf::Color::Yellow);
        if (attackTimer <= 0.f) {
            isAttacking = false;
            shape.setFillColor(sf::Color::Cyan);
        }
    }

    // 1. Update Physics (only if not attacking)
    if (!isAttacking) {
        kinematic.position += kinematic.velocity * dt;
        kinematic.orientation += kinematic.rotation * dt;
        kinematic.orientation = mapToRange(kinematic.orientation);
    }


    // 2. Update Visuals
    breadcrumbs.update(kinematic.position);
    
    shape.setPosition(kinematic.position);
    shape.setRotation(sf::degrees(kinematic.orientation * 180.f / PI));
}

void Character::draw(sf::RenderWindow &win) {
    breadcrumbs.draw(win);
    win.draw(shape);
}

bool Character::isPathComplete() const {
    return path.empty() || currentWaypoint >= (int)path.size();
}