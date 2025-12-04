#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>

// --- SHARED DEFINITIONS ---

enum class ActionType {
    WANDER,
    FLEE_ENEMY,
    SEEK_CENTER,
    ATTACK,
    HIDE,
    CHASE,
    NONE
};

// Represents the "Parameters" of the environment mentioned in HW4
struct WorldState {
    bool enemyNear;      // Is enemy within threat range?
    bool isNearWall;
    bool canSeeEnemy;
    bool canHide;        // Is there a reachable hiding spot nearby?
    
    // Helper to print state for debugging
    std::string toString() const {
        return "EnemyNear:" + std::string(enemyNear ? "T" : "F") + 
               " IsNearWall:" + std::string(isNearWall ? "T" : "F") +
               " CanSeeEnemy:" + std::string(canSeeEnemy ? "T" : "F") +
               " CanHide:" + std::string(canHide ? "T" : "F");
    }
};

// --- DECISION TREE CLASSES ---

class DTNode {
public:
    virtual ~DTNode() = default;
    virtual ActionType makeDecision(const WorldState& state) = 0;
    virtual void print(int depth = 0) = 0;
};

class DTAction : public DTNode {
    ActionType action;
public:
    DTAction(ActionType a) : action(a) {}
    // Fix: Comment out unused parameter name to silence warning
    ActionType makeDecision(const WorldState& /*state*/) override { return action; }
    void print(int depth = 0) override;
};

class DTDecision : public DTNode {
public:
    std::string attribute; // "enemyNear", "energyLow", etc.
    std::unique_ptr<DTNode> trueBranch;
    std::unique_ptr<DTNode> falseBranch;

    DTDecision(std::string attr, std::unique_ptr<DTNode> t, std::unique_ptr<DTNode> f)
        : attribute(attr), trueBranch(std::move(t)), falseBranch(std::move(f)) {}

    ActionType makeDecision(const WorldState& state) override;
    void print(int depth = 0) override;
};