#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <iostream>

// --- SHARED DEFINITIONS ---

enum class ActionType {
    WANDER,
    SEEK_GOAL,
    FLEE_ENEMY,
    RECHARGE,
    SEEK_CENTER,
    NONE
};

// Represents the "Parameters" of the environment mentioned in HW4
struct WorldState {
    bool enemyNear;      // Is enemy within threat range?
    bool energyLow;      // Is energy < threshold?
    bool goalVisible;    // Do we have line of sight to goal? (simplified to distance for now)
    bool isAtMaxSpeed;
    bool isNearWall;
    bool isMonsterNear;
    
    // Helper to print state for debugging
    std::string toString() const {
        return "Enemy:" + std::string(enemyNear ? "T" : "F") + 
               " EnergyLow:" + std::string(energyLow ? "T" : "F") + 
               " GoalVis:" + std::string(goalVisible ? "T" : "F") +
               " AtMaxSpeed:" + std::string(isAtMaxSpeed ? "T" : "F") +
               " NearWall:" + std::string(isNearWall ? "T" : "F") +
               " MonsterNear:" + std::string(isMonsterNear ? "T" : "F");
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

// --- BEHAVIOR TREE CLASSES ---

enum class BTStatus { SUCCESS, FAILURE, RUNNING };

class BTNode {
public:
    virtual ~BTNode() = default;
    virtual BTStatus tick(const WorldState& state) = 0;
};

class BTAction : public BTNode {
    ActionType action;
public:
    BTAction(ActionType a) : action(a) {}
    // Fix: Comment out unused parameter name to silence warning
    BTStatus tick(const WorldState& /*state*/) override {
        // Actions in this simple simulation are instantaneous command selections
        return BTStatus::SUCCESS; 
    }
    ActionType getAction() const { return action; }
};

class BTCondition : public BTNode {
    std::string attribute;
    bool expectedValue;
public:
    BTCondition(std::string attr, bool expected) : attribute(attr), expectedValue(expected) {}
    BTStatus tick(const WorldState& state) override;
};

class BTSelector : public BTNode {
    std::vector<std::unique_ptr<BTNode>> children;
public:
    void addChild(std::unique_ptr<BTNode> c) { children.push_back(std::move(c)); }
    BTStatus tick(const WorldState& state) override;
};

class BTSequence : public BTNode {
    std::vector<std::unique_ptr<BTNode>> children;
public:
    void addChild(std::unique_ptr<BTNode> c) { children.push_back(std::move(c)); }
    BTStatus tick(const WorldState& state) override;
};

// --- LEARNING (ID3) ---

struct TrainingExample {
    WorldState state;
    ActionType action;
};

class ID3Learner {
public:
    // Takes a dataset and returns the root of a learned Decision Tree
    static std::unique_ptr<DTNode> learn(const std::vector<TrainingExample>& examples, 
                                         const std::vector<std::string>& attributes);
};