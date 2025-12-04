#pragma once
#include "steering.h" // For Kinematic
#include "graph.h"    // For Graph
#include <vector>
#include <memory>
#include <functional>
#include <SFML/Graphics.hpp>

struct EnemyContext {
    Character& enemy;
    const Kinematic& player;
    const std::vector<sf::FloatRect>& walls;
    const Graph& graph;
    float dt;
};

enum class BTStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

class BTNode {
public:
    virtual ~BTNode() = default;
    virtual BTStatus tick(EnemyContext& ctx) = 0;
};

class BTComposite : public BTNode {
protected:
    std::vector<std::unique_ptr<BTNode>> children;
public:
    void addChild(std::unique_ptr<BTNode> child) {
        children.push_back(std::move(child));
    }
};

class BTSelector : public BTComposite {
public:
    BTStatus tick(EnemyContext& ctx) override;
};

class BTSequence : public BTComposite {
public:
    BTStatus tick(EnemyContext& ctx) override;
};

class BTAction : public BTNode {
    std::function<BTStatus(EnemyContext&)> actionFn;
public:
    BTAction(std::function<BTStatus(EnemyContext&)> fn) : actionFn(fn) {}
    BTStatus tick(EnemyContext& ctx) override {
        return actionFn(ctx);
    }
};

class BTCondition : public BTNode {
    std::function<bool(EnemyContext&)> predicate;
public:
    BTCondition(std::function<bool(EnemyContext&)> p) : predicate(p) {}
    BTStatus tick(EnemyContext& ctx) override {
        return predicate(ctx) ? BTStatus::SUCCESS : BTStatus::FAILURE;
    }
};
