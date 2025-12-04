#include "ai.h"
#include <cmath>
#include <algorithm>
#include <map>

// --- DECISION TREE IMPLEMENTATION ---

void DTAction::print(int depth) {
    for(int i=0; i<depth; ++i) std::cout << "  ";
    std::cout << "-> ACTION: " << (int)action << "\n";
}

ActionType DTDecision::makeDecision(const WorldState& state) {
    bool val = false;
    if (attribute == "enemyNear") val = state.enemyNear;
    else if (attribute == "isNearWall") val = state.isNearWall;

    if (val) return trueBranch->makeDecision(state);
    else return falseBranch->makeDecision(state);
}

void DTDecision::print(int depth) {
    for(int i=0; i<depth; ++i) std::cout << "  ";
    std::cout << "?" << attribute << "\n";
    trueBranch->print(depth + 1);
    falseBranch->print(depth + 1);
}