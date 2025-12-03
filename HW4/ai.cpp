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
    else if (attribute == "energyLow") val = state.energyLow;
    else if (attribute == "goalVisible") val = state.goalVisible;

    if (val) return trueBranch->makeDecision(state);
    else return falseBranch->makeDecision(state);
}

void DTDecision::print(int depth) {
    for(int i=0; i<depth; ++i) std::cout << "  ";
    std::cout << "?" << attribute << "\n";
    trueBranch->print(depth + 1);
    falseBranch->print(depth + 1);
}

// --- BEHAVIOR TREE IMPLEMENTATION ---

BTStatus BTCondition::tick(const WorldState& state) {
    bool val = false;
    if (attribute == "enemyNear") val = state.enemyNear;
    else if (attribute == "energyLow") val = state.energyLow;
    else if (attribute == "goalVisible") val = state.goalVisible;

    return (val == expectedValue) ? BTStatus::SUCCESS : BTStatus::FAILURE;
}

BTStatus BTSelector::tick(const WorldState& state) {
    for (auto& child : children) {
        if (child->tick(state) != BTStatus::FAILURE) return BTStatus::SUCCESS;
    }
    return BTStatus::FAILURE;
}

BTStatus BTSequence::tick(const WorldState& state) {
    for (auto& child : children) {
        if (child->tick(state) != BTStatus::SUCCESS) return BTStatus::FAILURE;
    }
    return BTStatus::SUCCESS;
}

// --- ID3 LEARNING IMPLEMENTATION ---

// Helper to calculate entropy of a set of examples
double calculateEntropy(const std::vector<TrainingExample>& examples) {
    if (examples.empty()) return 0.0;
    std::map<ActionType, int> counts;
    for (const auto& ex : examples) counts[ex.action]++;

    double entropy = 0.0;
    for (auto const& [action, count] : counts) {
        double p = (double)count / examples.size();
        entropy -= p * std::log2(p);
    }
    return entropy;
}

// Helper to filter examples based on attribute value
std::vector<TrainingExample> filterExamples(const std::vector<TrainingExample>& examples, 
                                            const std::string& attr, bool val) {
    std::vector<TrainingExample> subset;
    for (const auto& ex : examples) {
        bool prop = false;
        if (attr == "enemyNear") prop = ex.state.enemyNear;
        else if (attr == "energyLow") prop = ex.state.energyLow;
        else if (attr == "goalVisible") prop = ex.state.goalVisible;

        if (prop == val) subset.push_back(ex);
    }
    return subset;
}

std::unique_ptr<DTNode> ID3Learner::learn(const std::vector<TrainingExample>& examples, 
                                          const std::vector<std::string>& attributes) {
    // 1. If all examples have the same classification, return Leaf
    bool allSame = true;
    ActionType firstAction = examples[0].action;
    for (const auto& ex : examples) {
        if (ex.action != firstAction) { allSame = false; break; }
    }
    if (allSame) return std::make_unique<DTAction>(firstAction);

    // 2. If attributes is empty, return Leaf with majority class
    if (attributes.empty()) {
        std::map<ActionType, int> counts;
        for (const auto& ex : examples) counts[ex.action]++;
        ActionType bestAction = ActionType::NONE;
        int maxCount = -1;
        for (auto p : counts) {
            if (p.second > maxCount) { maxCount = p.second; bestAction = p.first; }
        }
        return std::make_unique<DTAction>(bestAction);
    }

    // 3. Find best attribute (Information Gain)
    double baseEntropy = calculateEntropy(examples);
    std::string bestAttr;
    double maxGain = -1.0;

    for (const auto& attr : attributes) {
        auto trueSet = filterExamples(examples, attr, true);
        auto falseSet = filterExamples(examples, attr, false);
        
        double trueEntropy = calculateEntropy(trueSet);
        double falseEntropy = calculateEntropy(falseSet);
        
        double remainder = ((double)trueSet.size()/examples.size()) * trueEntropy + 
                           ((double)falseSet.size()/examples.size()) * falseEntropy;
        double gain = baseEntropy - remainder;

        if (gain > maxGain) {
            maxGain = gain;
            bestAttr = attr;
        }
    }

    // 4. Split and Recurse
    auto trueExamples = filterExamples(examples, bestAttr, true);
    auto falseExamples = filterExamples(examples, bestAttr, false);

    // Handle edge case where a split is empty (use majority of parent)
    if (trueExamples.empty() || falseExamples.empty()) {
         std::map<ActionType, int> counts;
        for (const auto& ex : examples) counts[ex.action]++;
        ActionType bestAction = ActionType::NONE;
        int maxCount = -1;
        for (auto p : counts) {
            if (p.second > maxCount) { maxCount = p.second; bestAction = p.first; }
        }
        return std::make_unique<DTAction>(bestAction);
    }

    std::vector<std::string> remainingAttrs;
    for (const auto& a : attributes) if (a != bestAttr) remainingAttrs.push_back(a);

    return std::make_unique<DTDecision>(
        bestAttr,
        learn(trueExamples, remainingAttrs),
        learn(falseExamples, remainingAttrs)
    );
}