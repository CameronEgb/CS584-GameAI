#include "dt_learner.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <iostream>
#include <set>

struct Example {
    WorldState state;
    ActionType action;
};

// Helper to parse CSV line
Example parseLine(const std::string& line) {
    std::stringstream ss(line);
    std::string segment;
    std::vector<std::string> seglist;
    while(std::getline(ss, segment, ',')) {
        seglist.push_back(segment);
    }
    Example ex;
    // Default values if parsing fails
    ex.state.enemyNear = false;
    ex.state.isNearWall = false;
    ex.state.canSeeEnemy = false;
    ex.action = ActionType::NONE;

    if (seglist.size() >= 4) {
        ex.state.enemyNear = (seglist[0] == "1");
        ex.state.isNearWall = (seglist[1] == "1");
        ex.state.canSeeEnemy = (seglist[2] == "1");
        try {
            ex.action = (ActionType)std::stoi(seglist[3]);
        } catch (...) {}
    }
    return ex;
}

// Entropy calculation
double calculateEntropy(const std::vector<Example>& examples) {
    std::map<ActionType, int> counts;
    for (const auto& ex : examples) counts[ex.action]++;
    
    double entropy = 0.0;
    double total = examples.size();
    for (auto const& [action, count] : counts) {
        double p = count / total;
        if (p > 0) entropy -= p * std::log2(p);
    }
    return entropy;
}

// Split examples based on attribute
std::pair<std::vector<Example>, std::vector<Example>> split(const std::vector<Example>& examples, const std::string& attr) {
    std::vector<Example> trueSet, falseSet;
    for (const auto& ex : examples) {
        bool val = false;
        if (attr == "enemyNear") val = ex.state.enemyNear;
        else if (attr == "isNearWall") val = ex.state.isNearWall;
        else if (attr == "canSeeEnemy") val = ex.state.canSeeEnemy;
        
        if (val) trueSet.push_back(ex);
        else falseSet.push_back(ex);
    }
    return {trueSet, falseSet};
}

// Helper for majority vote
std::unique_ptr<DTNode> getMajority(const std::vector<Example>& examples) {
    std::map<ActionType, int> counts;
    for (const auto& ex : examples) counts[ex.action]++;
    ActionType best = ActionType::NONE; 
    int maxC = -1;
    for(auto [a, c] : counts) {
        if(c > maxC) { maxC = c; best = a; }
    }
    return std::make_unique<DTAction>(best);
}

std::unique_ptr<DTNode> buildTree(std::vector<Example>& examples, std::vector<std::string> attributes) {
    // Base cases
    if (examples.empty()) return std::make_unique<DTAction>(ActionType::NONE);

    // Check if all examples have same action
    ActionType firstAction = examples[0].action;
    bool allSame = true;
    for (const auto& ex : examples) {
        if (ex.action != firstAction) {
            allSame = false;
            break;
        }
    }
    if (allSame) return std::make_unique<DTAction>(firstAction);

    if (attributes.empty()) return getMajority(examples);

    // Find best attribute
    double baseEntropy = calculateEntropy(examples);
    std::string bestAttr;
    double maxGain = -1.0;

    for (const auto& attr : attributes) {
        auto [trueSet, falseSet] = split(examples, attr);
        double pTrue = (double)trueSet.size() / examples.size();
        double pFalse = (double)falseSet.size() / examples.size();
        double gain = baseEntropy - (pTrue * calculateEntropy(trueSet) + pFalse * calculateEntropy(falseSet));
        
        if (gain > maxGain) {
            maxGain = gain;
            bestAttr = attr;
        }
    }

    if (maxGain <= 0.0001) return getMajority(examples);

    // Recursion
    auto [trueSet, falseSet] = split(examples, bestAttr);
    std::vector<std::string> nextAttrs;
    for (const auto& a : attributes) if (a != bestAttr) nextAttrs.push_back(a);

    std::unique_ptr<DTNode> trueBranch, falseBranch;

    if (trueSet.empty()) trueBranch = getMajority(examples);
    else trueBranch = buildTree(trueSet, nextAttrs);

    if (falseSet.empty()) falseBranch = getMajority(examples);
    else falseBranch = buildTree(falseSet, nextAttrs);
    
    return std::make_unique<DTDecision>(bestAttr, std::move(trueBranch), std::move(falseBranch));
}

std::unique_ptr<DTNode> learnDT(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        std::cerr << "Could not open " << filename << " for learning. Returning default tree.\n";
        // Return a simple default tree for ENEMY
        auto chase = std::make_unique<DTAction>(ActionType::CHASE);
        auto search = std::make_unique<DTAction>(ActionType::SEEK_CENTER);
        auto root = std::make_unique<DTDecision>("canSeeEnemy", std::move(chase), std::move(search));
        return root;
    }

    std::vector<Example> examples;
    std::string line;
    std::getline(inFile, line); // Skip header
    while (std::getline(inFile, line)) {
        if (!line.empty()) examples.push_back(parseLine(line));
    }

    if (examples.empty()) {
        std::cerr << "No data in file. Returning default tree.\n";
        auto wander = std::make_unique<DTAction>(ActionType::WANDER);
        return wander;
    }

    std::vector<std::string> attributes = {"enemyNear", "isNearWall", "canSeeEnemy"};
    return buildTree(examples, attributes);
}
