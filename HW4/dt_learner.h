#pragma once
#include "ai.h"
#include <string>
#include <memory>

// Learns a decision tree from a CSV file (header: enemyNear,isNearWall,canSeeEnemy,action)
std::unique_ptr<DTNode> learnDT(const std::string& filename);
