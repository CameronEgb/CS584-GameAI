#include "bt.h"
#include <algorithm>
#include <random>
#include <vector>

BTStatus BTSelector::tick(EnemyContext& ctx) {
    for (auto& child : children) {
        BTStatus status = child->tick(ctx);
        if (status != BTStatus::FAILURE) {
            return status;
        }
    }
    return BTStatus::FAILURE;
}

BTStatus BTSequence::tick(EnemyContext& ctx) {
    for (auto& child : children) {
        BTStatus status = child->tick(ctx);
        if (status != BTStatus::SUCCESS) {
            return status;
        }
    }
    return BTStatus::SUCCESS;
}

BTStatus BTRandomSelector::tick(EnemyContext& ctx) {
    if (children.empty()) return BTStatus::FAILURE;

    std::vector<size_t> indices(children.size());
    for(size_t i = 0; i < children.size(); ++i) indices[i] = i;

    static std::random_device rd;
    static std::mt19937 g(rd());
    std::shuffle(indices.begin(), indices.end(), g);

    for (size_t idx : indices) {
        BTStatus status = children[idx]->tick(ctx);
        if (status != BTStatus::FAILURE) {
            return status;
        }
    }
    return BTStatus::FAILURE;
}
