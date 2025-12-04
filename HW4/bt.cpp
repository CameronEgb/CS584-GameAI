#include "bt.h"

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
