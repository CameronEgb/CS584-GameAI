#include "recorder.h"
#include <iostream>

DataRecorder::DataRecorder(const std::string& filename) {
    outFile.open(filename);
    if (outFile.is_open()) {
        // Header
        outFile << "enemyNear,isNearWall,canSeeEnemy,action\n";
    } else {
        std::cerr << "Failed to open " << filename << " for recording.\n";
    }
}

DataRecorder::~DataRecorder() {
    if (outFile.is_open()) outFile.close();
}

void DataRecorder::record(const WorldState& state, ActionType action) {
    if (!outFile.is_open()) return;
    outFile << (state.enemyNear ? "1" : "0") << ","
            << (state.isNearWall ? "1" : "0") << ","
            << (state.canSeeEnemy ? "1" : "0") << ","
            << (int)action << "\n";
}

