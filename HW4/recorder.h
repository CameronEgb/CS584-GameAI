#pragma once
#include "ai.h"
#include <fstream>
#include <string>

class DataRecorder {
    std::ofstream outFile;
public:
    DataRecorder(const std::string& filename);
    ~DataRecorder();
    void record(const WorldState& state, ActionType action);
};
