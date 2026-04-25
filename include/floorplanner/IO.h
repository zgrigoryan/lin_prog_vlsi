#pragma once

#include "floorplanner/Annealer.h"
#include "floorplanner/DataModel.h"
#include "floorplanner/SequencePair.h"

#include <string>

namespace fp {

struct RunMetadata {
    std::string mode;
    std::string solver;
    int iterations = 0;
    unsigned seed = 0;
    int epochLength = 0;
    double coolingRatio = 0.0;
    int numBlocks = 0;
    int numNets = 0;
    bool hasFixedOutline = false;
    double fixedOutlineWidth = 0.0;
    double fixedOutlineHeight = 0.0;
    double chipAspectLower = 0.0;
    double chipAspectUpper = 0.0;
};

FloorplanProblem readProblemJson(const std::string& path);
void writePlacementCsv(const std::string& path, const FloorplanSolution& solution);
void writeSummaryJson(const std::string& path, const FloorplanSolution& solution, const SequencePair& sp, double runtimeSeconds, const RunMetadata& metadata);
void printSolution(const FloorplanSolution& solution, const SequencePair& sp, double runtimeSeconds);

} // namespace fp
