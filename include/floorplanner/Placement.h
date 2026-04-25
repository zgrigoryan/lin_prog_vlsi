#pragma once

#include "floorplanner/DataModel.h"
#include "floorplanner/SequencePair.h"

namespace fp {

struct CompactPlacementResult {
    FloorplanSolution solution;
    std::vector<Block> blocks;
};

double computeHPWL(const FloorplanProblem& problem, const std::vector<Block>& placedBlocks);
double computeObjective(const FloorplanProblem& problem, double chipWidth, double chipHeight, double wirelength);
CompactPlacementResult compactPlacement(const FloorplanProblem& problem, const SequencePair& sp, const std::vector<Block>& blocksWithDims);
FloorplanSolution makeSolution(const FloorplanProblem& problem, const std::vector<Block>& placedBlocks, double chipWidth, double chipHeight, bool feasible, const std::string& status);

} // namespace fp
