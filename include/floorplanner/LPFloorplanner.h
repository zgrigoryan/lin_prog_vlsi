#pragma once

#include "floorplanner/DataModel.h"
#include "floorplanner/LPSolver.h"
#include "floorplanner/SequencePair.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace fp {

struct LPOptions {
    int maxAreaCorrectionIterations = 8;
    double areaTolerance = 1e-6;
    bool verboseAreaCorrection = false;
    bool fixHardOrientationsUsingConstruction = true;
};

struct LPVariableIndex {
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> w;
    std::vector<int> h;
    int W = -1;
    int H = -1;
    std::vector<int> netLeft;
    std::vector<int> netRight;
    std::vector<int> netBottom;
    std::vector<int> netTop;
    std::vector<int> netWidth;
    std::vector<int> netHeight;
};

struct LPBuildResult {
    LPModel model;
    LPVariableIndex vars;
};

LPBuildResult buildLPModel(const FloorplanProblem& problem, const SequencePair& sp, const std::vector<double>& alpha);
FloorplanProblem prepareProblemForLP(const FloorplanProblem& problem, const SequencePair& sp, const LPOptions& options = {});
FloorplanSolution optimizeByLP(const FloorplanProblem& problem, const SequencePair& sp, LPSolver& solver, const LPOptions& options = {});
void writeLPModel(const std::string& path, const LPModel& model);
void writeMPSModel(const std::string& path, const LPModel& model);
LPBuildResult buildInitialLPModelForExport(const FloorplanProblem& problem, const SequencePair& sp, const LPOptions& options = {});

} // namespace fp
