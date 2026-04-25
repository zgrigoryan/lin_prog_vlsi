#pragma once

#include "floorplanner/Construction.h"
#include "floorplanner/DataModel.h"
#include "floorplanner/LPFloorplanner.h"
#include "floorplanner/LPSolver.h"
#include "floorplanner/SequencePair.h"

#include <random>

namespace fp {

enum class EvaluationMode { CT, LP, SA_CT, SA_LP, SA_CT_LP };

struct AnnealerOptions {
    int iterations = 10000;
    int epochLength = 100;
    int maxEpochsWithoutImprovement = 50;
    double initialTemperature = 100.0;
    double coolingRatio = 0.95;
    unsigned seed = 1;
};

struct AnnealerResult {
    FloorplanSolution solution;
    SequencePair sequencePair;
};

EvaluationMode parseMode(const std::string& text);
std::string toString(EvaluationMode mode);

double annealingObjective(const FloorplanSolution& solution);
FloorplanSolution evaluateSequencePair(const FloorplanProblem& problem, const SequencePair& sp, EvaluationMode mode, LPSolver* solver);
AnnealerResult runAnnealing(const FloorplanProblem& problem, EvaluationMode mode, LPSolver* solver, const AnnealerOptions& options);

} // namespace fp
