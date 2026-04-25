#pragma once

#include "floorplanner/DataModel.h"
#include "floorplanner/Placement.h"
#include "floorplanner/SequencePair.h"

namespace fp {

struct ConstructionOptions {
    int softAspectCandidates = 5;
};

FloorplanSolution constructByKimKim(const FloorplanProblem& problem, const SequencePair& sp, const ConstructionOptions& options = {});

} // namespace fp
