#include "floorplanner/Annealer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fp {

EvaluationMode parseMode(const std::string& text) {
    if (text == "CT") return EvaluationMode::CT;
    if (text == "LP") return EvaluationMode::LP;
    if (text == "SA-CT") return EvaluationMode::SA_CT;
    if (text == "SA-LP") return EvaluationMode::SA_LP;
    if (text == "SA-CT-LP" || text == "SA-CT+LP") return EvaluationMode::SA_CT_LP;
    throw std::runtime_error("unknown mode: " + text);
}

std::string toString(EvaluationMode mode) {
    switch (mode) {
        case EvaluationMode::CT: return "CT";
        case EvaluationMode::LP: return "LP";
        case EvaluationMode::SA_CT: return "SA-CT";
        case EvaluationMode::SA_LP: return "SA-LP";
        case EvaluationMode::SA_CT_LP: return "SA-CT-LP";
    }
    return "unknown";
}

FloorplanSolution evaluateSequencePair(const FloorplanProblem& problem, const SequencePair& sp, EvaluationMode mode, LPSolver* solver) {
    if (mode == EvaluationMode::LP || mode == EvaluationMode::SA_LP) {
        if (!solver || !solver->available()) {
            FloorplanSolution s;
            s.status = "LP mode requires an available LP solver";
            return s;
        }
        FloorplanSolution lp = optimizeByLP(problem, sp, *solver);
        if (lp.feasible || mode == EvaluationMode::LP) return lp;

        // Paper deviation / engineering fallback:
        // In the paper, SA-LP evaluates topologies by LP. In fixed-outline
        // MCNC runs, many early topologies are LP-infeasible; treating every
        // failure as the same infinite objective prevents search progress.
        // We therefore use compact construction's outline/aspect penalty only
        // as a search signal for infeasible SA-LP candidates. Feasible
        // candidates and final reported LP solutions still come from the LP.
        FloorplanSolution penalty = constructByKimKim(problem, sp);
        penalty.feasible = false;
        penalty.status = "lp_infeasible; " + penalty.status;
        return penalty;
    }
    return constructByKimKim(problem, sp);
}

double annealingObjective(const FloorplanSolution& solution) {
    // LP failures and true infeasible placements should not look attractive to
    // simulated annealing just because a default objective was left behind.
    // Construction-mode aspect/outline violations already carry a penalty, but
    // they still remain infeasible and are therefore treated as very bad here.
    if (!std::isfinite(solution.objectiveValue)) return 1e100;
    return solution.objectiveValue;
}

AnnealerResult runAnnealing(const FloorplanProblem& problem, EvaluationMode mode, LPSolver* solver, const AnnealerOptions& options) {
    // SA-CT evaluates topology moves with the construction method.
    // SA-LP evaluates each topology by LP coordinate/dimension optimization.
    // SA-CT-LP searches cheaply with construction, then refines the best pair by LP.
    const int n = static_cast<int>(problem.blocks.size());
    std::mt19937 rng(options.seed);
    SequencePair current = SequencePair::random(n, rng);
    const EvaluationMode evalMode = mode == EvaluationMode::SA_LP ? EvaluationMode::SA_LP : EvaluationMode::CT;
    FloorplanSolution currentSol = evaluateSequencePair(problem, current, evalMode, solver);
    SequencePair best = current;
    FloorplanSolution bestSol = currentSol;

    std::uniform_real_distribution<double> unit(0.0, 1.0);
    double T = options.initialTemperature;
    int noImproveEpochs = 0;
    int sinceEpoch = 0;
    for (int iter = 0; iter < options.iterations && noImproveEpochs < options.maxEpochsWithoutImprovement; ++iter) {
        SequencePair candidate = current;
        candidate.mutate(rng);
        FloorplanSolution candSol = evaluateSequencePair(problem, candidate, evalMode, solver);
        const double candObj = annealingObjective(candSol);
        const double currentObj = annealingObjective(currentSol);
        bool accept = false;
        if (candObj < currentObj) {
            accept = true;
        } else if (std::isfinite(candObj) && std::isfinite(currentObj)) {
            const double delta = candObj - currentObj;
            accept = unit(rng) < std::exp(-delta / std::max(1e-12, T));
        }
        if (accept) {
            current = candidate;
            currentSol = candSol;
        }
        if (candObj < annealingObjective(bestSol)) {
            best = candidate;
            bestSol = candSol;
            noImproveEpochs = 0;
        }
        if (++sinceEpoch >= options.epochLength) {
            sinceEpoch = 0;
            T *= options.coolingRatio;
            ++noImproveEpochs;
        }
    }

    if (mode == EvaluationMode::SA_CT_LP && solver && solver->available()) {
        FloorplanSolution refined = optimizeByLP(problem, best, *solver);
        if (refined.feasible) bestSol = refined;
    }
    return {bestSol, best};
}

} // namespace fp
