#include "floorplanner/Annealer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
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
    // LP failures should not look attractive to simulated annealing just
    // because a default objective was left behind. Infeasible construction
    // placements may still carry a finite penalty; that penalty is useful early
    // in fixed-outline MCNC searches before the first feasible topology appears.
    if (!std::isfinite(solution.objectiveValue)) return 1e100;
    return solution.objectiveValue;
}

namespace {

constexpr double kHugeObjective = 1e90;

bool betterBest(const FloorplanSolution& candidate, const FloorplanSolution& best) {
    // Best-so-far is reserved for feasible solutions once any feasible solution
    // has been seen. Before that, finite construction penalties still provide a
    // useful ordering among infeasible states.
    if (candidate.feasible != best.feasible) return candidate.feasible;
    return annealingObjective(candidate) < annealingObjective(best);
}

bool acceptCandidate(const FloorplanSolution& current,
                     const FloorplanSolution& candidate,
                     double temperature,
                     std::mt19937& rng) {
    // 3D repo best practice adopted here for the 2D sequence-pair search:
    // once the current state is feasible, infeasible candidates are locked out.
    // Early exploration can still move among infeasible states, which matters
    // for tight fixed-outline MCNC cases.
    if (current.feasible && !candidate.feasible) return false;
    if (!current.feasible && candidate.feasible) return true;

    const double candObj = annealingObjective(candidate);
    const double currentObj = annealingObjective(current);
    if (candObj < currentObj) return true;
    if (candObj >= kHugeObjective || currentObj >= kHugeObjective) return false;

    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double delta = candObj - currentObj;
    return unit(rng) < std::exp(-delta / std::max(1e-12, temperature));
}

double calibrateInitialTemperature(const FloorplanProblem& problem,
                                   const SequencePair& initial,
                                   EvaluationMode evalMode,
                                   LPSolver* solver,
                                   const AnnealerOptions& options) {
    std::mt19937 rng(options.seed ^ 0x9e3779b9U);
    SequencePair current = initial;
    FloorplanSolution currentSol = evaluateSequencePair(problem, current, evalMode, solver);
    double currentObj = annealingObjective(currentSol);

    const int samples = std::max(1, options.temperatureCalibrationSamples);
    double sumDelta = 0.0;
    int useful = 0;
    for (int k = 0; k < samples; ++k) {
        SequencePair candidate = current;
        candidate.mutate(rng);
        FloorplanSolution candSol = evaluateSequencePair(problem, candidate, evalMode, solver);
        const double candObj = annealingObjective(candSol);
        if (candObj < kHugeObjective && currentObj < kHugeObjective) {
            sumDelta += std::abs(candObj - currentObj);
            ++useful;
        }
        current = candidate;
        currentSol = candSol;
        currentObj = candObj;
    }

    if (useful == 0) return options.initialTemperature > 0.0 ? options.initialTemperature : 100.0;
    const double avgDelta = sumDelta / useful;
    if (avgDelta < 1.0) return 100.0;
    const double target = std::clamp(options.targetAcceptanceProbability, 0.01, 0.99);
    return -avgDelta / std::log(target);
}

} // namespace

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

    double T = options.autoInitialTemperature || options.initialTemperature <= 0.0
                   ? calibrateInitialTemperature(problem, current, evalMode, solver, options)
                   : options.initialTemperature;
    const double startTemperature = T;
    const int epochLength = options.epochLength > 0 ? options.epochLength : std::max(200, n * n);
    if (options.verbose) {
        std::cout << "  [SA] start T=" << T
                  << "  epoch_length=" << epochLength
                  << "  initial_feasible=" << (currentSol.feasible ? "yes" : "no")
                  << "  initial_obj=" << annealingObjective(currentSol) << "\n";
    }
    int noImproveEpochs = 0;
    int sinceEpoch = 0;
    int epoch = 0;
    long long totalMoves = 0;
    long long acceptedMoves = 0;
    long long feasibilityLockedRejects = 0;
    for (int iter = 0; iter < options.iterations && noImproveEpochs < options.maxEpochsWithoutImprovement; ++iter) {
        SequencePair candidate = current;
        candidate.mutate(rng);
        FloorplanSolution candSol = evaluateSequencePair(problem, candidate, evalMode, solver);
        ++totalMoves;

        const bool feasibilityLocked = currentSol.feasible && !candSol.feasible;
        const bool accept = acceptCandidate(currentSol, candSol, T, rng);
        if (feasibilityLocked && !accept) {
            ++feasibilityLockedRejects;
        }
        if (accept) {
            current = candidate;
            currentSol = candSol;
            ++acceptedMoves;
        }
        if (betterBest(candSol, bestSol)) {
            best = candidate;
            bestSol = candSol;
            noImproveEpochs = 0;
        }
        if (++sinceEpoch >= epochLength) {
            sinceEpoch = 0;
            T *= options.coolingRatio;
            ++noImproveEpochs;
            ++epoch;
            if (options.verbose && options.progressIntervalEpochs > 0 && epoch % options.progressIntervalEpochs == 0) {
                const double rate = totalMoves > 0 ? 100.0 * static_cast<double>(acceptedMoves) / static_cast<double>(totalMoves) : 0.0;
                std::cout << "  [SA] epoch=" << epoch
                          << "  iter=" << iter + 1
                          << "  T=" << T
                          << "  best=" << annealingObjective(bestSol)
                          << "  best_feasible=" << (bestSol.feasible ? "yes" : "no")
                          << "  current=" << annealingObjective(currentSol)
                          << "  current_feasible=" << (currentSol.feasible ? "yes" : "no")
                          << "  accept=" << rate << "%\n";
            }
        }
    }

    if (mode == EvaluationMode::SA_CT_LP && solver && solver->available()) {
        FloorplanSolution refined = optimizeByLP(problem, best, *solver);
        if (refined.feasible) bestSol = refined;
    }
    if (options.verbose) {
        const double rate = totalMoves > 0 ? 100.0 * static_cast<double>(acceptedMoves) / static_cast<double>(totalMoves) : 0.0;
        std::cout << "  [SA] done: moves=" << totalMoves
                  << "  accepted=" << acceptedMoves
                  << "  accept_rate=" << rate << "%"
                  << "  feasibility_locked_rejects=" << feasibilityLockedRejects
                  << "  best_feasible=" << (bestSol.feasible ? "yes" : "no")
                  << "  best=" << annealingObjective(bestSol) << "\n";
    }
    AnnealerResult result;
    result.solution = bestSol;
    result.sequencePair = best;
    result.initialTemperatureUsed = startTemperature;
    result.epochLengthUsed = epochLength;
    result.totalMoves = totalMoves;
    result.acceptedMoves = acceptedMoves;
    return result;
}

} // namespace fp
