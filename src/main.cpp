#include "floorplanner/Annealer.h"
#include "floorplanner/IO.h"
#include "floorplanner/LPFloorplanner.h"
#include "floorplanner/LPSolver.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Args {
    std::string input;
    std::string output = "out";
    std::string mode = "SA-LP";
    std::string solver = "highs";
    int iterations = 10000;
    int epochLength = 100;
    int maxNoImproveEpochs = 50;
    double initialTemperature = 100.0;
    double coolingRatio = 0.95;
    unsigned seed = 1;
    std::string exportLp;
    std::string exportMps;
};

void usage() {
    std::cerr << "Usage: floorplanner --input examples/small.json --mode SA-LP --solver highs --iterations 10000 --output out/ [--export-lp out/model.lp] [--export-mps out/model.mps]\n";
}

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        auto need = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + name);
            return argv[++i];
        };
        if (key == "--input") a.input = need(key);
        else if (key == "--output") a.output = need(key);
        else if (key == "--mode") a.mode = need(key);
        else if (key == "--solver") a.solver = need(key);
        else if (key == "--iterations") a.iterations = std::stoi(need(key));
        else if (key == "--epoch-length") a.epochLength = std::stoi(need(key));
        else if (key == "--max-no-improve-epochs") a.maxNoImproveEpochs = std::stoi(need(key));
        else if (key == "--initial-temperature") a.initialTemperature = std::stod(need(key));
        else if (key == "--cooling-ratio") a.coolingRatio = std::stod(need(key));
        else if (key == "--seed") a.seed = static_cast<unsigned>(std::stoul(need(key)));
        else if (key == "--export-lp") a.exportLp = need(key);
        else if (key == "--export-mps") a.exportMps = need(key);
        else if (key == "--help" || key == "-h") {
            usage();
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }
    if (a.input.empty()) throw std::runtime_error("--input is required");
    return a;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parseArgs(argc, argv);
        auto problem = fp::readProblemJson(args.input);
        auto solver = fp::createSolver(args.solver);
        const fp::EvaluationMode mode = fp::parseMode(args.mode);
        const bool needsLpSolver = mode == fp::EvaluationMode::LP ||
                                   mode == fp::EvaluationMode::SA_LP ||
                                   mode == fp::EvaluationMode::SA_CT_LP;
        if (needsLpSolver && !solver->available()) {
            if (args.solver != "none" && args.solver != "compact" && !solver->unavailableReason().empty()) {
                throw std::runtime_error(solver->unavailableReason());
            }
            throw std::runtime_error("LP mode requires an available LP solver");
        }
        const auto start = std::chrono::steady_clock::now();

        fp::AnnealerResult result;
        if (mode == fp::EvaluationMode::CT || mode == fp::EvaluationMode::LP) {
            result.sequencePair = fp::SequencePair::identity(static_cast<int>(problem.blocks.size()));
            result.solution = fp::evaluateSequencePair(problem, result.sequencePair, mode, solver.get());
        } else {
            fp::AnnealerOptions options;
            options.iterations = args.iterations;
            options.epochLength = args.epochLength;
            options.maxEpochsWithoutImprovement = args.maxNoImproveEpochs;
            options.initialTemperature = args.initialTemperature;
            options.coolingRatio = args.coolingRatio;
            options.seed = args.seed;
            result = fp::runAnnealing(problem, mode, solver.get(), options);
        }

        const auto stop = std::chrono::steady_clock::now();
        const double runtime = std::chrono::duration<double>(stop - start).count();
        std::filesystem::create_directories(args.output);
        auto createParent = [](const std::string& path) {
            const auto parent = std::filesystem::path(path).parent_path();
            if (!parent.empty()) std::filesystem::create_directories(parent);
        };
        if (!args.exportLp.empty() || !args.exportMps.empty()) {
            // Export the LP corresponding to the final selected sequence-pair.
            // For SA modes this avoids writing thousands of intermediate LPs.
            const auto build = fp::buildInitialLPModelForExport(problem, result.sequencePair);
            if (!args.exportLp.empty()) {
                createParent(args.exportLp);
                fp::writeLPModel(args.exportLp, build.model);
            }
            if (!args.exportMps.empty()) {
                createParent(args.exportMps);
                fp::writeMPSModel(args.exportMps, build.model);
            }
        }
        fp::RunMetadata metadata;
        metadata.mode = fp::toString(mode);
        metadata.solver = args.solver;
        metadata.iterations = args.iterations;
        metadata.seed = args.seed;
        metadata.epochLength = args.epochLength;
        metadata.coolingRatio = args.coolingRatio;
        metadata.numBlocks = static_cast<int>(problem.blocks.size());
        metadata.numNets = static_cast<int>(problem.nets.size());
        metadata.hasFixedOutline = problem.hasFixedOutline;
        metadata.fixedOutlineWidth = problem.fixedOutlineWidth;
        metadata.fixedOutlineHeight = problem.fixedOutlineHeight;
        metadata.chipAspectLower = problem.chipAspectLower;
        metadata.chipAspectUpper = problem.chipAspectUpper;
        fp::writePlacementCsv((std::filesystem::path(args.output) / "placements.csv").string(), result.solution);
        fp::writeSummaryJson((std::filesystem::path(args.output) / "summary.json").string(), result.solution, result.sequencePair, runtime, metadata);
        fp::printSolution(result.solution, result.sequencePair, runtime);
        if (!result.solution.feasible) return 2;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        usage();
        return 1;
    }
}
