#include "floorplanner/Construction.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace fp {

namespace {

struct CandidateDim {
    double width = 0.0;
    double height = 0.0;
    Orientation orientation = Orientation::HORIZONTAL;
};

double blockEffect(const Block& b) {
    const double area = b.type == BlockType::HARD ? b.fixedWidth * b.fixedHeight : b.area;
    double extreme = 1.0;
    if (b.type == BlockType::HARD) {
        const double r = b.fixedHeight / std::max(1e-12, b.fixedWidth);
        extreme = std::max(r, 1.0 / std::max(1e-12, r));
    } else {
        extreme = std::max(b.maxAspectRatio, 1.0 / std::max(1e-12, b.minAspectRatio));
    }
    return area * (1.0 + 0.1 * std::log1p(extreme));
}

std::vector<CandidateDim> candidates(const Block& b, int softAspectCandidates) {
    std::vector<CandidateDim> out;
    if (b.type == BlockType::HARD) {
        out.push_back({b.fixedWidth, b.fixedHeight, Orientation::HORIZONTAL});
        if (std::abs(b.fixedWidth - b.fixedHeight) > 1e-12) out.push_back({b.fixedHeight, b.fixedWidth, Orientation::VERTICAL});
        return out;
    }
    const int k = std::max(1, softAspectCandidates);
    if (k == 1 || std::abs(b.maxAspectRatio - b.minAspectRatio) <= 1e-12) {
        const double r = std::sqrt(b.minAspectRatio * b.maxAspectRatio);
        out.push_back({std::sqrt(b.area / r), std::sqrt(b.area * r), Orientation::HORIZONTAL});
        return out;
    }
    // This follows Kim and Kim structurally, but uses log-spaced
    // candidate aspect ratios for soft modules.
    for (int i = 0; i < k; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(k - 1);
        const double logR = std::log(b.minAspectRatio) * (1.0 - t) + std::log(b.maxAspectRatio) * t;
        const double r = std::exp(logR);
        out.push_back({std::sqrt(b.area / r), std::sqrt(b.area * r), Orientation::HORIZONTAL});
    }
    return out;
}

} // namespace

FloorplanSolution constructByKimKim(const FloorplanProblem& problem, const SequencePair& sp, const ConstructionOptions& options) {
    std::vector<Block> assigned = problem.blocks;
    for (auto& b : assigned) {
        if (b.type == BlockType::HARD) {
            b.width = b.fixedWidth;
            b.height = b.fixedHeight;
        } else {
            const double r = std::sqrt(b.minAspectRatio * b.maxAspectRatio);
            b.width = std::sqrt(b.area / r);
            b.height = std::sqrt(b.area * r);
        }
    }

    // Kim and Kim's construction method fixes module dimensions for a given
    // sequence-pair by processing high-impact blocks first and testing a small
    // candidate set with compact longest-path placement.
    std::vector<int> order(assigned.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return blockEffect(assigned[a]) > blockEffect(assigned[b]);
    });

    FloorplanSolution bestSoFar;
    for (int id : order) {
        FloorplanSolution bestCandidate;
        std::vector<Block> bestBlocks = assigned;
        for (const auto& cand : candidates(problem.blocks[id], options.softAspectCandidates)) {
            std::vector<Block> trial = assigned;
            trial[id].width = cand.width;
            trial[id].height = cand.height;
            trial[id].orientation = cand.orientation;
            auto placed = compactPlacement(problem, sp, trial);
            if (placed.solution.objectiveValue < bestCandidate.objectiveValue) {
                bestCandidate = placed.solution;
                bestBlocks = placed.blocks;
            }
        }
        assigned = bestBlocks;
        bestSoFar = bestCandidate;
    }
    if (order.empty()) return makeSolution(problem, assigned, 0.0, 0.0, true, "empty");
    return bestSoFar;
}

} // namespace fp
