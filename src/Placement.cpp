#include "floorplanner/Placement.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace fp {

namespace {

void addEdge(std::vector<std::vector<std::pair<int, double>>>& graph, int from, int to, double weight) {
    graph[from].push_back({to, weight});
}

std::vector<double> longestPathsDag(const std::vector<std::vector<std::pair<int, double>>>& graph) {
    const int n = static_cast<int>(graph.size());
    std::vector<int> indeg(n, 0);
    for (int u = 0; u < n; ++u) {
        for (const auto& edge : graph[u]) ++indeg[edge.first];
    }
    std::queue<int> q;
    for (int i = 0; i < n; ++i) {
        if (indeg[i] == 0) q.push(i);
    }
    std::vector<double> dist(n, 0.0);
    int seen = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        ++seen;
        for (const auto& [v, w] : graph[u]) {
            dist[v] = std::max(dist[v], dist[u] + w);
            if (--indeg[v] == 0) q.push(v);
        }
    }
    if (seen != n) throw std::runtime_error("constraint graph has a cycle");
    return dist;
}

} // namespace

double computeHPWL(const FloorplanProblem& problem, const std::vector<Block>& placedBlocks) {
    // Simplification for experimentation: nets use block centers for HPWL.
    // Pin-aware offsets and orientation-dependent pin locations are not modeled.
    double total = 0.0;
    for (const auto& net : problem.nets) {
        if (net.blockIds.empty() && net.pads.empty()) continue;
        double minX = 1e100, maxX = -1e100, minY = 1e100, maxY = -1e100;
        for (int id : net.blockIds) {
            const auto& b = placedBlocks[id];
            const double cx = b.x + 0.5 * b.width;
            const double cy = b.y + 0.5 * b.height;
            minX = std::min(minX, cx);
            maxX = std::max(maxX, cx);
            minY = std::min(minY, cy);
            maxY = std::max(maxY, cy);
        }
        for (const auto& p : net.pads) {
            minX = std::min(minX, p.x);
            maxX = std::max(maxX, p.x);
            minY = std::min(minY, p.y);
            maxY = std::max(maxY, p.y);
        }
        total += (maxX - minX) + (maxY - minY);
    }
    return total;
}

double computeObjective(const FloorplanProblem& problem, double chipWidth, double chipHeight, double wirelength) {
    // The LP and construction evaluator use W + H as a linear chip-size proxy,
    // matching the paper's LP-friendly objective rather than nonlinear W * H.
    return problem.areaWeight * (chipWidth + chipHeight) + problem.wireWeight * wirelength;
}

CompactPlacementResult compactPlacement(const FloorplanProblem& problem, const SequencePair& sp, const std::vector<Block>& blocksWithDims) {
    // Compact placement solves longest paths in horizontal and vertical
    // precedence DAGs induced by the sequence-pair.
    const int n = static_cast<int>(blocksWithDims.size());
    if (!sp.validate(n)) throw std::runtime_error("invalid sequence-pair");
    std::vector<std::vector<std::pair<int, double>>> horizontal(n), vertical(n);
    for (const auto& rel : sp.allRelations()) {
        const int i = rel.i;
        const int j = rel.j;
        switch (rel.relation) {
            case Relation::LEFT_OF:
                addEdge(horizontal, i, j, blocksWithDims[i].width);
                break;
            case Relation::RIGHT_OF:
                addEdge(horizontal, j, i, blocksWithDims[j].width);
                break;
            case Relation::BELOW:
                addEdge(vertical, i, j, blocksWithDims[i].height);
                break;
            case Relation::ABOVE:
                addEdge(vertical, j, i, blocksWithDims[j].height);
                break;
        }
    }
    std::vector<Block> placed = blocksWithDims;
    const auto xs = longestPathsDag(horizontal);
    const auto ys = longestPathsDag(vertical);
    double W = 0.0, H = 0.0;
    for (int i = 0; i < n; ++i) {
        placed[i].x = xs[i];
        placed[i].y = ys[i];
        W = std::max(W, placed[i].x + placed[i].width);
        H = std::max(H, placed[i].y + placed[i].height);
    }
    const double lowerViolation = std::max(0.0, problem.chipAspectLower * W - H);
    const double upperViolation = std::max(0.0, H - problem.chipAspectUpper * W);
    const double outlineWViolation = problem.hasFixedOutline ? std::max(0.0, W - problem.fixedOutlineWidth) : 0.0;
    const double outlineHViolation = problem.hasFixedOutline ? std::max(0.0, H - problem.fixedOutlineHeight) : 0.0;
    const bool aspectOk = lowerViolation <= 1e-9 && upperViolation <= 1e-9;
    const bool outlineOk = outlineWViolation <= 1e-9 && outlineHViolation <= 1e-9;
    CompactPlacementResult out;
    out.blocks = placed;
    out.solution = makeSolution(problem, placed, W, H, aspectOk && outlineOk, aspectOk && outlineOk ? "compact_feasible" : "compact_aspect_or_outline_violation");
    if (!out.solution.feasible) {
        out.solution.objectiveValue += 1e6 * (lowerViolation + upperViolation + outlineWViolation + outlineHViolation);
    }
    return out;
}

FloorplanSolution makeSolution(const FloorplanProblem& problem, const std::vector<Block>& placedBlocks, double chipWidth, double chipHeight, bool feasible, const std::string& status) {
    FloorplanSolution sol;
    sol.chipWidth = chipWidth;
    sol.chipHeight = chipHeight;
    sol.chipArea = chipWidth * chipHeight;
    sol.totalWirelength = computeHPWL(problem, placedBlocks);
    sol.objectiveValue = computeObjective(problem, chipWidth, chipHeight, sol.totalWirelength);
    sol.feasible = feasible;
    sol.status = status;
    sol.placements.reserve(placedBlocks.size());
    for (const auto& b : placedBlocks) {
        sol.placements.push_back({b.name, b.type, b.x, b.y, b.width, b.height});
    }
    return sol;
}

} // namespace fp
