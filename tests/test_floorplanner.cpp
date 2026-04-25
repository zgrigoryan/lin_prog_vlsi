#include "floorplanner/Annealer.h"
#include "floorplanner/LPFloorplanner.h"
#include "floorplanner/LPSolver.h"
#include "floorplanner/Placement.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool cond, const char* msg) {
    if (!cond) throw std::runtime_error(msg);
}

fp::FloorplanProblem tinyProblem() {
    fp::FloorplanProblem p;
    p.blocks = {
        {0, "a", fp::BlockType::HARD, 4.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0},
        {1, "b", fp::BlockType::HARD, 6.0, 3.0, 2.0, 1.0, 1.0, 3.0, 2.0},
        {2, "c", fp::BlockType::SOFT, 9.0, 0.0, 0.0, 0.5, 2.0, 3.0, 3.0}
    };
    p.nets = {{0, "n1", {0, 1, 2}, {}}};
    p.chipAspectLower = 0.1;
    p.chipAspectUpper = 10.0;
    p.rebuildIndex();
    return p;
}

void testRelations() {
    fp::SequencePair identity({0, 1, 2}, {0, 1, 2});
    require(identity.relation(0, 1) == fp::Relation::LEFT_OF, "identity 0 left of 1");
    require(identity.relation(0, 2) == fp::Relation::LEFT_OF, "identity 0 left of 2");
    require(identity.relation(1, 2) == fp::Relation::LEFT_OF, "identity 1 left of 2");
    fp::SequencePair sp({0, 1, 2}, {0, 2, 1});
    require(sp.relation(0, 1) == fp::Relation::LEFT_OF, "0 left of 1");
    require(sp.relation(1, 2) == fp::Relation::ABOVE, "1 above 2");
    require(sp.relation(2, 1) == fp::Relation::BELOW, "2 below 1");
}

void testHPWL() {
    fp::FloorplanProblem two;
    two.blocks = {
        {0, "u", fp::BlockType::HARD, 4.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0},
        {1, "v", fp::BlockType::HARD, 4.0, 2.0, 2.0, 1.0, 1.0, 2.0, 2.0}
    };
    two.nets = {{0, "uv", {0, 1}, {}}};
    two.rebuildIndex();
    two.blocks[0].x = 0; two.blocks[0].y = 0;
    two.blocks[1].x = 4; two.blocks[1].y = 6;
    require(std::abs(fp::computeHPWL(two, two.blocks) - 10.0) < 1e-9, "two-block HPWL");

    auto p = tinyProblem();
    p.blocks[0].x = 0; p.blocks[0].y = 0;
    p.blocks[1].x = 4; p.blocks[1].y = 0;
    p.blocks[2].x = 1; p.blocks[2].y = 5;
    const double hpwl = fp::computeHPWL(p, p.blocks);
    require(std::abs(hpwl - 10.0) < 1e-9, "HPWL");
}

void testCompactPlacement() {
    auto p = tinyProblem();
    fp::SequencePair sp({0, 1, 2}, {0, 1, 2});
    auto out = fp::compactPlacement(p, sp, p.blocks);
    require(std::abs(out.blocks[1].x - 2.0) < 1e-9, "block b x");
    require(std::abs(out.blocks[2].x - 5.0) < 1e-9, "block c x");
    require(std::abs(out.solution.chipWidth - 8.0) < 1e-9, "compact width");
    require(out.solution.chipWidth > 0.0 && out.solution.chipHeight > 0.0, "positive chip dimensions");
    for (size_t i = 0; i < out.blocks.size(); ++i) {
        for (size_t j = i + 1; j < out.blocks.size(); ++j) {
            const auto& a = out.blocks[i];
            const auto& b = out.blocks[j];
            const bool separated = a.x + a.width <= b.x + 1e-9 ||
                                   b.x + b.width <= a.x + 1e-9 ||
                                   a.y + a.height <= b.y + 1e-9 ||
                                   b.y + b.height <= a.y + 1e-9;
            require(separated, "no overlap");
        }
    }
}

void testLPModelCreation() {
    auto p = tinyProblem();
    fp::SequencePair sp({0, 1, 2}, {0, 2, 1});
    auto build = fp::buildLPModel(p, sp, std::vector<double>(3, 1.0));
    require(!build.model.variables.empty(), "lp variables");
    require(!build.model.constraints.empty(), "lp constraints");
    require(build.vars.W >= 0 && build.vars.H >= 0, "chip vars");
    auto solver = fp::createSolver("highs");
    if (solver->available()) {
        auto sol = fp::optimizeByLP(p, sp, *solver);
        require(sol.feasible, "highs solves tiny LP");
    }
}

void testMutationValidity() {
    std::mt19937 rng(7);
    fp::SequencePair sp = fp::SequencePair::identity(10);
    for (int i = 0; i < 1000; ++i) {
        sp.mutate(rng);
        require(sp.validate(10), "mutation validity");
    }
}

} // namespace

int main() {
    try {
        testRelations();
        testHPWL();
        testCompactPlacement();
        testLPModelCreation();
        testMutationValidity();
        std::cout << "all tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test failure: " << e.what() << "\n";
        return 1;
    }
}
