#include "floorplanner/LPFloorplanner.h"

#include "floorplanner/Construction.h"
#include "floorplanner/Placement.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace fp {

namespace {

constexpr double INF = 1e30;

double softAreaLowerBound(const Block& b, double alpha) {
    // For all positive w,h with w*h >= A, the tangent to w + alpha*h at
    // h = sqrt(A/alpha), w = sqrt(A*alpha) is 2*sqrt(A*alpha).
    return 2.0 * std::sqrt(std::max(0.0, b.area) * std::max(1e-12, alpha));
}

double clampAlpha(double alpha) {
    return std::clamp(alpha, 1e-6, 1e6);
}

std::string lpName(std::string name) {
    for (char& c : name) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') c = '_';
    }
    if (name.empty()) return "unnamed";
    if (std::isdigit(static_cast<unsigned char>(name.front()))) name = "v_" + name;
    return name;
}

void addLe(LPModel& m, const std::string& name, std::initializer_list<std::pair<int, double>> terms, double rhs) {
    std::vector<int> ids;
    std::vector<double> vals;
    for (auto [id, v] : terms) {
        ids.push_back(id);
        vals.push_back(v);
    }
    m.addConstraint(name, std::move(ids), std::move(vals), ConstraintSense::LessEqual, rhs);
}

void addGe(LPModel& m, const std::string& name, std::initializer_list<std::pair<int, double>> terms, double rhs) {
    std::vector<int> ids;
    std::vector<double> vals;
    for (auto [id, v] : terms) {
        ids.push_back(id);
        vals.push_back(v);
    }
    m.addConstraint(name, std::move(ids), std::move(vals), ConstraintSense::GreaterEqual, rhs);
}

void addEq(LPModel& m, const std::string& name, std::initializer_list<std::pair<int, double>> terms, double rhs) {
    std::vector<int> ids;
    std::vector<double> vals;
    for (auto [id, v] : terms) {
        ids.push_back(id);
        vals.push_back(v);
    }
    m.addConstraint(name, std::move(ids), std::move(vals), ConstraintSense::Equal, rhs);
}

} // namespace

LPBuildResult buildLPModel(const FloorplanProblem& problem, const SequencePair& sp, const std::vector<double>& alpha) {
    LPBuildResult out;
    const int n = static_cast<int>(problem.blocks.size());
    const int nets = static_cast<int>(problem.nets.size());
    out.vars.x.resize(n);
    out.vars.y.resize(n);
    out.vars.w.resize(n, -1);
    out.vars.h.resize(n, -1);
    out.vars.netLeft.resize(nets);
    out.vars.netRight.resize(nets);
    out.vars.netBottom.resize(nets);
    out.vars.netTop.resize(nets);
    out.vars.netWidth.resize(nets);
    out.vars.netHeight.resize(nets);

    for (int i = 0; i < n; ++i) {
        out.vars.x[i] = out.model.addVariable("x_" + problem.blocks[i].name, 0.0, INF, 0.0);
        out.vars.y[i] = out.model.addVariable("y_" + problem.blocks[i].name, 0.0, INF, 0.0);
        if (problem.blocks[i].type == BlockType::SOFT) {
            out.vars.w[i] = out.model.addVariable("w_" + problem.blocks[i].name, 1e-9, INF, 0.0);
            out.vars.h[i] = out.model.addVariable("h_" + problem.blocks[i].name, 1e-9, INF, 0.0);
        }
    }
    // In fixed-outline experiments W and H are constants. The W + H term is
    // still present but constant, so optimization is driven by wirelength and
    // feasibility under the outline.
    out.vars.W = out.model.addVariable("W", problem.hasFixedOutline ? problem.fixedOutlineWidth : 0.0, problem.hasFixedOutline ? problem.fixedOutlineWidth : INF, problem.areaWeight);
    out.vars.H = out.model.addVariable("H", problem.hasFixedOutline ? problem.fixedOutlineHeight : 0.0, problem.hasFixedOutline ? problem.fixedOutlineHeight : INF, problem.areaWeight);

    for (int ni = 0; ni < nets; ++ni) {
        const auto suffix = "_" + problem.nets[ni].name;
        out.vars.netLeft[ni] = out.model.addVariable("netLeft" + suffix, -INF, INF, 0.0);
        out.vars.netRight[ni] = out.model.addVariable("netRight" + suffix, -INF, INF, 0.0);
        out.vars.netBottom[ni] = out.model.addVariable("netBottom" + suffix, -INF, INF, 0.0);
        out.vars.netTop[ni] = out.model.addVariable("netTop" + suffix, -INF, INF, 0.0);
        out.vars.netWidth[ni] = out.model.addVariable("netWidth" + suffix, 0.0, INF, problem.wireWeight);
        out.vars.netHeight[ni] = out.model.addVariable("netHeight" + suffix, 0.0, INF, problem.wireWeight);
    }

    auto widthTerm = [&](int block, double coef) -> std::pair<int, double> {
        const auto& b = problem.blocks[block];
        if (b.type == BlockType::SOFT) return {out.vars.w[block], coef};
        return {-1, coef * b.width};
    };
    auto heightTerm = [&](int block, double coef) -> std::pair<int, double> {
        const auto& b = problem.blocks[block];
        if (b.type == BlockType::SOFT) return {out.vars.h[block], coef};
        return {-1, coef * b.height};
    };
    auto addMixedLe = [&](const std::string& name, std::vector<std::pair<int, double>> terms, double rhs) {
        std::vector<int> ids;
        std::vector<double> vals;
        for (auto [id, v] : terms) {
            if (id >= 0) {
                ids.push_back(id);
                vals.push_back(v);
            } else {
                rhs -= v;
            }
        }
        out.model.addConstraint(name, std::move(ids), std::move(vals), ConstraintSense::LessEqual, rhs);
    };
    auto addMixedGe = [&](const std::string& name, std::vector<std::pair<int, double>> terms, double rhs) {
        std::vector<int> ids;
        std::vector<double> vals;
        for (auto [id, v] : terms) {
            if (id >= 0) {
                ids.push_back(id);
                vals.push_back(v);
            } else {
                rhs -= v;
            }
        }
        out.model.addConstraint(name, std::move(ids), std::move(vals), ConstraintSense::GreaterEqual, rhs);
    };

    for (int i = 0; i < n; ++i) {
        const auto& b = problem.blocks[i];
        if (b.type == BlockType::SOFT) {
            addGe(out.model, "aspect_min_" + b.name, {{out.vars.h[i], 1.0}, {out.vars.w[i], -b.minAspectRatio}}, 0.0);
            addLe(out.model, "aspect_max_" + b.name, {{out.vars.h[i], 1.0}, {out.vars.w[i], -b.maxAspectRatio}}, 0.0);
            const double a = alpha.size() > static_cast<size_t>(i) ? alpha[i] : 1.0;
            addGe(out.model, "area_surrogate_" + b.name, {{out.vars.w[i], 1.0}, {out.vars.h[i], a}}, softAreaLowerBound(b, a));
        }
    }

    // TODO: Kim and Kim remove redundant constraints to reduce LP size.
    // Current implementation keeps all pairwise precedence constraints for simplicity.
    for (const auto& rel : sp.allRelations()) {
        const int i = rel.i;
        const int j = rel.j;
        if (rel.relation == Relation::LEFT_OF) {
            addMixedLe("left_" + std::to_string(i) + "_" + std::to_string(j), {{out.vars.x[i], 1.0}, widthTerm(i, 1.0), {out.vars.x[j], -1.0}}, 0.0);
        } else if (rel.relation == Relation::RIGHT_OF) {
            addMixedLe("right_" + std::to_string(i) + "_" + std::to_string(j), {{out.vars.x[j], 1.0}, widthTerm(j, 1.0), {out.vars.x[i], -1.0}}, 0.0);
        } else if (rel.relation == Relation::BELOW) {
            addMixedLe("below_" + std::to_string(i) + "_" + std::to_string(j), {{out.vars.y[i], 1.0}, heightTerm(i, 1.0), {out.vars.y[j], -1.0}}, 0.0);
        } else {
            addMixedLe("above_" + std::to_string(i) + "_" + std::to_string(j), {{out.vars.y[j], 1.0}, heightTerm(j, 1.0), {out.vars.y[i], -1.0}}, 0.0);
        }
    }

    for (int i = 0; i < n; ++i) {
        addMixedLe("bound_x_" + problem.blocks[i].name, {{out.vars.x[i], 1.0}, widthTerm(i, 1.0), {out.vars.W, -1.0}}, 0.0);
        addMixedLe("bound_y_" + problem.blocks[i].name, {{out.vars.y[i], 1.0}, heightTerm(i, 1.0), {out.vars.H, -1.0}}, 0.0);
    }
    if (problem.chipAspectLower > 0.0) addGe(out.model, "chip_aspect_lower", {{out.vars.H, 1.0}, {out.vars.W, -problem.chipAspectLower}}, 0.0);
    if (problem.chipAspectUpper < INF / 2.0) addLe(out.model, "chip_aspect_upper", {{out.vars.H, 1.0}, {out.vars.W, -problem.chipAspectUpper}}, 0.0);

    for (int ni = 0; ni < nets; ++ni) {
        const auto& net = problem.nets[ni];
        for (int id : net.blockIds) {
            addMixedLe("net_left_" + std::to_string(ni) + "_" + std::to_string(id), {{out.vars.netLeft[ni], 1.0}, {out.vars.x[id], -1.0}, widthTerm(id, -0.5)}, 0.0);
            addMixedGe("net_right_" + std::to_string(ni) + "_" + std::to_string(id), {{out.vars.netRight[ni], 1.0}, {out.vars.x[id], -1.0}, widthTerm(id, -0.5)}, 0.0);
            addMixedLe("net_bottom_" + std::to_string(ni) + "_" + std::to_string(id), {{out.vars.netBottom[ni], 1.0}, {out.vars.y[id], -1.0}, heightTerm(id, -0.5)}, 0.0);
            addMixedGe("net_top_" + std::to_string(ni) + "_" + std::to_string(id), {{out.vars.netTop[ni], 1.0}, {out.vars.y[id], -1.0}, heightTerm(id, -0.5)}, 0.0);
        }
        for (const auto& pad : net.pads) {
            addLe(out.model, "pad_left_" + std::to_string(ni), {{out.vars.netLeft[ni], 1.0}}, pad.x);
            addGe(out.model, "pad_right_" + std::to_string(ni), {{out.vars.netRight[ni], 1.0}}, pad.x);
            addLe(out.model, "pad_bottom_" + std::to_string(ni), {{out.vars.netBottom[ni], 1.0}}, pad.y);
            addGe(out.model, "pad_top_" + std::to_string(ni), {{out.vars.netTop[ni], 1.0}}, pad.y);
        }
        addEq(out.model, "net_width_def_" + std::to_string(ni), {{out.vars.netWidth[ni], 1.0}, {out.vars.netRight[ni], -1.0}, {out.vars.netLeft[ni], 1.0}}, 0.0);
        addEq(out.model, "net_height_def_" + std::to_string(ni), {{out.vars.netHeight[ni], 1.0}, {out.vars.netTop[ni], -1.0}, {out.vars.netBottom[ni], 1.0}}, 0.0);
    }
    return out;
}

FloorplanProblem prepareProblemForLP(const FloorplanProblem& problem, const SequencePair& sp, const LPOptions& options) {
    if (!options.fixHardOrientationsUsingConstruction) return problem;
    FloorplanProblem oriented = problem;
    const FloorplanSolution constructed = constructByKimKim(problem, sp);
    for (auto& block : oriented.blocks) {
        if (block.type != BlockType::HARD) continue;
        for (const auto& placed : constructed.placements) {
            if (placed.name == block.name) {
                block.width = placed.width;
                block.height = placed.height;
                block.orientation = std::abs(block.width - block.fixedWidth) <= 1e-9 &&
                                    std::abs(block.height - block.fixedHeight) <= 1e-9
                                        ? Orientation::HORIZONTAL
                                        : Orientation::VERTICAL;
                break;
            }
        }
    }
    return oriented;
}

FloorplanSolution optimizeByLP(const FloorplanProblem& problem, const SequencePair& sp, LPSolver& solver, const LPOptions& options) {
    if (!solver.available()) {
        FloorplanSolution s;
        s.status = "LP mode requires an available LP solver";
        return s;
    }
    const FloorplanProblem lpProblem = prepareProblemForLP(problem, sp, options);
    const int n = static_cast<int>(lpProblem.blocks.size());
    std::vector<double> alpha(n, 1.0);
    for (int i = 0; i < n; ++i) {
        if (lpProblem.blocks[i].type == BlockType::SOFT) {
            const double r = std::sqrt(lpProblem.blocks[i].minAspectRatio * lpProblem.blocks[i].maxAspectRatio);
            alpha[i] = clampAlpha(1.0 / std::max(1e-12, r));
        }
    }

    FloorplanSolution last;
    for (int iter = 0; iter < std::max(1, options.maxAreaCorrectionIterations); ++iter) {
        const auto build = buildLPModel(lpProblem, sp, alpha);
        const auto lp = solver.solve(build.model);
        if (!lp.feasible) {
            last.status = lp.status;
            last.objectiveValue = std::numeric_limits<double>::infinity();
            return last;
        }
        std::vector<Block> placed = lpProblem.blocks;
        bool areaOk = true;
        for (int i = 0; i < n; ++i) {
            placed[i].x = lp.values[build.vars.x[i]];
            placed[i].y = lp.values[build.vars.y[i]];
            if (placed[i].type == BlockType::SOFT) {
                placed[i].width = lp.values[build.vars.w[i]];
                placed[i].height = lp.values[build.vars.h[i]];
                const double actualArea = placed[i].width * placed[i].height;
                if (options.verboseAreaCorrection) {
                    std::cerr << "area_correction iter=" << iter
                              << " block=" << placed[i].name
                              << " width=" << placed[i].width
                              << " height=" << placed[i].height
                              << " actual_area=" << actualArea
                              << " required_area=" << placed[i].area
                              << " alpha=" << alpha[i] << "\n";
                }
                if (actualArea + options.areaTolerance < placed[i].area) {
                    areaOk = false;
                    alpha[i] = clampAlpha(placed[i].width / std::max(1e-9, placed[i].height));
                }
            } else {
                placed[i].width = placed[i].width > 0.0 ? placed[i].width : placed[i].fixedWidth;
                placed[i].height = placed[i].height > 0.0 ? placed[i].height : placed[i].fixedHeight;
            }
        }
        const double W = lp.values[build.vars.W];
        const double H = lp.values[build.vars.H];
        last = makeSolution(lpProblem, placed, W, H, areaOk, areaOk ? "lp_optimal" : "lp_area_correction_needed");
        last.objectiveValue = lp.objective;
        if (areaOk) return last;
    }
    last.feasible = false;
    last.status = "lp_area_correction_limit";
    last.objectiveValue = std::numeric_limits<double>::infinity();
    return last;
}

LPBuildResult buildInitialLPModelForExport(const FloorplanProblem& problem, const SequencePair& sp, const LPOptions& options) {
    const FloorplanProblem lpProblem = prepareProblemForLP(problem, sp, options);
    std::vector<double> alpha(lpProblem.blocks.size(), 1.0);
    for (size_t i = 0; i < lpProblem.blocks.size(); ++i) {
        if (lpProblem.blocks[i].type == BlockType::SOFT) {
            const double r = std::sqrt(lpProblem.blocks[i].minAspectRatio * lpProblem.blocks[i].maxAspectRatio);
            alpha[i] = clampAlpha(1.0 / std::max(1e-12, r));
        }
    }
    return buildLPModel(lpProblem, sp, alpha);
}

void writeLPModel(const std::string& path, const LPModel& model) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write LP model: " + path);
    out << std::setprecision(12);
    out << (model.minimize ? "Minimize\n" : "Maximize\n") << " obj:";
    for (size_t i = 0; i < model.variables.size(); ++i) {
        const double c = model.variables[i].objective;
        if (std::abs(c) <= 1e-15) continue;
        out << (c >= 0.0 ? " + " : " - ") << std::abs(c) << " " << lpName(model.variables[i].name);
    }
    out << "\nSubject To\n";
    for (const auto& constraint : model.constraints) {
        out << " " << lpName(constraint.name) << ":";
        for (size_t k = 0; k < constraint.indices.size(); ++k) {
            const double v = constraint.values[k];
            out << (v >= 0.0 ? " + " : " - ") << std::abs(v) << " " << lpName(model.variables[constraint.indices[k]].name);
        }
        if (constraint.sense == ConstraintSense::LessEqual) out << " <= ";
        else if (constraint.sense == ConstraintSense::GreaterEqual) out << " >= ";
        else out << " = ";
        out << constraint.rhs << "\n";
    }
    out << "Bounds\n";
    for (const auto& var : model.variables) {
        const std::string name = lpName(var.name);
        const bool lowerInf = var.lower <= -INF / 2.0 || std::isinf(var.lower);
        const bool upperInf = var.upper >= INF / 2.0 || std::isinf(var.upper);
        if (lowerInf && upperInf) out << " " << name << " free\n";
        else if (std::abs(var.lower - var.upper) <= 1e-12) out << " " << name << " = " << var.lower << "\n";
        else if (lowerInf) out << " " << name << " <= " << var.upper << "\n";
        else if (upperInf) out << " " << var.lower << " <= " << name << "\n";
        else out << " " << var.lower << " <= " << name << " <= " << var.upper << "\n";
    }
    out << "End\n";
}

void writeMPSModel(const std::string& path, const LPModel& model) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write MPS model: " + path);
    out << std::setprecision(12);
    out << "NAME          FLOORPLAN\n";
    out << "ROWS\n";
    out << " N  OBJ\n";
    for (const auto& c : model.constraints) {
        const char rowType = c.sense == ConstraintSense::LessEqual ? 'L' : (c.sense == ConstraintSense::GreaterEqual ? 'G' : 'E');
        out << " " << rowType << "  " << lpName(c.name) << "\n";
    }
    out << "COLUMNS\n";
    for (size_t col = 0; col < model.variables.size(); ++col) {
        const std::string varName = lpName(model.variables[col].name);
        if (std::abs(model.variables[col].objective) > 1e-15) {
            out << "    " << varName << "  OBJ  " << model.variables[col].objective << "\n";
        }
        for (const auto& c : model.constraints) {
            for (size_t k = 0; k < c.indices.size(); ++k) {
                if (c.indices[k] == static_cast<int>(col) && std::abs(c.values[k]) > 1e-15) {
                    out << "    " << varName << "  " << lpName(c.name) << "  " << c.values[k] << "\n";
                }
            }
        }
    }
    out << "RHS\n";
    for (const auto& c : model.constraints) {
        if (std::abs(c.rhs) > 1e-15) out << "    RHS1  " << lpName(c.name) << "  " << c.rhs << "\n";
    }
    out << "BOUNDS\n";
    for (const auto& var : model.variables) {
        const std::string name = lpName(var.name);
        const bool lowerInf = var.lower <= -INF / 2.0 || std::isinf(var.lower);
        const bool upperInf = var.upper >= INF / 2.0 || std::isinf(var.upper);
        if (std::abs(var.lower - var.upper) <= 1e-12) out << " FX BND1  " << name << "  " << var.lower << "\n";
        else {
            if (lowerInf && upperInf) out << " FR BND1  " << name << "\n";
            else {
                if (lowerInf) out << " MI BND1  " << name << "\n";
                else out << " LO BND1  " << name << "  " << var.lower << "\n";
                if (!upperInf) out << " UP BND1  " << name << "  " << var.upper << "\n";
            }
        }
    }
    out << "ENDATA\n";
}

} // namespace fp
