#include "floorplanner/LPSolver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#ifdef FP_WITH_HIGHS
#include <Highs.h>
#endif

namespace fp {

int LPModel::addVariable(const std::string& name, double lower, double upper, double objective) {
    variables.push_back({name, lower, upper, objective});
    return static_cast<int>(variables.size()) - 1;
}

void LPModel::addConstraint(const std::string& name, std::vector<int> indices, std::vector<double> values, ConstraintSense sense, double rhs) {
    constraints.push_back({name, std::move(indices), std::move(values), sense, rhs});
}

UnavailableLPSolver::UnavailableLPSolver(std::string reason) : reason_(std::move(reason)) {}
std::string UnavailableLPSolver::name() const { return "unavailable"; }
bool UnavailableLPSolver::available() const { return false; }
std::string UnavailableLPSolver::unavailableReason() const { return reason_; }
LPSolution UnavailableLPSolver::solve(const LPModel&) {
    return {false, reason_, std::numeric_limits<double>::infinity(), {}};
}

#ifdef FP_WITH_HIGHS
namespace {

class HighsLPSolver final : public LPSolver {
public:
    std::string name() const override { return "highs"; }

    LPSolution solve(const LPModel& model) override {
        Highs highs;
        highs.setOptionValue("output_flag", false);
        HighsLp lp;
        lp.num_col_ = static_cast<HighsInt>(model.variables.size());
        lp.num_row_ = static_cast<HighsInt>(model.constraints.size());
        lp.sense_ = model.minimize ? ObjSense::kMinimize : ObjSense::kMaximize;
        lp.col_cost_.resize(model.variables.size());
        lp.col_lower_.resize(model.variables.size());
        lp.col_upper_.resize(model.variables.size());
        for (size_t i = 0; i < model.variables.size(); ++i) {
            lp.col_cost_[i] = model.variables[i].objective;
            lp.col_lower_[i] = model.variables[i].lower;
            lp.col_upper_[i] = std::isinf(model.variables[i].upper) ? kHighsInf : model.variables[i].upper;
        }
        lp.row_lower_.resize(model.constraints.size(), -kHighsInf);
        lp.row_upper_.resize(model.constraints.size(), kHighsInf);
        std::vector<HighsInt> starts(model.constraints.size() + 1, 0);
        std::vector<HighsInt> indices;
        std::vector<double> values;
        for (size_t r = 0; r < model.constraints.size(); ++r) {
            const auto& c = model.constraints[r];
            if (c.sense == ConstraintSense::LessEqual) lp.row_upper_[r] = c.rhs;
            else if (c.sense == ConstraintSense::GreaterEqual) lp.row_lower_[r] = c.rhs;
            else {
                lp.row_lower_[r] = c.rhs;
                lp.row_upper_[r] = c.rhs;
            }
            starts[r] = static_cast<HighsInt>(indices.size());
            for (size_t k = 0; k < c.indices.size(); ++k) {
                indices.push_back(c.indices[k]);
                values.push_back(c.values[k]);
            }
        }
        starts[model.constraints.size()] = static_cast<HighsInt>(indices.size());
        lp.a_matrix_.format_ = MatrixFormat::kRowwise;
        lp.a_matrix_.start_ = std::move(starts);
        lp.a_matrix_.index_ = std::move(indices);
        lp.a_matrix_.value_ = std::move(values);
        highs.passModel(lp);
        highs.run();
        const HighsModelStatus status = highs.getModelStatus();
        const bool feasible = status == HighsModelStatus::kOptimal;
        LPSolution sol;
        sol.feasible = feasible;
        sol.status = highs.modelStatusToString(status);
        if (feasible) {
            const auto info = highs.getInfo();
            const auto highsSol = highs.getSolution();
            sol.objective = info.objective_function_value;
            sol.values = highsSol.col_value;
        }
        return sol;
    }
};

} // namespace
#endif

std::unique_ptr<LPSolver> createSolver(const std::string& solverName) {
    std::string lower = solverName;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower == "none" || lower == "compact") return std::make_unique<UnavailableLPSolver>("LP solver disabled");
    if (lower == "highs") {
#ifdef FP_WITH_HIGHS
        return std::make_unique<HighsLPSolver>();
#else
        return std::make_unique<UnavailableLPSolver>("HiGHS backend was not enabled at compile time");
#endif
    }
    if (lower == "mosek" || lower == "cplex") {
        return std::make_unique<UnavailableLPSolver>("solver backend is not implemented yet: " + lower);
    }
    return std::make_unique<UnavailableLPSolver>("unknown solver: " + solverName);
}

} // namespace fp
