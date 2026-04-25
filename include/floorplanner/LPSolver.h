#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace fp {

enum class ConstraintSense { LessEqual, GreaterEqual, Equal };

struct LPVariable {
    std::string name;
    double lower = 0.0;
    double upper = std::numeric_limits<double>::infinity();
    double objective = 0.0;
};

struct LPConstraint {
    std::string name;
    std::vector<int> indices;
    std::vector<double> values;
    ConstraintSense sense = ConstraintSense::LessEqual;
    double rhs = 0.0;
};

struct LPModel {
    std::vector<LPVariable> variables;
    std::vector<LPConstraint> constraints;
    bool minimize = true;

    int addVariable(const std::string& name, double lower, double upper, double objective);
    void addConstraint(const std::string& name, std::vector<int> indices, std::vector<double> values, ConstraintSense sense, double rhs);
};

struct LPSolution {
    bool feasible = false;
    std::string status;
    double objective = std::numeric_limits<double>::infinity();
    std::vector<double> values;
};

class LPSolver {
public:
    virtual ~LPSolver() = default;
    virtual std::string name() const = 0;
    virtual bool available() const { return true; }
    virtual std::string unavailableReason() const { return {}; }
    virtual LPSolution solve(const LPModel& model) = 0;
};

class UnavailableLPSolver final : public LPSolver {
public:
    explicit UnavailableLPSolver(std::string reason);
    std::string name() const override;
    bool available() const override;
    std::string unavailableReason() const override;
    LPSolution solve(const LPModel& model) override;

private:
    std::string reason_;
};

std::unique_ptr<LPSolver> createSolver(const std::string& solverName);

} // namespace fp
