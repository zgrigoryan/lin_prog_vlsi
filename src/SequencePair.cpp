#include "floorplanner/SequencePair.h"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace fp {

SequencePair::SequencePair(int n) : gammaPlus(n), gammaMinus(n) {
    std::iota(gammaPlus.begin(), gammaPlus.end(), 0);
    gammaMinus = gammaPlus;
}

SequencePair::SequencePair(std::vector<int> plus, std::vector<int> minus)
    : gammaPlus(std::move(plus)), gammaMinus(std::move(minus)) {}

SequencePair SequencePair::identity(int n) {
    return SequencePair(n);
}

SequencePair SequencePair::random(int n, std::mt19937& rng) {
    SequencePair sp(n);
    std::shuffle(sp.gammaPlus.begin(), sp.gammaPlus.end(), rng);
    std::shuffle(sp.gammaMinus.begin(), sp.gammaMinus.end(), rng);
    return sp;
}

bool SequencePair::validate(int n) const {
    auto check = [n](const std::vector<int>& seq) {
        if (static_cast<int>(seq.size()) != n) return false;
        std::vector<int> seen(n, 0);
        for (int value : seq) {
            if (value < 0 || value >= n || seen[value]) return false;
            seen[value] = 1;
        }
        return true;
    };
    return check(gammaPlus) && check(gammaMinus);
}

std::vector<int> SequencePair::inversePlus() const {
    std::vector<int> inv(gammaPlus.size());
    for (int i = 0; i < static_cast<int>(gammaPlus.size()); ++i) inv[gammaPlus[i]] = i;
    return inv;
}

std::vector<int> SequencePair::inverseMinus() const {
    std::vector<int> inv(gammaMinus.size());
    for (int i = 0; i < static_cast<int>(gammaMinus.size()); ++i) inv[gammaMinus[i]] = i;
    return inv;
}

Relation SequencePair::relation(int i, int j) const {
    // Convenience method for tests and one-off queries. Hot paths should call
    // allRelations(), which computes inverse permutations once for all pairs.
    const auto p = inversePlus();
    const auto m = inverseMinus();
    const bool iBeforePlus = p[i] < p[j];
    const bool iBeforeMinus = m[i] < m[j];
    if (iBeforePlus && iBeforeMinus) return Relation::LEFT_OF;
    if (!iBeforePlus && !iBeforeMinus) return Relation::RIGHT_OF;
    if (!iBeforePlus && iBeforeMinus) return Relation::BELOW;
    return Relation::ABOVE;
}

std::vector<PairRelation> SequencePair::allRelations() const {
    // Sequence-pair topology interpretation: equal order in both permutations
    // gives horizontal precedence, while opposite order gives vertical
    // precedence.
    const int n = static_cast<int>(gammaPlus.size());
    const auto p = inversePlus();
    const auto m = inverseMinus();
    std::vector<PairRelation> out;
    out.reserve(n * (n - 1) / 2);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const bool iBeforePlus = p[i] < p[j];
            const bool iBeforeMinus = m[i] < m[j];
            Relation r = Relation::LEFT_OF;
            if (iBeforePlus && iBeforeMinus) r = Relation::LEFT_OF;
            else if (!iBeforePlus && !iBeforeMinus) r = Relation::RIGHT_OF;
            else if (!iBeforePlus && iBeforeMinus) r = Relation::BELOW;
            else r = Relation::ABOVE;
            out.push_back({i, j, r});
        }
    }
    return out;
}

void SequencePair::mutate(std::mt19937& rng) {
    const int n = static_cast<int>(gammaPlus.size());
    if (n < 2) return;
    std::uniform_int_distribution<int> blockDist(0, n - 1);
    int a = blockDist(rng);
    int b = blockDist(rng);
    while (b == a) b = blockDist(rng);
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    const double r = prob(rng);
    if (r < 0.3) {
        move1SwapBoth(a, b);
    } else if (r < 0.7) {
        move2SwapOne(a, b, prob(rng) < 0.5);
    } else {
        move3MoveBefore(a, b, prob(rng) < 0.5);
    }
}

void SequencePair::move1SwapBoth(int a, int b) {
    swapBlocks(gammaPlus, a, b);
    swapBlocks(gammaMinus, a, b);
}

void SequencePair::move2SwapOne(int a, int b, bool usePlus) {
    swapBlocks(usePlus ? gammaPlus : gammaMinus, a, b);
}

void SequencePair::move3MoveBefore(int movedBlock, int beforeBlock, bool usePlus) {
    moveBefore(usePlus ? gammaPlus : gammaMinus, movedBlock, beforeBlock);
}

std::string SequencePair::toString() const {
    auto seqToString = [](const std::vector<int>& seq) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < seq.size(); ++i) {
            if (i) oss << ",";
            oss << seq[i];
        }
        oss << "]";
        return oss.str();
    };
    return "plus=" + seqToString(gammaPlus) + " minus=" + seqToString(gammaMinus);
}

void SequencePair::swapBlocks(std::vector<int>& sequence, int a, int b) {
    auto ia = std::find(sequence.begin(), sequence.end(), a);
    auto ib = std::find(sequence.begin(), sequence.end(), b);
    if (ia != sequence.end() && ib != sequence.end()) std::iter_swap(ia, ib);
}

void SequencePair::moveBefore(std::vector<int>& sequence, int movedBlock, int beforeBlock) {
    if (movedBlock == beforeBlock) return;
    auto im = std::find(sequence.begin(), sequence.end(), movedBlock);
    auto ib = std::find(sequence.begin(), sequence.end(), beforeBlock);
    if (im == sequence.end() || ib == sequence.end()) return;
    sequence.erase(im);
    ib = std::find(sequence.begin(), sequence.end(), beforeBlock);
    sequence.insert(ib, movedBlock);
}

std::string toString(Relation relation) {
    switch (relation) {
        case Relation::LEFT_OF: return "LEFT_OF";
        case Relation::RIGHT_OF: return "RIGHT_OF";
        case Relation::BELOW: return "BELOW";
        case Relation::ABOVE: return "ABOVE";
    }
    return "UNKNOWN";
}

} // namespace fp
