#pragma once

#include <random>
#include <string>
#include <vector>

namespace fp {

enum class Relation { LEFT_OF, RIGHT_OF, BELOW, ABOVE };

struct PairRelation {
    int i = -1;
    int j = -1;
    Relation relation = Relation::LEFT_OF;
};

class SequencePair {
public:
    std::vector<int> gammaPlus;
    std::vector<int> gammaMinus;

    SequencePair() = default;
    explicit SequencePair(int n);
    SequencePair(std::vector<int> plus, std::vector<int> minus);

    static SequencePair identity(int n);
    static SequencePair random(int n, std::mt19937& rng);

    bool validate(int n) const;
    std::vector<int> inversePlus() const;
    std::vector<int> inverseMinus() const;
    Relation relation(int i, int j) const;
    std::vector<PairRelation> allRelations() const;

    void mutate(std::mt19937& rng);
    void move1SwapBoth(int a, int b);
    void move2SwapOne(int a, int b, bool usePlus);
    void move3MoveBefore(int movedBlock, int beforeBlock, bool usePlus);

    std::string toString() const;

private:
    static void swapBlocks(std::vector<int>& sequence, int a, int b);
    static void moveBefore(std::vector<int>& sequence, int movedBlock, int beforeBlock);
};

std::string toString(Relation relation);

} // namespace fp
