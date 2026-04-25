#include "floorplanner/DataModel.h"

#include <algorithm>
#include <stdexcept>

namespace fp {

void FloorplanProblem::rebuildIndex() {
    blockNameToId.clear();
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
        blocks[i].id = i;
        blockNameToId[blocks[i].name] = i;
        if (blocks[i].type == BlockType::HARD) {
            blocks[i].area = blocks[i].fixedWidth * blocks[i].fixedHeight;
            if (blocks[i].width <= 0.0) blocks[i].width = blocks[i].fixedWidth;
            if (blocks[i].height <= 0.0) blocks[i].height = blocks[i].fixedHeight;
        }
    }
    for (int i = 0; i < static_cast<int>(nets.size()); ++i) nets[i].id = i;
}

std::string toString(BlockType type) {
    return type == BlockType::HARD ? "HARD" : "SOFT";
}

std::string toString(Orientation orientation) {
    return orientation == Orientation::HORIZONTAL ? "HORIZONTAL" : "VERTICAL";
}

BlockType blockTypeFromString(const std::string& text) {
    std::string upper = text;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (upper == "HARD") return BlockType::HARD;
    if (upper == "SOFT") return BlockType::SOFT;
    throw std::runtime_error("unknown block type: " + text);
}

} // namespace fp
