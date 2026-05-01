#include "floorplanner/IO.h"

#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace fp {
namespace {

struct Json {
    using array = std::vector<Json>;
    using object = std::map<std::string, Json>;
    std::variant<std::nullptr_t, bool, double, std::string, array, object> value;
    bool isObject() const { return std::holds_alternative<object>(value); }
    bool isArray() const { return std::holds_alternative<array>(value); }
    const object& asObject() const { return std::get<object>(value); }
    const array& asArray() const { return std::get<array>(value); }
    const std::string& asString() const { return std::get<std::string>(value); }
    double asNumber() const { return std::get<double>(value); }
};

class Parser {
public:
    explicit Parser(std::string text) : text_(std::move(text)) {}
    Json parse() {
        Json v = parseValue();
        skipWs();
        if (pos_ != text_.size()) throw std::runtime_error("unexpected trailing JSON");
        return v;
    }
private:
    Json parseValue() {
        skipWs();
        if (pos_ >= text_.size()) throw std::runtime_error("unexpected end of JSON");
        if (text_[pos_] == '{') return parseObject();
        if (text_[pos_] == '[') return parseArray();
        if (text_[pos_] == '"') return Json{parseString()};
        if (std::isdigit(static_cast<unsigned char>(text_[pos_])) || text_[pos_] == '-') return Json{parseNumber()};
        if (text_.compare(pos_, 4, "true") == 0) { pos_ += 4; return Json{true}; }
        if (text_.compare(pos_, 5, "false") == 0) { pos_ += 5; return Json{false}; }
        if (text_.compare(pos_, 4, "null") == 0) { pos_ += 4; return Json{nullptr}; }
        throw std::runtime_error("invalid JSON value");
    }
    Json parseObject() {
        Json::object obj;
        expect('{');
        skipWs();
        if (peek('}')) return Json{obj};
        while (true) {
            std::string key = parseString();
            expect(':');
            obj[key] = parseValue();
            skipWs();
            if (peek('}')) break;
            expect(',');
        }
        return Json{obj};
    }
    Json parseArray() {
        Json::array arr;
        expect('[');
        skipWs();
        if (peek(']')) return Json{arr};
        while (true) {
            arr.push_back(parseValue());
            skipWs();
            if (peek(']')) break;
            expect(',');
        }
        return Json{arr};
    }
    std::string parseString() {
        skipWs();
        expect('"');
        std::string s;
        while (pos_ < text_.size() && text_[pos_] != '"') {
            if (text_[pos_] == '\\') {
                ++pos_;
                if (pos_ >= text_.size()) throw std::runtime_error("bad escape");
                const char c = text_[pos_++];
                if (c == '"' || c == '\\' || c == '/') s.push_back(c);
                else if (c == 'n') s.push_back('\n');
                else if (c == 't') s.push_back('\t');
                else throw std::runtime_error("unsupported JSON escape");
            } else {
                s.push_back(text_[pos_++]);
            }
        }
        expect('"');
        return s;
    }
    double parseNumber() {
        const size_t begin = pos_;
        if (text_[pos_] == '-') ++pos_;
        while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        if (pos_ < text_.size() && text_[pos_] == '.') {
            ++pos_;
            while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        }
        if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
            ++pos_;
            if (text_[pos_] == '+' || text_[pos_] == '-') ++pos_;
            while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) ++pos_;
        }
        return std::stod(text_.substr(begin, pos_ - begin));
    }
    void skipWs() {
        while (pos_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[pos_]))) ++pos_;
    }
    void expect(char c) {
        skipWs();
        if (pos_ >= text_.size() || text_[pos_] != c) throw std::runtime_error(std::string("expected '") + c + "'");
        ++pos_;
    }
    bool peek(char c) {
        skipWs();
        if (pos_ < text_.size() && text_[pos_] == c) {
            ++pos_;
            return true;
        }
        return false;
    }
    std::string text_;
    size_t pos_ = 0;
};

const Json* find(const Json::object& obj, const std::string& key) {
    auto it = obj.find(key);
    return it == obj.end() ? nullptr : &it->second;
}

double numberOr(const Json::object& obj, const std::string& key, double fallback) {
    const Json* v = find(obj, key);
    return v ? v->asNumber() : fallback;
}

std::string trim(const std::string& text) {
    size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) ++begin;
    size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) --end;
    return text.substr(begin, end - begin);
}

bool startsWith(const std::string& text, const std::string& prefix) {
    return text.rfind(prefix, 0) == 0;
}

std::string escape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

void writeJsonNumber(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";
}

} // namespace

FloorplanProblem readProblemJson(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open input: " + path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    const Json root = Parser(buffer.str()).parse();
    const auto& obj = root.asObject();
    FloorplanProblem p;
    p.chipAspectLower = numberOr(obj, "chipAspectLower", 0.0);
    p.chipAspectUpper = numberOr(obj, "chipAspectUpper", 1e30);
    p.areaWeight = numberOr(obj, "areaWeight", 1.0);
    p.wireWeight = numberOr(obj, "wireWeight", 1.0);
    if (find(obj, "fixedOutlineWidth") && find(obj, "fixedOutlineHeight")) {
        p.hasFixedOutline = true;
        p.fixedOutlineWidth = numberOr(obj, "fixedOutlineWidth", 0.0);
        p.fixedOutlineHeight = numberOr(obj, "fixedOutlineHeight", 0.0);
    }
    for (const auto& item : find(obj, "blocks")->asArray()) {
        const auto& bobj = item.asObject();
        Block b;
        b.name = find(bobj, "name")->asString();
        b.type = blockTypeFromString(find(bobj, "type")->asString());
        if (b.type == BlockType::HARD) {
            b.fixedWidth = numberOr(bobj, "width", 0.0);
            b.fixedHeight = numberOr(bobj, "height", 0.0);
            b.width = b.fixedWidth;
            b.height = b.fixedHeight;
            b.area = b.width * b.height;
        } else {
            b.area = numberOr(bobj, "area", 0.0);
            b.minAspectRatio = numberOr(bobj, "minAspectRatio", 1.0);
            b.maxAspectRatio = numberOr(bobj, "maxAspectRatio", 1.0);
            const double r = std::sqrt(b.minAspectRatio * b.maxAspectRatio);
            b.width = std::sqrt(b.area / r);
            b.height = std::sqrt(b.area * r);
        }
        p.blocks.push_back(b);
    }
    p.rebuildIndex();
    if (const Json* nets = find(obj, "nets")) {
        for (const auto& item : nets->asArray()) {
            const auto& nobj = item.asObject();
            Net net;
            net.name = find(nobj, "name")->asString();
            for (const auto& bname : find(nobj, "blocks")->asArray()) {
                auto it = p.blockNameToId.find(bname.asString());
                if (it == p.blockNameToId.end()) throw std::runtime_error("net references unknown block: " + bname.asString());
                net.blockIds.push_back(it->second);
            }
            if (const Json* pads = find(nobj, "pads")) {
                for (const auto& padJson : pads->asArray()) {
                    const auto& pobj = padJson.asObject();
                    net.pads.push_back({numberOr(pobj, "x", 0.0), numberOr(pobj, "y", 0.0)});
                }
            }
            p.nets.push_back(net);
        }
    }
    p.rebuildIndex();
    return p;
}

FloorplanProblem readMcncBenchmark(const std::string& blockPath, const std::string& netsPath) {
    FloorplanProblem p;
    std::unordered_map<std::string, PadLocation> padsByName;

    std::ifstream blockIn(blockPath);
    if (!blockIn) throw std::runtime_error("cannot open MCNC block file: " + blockPath);

    std::string line;
    int expectedBlocks = -1;
    int expectedTerminals = -1;
    while (std::getline(blockIn, line)) {
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;
        std::istringstream iss(line);
        std::string first;
        iss >> first;
        if (first == "Outline:") {
            p.hasFixedOutline = true;
            iss >> p.fixedOutlineWidth >> p.fixedOutlineHeight;
            p.chipAspectLower = p.fixedOutlineHeight / std::max(1e-12, p.fixedOutlineWidth);
            p.chipAspectUpper = p.chipAspectLower;
        } else if (first == "NumBlocks:") {
            iss >> expectedBlocks;
        } else if (first == "NumTerminals:") {
            iss >> expectedTerminals;
        } else {
            std::string second;
            iss >> second;
            if (second == "terminal") {
                PadLocation pad;
                iss >> pad.x >> pad.y;
                padsByName[first] = pad;
            } else {
                Block b;
                b.name = first;
                if (second == "soft" || second == "SOFT") {
                    b.type = BlockType::SOFT;
                    iss >> b.area >> b.minAspectRatio >> b.maxAspectRatio;
                    const double r = std::sqrt(b.minAspectRatio * b.maxAspectRatio);
                    b.width = std::sqrt(b.area / std::max(1e-12, r));
                    b.height = std::sqrt(b.area * r);
                } else {
                    b.type = BlockType::HARD;
                    b.fixedWidth = std::stod(second);
                    iss >> b.fixedHeight;
                    b.width = b.fixedWidth;
                    b.height = b.fixedHeight;
                    b.area = b.width * b.height;
                }
                p.blocks.push_back(b);
            }
        }
    }
    p.rebuildIndex();
    if (expectedBlocks >= 0 && expectedBlocks != static_cast<int>(p.blocks.size())) {
        throw std::runtime_error("MCNC block count mismatch in " + blockPath);
    }
    if (expectedTerminals >= 0 && expectedTerminals != static_cast<int>(padsByName.size())) {
        throw std::runtime_error("MCNC terminal count mismatch in " + blockPath);
    }

    std::ifstream netsIn(netsPath);
    if (!netsIn) throw std::runtime_error("cannot open MCNC nets file: " + netsPath);

    int expectedNets = -1;
    int netId = 0;
    while (std::getline(netsIn, line)) {
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;
        if (startsWith(line, "NumNets:")) {
            std::istringstream iss(line);
            std::string label;
            iss >> label >> expectedNets;
            continue;
        }
        if (!startsWith(line, "NetDegree:")) {
            throw std::runtime_error("expected NetDegree in MCNC nets file near: " + line);
        }
        std::istringstream degreeStream(line);
        std::string label;
        int degree = 0;
        degreeStream >> label >> degree;
        Net net;
        net.id = netId;
        net.name = "net" + std::to_string(netId);
        for (int k = 0; k < degree; ++k) {
            if (!std::getline(netsIn, line)) throw std::runtime_error("unexpected EOF in MCNC nets file");
            const std::string pinName = trim(line);
            auto blockIt = p.blockNameToId.find(pinName);
            if (blockIt != p.blockNameToId.end()) {
                net.blockIds.push_back(blockIt->second);
                continue;
            }
            auto padIt = padsByName.find(pinName);
            if (padIt != padsByName.end()) {
                net.pads.push_back(padIt->second);
                continue;
            }
            throw std::runtime_error("MCNC net references unknown block/terminal: " + pinName);
        }
        p.nets.push_back(std::move(net));
        ++netId;
    }
    if (expectedNets >= 0 && expectedNets != static_cast<int>(p.nets.size())) {
        throw std::runtime_error("MCNC net count mismatch in " + netsPath);
    }
    p.rebuildIndex();
    return p;
}

void writePlacementCsv(const std::string& path, const FloorplanSolution& solution) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write CSV: " + path);
    out << "block_name,x,y,width,height,type,layer\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& b : solution.placements) {
        out << b.name << "," << b.x << "," << b.y << "," << b.width << "," << b.height << "," << toString(b.type) << ",0\n";
    }
}

void writeSummaryJson(const std::string& path, const FloorplanSolution& s, const SequencePair& sp, double runtimeSeconds, const RunMetadata& metadata) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write summary: " + path);
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"objective\": "; writeJsonNumber(out, s.objectiveValue); out << ",\n";
    out << "  \"chipWidth\": "; writeJsonNumber(out, s.chipWidth); out << ",\n";
    out << "  \"chipHeight\": "; writeJsonNumber(out, s.chipHeight); out << ",\n";
    out << "  \"chipArea\": "; writeJsonNumber(out, s.chipArea); out << ",\n";
    out << "  \"totalWirelength\": "; writeJsonNumber(out, s.totalWirelength); out << ",\n";
    out << "  \"runtimeSeconds\": "; writeJsonNumber(out, runtimeSeconds); out << ",\n";
    out << "  \"feasible\": " << (s.feasible ? "true" : "false") << ",\n";
    out << "  \"status\": \"" << escape(s.status) << "\",\n";
    out << "  \"mode\": \"" << escape(metadata.mode) << "\",\n";
    out << "  \"solver\": \"" << escape(metadata.solver) << "\",\n";
    out << "  \"iterations\": " << metadata.iterations << ",\n";
    out << "  \"seed\": " << metadata.seed << ",\n";
    out << "  \"epochLength\": " << metadata.epochLength << ",\n";
    out << "  \"initialTemperature\": "; writeJsonNumber(out, metadata.initialTemperature); out << ",\n";
    out << "  \"initialTemperatureUsed\": "; writeJsonNumber(out, metadata.initialTemperatureUsed); out << ",\n";
    out << "  \"epochLengthUsed\": " << metadata.epochLengthUsed << ",\n";
    out << "  \"autoTemperature\": " << (metadata.autoTemperature ? "true" : "false") << ",\n";
    out << "  \"verboseAnnealing\": " << (metadata.verboseAnnealing ? "true" : "false") << ",\n";
    out << "  \"totalMoves\": " << metadata.totalMoves << ",\n";
    out << "  \"acceptedMoves\": " << metadata.acceptedMoves << ",\n";
    out << "  \"coolingRatio\": "; writeJsonNumber(out, metadata.coolingRatio); out << ",\n";
    out << "  \"numBlocks\": " << metadata.numBlocks << ",\n";
    out << "  \"numNets\": " << metadata.numNets << ",\n";
    out << "  \"hasFixedOutline\": " << (metadata.hasFixedOutline ? "true" : "false") << ",\n";
    out << "  \"fixedOutlineWidth\": "; writeJsonNumber(out, metadata.fixedOutlineWidth); out << ",\n";
    out << "  \"fixedOutlineHeight\": "; writeJsonNumber(out, metadata.fixedOutlineHeight); out << ",\n";
    out << "  \"chipAspectLower\": "; writeJsonNumber(out, metadata.chipAspectLower); out << ",\n";
    out << "  \"chipAspectUpper\": "; writeJsonNumber(out, metadata.chipAspectUpper); out << ",\n";
    out << "  \"gammaPlus\": [";
    for (size_t i = 0; i < sp.gammaPlus.size(); ++i) out << (i ? ", " : "") << sp.gammaPlus[i];
    out << "],\n  \"gammaMinus\": [";
    for (size_t i = 0; i < sp.gammaMinus.size(); ++i) out << (i ? ", " : "") << sp.gammaMinus[i];
    out << "]\n}\n";
}

void printSolution(const FloorplanSolution& s, const SequencePair& sp, double runtimeSeconds) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "objective: " << s.objectiveValue << "\n";
    std::cout << "chip_width: " << s.chipWidth << "\n";
    std::cout << "chip_height: " << s.chipHeight << "\n";
    std::cout << "chip_area: " << s.chipArea << "\n";
    std::cout << "wirelength: " << s.totalWirelength << "\n";
    std::cout << "runtime_seconds: " << runtimeSeconds << "\n";
    std::cout << "feasible: " << (s.feasible ? "true" : "false") << "\n";
    std::cout << "status: " << s.status << "\n";
    std::cout << "sequence_pair: " << sp.toString() << "\n";
    std::cout << "blocks:\n";
    for (const auto& b : s.placements) {
        std::cout << "  " << b.name << " x=" << b.x << " y=" << b.y << " w=" << b.width << " h=" << b.height << " type=" << toString(b.type) << "\n";
    }
}

} // namespace fp
