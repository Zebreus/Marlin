#pragma once

#include "../MarlinCore.h"
#include "../core/types.h"
#include "../core/macros.h"
#include "settings.h"
#include "planner.h"
#include "../module/motion.h"
#include <array>
#include <cmath>
/*

*/

namespace pushPull{

// in mm
struct Position{
    double x;
    double y;
    double z;
};

enum Direction{
    PUSH,
    PULL
};

struct Axis{
    Direction direction;
    Position origin;
    Position coreOffset;
    AxisEnum hardwareAxis;
};

using StepperPositions = std::array<int,5>;

constexpr bool debug = true;

constexpr std::array<Axis,5> axis{
    Axis{PULL, {  0,   0, 100}, {-0.7071*39.3, -0.7071*39.3, 0}, X_AXIS},
    Axis{PULL, {200,   0, 100}, { 0.7071*39.3, -0.7071*39.3, 0}, Y_AXIS},
    Axis{PULL, {200, 200, 100}, { 0.7071*39.3,  0.7071*39.3, 0}, Z_AXIS},
    Axis{PULL, {  0, 200, 100}, {-0.7071*39.3,  0.7071*39.3, 0}, I_AXIS},
    Axis{PUSH, {100, 100, 100}, { 0          ,            0, 0}, J_AXIS}
};

Position operator+(const Position& a, const Position& b);
void operator+=(Position& a, const Position& b);
Position operator-(const Position& a, const Position& b);
void operator-=(Position& a, const Position& b);

float distance(const Position& a, const Position& b);
float distance(const Position& a);

abce_long_t convertToSteps(const float &a, const float &b, const float &c, const float &e, const uint8_t extruder);
std::array<float, axis.size()> convertToMm(const float &a, const float &b, const float &c);
}

