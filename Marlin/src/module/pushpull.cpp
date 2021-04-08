#include "pushpull.h"

namespace pushPull{

Position operator+(const Position& a, const Position& b){
    return Position{a.x+b.x, a.y+b.y, a.z+b.z};
}
void operator+=(Position& a, const Position& b){
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
Position operator-(const Position& a, const Position& b){
    return Position{a.x-b.x, a.y-b.y, a.z-b.z};
}
void operator-=(Position& a, const Position& b){
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}

//TODO Rearrange for overflow
float distance(const Position& a, const Position& b){
    return std::sqrt(std::pow(a.x-b.x,2) + std::pow(a.y-b.y,2) + std::pow(a.z-b.z,2));
}
float distance(const Position& a){
    return std::sqrt(std::pow(a.x,2) + std::pow(a.y,2) + std::pow(a.z,2));
}

abce_long_t convertToSteps(const float &a, const float &b, const float &c, const float &e, const uint8_t extruder){
    std::array<float, axis.size()> positions = convertToMm(a,b,c);
    abce_long_t result;

    result.e = int32_t(LROUND(e * planner.settings.axis_steps_per_mm[E_AXIS_N(extruder)]));
    for(unsigned char i = 0; i<axis.size(); i++){
        switch(axis[i].hardwareAxis){
            case X_AXIS:
                result.x = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[X_AXIS]));
                break;
            case Y_AXIS:
                result.y = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[Y_AXIS]));
                break;
            case Z_AXIS:
                result.z = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[Z_AXIS]));
                break;
            #if NON_E_AXES >= 4
            case I_AXIS:
                result.i = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[I_AXIS]));
                break;
            #if NON_E_AXES >= 5
            case J_AXIS:
                result.j = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[J_AXIS]));
                break;
            #if NON_E_AXES >= 6
            case K_AXIS:
                result.k = int32_t(LROUND(positions[i] * planner.settings.axis_steps_per_mm[K_AXIS]));
                break;
            #endif
            #endif
            #endif
            default:
                SERIAL_ECHOLN("Error in pushpull.cpp");
        }
    }

    if constexpr(debug){
        SERIAL_ECHOLN("Requested Position (mm)");
        SERIAL_ECHO("X:");
        SERIAL_ECHO(a);
        SERIAL_ECHO(" Y:");
        SERIAL_ECHO(b);
        SERIAL_ECHO(" Z:");
        SERIAL_ECHOLN(c);

        SERIAL_ECHOLN("Calculated Position (mm)");
        SERIAL_ECHO("X:");
        SERIAL_ECHO(positions[0]);
        SERIAL_ECHO(" Y:");
        SERIAL_ECHO(positions[1]);
        SERIAL_ECHO(" Z:");
        SERIAL_ECHO(positions[2]);
        SERIAL_ECHO(" I:");
        SERIAL_ECHO(positions[3]);
        SERIAL_ECHO(" J:");
        SERIAL_ECHOLN(positions[4]);


        SERIAL_ECHOLN("Calculated Position (steps)");
        SERIAL_ECHO("X:");
        SERIAL_ECHO(result.x);
        SERIAL_ECHO(" Y:");
        SERIAL_ECHO(result.y);
        SERIAL_ECHO(" Z:");
        SERIAL_ECHO(result.z);
        SERIAL_ECHO(" I:");
        SERIAL_ECHO(result.i);
        SERIAL_ECHO(" J:");
        SERIAL_ECHOLN(result.j);
    }
    
    return result;
};

std::array<float, axis.size()> convertToMm(const float &a, const float &b, const float &c){
    Position core{a,b,c};
    std::array<float, axis.size()> positions;
    for(unsigned char i = 0; i<axis.size(); i++){
        positions[i] = distance(core+axis[i].coreOffset, axis[i].origin);
    }
    return positions;
    //return {0,0,0,0,0};
}

}