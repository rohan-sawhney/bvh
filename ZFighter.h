#ifndef Z_FIGHTER_H
#define Z_FIGHTER_H

#include "Types.h"

class ZFighter {
public:
    // solves z fighting by shifting overlapping triangles 
    void process(Mesh& mesh) const;
};

#endif
