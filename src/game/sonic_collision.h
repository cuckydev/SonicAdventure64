#ifndef _SONIC_COLLISION_H
#define _SONIC_COLLISION_H

#include "sonic.h"

void Sonic_NormalizeVectorTo(Vector *destination, const Vector *source);
s16 Sonic_VectorAngle(const Vector *from, const Vector *to);
void Sonic_QuaternionAngleAxis(Quaternion *q, s16 angle, const Vector *axis);
void Sonic_PerformCollision(SonicState *s);

#endif
