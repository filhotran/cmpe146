/*
 * fusion.h
 * Complementary filter for pitch and roll.
 */

#ifndef FUSION_H
#define FUSION_H

#include "imu_common.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.14159265f)

typedef struct {
    float pitch, roll, yaw;
    bool initialized;
} FusionState_t;

static inline void fusion_init(FusionState_t *f)
{
    f->pitch = 0;
    f->roll  = 0;
    f->yaw   = 0;
    f->initialized = false;
}

static inline void fusion_update(FusionState_t *f, const SensorSample_t *s,
    float alpha, float dt)
{
    /* Tilt angles from accelerometer */
    float p_acc = atan2f(s->ax, sqrtf(s->ay * s->ay + s->az * s->az)) * RAD_TO_DEG;
    float r_acc = atan2f(s->ay, sqrtf(s->ax * s->ax + s->az * s->az)) * RAD_TO_DEG;

    if (!f->initialized) {
        f->pitch = p_acc;
        f->roll  = r_acc;
        f->initialized = true;
        return;
    }

    /* Complementary filter */
    f->pitch = alpha * (f->pitch + s->gx * dt) + (1.0f - alpha) * p_acc;
    f->roll  = alpha * (f->roll  + s->gy * dt) + (1.0f - alpha) * r_acc;
    f->yaw  += s->gz * dt;
}

#endif
