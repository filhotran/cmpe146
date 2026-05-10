/*
 * fusion.h
 * Complementary filter for pitch and roll sensor fusion.
 *
 * HOW IT WORKS:
 *   The accelerometer gives absolute tilt (from gravity) but is noisy.
 *   The gyroscope gives smooth rotation rate but drifts over time.
 *   The complementary filter blends both:
 *
 *     angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
 *
 *   With alpha = 0.98:
 *     - 98% trust in gyro (smooth, responsive)
 *     - 2% trust in accel (corrects drift)
 *
 * IMPORTANT: We use single-precision math (atan2f, sqrtf) everywhere
 *   because the Cortex-M0+ has no FPU. Double-precision would be ~2x slower.
 */

#ifndef FUSION_H
#define FUSION_H

#include "imu_common.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.14159265f)

/* Filter state */
typedef struct {
    float pitch;
    float roll;
    float yaw;
    bool  initialized;
} FusionState_t;

/* Initialize filter to zero */
static inline void fusion_init(FusionState_t *state)
{
    state->pitch = 0.0f;
    state->roll  = 0.0f;
    state->yaw   = 0.0f;
    state->initialized = false;
}

/*
 * Compute pitch from accelerometer only.
 * pitch = atan2(ax, sqrt(ay^2 + az^2))
 * Correct but noisy — vibration causes spikes.
 */
static inline float compute_accel_pitch(float ax, float ay, float az)
{
    return atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

/*
 * Compute roll from accelerometer only.
 * roll = atan2(ay, sqrt(ax^2 + az^2))
 */
static inline float compute_accel_roll(float ax, float ay, float az)
{
    return atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
}

/*
 * Run one complementary filter update.
 *
 * On the first call, initializes directly from accelerometer
 * so the filter doesn't have to converge from zero.
 */
static inline void fusion_update_complementary(
    FusionState_t *state,
    const SensorSample_t *sample,
    float alpha,
    float dt)
{
    float pitch_accel = compute_accel_pitch(
        sample->accel_x, sample->accel_y, sample->accel_z);
    float roll_accel = compute_accel_roll(
        sample->accel_x, sample->accel_y, sample->accel_z);

    if (!state->initialized)
    {
        state->pitch = pitch_accel;
        state->roll  = roll_accel;
        state->yaw   = 0.0f;
        state->initialized = true;
        return;
    }

    /* The core formula */
    state->pitch = alpha * (state->pitch + sample->gyro_x * dt)
                 + (1.0f - alpha) * pitch_accel;

    state->roll  = alpha * (state->roll + sample->gyro_y * dt)
                 + (1.0f - alpha) * roll_accel;

    /* Yaw: gyro only, will drift without magnetometer */
    state->yaw += sample->gyro_z * dt;
}

/* Copy current orientation into output struct */
static inline void fusion_get_orientation(
    const FusionState_t *state,
    FusedOrientation_t *orient,
    uint32_t timestamp)
{
    orient->pitch     = state->pitch;
    orient->roll      = state->roll;
    orient->yaw       = state->yaw;
    orient->timestamp = timestamp;
}

#endif /* FUSION_H */
