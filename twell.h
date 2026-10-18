/*
 * Usage:
 *   #define TWELL_IMPL
 *   #include "twell.h"
 */

#ifndef TWELL_H
#define TWELL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TWELL_API
#  if defined(_WIN32) || defined(__CYGWIN__)
#    if defined(TWELL_BUILD_DLL) || defined(TWELL_IMPL)
#      define TWELL_API __declspec(dllexport)
#    else
#      define TWELL_API
#    endif
#  elif defined(__GNUC__) && __GNUC__ >= 4
#    define TWELL_API __attribute__((visibility("default")))
#  else
#    define TWELL_API
#  endif
#endif

#define TWELL_MAX_IMPULSES 8
#define TWELL_MAX_DRIVERS_PER_PROP 4
#define TWELL_GESTURE_HISTORY_SIZE 16
#define TWELL_INVALID_ID ((uint32_t)-1)

typedef uint32_t twell_property_id;
typedef uint32_t twell_gesture_id;

typedef struct twell_context twell_context;

typedef enum {
    TWELL_UNIT_PIXELS,     /* Default: eps_d = 0.1, eps_v = 0.5 */
    TWELL_UNIT_NORMALIZED, /* Default: eps_d = 0.0001, eps_v = 0.001 */
    TWELL_UNIT_DEGREES,    /* Default: eps_d = 0.05, eps_v = 0.1 */
    TWELL_UNIT_RADIANS,    /* Default: eps_d = 0.001, eps_v = 0.005 */
    TWELL_UNIT_CUSTOM      /* User-defined epsilons */
} twell_unit_type;

typedef enum {
    TWELL_IMPULSE_SPRING,
    TWELL_IMPULSE_DECAY
} twell_impulse_type;

typedef enum {
    TWELL_MAP_LINEAR,
    TWELL_MAP_EASE_IN,
    TWELL_MAP_EASE_OUT
} twell_mapping_curve;

typedef struct {
    double mass;
    double stiffness;
    double damping;
    double initial_velocity;
} twell_spring_config;

typedef struct { double x; double y; } twell_vector2;
typedef struct { double x; double y; double z; } twell_vector3;

typedef struct {
    double m11, m12, m13, m14;
    double m21, m22, m23, m24;
    double m31, m32, m33, m34;
    double m41, m42, m43, m44;
} twell_transform3d;

typedef struct {
    double x, y, z, w;
} twell_quaternion;

/* Core & Context */
TWELL_API size_t twell_get_memory_requirement(uint32_t max_properties, uint32_t max_gestures);

TWELL_API twell_context* twell_context_create(
    void* arena_memory,
    size_t arena_size_bytes,
    uint32_t max_properties,
    uint32_t max_gestures
);

TWELL_API uint32_t twell_context_tick(
    twell_context* ctx,
    double absolute_time,
    twell_property_id* out_resting,
    uint32_t max_resting
);

/* Analytical math kernel */
TWELL_API void twell_math_spring_evaluate(
    double t,
    twell_spring_config config,
    double initial_displacement,
    double* out_displacement,
    double* out_velocity
);

TWELL_API void twell_math_decay_evaluate(
    double t,
    double initial_velocity,
    double deceleration_rate,
    double* out_displacement,
    double* out_velocity
);

TWELL_API double twell_math_rubber_band(
    double input_displacement,
    double max_displacement,
    double tension
);

TWELL_API void twell_math_set_rest_thresholds(
    twell_context* ctx,
    double displacement_epsilon,
    double velocity_epsilon
);

/* Additive state machine, property creation */
TWELL_API twell_property_id twell_property_create(twell_context* ctx, double initial_value);
TWELL_API twell_property_id twell_property_create_with_unit(twell_context* ctx, double initial_value, twell_unit_type unit);
TWELL_API twell_property_id twell_property_create_2d(twell_context* ctx, twell_vector2 initial_value);
TWELL_API twell_property_id twell_property_create_2d_with_unit(twell_context* ctx, twell_vector2 initial_value, twell_unit_type unit);
TWELL_API twell_property_id twell_property_create_3d(twell_context* ctx, twell_vector3 initial_value);
TWELL_API twell_property_id twell_property_create_3d_with_unit(twell_context* ctx, twell_vector3 initial_value, twell_unit_type unit);

TWELL_API void twell_property_destroy(twell_context* ctx, twell_property_id id);

TWELL_API void twell_property_set_rest_thresholds(
    twell_context* ctx,
    twell_property_id id,
    double displacement_epsilon,
    double velocity_epsilon
);

TWELL_API double twell_property_get_model_value(twell_context* ctx, twell_property_id id);
TWELL_API twell_vector2 twell_property_get_model_value_2d(twell_context* ctx, twell_property_id id);
TWELL_API twell_vector3 twell_property_get_model_value_3d(twell_context* ctx, twell_property_id id);

TWELL_API double twell_property_get_presentation_value(twell_context* ctx, twell_property_id id);
TWELL_API twell_vector2 twell_property_get_presentation_value_2d(twell_context* ctx, twell_property_id id);
TWELL_API twell_vector3 twell_property_get_presentation_value_3d(twell_context* ctx, twell_property_id id);

TWELL_API void twell_property_set_value_immediate(twell_context* ctx, twell_property_id id, double value);
TWELL_API void twell_property_set_value_immediate_2d(twell_context* ctx, twell_property_id id, twell_vector2 value);
TWELL_API void twell_property_set_value_immediate_3d(twell_context* ctx, twell_property_id id, twell_vector3 value);

TWELL_API void twell_property_animate_to_target(
    twell_context* ctx,
    twell_property_id id,
    double target_value,
    twell_spring_config config,
    double absolute_time
);
TWELL_API void twell_property_animate_to_target_2d(
    twell_context* ctx,
    twell_property_id id,
    twell_vector2 target_value,
    twell_spring_config config,
    double absolute_time
);
TWELL_API void twell_property_animate_to_target_3d(
    twell_context* ctx,
    twell_property_id id,
    twell_vector3 target_value,
    twell_spring_config config,
    double absolute_time
);

TWELL_API void twell_property_animate_decay(
    twell_context* ctx,
    twell_property_id id,
    double initial_velocity,
    double deceleration_rate,
    double absolute_time
);

/* Gesture kinetics */
TWELL_API twell_gesture_id twell_gesture_create(twell_context* ctx);
TWELL_API void twell_gesture_destroy(twell_context* ctx, twell_gesture_id id);

TWELL_API void twell_gesture_add_touch(
    twell_context* ctx,
    twell_gesture_id id,
    double position,
    double absolute_time
);
TWELL_API void twell_gesture_add_touch_2d(
    twell_context* ctx,
    twell_gesture_id id,
    twell_vector2 position,
    double absolute_time
);
TWELL_API void twell_gesture_add_touch_3d(
    twell_context* ctx,
    twell_gesture_id id,
    twell_vector3 position,
    double absolute_time
);

TWELL_API double twell_gesture_get_velocity(twell_context* ctx, twell_gesture_id id);
TWELL_API twell_vector2 twell_gesture_get_velocity_2d(twell_context* ctx, twell_gesture_id id);
TWELL_API twell_vector3 twell_gesture_get_velocity_3d(twell_context* ctx, twell_gesture_id id);

TWELL_API void twell_property_track_gesture(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double boundary_min,
    double boundary_max
);
TWELL_API void twell_property_track_gesture_2d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector2 boundary_min,
    twell_vector2 boundary_max
);
TWELL_API void twell_property_track_gesture_3d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector3 boundary_min,
    twell_vector3 boundary_max
);

TWELL_API void twell_property_release_gesture_spring(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double target_value,
    twell_spring_config config,
    double absolute_time
);
TWELL_API void twell_property_release_gesture_spring_2d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector2 target_value,
    twell_spring_config config,
    double absolute_time
);
TWELL_API void twell_property_release_gesture_spring_3d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector3 target_value,
    twell_spring_config config,
    double absolute_time
);

TWELL_API void twell_property_release_gesture_decay(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double deceleration_rate,
    double absolute_time
);

/* Spatial geometry, driven properties */
TWELL_API twell_transform3d twell_transform3d_identity(void);
TWELL_API twell_transform3d twell_transform3d_make_translation(double tx, double ty, double tz);
TWELL_API twell_transform3d twell_transform3d_make_scale(double sx, double sy, double sz);
TWELL_API void twell_transform3d_set_perspective(twell_transform3d* transform, double depth);

TWELL_API twell_quaternion twell_quaternion_make_axis_angle(double x, double y, double z, double angle);
TWELL_API twell_quaternion twell_quaternion_make_euler(double pitch, double yaw, double roll);
TWELL_API twell_quaternion twell_quaternion_slerp(twell_quaternion q1, twell_quaternion q2, double t);
TWELL_API twell_transform3d twell_transform3d_from_quaternion(twell_quaternion q);

TWELL_API void twell_property_add_driver(
    twell_context* ctx,
    twell_property_id driven_id,
    twell_property_id driver_id,
    double driver_range_min,
    double driver_range_max,
    double driven_range_min,
    double driven_range_max,
    twell_mapping_curve curve_type,
    bool clamp_to_range
);

TWELL_API void twell_property_remove_driver(
    twell_context* ctx,
    twell_property_id driven_id,
    twell_property_id driver_id
);

#ifdef __cplusplus
}
#endif

// IMPLEMENTATION

#ifdef TWELL_IMPL

#include <string.h>

typedef struct {
    twell_impulse_type type;
    double start_time;
    double initial_displacement;
    double initial_velocity;

    twell_spring_config config;
    double deceleration_rate;

    /* Derived parameters */
    double A;
    double B;
    double omega_0;
    double omega_d;
    double zeta;
    double gamma1;
    double gamma2;

    bool active;
} twell_impulse;

typedef struct {
    twell_property_id driver_id;
    double driver_min;
    double driver_max;
    double driven_min;
    double driven_max;
    twell_mapping_curve curve;
    bool clamp;
    bool active;
} twell_driver_link;

typedef struct {
    double pos[3];
    double time;
} twell_touch_sample;

typedef struct {
    bool allocated;
    uint32_t dimensions;
    twell_touch_sample samples[TWELL_GESTURE_HISTORY_SIZE];
    uint32_t sample_count;
    uint32_t sample_head;
} twell_gesture_data;

struct twell_context {
    uint32_t max_properties;
    uint32_t max_gestures;

    /* Allocation and dimensionality tracking */
    uint8_t* prop_allocated;  /* 1 if allocated, 0 if free */
    uint8_t* prop_dimensions; /* 1, 2, or 3 for primary handle; 0 for component handle */

    /* SoA layout for 1D scalar properties */
    double* base_value;
    double* presentation_value;
    double* target_value;
    double* eps_displacement;
    double* eps_velocity;
    uint8_t* is_resting;
    uint8_t* unit_type;

    /* Active gesture tracking per property */
    twell_gesture_id* tracked_gesture;
    double* boundary_min;
    double* boundary_max;

    /* Ring buffer of additive impulses: size = max_properties * TWELL_MAX_IMPULSES */
    twell_impulse* impulses;
    uint8_t* impulse_head;
    uint8_t* impulse_count;

    /* Driver links: size = max_properties * TWELL_MAX_DRIVERS_PER_PROP */
    twell_driver_link* driver_links;

    /* Gesture pool: size = max_gestures */
    twell_gesture_data* gestures;

    /* Global threshold overrides */
    double global_eps_d;
    double global_eps_v;
    bool use_global_eps;

    /* Resting queue */
    twell_property_id* resting_queue;
    uint32_t resting_queue_head;
    uint32_t resting_queue_count;

    double current_time;
};

/* Helper function to initialize spring/decay impulse parameters */
static void twell_init_impulse_params(
    twell_impulse* imp,
    twell_impulse_type type,
    double start_time,
    double initial_displacement,
    double initial_velocity,
    twell_spring_config config,
    double deceleration_rate
) {
    imp->type = type;
    imp->start_time = start_time;
    imp->initial_displacement = initial_displacement;
    imp->initial_velocity = initial_velocity;
    imp->active = true;

    if (type == TWELL_IMPULSE_SPRING) {
        imp->config = config;
        double mass = config.mass > 1e-9 ? config.mass : 1e-9;
        double stiffness = config.stiffness > 1e-9 ? config.stiffness : 1e-9;
        imp->omega_0 = sqrt(stiffness / mass);
        imp->zeta = config.damping / (2.0 * sqrt(mass * stiffness));

        if (imp->zeta < 0.9999) {
            /* Underdamped */
            imp->omega_d = imp->omega_0 * sqrt(1.0 - imp->zeta * imp->zeta);
            imp->A = initial_displacement;
            imp->B = (initial_velocity + imp->zeta * imp->omega_0 * initial_displacement) / (imp->omega_d > 1e-9 ? imp->omega_d : 1e-9);
        } else if (imp->zeta <= 1.0001) {
            /* Critically damped */
            imp->omega_d = 0.0;
            imp->A = initial_displacement;
            imp->B = initial_velocity + imp->omega_0 * initial_displacement;
        } else {
            /* Overdamped */
            double sq = sqrt(imp->zeta * imp->zeta - 1.0);
            imp->gamma1 = -imp->zeta * imp->omega_0 + imp->omega_0 * sq;
            imp->gamma2 = -imp->zeta * imp->omega_0 - imp->omega_0 * sq;
            double denom = imp->gamma1 - imp->gamma2;
            if (fabs(denom) < 1e-9) denom = 1e-9;
            imp->A = (initial_velocity - imp->gamma2 * initial_displacement) / denom;
            imp->B = initial_displacement - imp->A;
        }
    } else {
        /* Decay */
        imp->deceleration_rate = deceleration_rate > 1e-9 ? deceleration_rate : 1e-9;
    }
}

static void twell_eval_impulse(const twell_impulse* imp, double t_now, double* out_x, double* out_v) {
    if (!imp->active) {
        *out_x = 0.0;
        *out_v = 0.0;
        return;
    }

    double t = t_now - imp->start_time;
    if (t < 0.0) t = 0.0;

    if (imp->type == TWELL_IMPULSE_SPRING) {
        if (imp->zeta < 0.9999) {
            /* Underdamped */
            double env = exp(-imp->zeta * imp->omega_0 * t);
            double cos_wt = cos(imp->omega_d * t);
            double sin_wt = sin(imp->omega_d * t);

            *out_x = env * (imp->A * cos_wt + imp->B * sin_wt);
            double term1 = -imp->zeta * imp->omega_0 * (imp->A * cos_wt + imp->B * sin_wt);
            double term2 = imp->omega_d * (-imp->A * sin_wt + imp->B * cos_wt);
            *out_v = env * (term1 + term2);
        } else if (imp->zeta <= 1.0001) {
            /* Critically damped */
            double env = exp(-imp->omega_0 * t);
            *out_x = (imp->A + imp->B * t) * env;
            *out_v = (imp->B - imp->omega_0 * (imp->A + imp->B * t)) * env;
        } else {
            /* Overdamped */
            double e1 = exp(imp->gamma1 * t);
            double e2 = exp(imp->gamma2 * t);
            *out_x = imp->A * e1 + imp->B * e2;
            *out_v = imp->A * imp->gamma1 * e1 + imp->B * imp->gamma2 * e2;
        }
    } else {
        /* Viscous Fluid Decay */
        double d = imp->deceleration_rate;
        double v = imp->initial_velocity * exp(-d * t);
        *out_v = v;
        /* Displacement remaining relative to starting point: (v0/d) * (1 - e^(-d*t)) */
        *out_x = (imp->initial_velocity / d) * (1.0 - exp(-d * t));
        /* For additive impulse of decay, total travel destination is v0/d.
         * The remaining delta to destination is (v0/d) - x(t) = v(t)/d.
         */
    }
}

static void twell_get_default_epsilons(twell_unit_type unit, double* eps_d, double* eps_v) {
    switch (unit) {
        case TWELL_UNIT_PIXELS:
            *eps_d = 0.1;
            *eps_v = 0.5;
            break;
        case TWELL_UNIT_NORMALIZED:
            *eps_d = 0.0001;
            *eps_v = 0.001;
            break;
        case TWELL_UNIT_DEGREES:
            *eps_d = 0.05;
            *eps_v = 0.1;
            break;
        case TWELL_UNIT_RADIANS:
            *eps_d = 0.001;
            *eps_v = 0.005;
            break;
        case TWELL_UNIT_CUSTOM:
        default:
            *eps_d = 0.01;
            *eps_v = 0.01;
            break;
    }
}

size_t twell_get_memory_requirement(uint32_t max_properties, uint32_t max_gestures) {
    size_t total = 0;
    
    total += ((sizeof(twell_context) + 7) & ~(size_t)7);
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* prop_allocated */
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* prop_dimensions */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* base_value */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* presentation_value */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* target_value */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* eps_displacement */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* eps_velocity */
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* is_resting */
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* unit_type */
    total += ((max_properties * sizeof(twell_gesture_id) + 7) & ~(size_t)7); /* tracked_gesture */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* boundary_min */
    total += ((max_properties * sizeof(double) + 7) & ~(size_t)7);  /* boundary_max */
    total += ((max_properties * TWELL_MAX_IMPULSES * sizeof(twell_impulse) + 7) & ~(size_t)7); /* impulses */
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* impulse_head */
    total += ((max_properties * sizeof(uint8_t) + 7) & ~(size_t)7); /* impulse_count */
    total += ((max_properties * TWELL_MAX_DRIVERS_PER_PROP * sizeof(twell_driver_link) + 7) & ~(size_t)7); /* driver_links */
    total += ((max_gestures * sizeof(twell_gesture_data) + 7) & ~(size_t)7); /* gestures */
    total += ((max_properties * sizeof(twell_property_id) + 7) & ~(size_t)7); /* resting_queue */

    return total;
}

static void* twell_arena_alloc(uint8_t** cursor, size_t size, uint8_t* arena_end) {
    uint8_t* cur = *cursor;
    size_t aligned_size = (size + 7) & ~(size_t)7;
    if (cur + aligned_size > arena_end) {
        return NULL;
    }
    *cursor = cur + aligned_size;
    return (void*)cur;
}

twell_context* twell_context_create(
    void* arena_memory,
    size_t arena_size_bytes,
    uint32_t max_properties,
    uint32_t max_gestures
) {
    if (!arena_memory || max_properties == 0) return NULL;

    uint8_t* cursor = (uint8_t*)arena_memory;
    uint8_t* arena_end = cursor + arena_size_bytes;

    twell_context* ctx = (twell_context*)twell_arena_alloc(&cursor, sizeof(twell_context), arena_end);
    if (!ctx) return NULL;

    memset(ctx, 0, sizeof(twell_context));
    ctx->max_properties = max_properties;
    ctx->max_gestures = max_gestures;

    ctx->prop_allocated  = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->prop_dimensions = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->base_value         = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->presentation_value = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->target_value       = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->eps_displacement   = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->eps_velocity       = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->is_resting         = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->unit_type          = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->tracked_gesture    = (twell_gesture_id*)twell_arena_alloc(&cursor, max_properties * sizeof(twell_gesture_id), arena_end);
    ctx->boundary_min       = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->boundary_max       = (double*)twell_arena_alloc(&cursor, max_properties * sizeof(double), arena_end);
    ctx->impulses           = (twell_impulse*)twell_arena_alloc(&cursor, max_properties * TWELL_MAX_IMPULSES * sizeof(twell_impulse), arena_end);
    ctx->impulse_head       = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->impulse_count      = (uint8_t*)twell_arena_alloc(&cursor, max_properties * sizeof(uint8_t), arena_end);
    ctx->driver_links       = (twell_driver_link*)twell_arena_alloc(&cursor, max_properties * TWELL_MAX_DRIVERS_PER_PROP * sizeof(twell_driver_link), arena_end);
    ctx->gestures           = (twell_gesture_data*)twell_arena_alloc(&cursor, max_gestures * sizeof(twell_gesture_data), arena_end);
    ctx->resting_queue      = (twell_property_id*)twell_arena_alloc(&cursor, max_properties * sizeof(twell_property_id), arena_end);

    if (!ctx->prop_allocated || !ctx->prop_dimensions ||
        !ctx->base_value || !ctx->presentation_value || !ctx->target_value ||
        !ctx->eps_displacement || !ctx->eps_velocity ||
        !ctx->is_resting || !ctx->unit_type ||
        !ctx->tracked_gesture || !ctx->boundary_min || !ctx->boundary_max ||
        !ctx->impulses || !ctx->impulse_head || !ctx->impulse_count ||
        !ctx->driver_links ||
        (max_gestures > 0 && !ctx->gestures) ||
        !ctx->resting_queue) {
        return NULL;
    }

    size_t allocated_bytes = (size_t)(cursor - (uint8_t*)arena_memory);
    memset((uint8_t*)arena_memory + sizeof(twell_context), 0, allocated_bytes - sizeof(twell_context));

    for (uint32_t i = 0; i < max_properties; i++) {
        ctx->tracked_gesture[i] = TWELL_INVALID_ID;
        ctx->boundary_min[i] = -INFINITY;
        ctx->boundary_max[i] = INFINITY;
        ctx->is_resting[i] = 1;
    }

    return ctx;
}

void twell_math_spring_evaluate(
    double t,
    twell_spring_config config,
    double initial_displacement,
    double* out_displacement,
    double* out_velocity
) {
    twell_impulse imp;
    twell_init_impulse_params(&imp, TWELL_IMPULSE_SPRING, 0.0, initial_displacement, config.initial_velocity, config, 0.0);
    twell_eval_impulse(&imp, t, out_displacement, out_velocity);
}

void twell_math_decay_evaluate(
    double t,
    double initial_velocity,
    double deceleration_rate,
    double* out_displacement,
    double* out_velocity
) {
    twell_spring_config dummy = {0};
    twell_impulse imp;
    twell_init_impulse_params(&imp, TWELL_IMPULSE_DECAY, 0.0, 0.0, initial_velocity, dummy, deceleration_rate);
    twell_eval_impulse(&imp, t, out_displacement, out_velocity);
}

double twell_math_rubber_band(
    double input_displacement,
    double max_displacement,
    double tension
) {
    if (max_displacement <= 0.0) return 0.0;
    double d_in = fabs(input_displacement);
    double c = tension > 0.0 ? tension : 0.55;
    double d_out = (d_in * c * max_displacement) / (d_in * c + max_displacement);
    return input_displacement < 0.0 ? -d_out : d_out;
}

void twell_math_set_rest_thresholds(
    twell_context* ctx,
    double displacement_epsilon,
    double velocity_epsilon
) {
    if (!ctx) return;
    ctx->global_eps_d = displacement_epsilon;
    ctx->global_eps_v = velocity_epsilon;
    ctx->use_global_eps = true;
    for (uint32_t i = 0; i < ctx->max_properties; i++) {
        if (ctx->prop_allocated[i]) {
            ctx->eps_displacement[i] = displacement_epsilon;
            ctx->eps_velocity[i] = velocity_epsilon;
        }
    }
}

/* Internal helper */
static void twell_property_push_impulse(
    twell_context* ctx,
    twell_property_id id,
    twell_impulse_type type,
    double start_time,
    double initial_displacement,
    double initial_velocity,
    twell_spring_config config,
    double deceleration_rate
) {
    uint8_t count = ctx->impulse_count[id];
    uint8_t head = ctx->impulse_head[id];
    twell_impulse* prop_impulses = &ctx->impulses[id * TWELL_MAX_IMPULSES];

    if (count < TWELL_MAX_IMPULSES) {
        uint8_t slot = (head + count) % TWELL_MAX_IMPULSES;
        twell_init_impulse_params(&prop_impulses[slot], type, start_time, initial_displacement, initial_velocity, config, deceleration_rate);
        ctx->impulse_count[id]++;
    } else {
        /* Buffer Overflow!
         * 1. Evaluate oldest impulse (head) at start_time.
         * 2. Bake its displacement into base_value.
         * 3. Transfer its remaining velocity into next oldest impulse (head + 1).
         */
        uint8_t oldest_slot = head;
        double x_culled, v_culled;
        twell_eval_impulse(&prop_impulses[oldest_slot], start_time, &x_culled, &v_culled);

        if (prop_impulses[oldest_slot].type == TWELL_IMPULSE_SPRING) {
            ctx->base_value[id] += x_culled;
        } else {
            /* Decay impulse destination total is dest = v0/d, displacement so far is x_culled.
             * Remaining offset contribution is (x_culled - dest). Bake into base_value.
             */
            double d = prop_impulses[oldest_slot].deceleration_rate;
            double dest = prop_impulses[oldest_slot].initial_velocity / d;
            ctx->base_value[id] += (x_culled - dest);
        }

        uint8_t next_oldest_slot = (oldest_slot + 1) % TWELL_MAX_IMPULSES;
        double x_next, v_next;
        twell_eval_impulse(&prop_impulses[next_oldest_slot], start_time, &x_next, &v_next);

        /* Re-initialize next oldest impulse with transferred velocity */
        twell_init_impulse_params(
            &prop_impulses[next_oldest_slot],
            prop_impulses[next_oldest_slot].type,
            start_time,
            x_next,
            v_next + v_culled,
            prop_impulses[next_oldest_slot].config,
            prop_impulses[next_oldest_slot].deceleration_rate
        );

        /* Overwrite oldest slot with new impulse */
        twell_init_impulse_params(&prop_impulses[oldest_slot], type, start_time, initial_displacement, initial_velocity, config, deceleration_rate);
        ctx->impulse_head[id] = (head + 1) % TWELL_MAX_IMPULSES;
    }

    ctx->is_resting[id] = 0;
}

twell_property_id twell_property_create_with_unit(twell_context* ctx, double initial_value, twell_unit_type unit) {
    if (!ctx) return TWELL_INVALID_ID;

    for (uint32_t i = 0; i < ctx->max_properties; i++) {
        if (!ctx->prop_allocated[i]) {
            ctx->prop_allocated[i] = 1;
            ctx->prop_dimensions[i] = 1;
            ctx->base_value[i] = initial_value;
            ctx->presentation_value[i] = initial_value;
            ctx->target_value[i] = initial_value;
            ctx->unit_type[i] = (uint8_t)unit;
            ctx->is_resting[i] = 1;
            ctx->impulse_head[i] = 0;
            ctx->impulse_count[i] = 0;
            ctx->tracked_gesture[i] = TWELL_INVALID_ID;

            if (ctx->use_global_eps) {
                ctx->eps_displacement[i] = ctx->global_eps_d;
                ctx->eps_velocity[i] = ctx->global_eps_v;
            } else {
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i], &ctx->eps_velocity[i]);
            }

            return i;
        }
    }

    return TWELL_INVALID_ID;
}

twell_property_id twell_property_create(twell_context* ctx, double initial_value) {
    return twell_property_create_with_unit(ctx, initial_value, TWELL_UNIT_PIXELS);
}

twell_property_id twell_property_create_2d_with_unit(twell_context* ctx, twell_vector2 initial_value, twell_unit_type unit) {
    if (!ctx) return TWELL_INVALID_ID;

    for (uint32_t i = 0; i + 1 < ctx->max_properties; i++) {
        if (!ctx->prop_allocated[i] && !ctx->prop_allocated[i + 1]) {
            ctx->prop_allocated[i] = 1;
            ctx->prop_allocated[i + 1] = 1;
            ctx->prop_dimensions[i] = 2;
            ctx->prop_dimensions[i + 1] = 0; /* component */

            ctx->base_value[i] = initial_value.x;
            ctx->presentation_value[i] = initial_value.x;
            ctx->target_value[i] = initial_value.x;
            ctx->unit_type[i] = (uint8_t)unit;
            ctx->is_resting[i] = 1;
            ctx->impulse_head[i] = 0;
            ctx->impulse_count[i] = 0;
            ctx->tracked_gesture[i] = TWELL_INVALID_ID;

            ctx->base_value[i + 1] = initial_value.y;
            ctx->presentation_value[i + 1] = initial_value.y;
            ctx->target_value[i + 1] = initial_value.y;
            ctx->unit_type[i + 1] = (uint8_t)unit;
            ctx->is_resting[i + 1] = 1;
            ctx->impulse_head[i + 1] = 0;
            ctx->impulse_count[i + 1] = 0;
            ctx->tracked_gesture[i + 1] = TWELL_INVALID_ID;

            if (ctx->use_global_eps) {
                ctx->eps_displacement[i] = ctx->global_eps_d;
                ctx->eps_velocity[i] = ctx->global_eps_v;
                ctx->eps_displacement[i + 1] = ctx->global_eps_d;
                ctx->eps_velocity[i + 1] = ctx->global_eps_v;
            } else {
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i], &ctx->eps_velocity[i]);
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i + 1], &ctx->eps_velocity[i + 1]);
            }

            return i;
        }
    }
    return TWELL_INVALID_ID;
}

twell_property_id twell_property_create_2d(twell_context* ctx, twell_vector2 initial_value) {
    return twell_property_create_2d_with_unit(ctx, initial_value, TWELL_UNIT_PIXELS);
}

twell_property_id twell_property_create_3d_with_unit(twell_context* ctx, twell_vector3 initial_value, twell_unit_type unit) {
    if (!ctx) return TWELL_INVALID_ID;

    for (uint32_t i = 0; i + 2 < ctx->max_properties; i++) {
        if (!ctx->prop_allocated[i] && !ctx->prop_allocated[i + 1] && !ctx->prop_allocated[i + 2]) {
            ctx->prop_allocated[i] = 1;
            ctx->prop_allocated[i + 1] = 1;
            ctx->prop_allocated[i + 2] = 1;
            ctx->prop_dimensions[i] = 3;
            ctx->prop_dimensions[i + 1] = 0;
            ctx->prop_dimensions[i + 2] = 0;

            ctx->base_value[i] = initial_value.x;
            ctx->presentation_value[i] = initial_value.x;
            ctx->target_value[i] = initial_value.x;
            ctx->unit_type[i] = (uint8_t)unit;
            ctx->is_resting[i] = 1;
            ctx->impulse_head[i] = 0;
            ctx->impulse_count[i] = 0;
            ctx->tracked_gesture[i] = TWELL_INVALID_ID;

            ctx->base_value[i + 1] = initial_value.y;
            ctx->presentation_value[i + 1] = initial_value.y;
            ctx->target_value[i + 1] = initial_value.y;
            ctx->unit_type[i + 1] = (uint8_t)unit;
            ctx->is_resting[i + 1] = 1;
            ctx->impulse_head[i + 1] = 0;
            ctx->impulse_count[i + 1] = 0;
            ctx->tracked_gesture[i + 1] = TWELL_INVALID_ID;

            ctx->base_value[i + 2] = initial_value.z;
            ctx->presentation_value[i + 2] = initial_value.z;
            ctx->target_value[i + 2] = initial_value.z;
            ctx->unit_type[i + 2] = (uint8_t)unit;
            ctx->is_resting[i + 2] = 1;
            ctx->impulse_head[i + 2] = 0;
            ctx->impulse_count[i + 2] = 0;
            ctx->tracked_gesture[i + 2] = TWELL_INVALID_ID;

            if (ctx->use_global_eps) {
                ctx->eps_displacement[i] = ctx->global_eps_d;
                ctx->eps_velocity[i] = ctx->global_eps_v;
                ctx->eps_displacement[i + 1] = ctx->global_eps_d;
                ctx->eps_velocity[i + 1] = ctx->global_eps_v;
                ctx->eps_displacement[i + 2] = ctx->global_eps_d;
                ctx->eps_velocity[i + 2] = ctx->global_eps_v;
            } else {
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i], &ctx->eps_velocity[i]);
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i + 1], &ctx->eps_velocity[i + 1]);
                twell_get_default_epsilons(unit, &ctx->eps_displacement[i + 2], &ctx->eps_velocity[i + 2]);
            }

            return i;
        }
    }
    return TWELL_INVALID_ID;
}

twell_property_id twell_property_create_3d(twell_context* ctx, twell_vector3 initial_value) {
    return twell_property_create_3d_with_unit(ctx, initial_value, TWELL_UNIT_PIXELS);
}

void twell_property_destroy(twell_context* ctx, twell_property_id id) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return;

    twell_property_id parent_id = id;
    if (ctx->prop_dimensions[id] == 0) {
        if (id > 0 && ctx->prop_dimensions[id - 1] == 2) parent_id = id - 1;
        else if (id > 0 && ctx->prop_dimensions[id - 1] == 3) parent_id = id - 1;
        else if (id > 1 && ctx->prop_dimensions[id - 2] == 3) parent_id = id - 2;
    }

    uint32_t count = ctx->prop_dimensions[parent_id];
    if (count == 0) count = 1;

    for (uint32_t k = 0; k < count && (parent_id + k) < ctx->max_properties; k++) {
        uint32_t idx = parent_id + k;
        ctx->prop_allocated[idx] = 0;
        ctx->prop_dimensions[idx] = 0;
        ctx->impulse_count[idx] = 0;
        ctx->impulse_head[idx] = 0;
        ctx->tracked_gesture[idx] = TWELL_INVALID_ID;

        /* Remove driver links */
        for (uint32_t d = 0; d < TWELL_MAX_DRIVERS_PER_PROP; d++) {
            ctx->driver_links[idx * TWELL_MAX_DRIVERS_PER_PROP + d].active = false;
        }
    }
}

void twell_property_set_rest_thresholds(
    twell_context* ctx,
    twell_property_id id,
    double displacement_epsilon,
    double velocity_epsilon
) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return;
    ctx->eps_displacement[id] = displacement_epsilon;
    ctx->eps_velocity[id] = velocity_epsilon;
}

double twell_property_get_model_value(twell_context* ctx, twell_property_id id) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return 0.0;
    return ctx->target_value[id];
}

twell_vector2 twell_property_get_model_value_2d(twell_context* ctx, twell_property_id id) {
    twell_vector2 res = {0.0, 0.0};
    res.x = twell_property_get_model_value(ctx, id);
    res.y = twell_property_get_model_value(ctx, id + 1);
    return res;
}

twell_vector3 twell_property_get_model_value_3d(twell_context* ctx, twell_property_id id) {
    twell_vector3 res = {0.0, 0.0, 0.0};
    res.x = twell_property_get_model_value(ctx, id);
    res.y = twell_property_get_model_value(ctx, id + 1);
    res.z = twell_property_get_model_value(ctx, id + 2);
    return res;
}

double twell_property_get_presentation_value(twell_context* ctx, twell_property_id id) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return 0.0;
    return ctx->presentation_value[id];
}

twell_vector2 twell_property_get_presentation_value_2d(twell_context* ctx, twell_property_id id) {
    twell_vector2 res = {0.0, 0.0};
    res.x = twell_property_get_presentation_value(ctx, id);
    res.y = twell_property_get_presentation_value(ctx, id + 1);
    return res;
}

twell_vector3 twell_property_get_presentation_value_3d(twell_context* ctx, twell_property_id id) {
    twell_vector3 res = {0.0, 0.0, 0.0};
    res.x = twell_property_get_presentation_value(ctx, id);
    res.y = twell_property_get_presentation_value(ctx, id + 1);
    res.z = twell_property_get_presentation_value(ctx, id + 2);
    return res;
}

void twell_property_set_value_immediate(twell_context* ctx, twell_property_id id, double value) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return;
    ctx->base_value[id] = value;
    ctx->presentation_value[id] = value;
    ctx->target_value[id] = value;
    ctx->impulse_count[id] = 0;
    ctx->impulse_head[id] = 0;
    ctx->is_resting[id] = 1;
}

void twell_property_set_value_immediate_2d(twell_context* ctx, twell_property_id id, twell_vector2 value) {
    twell_property_set_value_immediate(ctx, id, value.x);
    twell_property_set_value_immediate(ctx, id + 1, value.y);
}

void twell_property_set_value_immediate_3d(twell_context* ctx, twell_property_id id, twell_vector3 value) {
    twell_property_set_value_immediate(ctx, id, value.x);
    twell_property_set_value_immediate(ctx, id + 1, value.y);
    twell_property_set_value_immediate(ctx, id + 2, value.z);
}

void twell_property_animate_to_target(
    twell_context* ctx,
    twell_property_id id,
    double target_value,
    twell_spring_config config,
    double absolute_time
) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return;

    double delta = ctx->target_value[id] - target_value;
    ctx->target_value[id] = target_value;
    ctx->base_value[id] = target_value;

    twell_property_push_impulse(ctx, id, TWELL_IMPULSE_SPRING, absolute_time, delta, config.initial_velocity, config, 0.0);
}

void twell_property_animate_to_target_2d(
    twell_context* ctx,
    twell_property_id id,
    twell_vector2 target_value,
    twell_spring_config config,
    double absolute_time
) {
    twell_property_animate_to_target(ctx, id, target_value.x, config, absolute_time);
    twell_property_animate_to_target(ctx, id + 1, target_value.y, config, absolute_time);
}

void twell_property_animate_to_target_3d(
    twell_context* ctx,
    twell_property_id id,
    twell_vector3 target_value,
    twell_spring_config config,
    double absolute_time
) {
    twell_property_animate_to_target(ctx, id, target_value.x, config, absolute_time);
    twell_property_animate_to_target(ctx, id + 1, target_value.y, config, absolute_time);
    twell_property_animate_to_target(ctx, id + 2, target_value.z, config, absolute_time);
}

void twell_property_animate_decay(
    twell_context* ctx,
    twell_property_id id,
    double initial_velocity,
    double deceleration_rate,
    double absolute_time
) {
    if (!ctx || id >= ctx->max_properties || !ctx->prop_allocated[id]) return;

    double d = deceleration_rate > 1e-9 ? deceleration_rate : 1e-9;
    double total_travel = initial_velocity / d;
    ctx->target_value[id] = ctx->base_value[id] + total_travel;
    ctx->base_value[id] = ctx->target_value[id];

    twell_spring_config dummy = {0};
    twell_property_push_impulse(ctx, id, TWELL_IMPULSE_DECAY, absolute_time, 0.0, initial_velocity, dummy, deceleration_rate);
}

twell_gesture_id twell_gesture_create(twell_context* ctx) {
    if (!ctx) return TWELL_INVALID_ID;

    for (uint32_t i = 0; i < ctx->max_gestures; i++) {
        if (!ctx->gestures[i].allocated) {
            ctx->gestures[i].allocated = true;
            ctx->gestures[i].dimensions = 1;
            ctx->gestures[i].sample_count = 0;
            ctx->gestures[i].sample_head = 0;
            return i;
        }
    }
    return TWELL_INVALID_ID;
}

void twell_gesture_destroy(twell_context* ctx, twell_gesture_id id) {
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return;
    ctx->gestures[id].allocated = false;
    ctx->gestures[id].sample_count = 0;
}

void twell_gesture_add_touch(
    twell_context* ctx,
    twell_gesture_id id,
    double position,
    double absolute_time
) {
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return;

    twell_gesture_data* g = &ctx->gestures[id];
    g->dimensions = 1;
    uint32_t slot = g->sample_head;
    g->samples[slot].pos[0] = position;
    g->samples[slot].pos[1] = 0.0;
    g->samples[slot].pos[2] = 0.0;
    g->samples[slot].time = absolute_time;

    g->sample_head = (g->sample_head + 1) % TWELL_GESTURE_HISTORY_SIZE;
    if (g->sample_count < TWELL_GESTURE_HISTORY_SIZE) {
        g->sample_count++;
    }
}

void twell_gesture_add_touch_2d(
    twell_context* ctx,
    twell_gesture_id id,
    twell_vector2 position,
    double absolute_time
) {
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return;

    twell_gesture_data* g = &ctx->gestures[id];
    g->dimensions = 2;
    uint32_t slot = g->sample_head;
    g->samples[slot].pos[0] = position.x;
    g->samples[slot].pos[1] = position.y;
    g->samples[slot].pos[2] = 0.0;
    g->samples[slot].time = absolute_time;

    g->sample_head = (g->sample_head + 1) % TWELL_GESTURE_HISTORY_SIZE;
    if (g->sample_count < TWELL_GESTURE_HISTORY_SIZE) {
        g->sample_count++;
    }
}

void twell_gesture_add_touch_3d(
    twell_context* ctx,
    twell_gesture_id id,
    twell_vector3 position,
    double absolute_time
) {
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return;

    twell_gesture_data* g = &ctx->gestures[id];
    g->dimensions = 3;
    uint32_t slot = g->sample_head;
    g->samples[slot].pos[0] = position.x;
    g->samples[slot].pos[1] = position.y;
    g->samples[slot].pos[2] = position.z;
    g->samples[slot].time = absolute_time;

    g->sample_head = (g->sample_head + 1) % TWELL_GESTURE_HISTORY_SIZE;
    if (g->sample_count < TWELL_GESTURE_HISTORY_SIZE) {
        g->sample_count++;
    }
}

/* Time-weighted least-squares linear regression over rolling window */
static double twell_gesture_calc_component_velocity(twell_gesture_data* g, uint32_t comp) {
    if (g->sample_count < 2) return 0.0;

    uint32_t latest_idx = (g->sample_head + TWELL_GESTURE_HISTORY_SIZE - 1) % TWELL_GESTURE_HISTORY_SIZE;
    double t_latest = g->samples[latest_idx].time;
    double window = 0.080; /* 80ms rolling window */

    double sum_w = 0.0;
    double sum_wt = 0.0;
    double sum_wx = 0.0;
    uint32_t valid_count = 0;

    /* First pass: calculate weighted means */
    for (uint32_t i = 0; i < g->sample_count; i++) {
        uint32_t idx = (g->sample_head + TWELL_GESTURE_HISTORY_SIZE - 1 - i) % TWELL_GESTURE_HISTORY_SIZE;
        double dt = g->samples[idx].time - t_latest;
        if (dt < -window) break;

        double w = exp(15.0 * dt); /* exponential time weighting relative to t_latest */
        sum_w += w;
        sum_wt += w * dt;
        sum_wx += w * g->samples[idx].pos[comp];
        valid_count++;
    }

    if (valid_count < 2 || sum_w < 1e-9) return 0.0;

    double mean_t = sum_wt / sum_w;
    double mean_x = sum_wx / sum_w;

    double num = 0.0;
    double den = 0.0;

    for (uint32_t i = 0; i < g->sample_count; i++) {
        uint32_t idx = (g->sample_head + TWELL_GESTURE_HISTORY_SIZE - 1 - i) % TWELL_GESTURE_HISTORY_SIZE;
        double dt = g->samples[idx].time - t_latest;
        if (dt < -window) break;

        double w = exp(15.0 * dt);
        double diff_t = dt - mean_t;
        double diff_x = g->samples[idx].pos[comp] - mean_x;

        num += w * diff_t * diff_x;
        den += w * diff_t * diff_t;
    }

    if (den < 1e-12) return 0.0;
    return num / den;
}

double twell_gesture_get_velocity(twell_context* ctx, twell_gesture_id id) {
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return 0.0;
    return twell_gesture_calc_component_velocity(&ctx->gestures[id], 0);
}

twell_vector2 twell_gesture_get_velocity_2d(twell_context* ctx, twell_gesture_id id) {
    twell_vector2 v = {0.0, 0.0};
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return v;
    v.x = twell_gesture_calc_component_velocity(&ctx->gestures[id], 0);
    v.y = twell_gesture_calc_component_velocity(&ctx->gestures[id], 1);
    return v;
}

twell_vector3 twell_gesture_get_velocity_3d(twell_context* ctx, twell_gesture_id id) {
    twell_vector3 v = {0.0, 0.0, 0.0};
    if (!ctx || id >= ctx->max_gestures || !ctx->gestures[id].allocated) return v;
    v.x = twell_gesture_calc_component_velocity(&ctx->gestures[id], 0);
    v.y = twell_gesture_calc_component_velocity(&ctx->gestures[id], 1);
    v.z = twell_gesture_calc_component_velocity(&ctx->gestures[id], 2);
    return v;
}

void twell_property_track_gesture(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double boundary_min,
    double boundary_max
) {
    if (!ctx || prop_id >= ctx->max_properties || !ctx->prop_allocated[prop_id]) return;
    ctx->tracked_gesture[prop_id] = gesture_id;
    ctx->boundary_min[prop_id] = boundary_min;
    ctx->boundary_max[prop_id] = boundary_max;
    ctx->is_resting[prop_id] = 0;
    ctx->impulse_count[prop_id] = 0;
    ctx->impulse_head[prop_id] = 0;
}

void twell_property_track_gesture_2d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector2 boundary_min,
    twell_vector2 boundary_max
) {
    twell_property_track_gesture(ctx, prop_id, gesture_id, boundary_min.x, boundary_max.x);
    twell_property_track_gesture(ctx, prop_id + 1, gesture_id, boundary_min.y, boundary_max.y);
}

void twell_property_track_gesture_3d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector3 boundary_min,
    twell_vector3 boundary_max
) {
    twell_property_track_gesture(ctx, prop_id, gesture_id, boundary_min.x, boundary_max.x);
    twell_property_track_gesture(ctx, prop_id + 1, gesture_id, boundary_min.y, boundary_max.y);
    twell_property_track_gesture(ctx, prop_id + 2, gesture_id, boundary_min.z, boundary_max.z);
}

void twell_property_release_gesture_spring(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double target_value,
    twell_spring_config config,
    double absolute_time
) {
    if (!ctx || prop_id >= ctx->max_properties || !ctx->prop_allocated[prop_id]) return;

    double v = twell_gesture_get_velocity(ctx, gesture_id);
    config.initial_velocity = v;

    ctx->tracked_gesture[prop_id] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id, target_value, config, absolute_time);
}

void twell_property_release_gesture_spring_2d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector2 target_value,
    twell_spring_config config,
    double absolute_time
) {
    twell_vector2 v = twell_gesture_get_velocity_2d(ctx, gesture_id);

    twell_spring_config config_x = config;
    config_x.initial_velocity = v.x;
    ctx->tracked_gesture[prop_id] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id, target_value.x, config_x, absolute_time);

    twell_spring_config config_y = config;
    config_y.initial_velocity = v.y;
    ctx->tracked_gesture[prop_id + 1] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id + 1, target_value.y, config_y, absolute_time);
}

void twell_property_release_gesture_spring_3d(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    twell_vector3 target_value,
    twell_spring_config config,
    double absolute_time
) {
    twell_vector3 v = twell_gesture_get_velocity_3d(ctx, gesture_id);

    twell_spring_config config_x = config;
    config_x.initial_velocity = v.x;
    ctx->tracked_gesture[prop_id] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id, target_value.x, config_x, absolute_time);

    twell_spring_config config_y = config;
    config_y.initial_velocity = v.y;
    ctx->tracked_gesture[prop_id + 1] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id + 1, target_value.y, config_y, absolute_time);

    twell_spring_config config_z = config;
    config_z.initial_velocity = v.z;
    ctx->tracked_gesture[prop_id + 2] = TWELL_INVALID_ID;
    twell_property_animate_to_target(ctx, prop_id + 2, target_value.z, config_z, absolute_time);
}

void twell_property_release_gesture_decay(
    twell_context* ctx,
    twell_property_id prop_id,
    twell_gesture_id gesture_id,
    double deceleration_rate,
    double absolute_time
) {
    if (!ctx || prop_id >= ctx->max_properties || !ctx->prop_allocated[prop_id]) return;

    double v = twell_gesture_get_velocity(ctx, gesture_id);
    ctx->tracked_gesture[prop_id] = TWELL_INVALID_ID;
    twell_property_animate_decay(ctx, prop_id, v, deceleration_rate, absolute_time);
}

twell_transform3d twell_transform3d_identity(void) {
    twell_transform3d m = {0};
    m.m11 = 1.0; m.m22 = 1.0; m.m33 = 1.0; m.m44 = 1.0;
    return m;
}

twell_transform3d twell_transform3d_make_translation(double tx, double ty, double tz) {
    twell_transform3d m = twell_transform3d_identity();
    m.m41 = tx; m.m42 = ty; m.m43 = tz;
    return m;
}

twell_transform3d twell_transform3d_make_scale(double sx, double sy, double sz) {
    twell_transform3d m = twell_transform3d_identity();
    m.m11 = sx; m.m22 = sy; m.m33 = sz;
    return m;
}

void twell_transform3d_set_perspective(twell_transform3d* transform, double depth) {
    if (!transform || depth <= 1e-9) return;
    transform->m34 = -1.0 / depth;
}

twell_quaternion twell_quaternion_make_axis_angle(double x, double y, double z, double angle) {
    twell_quaternion q = {0.0, 0.0, 0.0, 1.0};
    double len = sqrt(x * x + y * y + z * z);
    if (len < 1e-9) return q;

    double half = angle * 0.5;
    double s = sin(half) / len;
    q.x = x * s;
    q.y = y * s;
    q.z = z * s;
    q.w = cos(half);
    return q;
}

twell_quaternion twell_quaternion_make_euler(double pitch, double yaw, double roll) {
    double p = pitch * 0.5;
    double y = yaw * 0.5;
    double r = roll * 0.5;

    double cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);
    double cr = cos(r), sr = sin(r);

    twell_quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

twell_quaternion twell_quaternion_slerp(twell_quaternion q1, twell_quaternion q2, double t) {
    double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

    if (dot < 0.0) {
        q2.x = -q2.x; q2.y = -q2.y; q2.z = -q2.z; q2.w = -q2.w;
        dot = -dot;
    }

    if (dot > 0.9995) {
        twell_quaternion result;
        result.x = q1.x + t * (q2.x - q1.x);
        result.y = q1.y + t * (q2.y - q1.y);
        result.z = q1.z + t * (q2.z - q1.z);
        result.w = q1.w + t * (q2.w - q1.w);
        double len = sqrt(result.x * result.x + result.y * result.y + result.z * result.z + result.w * result.w);
        if (len > 1e-9) {
            result.x /= len; result.y /= len; result.z /= len; result.w /= len;
        }
        return result;
    }

    double theta_0 = acos(dot);
    double theta = theta_0 * t;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    twell_quaternion result;
    result.x = (s0 * q1.x) + (s1 * q2.x);
    result.y = (s0 * q1.y) + (s1 * q2.y);
    result.z = (s0 * q1.z) + (s1 * q2.z);
    result.w = (s0 * q1.w) + (s1 * q2.w);
    return result;
}

twell_transform3d twell_transform3d_from_quaternion(twell_quaternion q) {
    twell_transform3d m = twell_transform3d_identity();

    double x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    double xx = q.x * x2,  xy = q.x * y2,  xz = q.x * z2;
    double yy = q.y * y2,  yz = q.y * z2,  zz = q.z * z2;
    double wx = q.w * x2,  wy = q.w * y2,  wz = q.w * z2;

    m.m11 = 1.0 - (yy + zz);
    m.m12 = xy + wz;
    m.m13 = xz - wy;

    m.m21 = xy - wz;
    m.m22 = 1.0 - (xx + zz);
    m.m23 = yz + wx;

    m.m31 = xz + wy;
    m.m32 = yz - wx;
    m.m33 = 1.0 - (xx + yy);

    return m;
}

void twell_property_add_driver(
    twell_context* ctx,
    twell_property_id driven_id,
    twell_property_id driver_id,
    double driver_range_min,
    double driver_range_max,
    double driven_range_min,
    double driven_range_max,
    twell_mapping_curve curve_type,
    bool clamp_to_range
) {
    if (!ctx || driven_id >= ctx->max_properties || driver_id >= ctx->max_properties) return;
    if (!ctx->prop_allocated[driven_id] || !ctx->prop_allocated[driver_id]) return;

    twell_driver_link* links = &ctx->driver_links[driven_id * TWELL_MAX_DRIVERS_PER_PROP];
    for (uint32_t i = 0; i < TWELL_MAX_DRIVERS_PER_PROP; i++) {
        if (!links[i].active || links[i].driver_id == driver_id) {
            links[i].active = true;
            links[i].driver_id = driver_id;
            links[i].driver_min = driver_range_min;
            links[i].driver_max = driver_range_max;
            links[i].driven_min = driven_range_min;
            links[i].driven_max = driven_range_max;
            links[i].curve = curve_type;
            links[i].clamp = clamp_to_range;
            return;
        }
    }
}

void twell_property_remove_driver(
    twell_context* ctx,
    twell_property_id driven_id,
    twell_property_id driver_id
) {
    if (!ctx || driven_id >= ctx->max_properties) return;

    twell_driver_link* links = &ctx->driver_links[driven_id * TWELL_MAX_DRIVERS_PER_PROP];
    for (uint32_t i = 0; i < TWELL_MAX_DRIVERS_PER_PROP; i++) {
        if (links[i].active && links[i].driver_id == driver_id) {
            links[i].active = false;
        }
    }
}

/* Physics tick */

uint32_t twell_context_tick(
    twell_context* ctx,
    double absolute_time,
    twell_property_id* out_resting,
    uint32_t max_resting
) {
    if (!ctx) return 0;
    ctx->current_time = absolute_time;

    /* handle resting queue flush if out_resting is NULL */
    if (out_resting == NULL) {
        ctx->resting_queue_count = 0;
        ctx->resting_queue_head = 0;
    }

    /* pass 1: Evaluate gestures and additive impulse stacks */
    for (uint32_t i = 0; i < ctx->max_properties; i++) {
        if (!ctx->prop_allocated[i]) continue;

        /* 1. evaluate active gesture tracking */
        twell_gesture_id g_id = ctx->tracked_gesture[i];
        if (g_id != TWELL_INVALID_ID && g_id < ctx->max_gestures && ctx->gestures[g_id].allocated) {
            twell_gesture_data* g = &ctx->gestures[g_id];
            if (g->sample_count > 0) {
                uint32_t latest_idx = (g->sample_head + TWELL_GESTURE_HISTORY_SIZE - 1) % TWELL_GESTURE_HISTORY_SIZE;
                uint32_t comp = 0;
                /* determine component index based on handle offset */
                if (ctx->prop_dimensions[i] == 0) {
                    /* secondary component */
                    if (i > 0 && ctx->prop_dimensions[i - 1] == 2) comp = 1;
                    else if (i > 0 && ctx->prop_dimensions[i - 1] == 3) comp = 1;
                    else if (i > 1 && ctx->prop_dimensions[i - 2] == 3) comp = 2;
                }
                double raw_pos = g->samples[latest_idx].pos[comp];
                double min_b = ctx->boundary_min[i];
                double max_b = ctx->boundary_max[i];

                double tracked_val = raw_pos;
                if (raw_pos > max_b) {
                    double over = raw_pos - max_b;
                    tracked_val = max_b + twell_math_rubber_band(over, 200.0, 0.55);
                } else if (raw_pos < min_b) {
                    double under = raw_pos - min_b;
                    tracked_val = min_b + twell_math_rubber_band(under, 200.0, 0.55);
                }

                ctx->presentation_value[i] = tracked_val;
                ctx->base_value[i] = tracked_val;
                ctx->target_value[i] = tracked_val;
                ctx->is_resting[i] = 0;
                continue;
            }
        }

        /* 2. evaluate additive stack */
        uint8_t count = ctx->impulse_count[i];
        if (count == 0) {
            ctx->presentation_value[i] = ctx->base_value[i];
            continue;
        }

        double pres = ctx->base_value[i];
        uint8_t head = ctx->impulse_head[i];
        twell_impulse* prop_impulses = &ctx->impulses[i * TWELL_MAX_IMPULSES];

        bool all_at_rest = true;
        double eps_d = ctx->eps_displacement[i];
        double eps_v = ctx->eps_velocity[i];

        for (uint8_t k = 0; k < count; k++) {
            uint8_t slot = (head + k) % TWELL_MAX_IMPULSES;
            twell_impulse* imp = &prop_impulses[slot];
            if (!imp->active) continue;

            double dx, dv;
            twell_eval_impulse(imp, absolute_time, &dx, &dv);

            if (imp->type == TWELL_IMPULSE_SPRING) {
                pres += dx;
                if (fabs(dx) > eps_d || fabs(dv) > eps_v) {
                    all_at_rest = false;
                }
            } else {
                /* decay */
                double d = imp->deceleration_rate;
                double dest = imp->initial_velocity / d;
                pres += (dx - dest); /* dx is traveled, dest is total. additive offset = dx - dest */
                if (fabs(dv) > eps_v) {
                    all_at_rest = false;
                }
            }
        }

        ctx->presentation_value[i] = pres;

        if (all_at_rest) {
            /* Property reached resting state */
            ctx->impulse_count[i] = 0;
            ctx->impulse_head[i] = 0;
            ctx->base_value[i] = ctx->target_value[i];
            ctx->presentation_value[i] = ctx->target_value[i];

            if (!ctx->is_resting[i]) {
                ctx->is_resting[i] = 1;
                /* Push to resting queue */
                if (ctx->resting_queue_count < ctx->max_properties) {
                    uint32_t q_slot = (ctx->resting_queue_head + ctx->resting_queue_count) % ctx->max_properties;
                    ctx->resting_queue[q_slot] = i;
                    ctx->resting_queue_count++;
                }
            }
        } else {
            ctx->is_resting[i] = 0;
        }
    }

    /* Pass 2: evaluate driven links (maps updated presentation values of drivers to driven properties) */
    for (uint32_t i = 0; i < ctx->max_properties; i++) {
        if (!ctx->prop_allocated[i]) continue;

        twell_driver_link* links = &ctx->driver_links[i * TWELL_MAX_DRIVERS_PER_PROP];
        for (uint32_t d = 0; d < TWELL_MAX_DRIVERS_PER_PROP; d++) {
            if (links[d].active) {
                double driver_val = ctx->presentation_value[links[d].driver_id];
                double span = links[d].driver_max - links[d].driver_min;
                double u = span > 1e-9 ? (driver_val - links[d].driver_min) / span : 0.0;
                if (links[d].clamp) {
                    if (u < 0.0) u = 0.0;
                    if (u > 1.0) u = 1.0;
                }

                double f_u = u;
                if (links[d].curve == TWELL_MAP_EASE_IN) f_u = u * u;
                else if (links[d].curve == TWELL_MAP_EASE_OUT) f_u = u * (2.0 - u);

                double mapped = links[d].driven_min + f_u * (links[d].driven_max - links[d].driven_min);
                twell_property_set_value_immediate(ctx, i, mapped);
            }
        }
    }

    /* Drain resting events into out_resting */
    uint32_t written = 0;
    if (out_resting && max_resting > 0) {
        while (written < max_resting && ctx->resting_queue_count > 0) {
            out_resting[written] = ctx->resting_queue[ctx->resting_queue_head];
            ctx->resting_queue_head = (ctx->resting_queue_head + 1) % ctx->max_properties;
            ctx->resting_queue_count--;
            written++;
        }
    }

    return written;
}

#endif /* TWELL_IMPL */

#endif /* TWELL_H */
