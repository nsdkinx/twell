# Twell

**A tiny, embeddable, zero-allocation physics engine for remarkably fluid graphical user interfaces.**

Twell brings Apple-style, physics-driven, interruptible UI animations to *any* rendering engine or UI framework.  

Built as an single-header C library with native Python bindings, Twell is perfectly suitable for desktop GUI frameworks, mobile apps, embedded systems and game engines - such as Raylib, LVGL, Pygame, or any other GUI stack.

---

* **Zero allocation**: Operates entirely within a user-provided memory arena.
* **Analytical spring kernel**: Uses exact closed-form differential equation solutions for critically damped, underdamped, and overdamped springs. Zero numerical integration drift or time-step instability. Just like iOS.
* **Additive state machine ($C^1$ Continuity)**: Seamlessly handles animation interruptions. Retargeting mid-flight preserves velocity and momentum without snapping or jarring speed resets.
* **Gesture kinetics and momentum handoff**: Low-pass touch tracking with automatic velocity estimation and 1:1 input locking. Smoothly transfers flick momentum into spring or decay dynamics upon release.
* **Asymptotic rubber banding**: Built-in iOS-style rational resistance function for overscroll and drag boundaries.
* **Viscous fluid decay**: Exponential velocity decay modeling natural inertial scrolling (`UIScrollView` mechanics).
* **Kinematic property drivers**: Link scalar properties together (e.g. map scroll offset to header scale or background blur) with linear, ease-in, or ease-out curves.
* **Unit-aware resting thresholds**: Automatic resting epsilons for `Pixels`, `Normalized`, `Degrees`, and `Radians` to save CPU cycles when animations settle.
* **Spatial 3D geometry**: Built-in 3D transform matrices ([`CATransform3D`](docs/ios-to-twell.md#catransform3d) layout compatible), quaternions, SLERP, and perspective projection math (`m34`).
* **Also supports Python**: Just `import twell` for object-oriented Python bindings.

---

## Twell is all about Model and Presentation values

Twell enforces strict separation between where an object **logically is** and where it **physically renders**:

* **Model value**: The target destination state. When a user opens a menu, the model value instantly becomes `1.0`.
* **Presentation value**: The current physics position on screen. During render loops, **always draw using presentation values**.

---

## Quick start

### C or C++

Include `twell.h` in your build. In one C file, define `TWELL_IMPL`:

```c
#define TWELL_IMPL
#include "twell.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    // Ask Twell how large the arena must be, then allocate exactly that
    size_t arena_size = twell_get_memory_requirement(128 /* max props */, 32 /* max gestures */);
    uint8_t* arena = malloc(arena_size);
    twell_context* ctx = twell_context_create(arena, arena_size, 128, 32);
    if (!ctx) {
        fprintf(stderr, "failed to create twell context\n");
        free(arena);
        return 1;
    }

    // create a 2D position property (e.g. X and Y coords in pixels)
    twell_vector2 start_pos = { 100.0, 100.0 };
    twell_property_id prop = twell_property_create_2d_with_unit(ctx, start_pos, TWELL_UNIT_PIXELS);

    // yeee bouncy spring
    twell_spring_config spring = {
        .mass = 1.0,
        .stiffness = 250.0,
        .damping = 18.0,
        .initial_velocity = 0.0
    };

    // animate to new target destination at absolute time
    double current_time = 0.0;
    twell_vector2 target = { 400.0, 300.0 };
    twell_property_animate_to_target_2d(ctx, prop, target, spring, current_time);

    // main loop
    while (current_time < 2.0) {
        current_time += 1.0 / 60.0; // simulate 60 FPS

        // step physics forward using absolute hardware time
        twell_context_tick(ctx, current_time, NULL, 0);

        // fetch current on-screen location for rendering
        twell_vector2 pos = twell_property_get_presentation_value_2d(ctx, prop);
        printf("Time: %.2fs | Render position: (%.1f, %.1f)\n", current_time, pos.x, pos.y);
    }

    free(arena);
    return 0;
}
```

### 2. Python

#### Installation & Building

Install `twell` directly into your Python environment:

```bash
pip install .
```

#### Usage Example

Import `twell` alongside your renderer (e.g., Pygame, Raylib, or Arcade):

```python
import time
from twell import Context, SpringConfig, Vector2, UnitType

ctx = Context(max_properties=128, max_gestures=32)

# create a 2D animatable property
card_pos = ctx.create_property_2d(
    Vector2(100.0, 100.0), unit=UnitType.PIXELS
)

# configure a spring
spring = SpringConfig(mass=1.0, stiffness=250.0, damping=18.0)

# trigger retargeting animation
start_time = time.time()
card_pos.animate_to_target(
    Vector2(400.0, 300.0), spring, start_time
)

# render loop simulation
current_time = start_time
for _ in range(60):
    current_time += 1.0 / 60.0
    ctx.tick(current_time)
    
    # draw at the current presentation value
    pos = card_pos.presentation_value
    print(f"Render position: ({pos.x:.1f}, {pos.y:.1f})")
```

That's it! Now you can read [the beginner's guide](docs/beginners-guide.md).

---

## Core concepts

### Additive state machine
When an ongoing spring animation is retargeted before reaching its destination, traditional animation engines either snap position or reset velocity. Twell stacks impulse functions mathematically, blending momentum seamlessly so interrupted animations feel natural and responsive.

### 2. Gesture Tracking & Spring Handoff
Bind touch/mouse movement directly to properties with optional boundary constraints:

```c
// Lock property to touch gesture with rubber-banding boundaries
twell_property_track_gesture_2d(ctx, prop_id, gesture_id, bounds_min, bounds_max);

// Touch released: automatically hand off to a spring, preserving touch flick velocity
twell_property_release_gesture_spring_2d(ctx, prop_id, gesture_id, target_rest_pos, spring_cfg, time);
```

### 3. Driven Properties (Kinematic Coupling)
Create reactive UI relationships where one property automatically drives another:

```c
// Map vertical scroll offset (600px -> 0px) to background card scale (1.0 -> 0.8)
twell_property_add_driver(
    ctx, 
    bg_scale_id,         // Driven property
    scroll_y_id,         // Driver property
    600.0, 0.0,          // Driver input range
    1.0, 0.8,            // Driven output range
    TWELL_MAP_LINEAR,    // Mapping curve
    true                 // Clamp to output range
);
```

---

## Wait, isn't it literally CoreAnimation?

For developers coming from iOS (UIKit /SwiftUI/CoreAnimation), Twell is remarkably similar to Apple's motion architecture:

| Apple | Twell |
| :--- | :--- |
| `layer.position` (Target) | `twell_property_get_model_value` / `prop.model_value` |
| `layer.presentation()` (Current frame) | `twell_property_get_presentation_value` / `prop.presentation_value` |
| `CACurrentMediaTime()` | Your render loop's `get_absolute_time()` |
| Additive animations (iOS 8+) | Native impulse ring-buffer blend kernel |
| `CASpringAnimation` | `twell_spring_config` + `twell_property_animate_to_target` |
| `UIPanGestureRecognizer` | `twell_gesture_id` + low-pass touch history |
| `UIScrollView` rubber-banding | `twell_property_track_gesture` (boundary limits) |
| `UIScrollView.DecelerationRate` | `twell_property_release_gesture_decay` |
| `CATransform3D` | `twell_transform3d` (includes perspective `m34` field) |

*For a complete guide on transitioning from iOS to Twell, read the [iOS to Twell](docs/ios-to-twell.md) guide.*
