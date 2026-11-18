# Physics Conventions

These conventions are binding for all simulation code. The ones that depend on Godot behaviour are
checked by `Tests/test_conventions.gd`.

## Units and coordinates
- SI: m, kg, s, N, N·m, N·s (impulse), kg·m² (inertia).
- Right-handed, +X right, +Y up, local **-Z forward** (`Vector3.FORWARD`). X × Y = Z.
- Positive rotation about an axis is counter-clockwise looking down the axis toward the origin (right-hand rule).
- Default gravity (0, -9.81, 0) m/s², configurable in `SimConfig`.

## State
- Body position is the **center of mass (COM) in world coordinates**. It is the sole integration reference.
  Visual/mesh origins that differ from the COM are a presentation concern.
- Orientation: unit `Quaternion` rotating **local → world** (`Basis(q)` is the local→world matrix R).
  The body setter re-normalizes on every assignment.
- Linear velocity: velocity of the COM, world frame.
- Angular velocity ω: **world frame**, rad/s.
- Quaternion product `a * b` is the Hamilton product: apply `b` first, then `a`. A world-frame rotation increment
  `dq` is applied as `q' = dq * q`.

## Inertia
- `local_inertia` is a 3×3 `Basis` about the COM, in the body frame. Stored as a full symmetric matrix
  (diagonal principal tensors are just a special case) so full tensors can be used later.
- `I_world = R · I_local · Rᵀ`, `I_world⁻¹ = R · I_local⁻¹ · Rᵀ`. Recomputed whenever orientation changes.
- Static body: mass 0, inverse mass 0, inverse inertia = zero matrix. A **finite** mass ≤ 0 means static (the inertia argument is
  ignored). A non-finite mass is invalid, never static.
- **Validity of a dynamic body's inertia** (`SimMassProperties.validate_inertia`, M0.1): the tensor must be (1) finite,
  (2) symmetric, (3) positive definite, (4) well conditioned. A positive determinant alone proves none of this
  (e.g. `diag(-1,-1,1)`). Method: normalize by the largest |element| (verdict independent of physical scale), all arithmetic in
  float64; symmetry within 1e-5 of the largest element (~84 float32 ulps: allows R·D·Rᵀ rounding); Cholesky with strictly
  positive pivots; reciprocal 1-norm condition number ≥ 1e-5 (below that fewer than ~2 digits of the float32 inverse survive).
  Slender bodies are fine (rod r=1 cm, L=1 m has rcond ≈ 6e-4); the triangle inequality of principal moments is not enforced.
- Invalid mass properties are observable (`valid == false`, `error_message`, `push_error`), carry zero inverse mass/inertia, and are
  refused by `SimWorld.add_body` (returns null) and `SimRigidBody.set_mass_properties` (returns false); `SimWorld.step()` also
  refuses a world containing one. The inverse inertia is computed in float64 and stored as float32 (`Basis.inverse()` overflows
  float32 for tensors above ~1e12 although the true inverse is representable).
- Box: I = m/12·diag(y²+z², x²+z², x²+y²) with full side lengths. Sphere: 2/5·m·r².

## Forces, torques, impulses
- Force (N) and torque (N·m) accumulate over a step and are consumed (then cleared) by that step.
- Force at a world point p: force += F, torque += (p − x_com) × F.
- Impulse (N·s) changes velocity immediately: Δv = J/m, Δω = I_world⁻¹ ((p − x_com) × J). Impulses never touch the accumulators.
- Angular impulse: Δω = I_world⁻¹ L.

## Rotational dynamics
α = I_world⁻¹ (τ − ω × (I_world ω)) — the gyroscopic term is included.

## Constraint Jacobian conventions
- A row is `J = [Jv_a, Jw_a, Jv_b, Jw_b]` (world frame). Constraint velocity `Cdot = Jv_a·v_a + Jw_a·ω_a + Jv_b·v_b + Jw_b·ω_b`.
- A scalar impulse λ applies `Δv_a = m_a⁻¹ Jv_a λ`, `Δω_a = I_a⁻¹ Jw_a λ` (same for b). Impulses are therefore along +Jᵀ.
- Effective mass `1/(K + γ)` with `K = J M⁻¹ Jᵀ`, softness γ ≥ 0.
- Row solve: `λ = −(Cdot + bias + γ·λ_acc)/(K+γ)`, then `λ_acc` is clamped to `[min, max]` and only the clamped delta is applied.
  Bilateral rows use (−∞, +∞); unilateral (contact-like) rows use `min = 0`.
- Ball joint: `C = (x_b + r_b) − (x_a + r_a)`, with `r = R · local_anchor`. Rows along world X, Y, Z:
  `Jv_a = −e, Jw_a = −(r_a × e), Jv_b = +e, Jw_b = r_b × e`.

## Stabilization
- Velocity-level Baumgarte: `bias = (β/Δt)·C` so the solver drives `Cdot → −(β/Δt) C`, removing a fraction β of the position error per step.
  Purpose: counter *constraint drift* caused by first-order position integration. β = `SimConfig.baumgarte_factor` (default 0.2).
- Known limitation: Baumgarte adds/removes a small amount of energy proportional to the error. Split impulses or post-stabilization is deferred.
- Distinguish: **numerical error** (integration order), **constraint drift** (anchor separation, bounded by Baumgarte), **physical motion** (swing).

## Time
- Fixed timestep default 1/120 s, independent of render rate. Accumulator with frame-time clamp and step cap to avoid a spiral of death.
- Rendering interpolates between the previous and the published state: `alpha = accumulator / dt`.

## Numerical precision (verified on Godot 4.7.stable.official.5b4e0cb0f)
- `Vector3`, `Quaternion`, `Basis`, `Transform3D` are **single precision (float32)** in the standard Godot build; GDScript scalar `float` is 64-bit.
  Anything stored in a vector/basis/quaternion (positions, velocities, inertia, orientation) carries ~1e-7 relative error.
- Inertia validation thresholds above are chosen from this float32 epsilon (1.19e-7); tests use I·I⁻¹ ≈ identity at 1e-5.
- Test tolerances therefore use >= 1e-6 (relative to magnitude) for vector/basis quantities. Do NOT write 1e-9 tolerances on vector results.
- Consequence for future work: long-running or large-coordinate scenes lose position resolution (float32 at 100 m ~ 8e-6 m).
  A double-precision Godot build (`precision=double`) is the migration path if this becomes limiting; record any such decision here.
- Frame-clamp interplay: `max_frame_time / fixed_timestep` (0.1 s -> 12 steps) exceeds `max_steps_per_frame` (10), so the step cap is
  the binding limit for frames longer than ~0.083 s; the remaining backlog is dropped and reported in `dropped_time`.
