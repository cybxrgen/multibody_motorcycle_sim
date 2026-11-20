# Physics Conventions

**Authoritative for:** units, coordinate frames, quaternion/angular-velocity, mass and inertia, force/impulse, constraint/Jacobian, hinge-joint,
contact and friction, stabilization, time-stepping and numerical-precision conventions. Binding for all simulation code.
**Belongs elsewhere:** implementation progress and known limitations → `PROJECT_STATE.md`; module design → `ARCHITECTURE.md`;
test results and methodology → `TESTING.md`.
**Update when:** a convention, equation, sign rule or numerical assumption changes. Conventions that depend on Godot behaviour are checked by `Tests/test_conventions.gd`.

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
- **Validity of a dynamic body's inertia** (`SimMassProperties.validate_inertia`): the tensor must be (1) finite,
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
- Split row solve (positional correction, see "Stabilization: split impulses"): the same `J` and effective mass act on the **split velocities** `v_s, w_s`:
  `lambda_s = -(J.v_s + b_s + gamma*lambda_s_acc)/(K+gamma)`, `lambda_s_acc` clamped to the row's split bounds, only the clamped delta is applied to `v_s, w_s` (never to `v, w`).
  `b_s` is the split bias; `lambda_s_acc` starts at 0 every step (no warm start).
- Ball joint: `C = (x_b + r_b) − (x_a + r_a)`, with `r = R · local_anchor`. Rows along world X, Y, Z:
  `Jv_a = −e, Jw_a = −(r_a × e), Jv_b = +e, Jw_b = r_b × e`. Physical bias 0 (`Cdot = 0`); split bias `b_s = (beta/dt) C.e` (bilateral).

## Hinge joint
- Frame data per body, local: anchor, unit axis, unit reference direction perpendicular to the axis. Zero angle <=> `R_a*ref_a == R_b*ref_b`. `HingeJointConstraint.from_world(a, b, anchor, axis, reference)` (`hinge_joint_constraint.gd`) makes the current pose the zero angle. Input is validated, never repaired: unit length / perpendicularity within 1e-6 (8 float32 ulps); valid input is then re-normalized exactly.
- World frame (A): `a = R_a*axis_a`, `p = R_a*ref_a`, `q = a x p`; `b = R_b*axis_b`.
- Axis alignment rows: `C_p = p.b`, `C_q = q.b` (sine-type angle error, zero for b = +-a). `Jw_a = p x b`, `Jw_b = b x p` (and q), physical `bias = 0`, split `b_s = (beta/dt) C` (bilateral). Perpendicular to the axis, so rotation about it is free.
- Angle: `atan2(w.q, w.p)`, `w = R_b*ref_b`; positive right-handed about +a (B turns counter-clockwise seen from +a relative to A). Continuous: only wrapped differences (`wrapf(d, -PI, PI)`) are accumulated; valid while the relative rotation between two samples is < PI (every step samples). A tilt of b changes it at second order.
- Hinge-axis rows: `Jw_b = +a, Jw_a = -a` (`Cdot = (w_b - w_a).a`, the angle rate); the upper limit uses the negated pair.
- Limits (unilateral, impulse in [0, +inf)): lower `C = angle - lower`, upper `C = upper - angle`. Physical bias `C/dt` while `C >= 0` (speculative: engages only if the limit would be crossed this step, not a correction) and `0` when violated (the angle may not move further out: `Cdot >= 0`); continuous at 0. A violated limit is corrected by the split channel only: split bias `(beta/dt) C` (negative), split bounds `[0, +inf)`; the split row is inactive while `C >= 0`. Lower and upper must be finite, `lower <= upper`; both enabled together.
- Motor: row bias `-target_speed`; positive speed => positive angle rate (B along +a, A along -a); impulse bound `+-max_torque*dt` (`dt = config.fixed_timestep`); `max_torque = 0` or disabled => bounds [0,0]. Target is first clamped into the rate interval the enabled limit rows admit (`[-bias_lower, +bias_upper]`, the **physical** limit biases: a violated limit contributes 0), so motor and limits never push against each other and the motor never drives further into a violated limit. The motor row has no split channel.
- Momentum: all rows except the anchor rows are pure equal-and-opposite angular impulses; anchor rows change total angular momentum about the origin by `C x lambda`. The split pose correction adds the term described in "Stabilization: split impulses".

## Contact (`ContactConstraint`)
One-point contact between two bodies; the caller supplies the geometry and creates, refreshes and removes the constraint (no collision detection exists).
Several contact points are several independent constraints.
- **Data.** Anchor on each body (local frame), unit normal `n` in body A's local frame, friction coefficient `mu >= 0`, optional `tangent_rotation`. World: `pa = x_a + R_a*anchor_a`, `pb = x_b + R_b*anchor_b`, `n = R_a*normal_a`. Validated, never repaired: finite anchors, finite nonzero normal of unit length within 1e-6 (valid input is then normalized exactly), finite `mu >= 0`, finite `tangent_rotation`, two different bodies, not both static.
- **Sign convention.** `n` points from A toward B. Gap `g = n.(pb - pa)`: `g > 0` separated, `g < 0` penetrating. Normal relative velocity `vn = n.(v_b(p) - v_a(p))`: `vn > 0` separating, `vn < 0` approaching.
- **Shared contact point.** `p = (pa + pb)/2`. All rows use lever arms `r_a = p - x_a`, `r_b = p - x_b`, so the equal-and-opposite impulses act at one point (exact conservation of linear momentum and of angular momentum about any origin, up to float32). A tangential offset between `pa` and `pb` is not constrained.
- **Caller duty.** Anchors are fixed to the bodies. When the contact patch moves relative to a body (sliding, rolling) the caller must re-seat the anchors, otherwise lever arms and gap drift away from the real contact.
- **Normal row.** `Jv_a = -n, Jw_a = -(r_a x n), Jv_b = +n, Jw_b = r_b x n`, so `Cdot = vn`. Impulse bounds `[0, +inf)`: it can push, never pull. The row solve enforces `Cdot + bias >= 0`, i.e. `vn >= -bias`.
  - Speculative, `g >= 0`: require the gap not to become negative, `g + vn*dt >= 0`  =>  `bias = g/dt`. Inactive (zero impulse) unless the bodies would cross within this step. Closing is judged with the current velocities (frozen Jacobian, linear in `vn`). Physical channel; the split row is inactive.
  - Penetrating, `g < 0`: the **physical** normal row has `bias = 0` (`vn >= 0`: no further approach, never an attractive impulse; continuous with the speculative bias at `g = 0`). The penetration itself is removed by the **split** normal row: split bias `(beta/dt) g` (negative, `beta = SimConfig.baumgarte_factor`; no separate parameter), bounds `[0, +inf)`, i.e. `vn_split >= beta|g|/dt`, so the pose moves by `beta|g|` per step (`g + vn_split*dt = (1 - beta) g`) and no velocity is added to either body. Same form as the hinge limits (`C = g`), with the sign fixed by the convention above.
  - No restitution: contacts are perfectly inelastic in the normal direction.
- **Friction (isotropic Coulomb).** Tangent rows `t1, t2` have the same Jacobian pattern with `t` instead of `n` and bias 0 (the tangential relative velocity at `p` is driven to zero). With accumulated tangent impulses `lambda_t = (l1, l2)` and normal impulse `lambda_n`: admissible set `|lambda_t| <= mu * lambda_n` (a disk, not a square). Per solver iteration, in this order:
  1. normal row (clamped to `[0, inf)`);
  2. block update `y = lambda_t - K^-1 c`, `K_ij = J_i M^-1 J_j^T` (2x2, current inertias), `c = (Cdot_t1, Cdot_t2)`;
  3. `lambda_t <- argmin_{|x| <= mu*lambda_n} (x - y)^T K (x - y)` (K-metric projection; `y` if already inside; zero if `mu*lambda_n = 0`);
  4. apply only `lambda_t_new - lambda_t_old`.
  Friction is physical and reads only the physical normal row's accumulated impulse; the split normal impulse is a different field and never enters the disk radius, the warm start or any friction quantity.
  Because step 3 minimizes the same quadratic as the unconstrained block update, it is the exact block Gauss-Seidel step for the friction subproblem; at a converged sliding contact the friction impulse is antiparallel to the remaining slip (maximum dissipation). A radial (Euclidean) scaling would not have this property when `K` is anisotropic and is therefore not used. Sticking = `y` inside the disk; sliding = boundary active. Static and kinetic friction share one `mu`.
  Projection: `x(nu) = (K + nu I)^-1 K y` with `|x(nu)|` strictly decreasing in `nu >= 0`; `nu` is found by 60 bisection steps (float64) and the end of the bracket inside the disk is used, so the bound is never exceeded.
- **Tangent basis.** `e` = world axis with the smallest `|n.e|`; `t1 = normalize(n x e)`, `t2 = n x t1` (right-handed with `n`), then both rotated about `n` by `tangent_rotation`. The block solve and the disk are rotation invariant, so results do not depend on the basis; it can be rebuilt every step.
- **Warm starting.** Normal impulse: like every row (scaled by `warm_start_factor`, clamped to `[0, inf)`). Friction: stored between steps as a world vector (the basis changes with `n`), scaled by `warm_start_factor`, projected into the current tangent plane, then radially clipped to the disk of radius `mu * (warm-started lambda_n)` before it is applied; so the carried impulse is admissible for the current normal impulse and `mu`. Without warm starting both start at zero.
- **Momentum.** Every row has `Jv_a = -Jv_b`; the lever arms share `p`. Total (physical) linear momentum is conserved, and the physical impulses conserve angular momentum about the origin up to float32 rounding (spheres in free flight; non-spherical bodies additionally carry the explicit gyroscopic error of the integrator). A split correction moves the bodies without changing their momentum; see "Stabilization: split impulses".
- **Numerical assumptions.** Rows use the Jacobians linearized at the start of the step (see the frozen-Jacobian limitation in `PROJECT_STATE.md`). Tests of a single-row solve use `1e-5` for float32 quantities of order 1.

## Stabilization: split impulses
Position error (constraint drift, contact penetration, a violated hinge limit) is corrected through a **separate velocity channel**, so numerical correction never changes physical velocity, momentum or kinetic energy.
- **Two velocity channels per body and fixed step.** Physical `v, w` (gravity, forces, motor, contact normal impulse, friction, joint velocity constraints, speculative biases) and split `v_s, w_s` (positional correction only). `v_s, w_s` are zero at the start of every step (`SimWorld` zeroes them before the solver and the solver zeroes those of the bodies it touches before the split pass), are written only by the split pass, are used only by the position integration, and are zero again right after it. They are never stored as body velocity and never accumulate across steps. Momentum, kinetic and potential energy, `velocity_at_point`, motor rate, friction limits and contact velocity use `v, w` only. Static bodies have zero inverse mass and inertia, so their split velocities stay exactly zero.
- **Split solve.** Same rows, Jacobians and effective masses (`K = J M^-1 J^T`, current world inverse inertia) as the physical solve, applied to `v_s, w_s`: `lambda_s = -(J.v_s + b_s + gamma*lambda_s_acc)/(K+gamma)`, clamped to the row's split bounds, with its own accumulated impulse. It runs after the physical pass, for `solver_iterations` iterations over all active constraints. It is **never warm started**: `lambda_s_acc` is reset in every `prepare`. The two passes do not read each other's state, so their order only fixes determinism.
- **Biases.** With `beta = SimConfig.baumgarte_factor` (the only stabilization parameter; no other tuning value exists) the split bias of a positional row is `b_s = (beta/dt) C`, so the converged split solve satisfies `J.v_s = -(beta/dt) C` and the pose moves by `-beta C` per step: a fraction beta of the error is removed. The physical bias of the same row is 0 (a joint row enforces `Cdot = 0`).
- **Which effects use which channel.**

  | Effect | Channel |
  |---|---|
  | Ball anchor, hinge anchor, hinge axis alignment error | split (`b_s = (beta/dt) C`, bilateral); physical `Cdot = 0` |
  | Violated hinge limit (`C < 0`) | split (`b_s = (beta/dt) C`, bounds `[0, +inf)`); physical bias 0 |
  | Penetrating contact (`g < 0`) | split (`b_s = (beta/dt) g`, bounds `[0, +inf)`); physical bias 0 |
  | Speculative contact (`g >= 0`), speculative hinge limit (`C >= 0`) | physical (`bias = C/dt`); split row inactive |
  | Hinge motor, contact friction, contact normal impulse against approach, gravity, forces | physical only |

- **Position integration.** `x' = x + dt (v + v_s)`, `q' = exp(dt (w + w_s)/2) q` (world-frame increment applied as `dq * q`, then normalized, exactly as before; a body with `v_s = w_s = 0` reproduces the previous update bit for bit). Nothing is teleported and no anchor is overwritten.
- **Consequences.**
  - *No injected kinetic energy.* A body at rest with constraint errors keeps `v = w = 0` exactly; its pose converges to the constraint manifold. For a single exactly solved row the error shrinks as `(1 - beta)^n`. The former velocity-level bias `(beta/dt) C` left `v = beta|C|/dt` in the body (0.48 m/s for a 2 cm contact penetration at the default beta and 120 Hz).
  - *Momentum.* The two split impulses of a constraint are equal and opposite, so they do not move the centre of mass of a dynamic pair, but they move `x` without changing `p`: total angular momentum about the origin changes by `sum m_i dx_i,split x v_i` per step (plus the physical `C x lambda` of joint anchor rows). Linear momentum is unaffected. `SimWorld.total_angular_momentum()` reports physical state and therefore shows this term.
  - *Frozen-Jacobian dissipation is no longer masked.* Rows are linearized once per step. Free hinge rotation about an offset COM leaves a tangential error `delta^2/2` (`delta = w dt`) after each centripetal impulse, so the rate obeys `dw/dt = -f w^3 dt/2`, i.e. `w(t)^-2 = w0^-2 + f dt t`, with `f = m d_perp^2 / I_pivot` (`d_perp`: COM distance from the axis). The former Baumgarte velocity added energy that happened to offset this loss; without it (beta = 0, or with split impulses) the loss is visible. It is `O(dt)` per unit time.
  - *Cold-started convergence.* Split impulses are not warm started, so high mass-ratio configurations need more iterations for the same correction accuracy than the warm-started physical rows (tracked in `PROJECT_STATE.md`).
- **Purpose and limits.** Distinguish: **numerical error** (integration order), **constraint drift** (anchor/axis separation, bounded by the split correction), **physical motion** (swing). beta is the fraction of error removed per step, range `[0, 1]` (beta > 1 would over-correct); there is no hidden damping. First-order drift remains: the steady-state anchor error scales with `v^2 dt^2/(2 L beta)`.

## Time
- Fixed timestep default 1/120 s, independent of render rate. Accumulator with frame-time clamp and step cap to avoid a spiral of death.
- Rendering interpolates between the previous and the published state: `alpha = accumulator / dt`.

## Numerical precision (Godot 4.7)
- `Vector3`, `Quaternion`, `Basis`, `Transform3D` are **single precision (float32)** in the standard Godot build; GDScript scalar `float` is 64-bit.
  Anything stored in a vector/basis/quaternion (positions, velocities, inertia, orientation) carries ~1e-7 relative error.
- Inertia validation thresholds above are chosen from this float32 epsilon (1.19e-7); tests use I·I⁻¹ ≈ identity at 1e-5.
- Test tolerances therefore use >= 1e-6 (relative to magnitude) for vector/basis quantities. Do NOT write 1e-9 tolerances on vector results.
  Tighter tolerances are legitimate only for float64 scalars (e.g. accumulator and simulation-time arithmetic) or exact discrete solutions computed in float64; see `TESTING.md`.
- Consequence for future work: long-running or large-coordinate scenes lose position resolution (float32 at 100 m ~ 8e-6 m).
  A double-precision Godot build (`precision=double`) is the migration path if this becomes limiting; record any such decision here.
- Frame-clamp interplay: `max_frame_time / fixed_timestep` (0.1 s -> 12 steps) exceeds `max_steps_per_frame` (10), so the step cap is
  the binding limit for frames longer than ~0.083 s; the remaining backlog is dropped and reported in `dropped_time`.
