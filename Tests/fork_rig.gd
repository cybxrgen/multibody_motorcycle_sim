extends RefCounted
## Shared test fixture (not a test file: the runner only discovers test_*.gd). A two-body prismatic rig with an arbitrary, raked
## joint axis and differently oriented bodies, plus TEST-OWNED copies of the reference geometry so the physical behaviour can be
## judged against the configured geometry instead of the constraint's own cached values.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")

## World joint point and axis of the standard rig (a generic raked axis, not along any world axis).
const POINT: Vector3 = Vector3(0.2, 0.5, -0.1)
static func axis() -> Vector3:
	return Vector3(0.3, 0.8, -0.5).normalized()


## Unit vector perpendicular to `u` (world basis vector least aligned with it, projected).
static func perp_reference(u: Vector3) -> Vector3:
	var best: Vector3 = Vector3.RIGHT
	var best_dot: float = absf(u.x)
	if absf(u.y) < best_dot:
		best = Vector3.UP
		best_dot = absf(u.y)
	if absf(u.z) < best_dot:
		best = Vector3.BACK
	return (best - u * u.dot(best)).normalized()


static func body(mass: float, pos: Vector3, rot: Quaternion, size: Vector3 = Vector3(0.3, 0.4, 0.2)) -> SimRigidBody:
	return SimRigidBody.new(SimMassProperties.new(mass, SimMassProperties.box_inertia(mass, size)), pos, rot)


## Builds world, bodies and a registered prismatic joint. Options (Dictionary, all optional):
##   mass_a, mass_b (3, 5); static_a (false); on_axis (false: COMs lie ON the joint axis, bodies unrotated relative to it);
##   isotropic (false: sphere-like inertia); gravity (zero); iterations (200); limits (Vector2(lower, upper) enables the limits).
## Returns {world, a, b, joint, u, point, pa, pb, ua, ub, ra, rb} where pa/pb/ua/ub/ra/rb are the test-owned local anchor, axis and
## reference vectors computed from the reference world geometry (never read back from the joint).
static func make(options: Dictionary = {}) -> Dictionary:
	var w: SimWorld = SimWorld.new()
	w.config.gravity = options.get("gravity", Vector3.ZERO)
	w.config.solver_iterations = options.get("iterations", 200)
	var u: Vector3 = axis()
	var qa: Quaternion = Quaternion(Vector3(1, 2, 3).normalized(), 0.7)
	var qb: Quaternion = Quaternion(Vector3(-2, 1, 1).normalized(), 1.1)
	var pos_a: Vector3 = POINT - u * 0.3 + Vector3(0.1, -0.05, 0.08)
	var pos_b: Vector3 = POINT + u * 0.4 + Vector3(-0.06, 0.07, 0.1)
	if options.get("on_axis", false):
		qa = Quaternion.IDENTITY
		qb = Quaternion.IDENTITY
		pos_a = POINT - u * 0.3
		pos_b = POINT + u * 0.4
	var a: SimRigidBody = body(options.get("mass_a", 3.0), pos_a, qa) if not options.get("static_a", false) else SimRigidBody.new(SimMassProperties.new(0.0), pos_a, qa)
	var b: SimRigidBody = body(options.get("mass_b", 5.0), pos_b, qb)
	if options.get("isotropic", false):
		# Equal principal moments: no gyroscopic torque for any spin, so spin about the joint axis is torque-free.
		if not options.get("static_a", false):
			a = SimRigidBody.new(SimMassProperties.new(options.get("mass_a", 3.0), SimMassProperties.sphere_inertia(options.get("mass_a", 3.0), 0.2)), pos_a, qa)
		b = SimRigidBody.new(SimMassProperties.new(options.get("mass_b", 5.0), SimMassProperties.sphere_inertia(options.get("mass_b", 5.0), 0.2)), pos_b, qb)
	w.add_body(a)
	w.add_body(b)
	var ref: Vector3 = perp_reference(u)
	var joint = PrismaticJoint.from_world(a, b, POINT, u, ref)
	if options.has("limits"):
		joint.limit_enabled = true
		joint.lower_limit = options["limits"].x
		joint.upper_limit = options["limits"].y
	w.add_constraint(joint)
	return {
		"world": w, "a": a, "b": b, "joint": joint, "u": u, "point": POINT, "ref": ref,
		"pa": a.orientation.inverse() * (POINT - a.position), "pb": b.orientation.inverse() * (POINT - b.position),
		"ua": a.orientation.inverse() * u, "ub": b.orientation.inverse() * u,
		"ra": a.orientation.inverse() * ref, "rb": b.orientation.inverse() * ref,
	}


## Independent measurement of the five constrained quantities and the free coordinate from the TEST-OWNED local data:
## {s (m), lateral (m), axis (rad), twist (rad)}.
static func measure(r: Dictionary) -> Dictionary:
	var a: SimRigidBody = r.a
	var b: SimRigidBody = r.b
	var anchor_a: Vector3 = a.position + a.orientation * r.pa
	var anchor_b: Vector3 = b.position + b.orientation * r.pb
	var ua: Vector3 = a.orientation * r.ua
	var ub: Vector3 = b.orientation * r.ub
	var d: Vector3 = anchor_b - anchor_a
	var p: Vector3 = a.orientation * r.ra
	var q: Vector3 = ua.cross(p)
	var w: Vector3 = b.orientation * r.rb
	return {
		"s": d.dot(ua), "lateral": (d - ua * d.dot(ua)).length(),
		"axis": atan2(ua.cross(ub).length(), ua.dot(ub)), "twist": atan2(w.dot(q), w.dot(p)),
	}
