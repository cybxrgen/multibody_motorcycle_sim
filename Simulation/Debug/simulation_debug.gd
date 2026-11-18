extends Node3D
## Debug overlay: world axes, COM markers, local axes, constraint anchors, and a text report.
## Reads published simulation state only (through interpolation); never writes it.

const DebugDraw3D = preload("res://Simulation/Debug/debug_draw_3d.gd")
const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

var world = null
var draw: DebugDraw3D = DebugDraw3D.new()

var show_world_axes: bool = true
var show_com: bool = true
var show_local_axes: bool = false
var show_anchors: bool = true


func _ready() -> void:
	add_child(draw)


func body_transform(body, alpha: float) -> Transform3D:
	return SimBodyState.interpolate_transform(body.previous_state, body.published_state, alpha)


func update_overlay(alpha: float) -> void:
	draw.begin_frame()
	if world != null:
		if show_world_axes:
			draw.axes(Transform3D(Basis.IDENTITY, Vector3.ZERO), 1.0)
		for body in world.bodies:
			var t: Transform3D = body_transform(body, alpha)
			if show_com:
				draw.cross(t.origin, 0.06, Color(1, 1, 0))
			if show_local_axes:
				draw.axes(t, 0.4)
		if show_anchors:
			for c in world.constraints:
				if "local_anchor_a" in c:
					var pa: Vector3 = body_transform(c.body_a, alpha) * c.local_anchor_a
					var pb: Vector3 = body_transform(c.body_b, alpha) * c.local_anchor_b
					draw.cross(pa, 0.05, Color(1, 0.4, 1))
					draw.cross(pb, 0.08, Color(0, 1, 1))
					draw.line(body_transform(c.body_a, alpha).origin, pa, Color(1, 0.4, 1, 0.6))
					draw.line(body_transform(c.body_b, alpha).origin, pb, Color(0, 1, 1, 0.6))
	draw.flush()


func build_report() -> String:
	if world == null:
		return ""
	var max_err: float = 0.0
	for c in world.constraints:
		if c.has_method("linear_error"):
			max_err = maxf(max_err, c.linear_error().length())
	var e_k: float = world.total_kinetic_energy()
	var e_p: float = world.total_potential_energy()
	return "t = %.3f s   steps = %d   dt = %.5f\nKE = %.3f J   PE = %.3f J   E = %.3f J\n|p| = %.3f   |L| = %.3f\nmax joint error = %.5f m   dropped = %.3f s" % [
		world.simulation_time, world.step_count, world.config.fixed_timestep,
		e_k, e_p, e_k + e_p,
		world.total_linear_momentum().length(), world.total_angular_momentum().length(),
		max_err, world.dropped_time]
