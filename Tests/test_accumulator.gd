extends "res://Tests/sim_test_case.gd"
## M0.1.1: SimWorld.advance() accumulator edge cases (relative step tolerance).
## Tolerances: sim-time/accumulator comparisons use 1e-9 * dt or 1e-12 s (double-precision sums of
## <= 1e4 terms of size ~dt carry ~1e-12 * dt error); velocities are float32 (1e-6 relative).

const DTS: Array = [1.0 / 120.0, 1.0e-3, 1.0e-6, 1.0e-9]


func _world(dt: float = 1.0 / 120.0) -> SimWorld:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	w.config.fixed_timestep = dt
	var b = w.add_body(make_sphere(1.0, 0.5, Vector3(0, 5, 0)))
	b.linear_velocity = Vector3(1, 0, 0)
	return w


func _jointed_world(dt: float) -> SimWorld:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	w.config.fixed_timestep = dt
	var anchor = w.add_body(make_static(Vector3.ZERO))
	var bob = w.add_body(make_sphere(1.0, 0.1, Vector3(1, 0, 0)))
	w.add_constraint(BallJoint.new(anchor, bob, Vector3.ZERO, Vector3(-1, 0, 0)))
	return w


func _state(w: SimWorld) -> Array:
	var out: Array = [w.step_count, w.simulation_time, w.accumulator, w.dropped_time]
	for b in w.bodies:
		out.append_array([b.position, b.orientation, b.linear_velocity, b.angular_velocity, b.force_accumulator])
	for c in w.constraints:
		for row in c.rows:
			out.append(row.accumulated_impulse)
	return out


func test_a_zero_elapsed_time_does_nothing() -> void:
	var w: SimWorld = _jointed_world(1.0 / 120.0)
	var before: Array = _state(w)
	check(w.advance(0.0) == 0, "advance(0) runs zero steps")
	check(before == _state(w), "state, accumulator and constraint impulses unchanged")
	w.advance(0.4 / 120.0)  # leave a partial step in the accumulator
	var acc: float = w.accumulator
	check(w.advance(0.0) == 0, "advance(0) with partial accumulator runs zero steps")
	check(w.accumulator == acc, "partial accumulator unchanged by advance(0)")


func test_b_tiny_timestep_zero_elapsed_runs_no_phantom_steps() -> void:
	for dt in [1.0e-13, 1.0e-12, 1.0e-15]:
		var w: SimWorld = _jointed_world(dt)
		check(w.config.is_valid(), "dt %s is an accepted configuration" % dt)
		var before: Array = _state(w)
		check(w.advance(0.0) == 0, "dt %s: advance(0) runs zero steps" % dt)
		check(before == _state(w), "dt %s: positions, velocities, constraint impulses, accumulator unchanged" % dt)


func test_c_less_than_one_timestep_accumulates() -> void:
	for dt in DTS + [1.0e-13]:
		var w: SimWorld = _world(dt)
		check(w.advance(0.5 * dt) == 0, "dt %s: half a step -> no step" % dt)
		check_near(w.accumulator, 0.5 * dt, 1.0e-9 * dt, "dt %s: elapsed time retained" % dt)
		check(w.advance(0.5 * dt) == 1, "dt %s: second half completes exactly one step" % dt)


func test_d_exactly_one_timestep() -> void:
	for dt in DTS + [1.0e-13]:
		var w: SimWorld = _world(dt)
		check(w.advance(dt) == 1, "dt %s: advance(dt) -> exactly one step" % dt)
		check(w.step_count == 1, "dt %s: step_count 1" % dt)
		check_near(w.simulation_time, dt, 1.0e-12 * dt, "dt %s: simulation_time = dt" % dt)
		check_near(w.bodies[0].linear_velocity.y, -9.81 * dt, 1.0e-6 * 9.81 * dt + 1.0e-30, "dt %s: v = g dt after one semi-implicit step" % dt)


func test_e_multiple_accumulated_timesteps() -> void:
	for dt in [1.0 / 120.0, 1.0e-3]:
		var w: SimWorld = _world(dt)
		check(w.advance(3.0 * dt) == 3, "dt %s: three steps" % dt)
		check_near(w.simulation_time, 3.0 * dt, 1.0e-12, "dt %s: time = 3 dt" % dt)
		check(w.accumulator <= 1.0e-9 * dt, "dt %s: no leftover that could cause a phantom step" % dt)
		check(w.advance(0.0) == 0, "dt %s: no phantom step afterwards" % dt)
		check(w.advance(4.5 * dt) == 4, "dt %s: 4.5 dt -> four steps" % dt)
		check_near(w.accumulator, 0.5 * dt, 1.0e-9 * dt, "dt %s: half step retained" % dt)


func test_f_split_intervals_accumulate_without_early_or_lost_steps() -> void:
	for dt in [1.0 / 120.0, 1.0e-3]:
		for parts in [3, 7, 10, 1000]:
			var w: SimWorld = _world(dt)
			var elapsed: float = 0.0
			var inc: float = dt / parts
			var early: bool = false
			for i in parts * 60:  # 60 timesteps of elapsed time in small increments
				w.advance(inc)
				elapsed += inc
				if w.simulation_time > elapsed + 1.0e-6 * dt:
					early = true
			var label: String = "dt %s, %d parts per step" % [dt, parts]
			check(not early, label + ": never stepped ahead of elapsed time")
			check(w.step_count == 60, label + ": exactly 60 steps (got %d)" % w.step_count)
			check_near(w.simulation_time + w.accumulator, elapsed, 1.0e-9 * dt, label + ": no elapsed time lost")


func test_g_default_configuration_behaves_as_before() -> void:
	var cfg = SimWorld.new().config
	check_near(cfg.fixed_timestep, 1.0 / 120.0, 0.0, "default dt unchanged")
	check(cfg.max_steps_per_frame == 10, "default max steps unchanged")
	check_near(SimWorld.step_tolerance(cfg.fixed_timestep), 1.0e-12, 0.0, "tolerance at default dt is exactly the previous 1e-12")
	var a: SimWorld = _world()
	var b: SimWorld = _world()
	check(a.advance(6.0 / 120.0) == 6, "0.05 s -> 6 steps")
	run_steps(b, 6)
	check(a.bodies[0].position == b.bodies[0].position and a.bodies[0].linear_velocity == b.bodies[0].linear_velocity, "advance() state identical to 6 direct steps")
	check(a.advance(1.0 / 120.0 - 1.0e-13) == 1, "an elapsed time within the 1e-12 tolerance of one step still steps (unchanged behavior)")
	check(a.advance(0.5 / 120.0) == 0, "half step -> none")


func test_h_max_steps_cap_preserves_policy() -> void:
	var dt: float = 1.0 / 120.0
	var w: SimWorld = _world(dt)
	w.config.max_frame_time = 1.0
	check(w.advance(0.5 + 0.3 * dt) == 10, "capped at max_steps_per_frame")
	check_near(w.accumulator, 0.3 * dt, 1.0e-9 * dt, "sub-step phase preserved")
	check_near(w.dropped_time, 0.5 - 10.0 * dt, 1.0e-9, "whole backlog steps dropped and recorded")
	var v: SimWorld = _world(dt)
	v.config.max_frame_time = 1.0
	check(v.advance(0.5) == 10, "exact multiple also capped")
	check(v.accumulator <= 1.0e-9 * dt, "no spurious leftover step after drop")
	# small timestep: relative tolerance must not break the cap logic
	var s: SimWorld = _world(1.0e-6)
	s.config.max_frame_time = 1.0
	s.config.max_steps_per_frame = 5
	check(s.advance(10.5e-6) == 5, "small dt: capped at 5 steps")
	check_near(s.accumulator, 0.5e-6, 1.0e-9 * 1.0e-6, "small dt: phase preserved")
