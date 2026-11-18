extends "res://Tests/sim_test_case.gd"
## M0.1: SimConfig.validate() and SimWorld refusal behavior for invalid configuration.

const SimConfig = preload("res://Simulation/Core/sim_config.gd")

# Every invalid value must yield exactly one error that names its parameter.
const INVALID_CASES: Array = [
	["fixed_timestep", 0.0], ["fixed_timestep", -1.0 / 120.0], ["fixed_timestep", NAN], ["fixed_timestep", INF],
	["max_frame_time", 0.0], ["max_frame_time", -0.1], ["max_frame_time", NAN], ["max_frame_time", INF],
	["max_steps_per_frame", 0], ["max_steps_per_frame", -3],
	["solver_iterations", 0], ["solver_iterations", -5],
	["baumgarte_factor", -0.01], ["baumgarte_factor", 1.01], ["baumgarte_factor", NAN], ["baumgarte_factor", INF],
	["warm_start_factor", -0.1], ["warm_start_factor", 1.5], ["warm_start_factor", NAN],
	["min_row_denominator", 0.0], ["min_row_denominator", -1.0e-12], ["min_row_denominator", NAN], ["min_row_denominator", INF],
	["gravity", Vector3(NAN, -9.81, 0)], ["gravity", Vector3(0, INF, 0)], ["gravity", Vector3(0, -9.81, -INF)],
]


func _world_with_body() -> SimWorld:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	var b = w.add_body(make_sphere(1.0, 0.5, Vector3(0, 5, 0)))
	b.linear_velocity = Vector3(1, 2, 3)
	b.apply_force(Vector3(4, 5, 6))
	return w


func _snapshot(w: SimWorld) -> Dictionary:
	var b = w.bodies[0]
	return {
		"step_count": w.step_count, "time": w.simulation_time, "accumulator": w.accumulator,
		"dropped": w.dropped_time, "pos": b.position, "orient": b.orientation, "vel": b.linear_velocity,
		"omega": b.angular_velocity, "force": b.force_accumulator, "torque": b.torque_accumulator,
		"prev": b.previous_state.position, "pub": b.published_state.position,
	}


func _check_untouched(w: SimWorld, before: Dictionary, label: String) -> void:
	var after: Dictionary = _snapshot(w)
	for key in before:
		check(before[key] == after[key], "%s: '%s' unchanged" % [label, key])


func test_default_config_is_valid() -> void:
	var cfg = SimConfig.new()
	check(cfg.validate().is_empty(), "default config has no errors")
	check(cfg.is_valid(), "default config is_valid")


func test_each_invalid_parameter_is_reported_once() -> void:
	for entry in INVALID_CASES:
		var cfg = SimConfig.new()
		cfg.set(entry[0], entry[1])
		var errors: PackedStringArray = cfg.validate()
		var label: String = "%s = %s" % [entry[0], entry[1]]
		check(errors.size() == 1, "exactly one error for " + label)
		check(errors.size() >= 1 and errors[0].begins_with(entry[0]), "error names the parameter for " + label)
		check(not cfg.is_valid(), "is_valid false for " + label)


func test_valid_boundary_values_accepted() -> void:
	var cases: Array = [
		["baumgarte_factor", 0.0], ["baumgarte_factor", 1.0], ["warm_start_factor", 0.0], ["warm_start_factor", 1.0],
		["solver_iterations", 1], ["max_steps_per_frame", 1], ["fixed_timestep", 1.0e-6], ["fixed_timestep", 1.0],
		["max_frame_time", 1.0e-9], ["min_row_denominator", 1.0e-300], ["gravity", Vector3.ZERO],
	]
	for entry in cases:
		var cfg = SimConfig.new()
		cfg.set(entry[0], entry[1])
		check(cfg.validate().is_empty(), "boundary accepted: %s = %s" % [entry[0], entry[1]])


func test_all_violations_reported_together_and_nothing_is_clamped() -> void:
	var cfg = SimConfig.new()
	cfg.fixed_timestep = 0.0
	cfg.solver_iterations = 0
	cfg.baumgarte_factor = 7.0
	check(cfg.validate().size() == 3, "three violations, three messages")
	check_near(cfg.fixed_timestep, 0.0, 0.0, "fixed_timestep not clamped")
	check(cfg.solver_iterations == 0, "solver_iterations not clamped")
	check_near(cfg.baumgarte_factor, 7.0, 0.0, "baumgarte_factor not clamped")


func test_advance_refuses_invalid_config_without_partial_step() -> void:
	for entry in INVALID_CASES:
		var w: SimWorld = _world_with_body()
		w.config.set(entry[0], entry[1])
		var before: Dictionary = _snapshot(w)
		var label: String = "advance with %s = %s" % [entry[0], entry[1]]
		var steps: int = w.advance(0.05)  # an infinite loop here (e.g. dt = 0) would hang the suite
		check(steps == 0, label + ": zero steps")
		_check_untouched(w, before, label)
		check(w.rejected_step_count == 1, label + ": refusal counted")
		check(w.last_error.begins_with("invalid SimConfig"), label + ": last_error set")


func test_direct_step_refuses_invalid_config_without_partial_step() -> void:
	for entry in INVALID_CASES:
		var w: SimWorld = _world_with_body()
		w.config.set(entry[0], entry[1])
		var before: Dictionary = _snapshot(w)
		var label: String = "step with %s = %s" % [entry[0], entry[1]]
		check(not w.step(), label + ": step returns false")
		_check_untouched(w, before, label)


func test_config_mutation_after_world_creation_is_caught_and_recoverable() -> void:
	var w: SimWorld = _world_with_body()
	var dt: float = w.config.fixed_timestep
	check(w.advance(dt) == 1, "valid world steps once")
	check(w.last_error == "", "no error while valid")
	w.config.fixed_timestep = 0.0
	var before: Dictionary = _snapshot(w)
	check(w.advance(0.05) == 0, "dt mutated to 0: advance refused")
	check(not w.step(), "dt mutated to 0: step refused")
	_check_untouched(w, before, "dt mutated to 0")
	check_near(w.interpolation_alpha(), 0.0, 0.0, "interpolation_alpha safe with invalid dt")
	w.config.fixed_timestep = dt
	w.config.solver_iterations = 0
	check(not w.step(), "iterations mutated to 0: step refused")
	w.config.solver_iterations = 20
	check(w.step(), "recovers after configuration is fixed")
	check(w.step_count == 2, "step_count continues from the last valid step")
	check(w.last_error == "", "last_error cleared after recovery")
	w.config = null
	check(w.advance(0.05) == 0 and not w.step(), "null config refused")
	check_near(w.interpolation_alpha(), 0.0, 0.0, "interpolation_alpha safe with null config")


func test_repeated_refusals_are_counted_and_reported_once() -> void:
	var w: SimWorld = _world_with_body()
	w.config.fixed_timestep = -1.0
	for i in 100:
		w.advance(0.05)
	check(w.rejected_step_count == 100, "every refusal counted")
	check(w.last_error != "" and w._reported_error == w.last_error, "one distinct error reported, not one per call")
	w.config.fixed_timestep = 1.0 / 120.0
	check(w.advance(1.0 / 120.0) == 1, "world runs again once valid")
	check(w._reported_error == "", "report latch reset after recovery")
