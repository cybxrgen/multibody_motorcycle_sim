extends RefCounted
## Shared fixtures for the M2.5.3 Stage D tests (not a test file: the runner only discovers test_*.gd).
## Everything works on the ACTUAL assembled motorcycle of MotorcycleBuilder (never on a patch built by hand). Pose changes move every dynamic
## body RIGIDLY (so all joints keep their reference geometry) and reset the published states; velocities are left as they are (zero by default).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81


## The assembled prototype with one distributed patch per wheel; `mutate` (optional Callable taking the MotorcycleConfig) edits the configuration first.
static func assemble(mutate: Callable = Callable(), world: SimWorld = null):
	var config = MotorcyclePrototype.config_with_patches()
	if mutate.is_valid():
		mutate.call(config)
	return MotorcycleBuilder.build(config, world)


static func step(assembly, count: int) -> void:
	for k in count:
		assembly.world.step()


## Rotates every dynamic body rigidly by `rotation` about the world origin (which lies on the ground plane) and then translates it by `offset`.
static func move_rigidly(assembly, rotation: Quaternion, offset: Vector3) -> void:
	for body in assembly.bodies():
		body.position = rotation * body.position + offset
		body.orientation = rotation * body.orientation
		body.linear_velocity = rotation * body.linear_velocity
		body.angular_velocity = rotation * body.angular_velocity
		body.reset_published_state()


static func shift(assembly, offset: Vector3) -> void:
	move_rigidly(assembly, Quaternion.IDENTITY, offset)


## Rolls the machine about the world Z axis (the forward axis) by `angle` (rad) and sinks it by `sink` (m).
static func roll(assembly, angle: float, sink: float = 0.0) -> void:
	move_rigidly(assembly, Quaternion(Vector3.BACK, angle), Vector3(0.0, -sink, 0.0))


## Gives every dynamic body the forward speed `speed` (m/s, along -Z) and every wheel the matching free-rolling spin about its axle.
static func roll_forward(assembly, speed: float) -> void:
	for body in assembly.bodies():
		body.linear_velocity = Vector3(0.0, 0.0, -speed)
	for pair in [[assembly.front_wheel, assembly.config.front_wheel_radius], [assembly.rear_wheel, assembly.config.rear_wheel_radius]]:
		var u: Vector3 = pair[0].orientation * Vector3.RIGHT
		pair[0].angular_velocity = -u * (speed / pair[1])


static func wheel_of(assembly, side: String):
	return assembly.front_wheel if side == "front" else assembly.rear_wheel


static func patch_of(assembly, side: String):
	return assembly.front_tire_patch if side == "front" else assembly.rear_tire_patch


## Steps once and returns the largest distance between the patch's reported active-sample positions and the samples the Stage A sampler gives for
## the wheel pose AT THE START of that step (the pose the patch must have used). Returns INF when the patch reports no sample.
static func start_of_step_pose_error(assembly, side: String) -> float:
	var wheel = wheel_of(assembly, side)
	var patch = patch_of(assembly, side)
	var position: Vector3 = wheel.position
	var orientation: Quaternion = wheel.orientation
	assembly.world.step()
	var data: Dictionary = TirePatchSampler.sample(position, orientation, patch.geometry, patch.wheel_radius, patch.plane_point, patch.plane_normal,
			patch.lateral_count, patch.circumferential_count)
	var by_index: Dictionary = {}
	for sample in data.samples:
		by_index[sample.lateral_index * patch.circumferential_count + sample.angular_index] = sample
	var report: Array = patch.sample_report()
	if report.is_empty():
		return INF
	var worst: float = 0.0
	for entry in report:
		var sample: Dictionary = by_index[entry.index]
		worst = maxf(worst, (entry.position - sample.position).length())
		worst = maxf(worst, absf(entry.gap - sample.gap))
	return worst
