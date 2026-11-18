extends RefCounted
## Immutable-by-convention snapshot of a body's kinematic state.
## The world publishes one per body at the end of each step; presentation code reads
## `previous_state` / `published_state` and never writes physical state.

var position: Vector3 = Vector3.ZERO
var orientation: Quaternion = Quaternion.IDENTITY
var linear_velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO


func capture_from(body) -> void:
	position = body.position
	orientation = body.orientation
	linear_velocity = body.linear_velocity
	angular_velocity = body.angular_velocity


func copy_from(other) -> void:
	position = other.position
	orientation = other.orientation
	linear_velocity = other.linear_velocity
	angular_velocity = other.angular_velocity


func get_transform() -> Transform3D:
	return Transform3D(Basis(orientation), position)


## Interpolated COM transform between two states (lerp position, slerp orientation).
static func interpolate_transform(a, b, alpha: float) -> Transform3D:
	var t: float = clampf(alpha, 0.0, 1.0)
	var q: Quaternion = a.orientation.slerp(b.orientation, t)
	return Transform3D(Basis(q), a.position.lerp(b.position, t))
