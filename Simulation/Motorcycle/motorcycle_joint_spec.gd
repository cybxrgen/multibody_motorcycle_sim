extends RefCounted
## Specification of ONE hinge of the motorcycle assembly, in WORLD coordinates at the reference pose.
## The builder converts it into each body's local frame (anchor, axis, reference direction), so the two local frames can
## never disagree and no per-body offsets are written down twice.

var joint_name: String = ""
## Hinge pivot (m), world coordinates at the reference pose.
var point: Vector3 = Vector3.ZERO
## Hinge axis, unit vector in world coordinates at the reference pose. The sign defines the positive angle direction
## (right-handed about the axis, `HingeJointConstraint.angle()`).
var axis: Vector3 = Vector3.RIGHT


func _init(p_name: String = "", p_point: Vector3 = Vector3.ZERO, p_axis: Vector3 = Vector3.RIGHT) -> void:
	joint_name = p_name
	point = p_point
	axis = p_axis
