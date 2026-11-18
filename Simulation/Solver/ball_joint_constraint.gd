extends "res://Simulation/Solver/sim_constraint.gd"
## Ball (point-to-point) joint: keeps local_anchor_a on body A coincident with local_anchor_b on body B.
## Three bilateral rows along world X, Y, Z.
##   C    = (x_b + r_b) - (x_a + r_a),  r = R * local_anchor
##   Cdot = v_b + w_b x r_b - v_a - w_a x r_a
##   row along e: Jv_a = -e, Jw_a = -(r_a x e), Jv_b = +e, Jw_b = r_b x e
##   bias = (beta/dt) * C . e          (Baumgarte, removes a fraction beta of the drift per step)
## Either body may be static (zero inverse mass/inertia).

var local_anchor_a: Vector3 = Vector3.ZERO
var local_anchor_b: Vector3 = Vector3.ZERO

var _r_a: Vector3 = Vector3.ZERO
var _r_b: Vector3 = Vector3.ZERO


func _init(p_body_a: SimRigidBody, p_body_b: SimRigidBody, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3) -> void:
	super(p_body_a, p_body_b)
	local_anchor_a = p_local_anchor_a
	local_anchor_b = p_local_anchor_b
	for i in 3:
		add_row()


func world_anchor_a() -> Vector3:
	return body_a.position + body_a.orientation * local_anchor_a


func world_anchor_b() -> Vector3:
	return body_b.position + body_b.orientation * local_anchor_b


## C = anchor_b - anchor_a in world space; zero when the joint is satisfied.
func linear_error() -> Vector3:
	return world_anchor_b() - world_anchor_a()


## Relative velocity of the two anchor points (B minus A), world space.
func relative_anchor_velocity() -> Vector3:
	var r_a: Vector3 = body_a.orientation * local_anchor_a
	var r_b: Vector3 = body_b.orientation * local_anchor_b
	return (body_b.linear_velocity + body_b.angular_velocity.cross(r_b)) \
		- (body_a.linear_velocity + body_a.angular_velocity.cross(r_a))


## Impulse this joint applied to body B during the last solve, as a world vector.
func accumulated_impulse_on_b() -> Vector3:
	return Vector3(rows[0].accumulated_impulse, rows[1].accumulated_impulse, rows[2].accumulated_impulse)


func _update_rows(dt: float, config) -> void:
	_r_a = body_a.orientation * local_anchor_a
	_r_b = body_b.orientation * local_anchor_b
	var c: Vector3 = (body_b.position + _r_b) - (body_a.position + _r_a)
	var bias_scale: float = config.baumgarte_factor / dt
	var axes: Array = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
	for i in 3:
		var e: Vector3 = axes[i]
		var row = rows[i]
		row.linear_a = -e
		row.angular_a = -_r_a.cross(e)
		row.linear_b = e
		row.angular_b = _r_b.cross(e)
		row.bias = bias_scale * c.dot(e)
		row.softness = 0.0
