extends RefCounted
## Sequential impulse / projected Gauss-Seidel solver. Operates on generic constraints only.
##  1. prepare every active constraint (Jacobians, bias, effective mass, warm start)
##  2. iterate `config.solver_iterations` times over all active constraints (`SimConstraint.solve_iteration()`;
##     the default solves each row in order, coupled constraints may override)
##  3. finish
## Bodies are modified only through their velocities.

var last_iterations: int = 0


func solve(constraints: Array, dt: float, config) -> void:
	var active: Array = []
	for c in constraints:
		if c.enabled:
			c.prepare(dt, config)
			active.append(c)
	last_iterations = 0
	for iteration in config.solver_iterations:
		for c in active:
			c.solve_iteration()
		last_iterations += 1
	for c in active:
		c.finish()
