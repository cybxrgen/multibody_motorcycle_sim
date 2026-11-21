extends RefCounted
## Sequential impulse / projected Gauss-Seidel solver. Operates on generic constraints only.
##  1. prepare every active constraint (Jacobians, physical and split biases, effective mass, physical warm start)
##  2. PHYSICAL pass: iterate `config.solver_iterations` times over all active constraints (`SimConstraint.solve_iteration()`;
##     the default solves each row in order, coupled constraints may override). Changes the physical velocities.
##  3. SPLIT pass (positional correction, M1.3): zero the split velocities of the bodies involved, then iterate
##     `config.solver_iterations` times over all active constraints (`SimConstraint.solve_split_iteration()`). Changes only the
##     bodies' split velocities, never the physical ones, and nothing is warm started.
##  4. finish
## The two passes are independent (the split pass reads only split velocities and its own accumulated impulses; the physical
## pass never reads them), so the order only fixes determinism. Contact friction therefore keeps using the PHYSICAL normal impulse.
## Bodies are modified only through their velocities (physical or split); positions are integrated by SimWorld.

var last_iterations: int = 0
var last_split_iterations: int = 0


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
		c.body_a.clear_split_velocity()
		c.body_b.clear_split_velocity()
	last_split_iterations = 0
	for iteration in config.solver_iterations:
		for c in active:
			c.solve_split_iteration()
		last_split_iterations += 1
	for c in active:
		c.finish()
