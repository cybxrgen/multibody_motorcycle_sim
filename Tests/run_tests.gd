extends SceneTree
## Headless test runner.
##
##   godot --headless --path <project_dir> --script res://Tests/run_tests.gd
##   godot --headless --path <project_dir> --script res://Tests/run_tests.gd -- --report=/tmp/report.txt
##
## Discovers res://Tests/test_*.gd, runs every method named test_*, prints a
## report and exits with 0 (all passed), 1 (failures) or 3 (no tests found).
## A GDScript runtime error inside a test aborts the whole run; that is reported
## by the engine on stderr and must be treated as a failure.


func _initialize() -> void:
	var code: int = _run()
	quit(code)


func _run() -> int:
	var lines: Array[String] = []
	var passed: int = 0
	var failed: int = 0
	var skipped: int = 0
	var total_checks: int = 0

	lines.append("=== Motorcycle Multibody Simulator test report ===")
	lines.append("Engine: %s" % Engine.get_version_info().get("string", "unknown"))

	var files: Array[String] = _discover_test_files()
	if files.is_empty():
		lines.append("No test files found in res://Tests.")
		_emit(lines)
		return 3

	for path in files:
		var script: GDScript = load(path)
		if script == null:
			failed += 1
			lines.append("[FAIL] %s: could not load script" % path)
			continue
		var instance = script.new()
		var names: Array[String] = []
		for m in instance.get_method_list():
			var n: String = m["name"]
			if n.begins_with("test_"):
				names.append(n)
		names.sort()
		for test_name in names:
			instance.reset_for_test()
			instance.call(test_name)
			var label: String = "%s::%s" % [path.get_file(), test_name]
			total_checks += instance.checks_run
			if instance.skip_reason != "":
				skipped += 1
				lines.append("[SKIP] %s (%s)" % [label, instance.skip_reason])
			elif instance.failures.is_empty():
				passed += 1
				lines.append("[PASS] %s (%d checks)" % [label, instance.checks_run])
			else:
				failed += 1
				lines.append("[FAIL] %s" % label)
				for f in instance.failures:
					lines.append("         - %s" % f)

	lines.append("--------------------------------------------------")
	lines.append("Tests passed: %d  failed: %d  skipped: %d  (total %d, checks %d)" % [passed, failed, skipped, passed + failed + skipped, total_checks])
	_emit(lines)
	return 1 if failed > 0 else 0


func _discover_test_files() -> Array[String]:
	var result: Array[String] = []
	var dir: DirAccess = DirAccess.open("res://Tests")
	if dir == null:
		return result
	dir.list_dir_begin()
	var entry: String = dir.get_next()
	while entry != "":
		if not dir.current_is_dir() and entry.begins_with("test_") and entry.ends_with(".gd"):
			result.append("res://Tests/" + entry)
		entry = dir.get_next()
	dir.list_dir_end()
	result.sort()
	return result


func _emit(lines: Array[String]) -> void:
	var text: String = "\n".join(lines)
	print(text)
	var report_path: String = "user://test_report.txt"
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--report="):
			report_path = arg.substr("--report=".length())
	var f: FileAccess = FileAccess.open(report_path, FileAccess.WRITE)
	if f != null:
		f.store_string(text + "\n")
		f.close()
