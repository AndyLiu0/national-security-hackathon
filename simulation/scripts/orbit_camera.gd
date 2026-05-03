extends Camera3D

# Camera modes cycled with [C]:
# 0 = orbit center, 1 = chase ARGUS, 2 = chase HCM.

@export var controller_path: NodePath
@export var argus_path: NodePath
@export var hcm_path: NodePath
@export var orbit_radius: float = 380.0
@export var orbit_height: float = 180.0
@export var orbit_speed: float = 0.04

var _t: float = 0.0
var _mode: int = 0
var _c_prev: bool = false
var controller: SimController
var argus: Node3D
var hcm: Node3D

func _ready() -> void:
	controller = get_node_or_null(controller_path) as SimController
	argus = get_node_or_null(argus_path) as Node3D
	hcm = get_node_or_null(hcm_path) as Node3D
	make_current()

func _process(delta: float) -> void:
	# Manual edge-detect on KEY_C — avoids action-map matching quirks.
	var c_now: bool = Input.is_key_pressed(KEY_C)
	if c_now and not _c_prev:
		_mode = (_mode + 1) % 3
		if controller:
			controller._camera_mode = _mode
	_c_prev = c_now

	_t += delta * orbit_speed
	match _mode:
		1:
			if argus:
				global_transform.origin = argus.position + Vector3(40, 25, 40)
				look_at(argus.position, Vector3.UP)
		2:
			if hcm:
				global_transform.origin = hcm.position + Vector3(60, 35, 60)
				look_at(hcm.position, Vector3.UP)
		_:
			global_transform.origin = Vector3(cos(_t) * orbit_radius, orbit_height, sin(_t) * orbit_radius)
			look_at(Vector3(0, 90, 0), Vector3.UP)
