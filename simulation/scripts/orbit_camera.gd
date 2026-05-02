extends Camera3D

# Lazy orbit camera. Modes are cycled via the SimController.
# 0 = orbit center, 1 = chase ARGUS, 2 = chase HCM.

@export var controller_path: NodePath
@export var argus_path: NodePath
@export var hcm_path: NodePath
@export var orbit_radius: float = 380.0
@export var orbit_height: float = 180.0
@export var orbit_speed: float = 0.04

var _t: float = 0.0
var controller: SimController
var argus: Node3D
var hcm: Node3D

func _ready() -> void:
	controller = get_node(controller_path) as SimController
	argus = get_node(argus_path) as Node3D
	hcm = get_node(hcm_path) as Node3D

func _process(delta: float) -> void:
	_t += delta * orbit_speed
	var mode := controller.camera_mode() if controller else 0
	match mode:
		1:
			var p: Vector3 = argus.position + Vector3(40, 25, 40)
			global_transform.origin = p
			look_at(argus.position, Vector3.UP)
		2:
			var p2: Vector3 = hcm.position + Vector3(60, 35, 60)
			global_transform.origin = p2
			look_at(hcm.position, Vector3.UP)
		_:
			var x := cos(_t) * orbit_radius
			var z := sin(_t) * orbit_radius
			global_transform.origin = Vector3(x, orbit_height, z)
			look_at(Vector3(0, 90, 0), Vector3.UP)
