# Godot 4 렌더러 어댑터: viz/godot_bridge.py가 TCP(127.0.0.1)로 보내는 중립 렌더 스트림(한 줄에 JSON 하나)을 그린다.
#   ~/Applications/godot/Godot_v4.7.2-stable_linux.x86_64 --path godot -- --port=4243
#   (sim.runner / sim.rt_link --godot가 중계기와 함께 띄운다)
# 좌표: 스트림은 ROS(REP-103: x 앞, y 왼쪽, z 위). 장면 맨 위 노드 world를 X축 -90° 돌려(ROS (x, y, z) -> Godot (x, z, -y))
# 그 아래에서는 ROS 좌표를 그대로 쓴다 (변환은 여기 한 곳). 두 좌표계 모두 오른손 좌표계라 회전만 하면 된다.
# 물리 계산은 하지 않는다 (받은 포즈로 놓기만).
extends Node3D

const FOOT_NAMES := ["FL", "FR", "RL", "RR"]
const MAX_PATH := 2000
# 카메라 (궤도 카메라: 바라보는 점 둘레를 돈다). 처음 시점은 왼쪽 뒤 위 (몸통에서 약 3.4 m)
const CAM_YAW0 := -45.0              # 도. 0이면 Godot +z 쪽(ROS 오른쪽 -y)에서 본다
const CAM_PITCH0 := 22.7             # 도. 위에서 내려다보는 각
const CAM_DIST0 := 3.37              # m
const CAM_DIST_MIN := 0.5
const CAM_DIST_MAX := 60.0
const ORBIT_DEG_PER_PX := 0.3
const ZOOM_STEP := 1.12              # 휠 한 칸에 거리 ×/÷
const HINT := "drag: rotate   wheel: zoom   right/middle drag: pan   F: follow on/off   R: reset view"
const L_HINT := "   L: points/footholds on/off"

var host := "127.0.0.1"
var port := 4243
var shot_path := ""                  # 시험용: N프레임 뒤 화면을 PNG로 저장하고 끝낸다 (--shot=경로 --shot-frames=N)
var shot_frames := 0
var tcp := StreamPeerTCP.new()
var buf := PackedByteArray()
var connected_once := false
var retry_at := 0.0

var world: Node3D                    # ROS 좌표계 (X축 -90°)
var bodies := {}                     # 바디 이름 -> Node3D (월드 포즈)
var mesh_cache := {}                 # OBJ 경로 -> ArrayMesh
var terrain_mi: MeshInstance3D
var terrain_idx_key := Vector2i(-1, -1)
var terrain_idx := PackedInt32Array()
var feet := []                       # 발 접촉 구
var path_mi: MeshInstance3D
var path_pts := PackedVector3Array()
var cmd_mi: MeshInstance3D
var cam: Camera3D
var cam_target := Vector3.ZERO       # 따라가는 점 (몸통, 부드럽게)
var cam_yaw := CAM_YAW0
var cam_pitch := CAM_PITCH0
var cam_dist := CAM_DIST0
var cam_pan := Vector3.ZERO          # 오른쪽 끌기로 옮긴 만큼 (따라가는 점에서 떨어진 양)
var cam_follow := true               # F: 몸통 따라가기 켜고 끄기
var follow_name := "base"            # 따라갈 바디: base(로봇 몸통), 없으면 장면의 첫 바디
var label: Label
var hint: Label
var clouds := {}                     # 점군 토픽 -> MeshInstance3D (중계기 --points일 때만 온다)
var clouds_visible := true           # L: 점군·디딜 곳 보이기/숨기기
var foothold_root: Node3D            # 디딜 곳 (구 + 옮긴 선분). 처음 받을 때 만든다
var foothold_spheres := []
var foothold_lines: MeshInstance3D
var frames := 0
var stats := {"scene": 0, "poses": 0, "terrain": 0, "status": 0, "points": 0, "markers": 0}


func _ready() -> void:
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--port="):
			port = int(a.substr(7))
		elif a.begins_with("--host="):
			host = a.substr(7)
		elif a.begins_with("--shot="):
			shot_path = a.substr(7)
		elif a.begins_with("--shot-frames="):
			shot_frames = int(a.substr(14))
		elif a.begins_with("--cam="):              # 처음 시점: --cam=yaw,pitch,거리 (도, 도, m)
			var c := a.substr(6).split(",")
			cam_yaw = float(c[0])
			cam_pitch = float(c[1])
			cam_dist = float(c[2])
	world = Node3D.new()
	world.name = "World_ROS"
	world.rotation_degrees = Vector3(-90, 0, 0)
	add_child(world)

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.62, 0.72, 0.82)
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.75, 0.75, 0.78)
	e.ambient_light_energy = 0.6
	env.environment = e
	add_child(env)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -35, 0)
	sun.light_energy = 1.1
	add_child(sun)

	cam = Camera3D.new()
	cam.position = Vector3(-2.5, 1.5, 2.5)
	cam.far = 200.0
	add_child(cam)
	cam.look_at(Vector3.ZERO)

	terrain_mi = MeshInstance3D.new()
	world.add_child(terrain_mi)
	for i in 4:
		var s := MeshInstance3D.new()
		var sm := SphereMesh.new()
		sm.radius = 0.03
		sm.height = 0.06
		s.mesh = sm
		s.material_override = _mat(Color(0.6, 0.6, 0.6, 0.6))
		s.visible = false
		world.add_child(s)
		feet.append(s)
	path_mi = MeshInstance3D.new()
	path_mi.material_override = _mat(Color(1.0, 0.85, 0.2), true)
	world.add_child(path_mi)
	cmd_mi = MeshInstance3D.new()
	cmd_mi.material_override = _mat(Color(0.25, 0.6, 1.0), true)
	world.add_child(cmd_mi)

	var layer := CanvasLayer.new()
	add_child(layer)
	var panel := PanelContainer.new()
	panel.position = Vector2(12, 12)
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0, 0, 0, 0.55)
	sb.set_content_margin_all(8)
	panel.add_theme_stylebox_override("panel", sb)
	layer.add_child(panel)
	label = Label.new()
	label.text = "connecting to %s:%d ..." % [host, port]
	label.add_theme_font_size_override("font_size", 15)
	panel.add_child(label)
	hint = Label.new()
	hint.text = HINT
	hint.add_theme_font_size_override("font_size", 13)
	hint.add_theme_color_override("font_color", Color(1, 1, 1, 0.8))
	hint.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.8))
	hint.anchor_top = 1.0
	hint.anchor_bottom = 1.0
	hint.offset_left = 12
	hint.offset_top = -28
	layer.add_child(hint)
	tcp.connect_to_host(host, port)


func _mat(c: Color, unshaded := false) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = c
	if c.a < 1.0:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	if unshaded:
		m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	return m


func _process(delta: float) -> void:
	frames += 1
	tcp.poll()
	var st := tcp.get_status()
	if st == StreamPeerTCP.STATUS_CONNECTED:
		connected_once = true
		var n := tcp.get_available_bytes()
		if n > 0:
			var r := tcp.get_data(n)
			if r[0] == OK:
				buf.append_array(r[1])
		var i := buf.find(10)
		while i >= 0:
			var line := buf.slice(0, i).get_string_from_utf8()
			buf = buf.slice(i + 1)
			var msg = JSON.parse_string(line)
			if msg is Dictionary:
				_handle(msg)
			i = buf.find(10)
	elif st == StreamPeerTCP.STATUS_NONE or st == StreamPeerTCP.STATUS_ERROR:
		var now := Time.get_ticks_msec() / 1000.0
		if not connected_once and now >= retry_at:        # 중계기가 아직 안 떴으면 다시 시도. 끝난 뒤에는 마지막 장면을 그대로 둔다
			retry_at = now + 0.5
			tcp = StreamPeerTCP.new()
			tcp.connect_to_host(host, port)
	# 카메라: 몸통을 부드럽게 따라가고(F로 끔), 마우스로 돈 각도·거리·옮긴 양을 더한다 (Godot 좌표)
	if cam_follow and bodies.has(follow_name):
		var target: Vector3 = bodies[follow_name].global_position
		cam_target = cam_target.lerp(target, clamp(delta * 4.0, 0.0, 1.0))
	_place_camera()
	if shot_path != "" and frames >= shot_frames:
		get_viewport().get_texture().get_image().save_png(shot_path)
		print("shot saved ", shot_path, " stats ", stats, " cam yaw %.1f pitch %.1f dist %.2f follow %s" % [cam_yaw, cam_pitch, cam_dist, cam_follow])
		get_tree().quit()


func _place_camera() -> void:
	var look := cam_target + cam_pan
	var p := deg_to_rad(cam_pitch)
	var y := deg_to_rad(cam_yaw)
	cam.position = look + cam_dist * Vector3(cos(p) * sin(y), sin(p), cos(p) * cos(y))
	cam.look_at(look)


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			cam_dist = clamp(cam_dist / ZOOM_STEP, CAM_DIST_MIN, CAM_DIST_MAX)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			cam_dist = clamp(cam_dist * ZOOM_STEP, CAM_DIST_MIN, CAM_DIST_MAX)
	elif event is InputEventMouseMotion:
		# 지금 눌린 버튼으로 판단 (창 밖에서 버튼을 떼도 끄는 상태가 남지 않는다)
		var mm := event as InputEventMouseMotion
		var d := mm.relative
		if mm.button_mask & MOUSE_BUTTON_MASK_LEFT:        # 돌리기
			cam_yaw -= d.x * ORBIT_DEG_PER_PX
			cam_pitch = clamp(cam_pitch + d.y * ORBIT_DEG_PER_PX, -5.0, 89.0)
		elif mm.button_mask & (MOUSE_BUTTON_MASK_RIGHT | MOUSE_BUTTON_MASK_MIDDLE):   # 옮기기: 화면 평면에서, 거리에 비례
			var k := cam_dist * 0.0015
			cam_pan += (-cam.global_transform.basis.x * d.x + cam.global_transform.basis.y * d.y) * k
	elif event is InputEventKey and event.pressed and not event.echo:
		var key := (event as InputEventKey).keycode
		if key == KEY_R:
			cam_yaw = CAM_YAW0
			cam_pitch = CAM_PITCH0
			cam_dist = CAM_DIST0
			cam_pan = Vector3.ZERO
			cam_follow = true
		elif key == KEY_F:
			cam_follow = not cam_follow
		elif key == KEY_L:
			clouds_visible = not clouds_visible
			for mi in clouds.values():
				mi.visible = clouds_visible
			if foothold_root != null:
				foothold_root.visible = clouds_visible


func _handle(msg: Dictionary) -> void:
	var t: String = msg.get("type", "")
	if stats.has(t):
		stats[t] += 1
	match t:
		"scene":
			_build_scene(msg)
		"poses":
			var p: Dictionary = msg["p"]
			for name in p:
				if bodies.has(name):
					var v: Array = p[name]
					var b: Node3D = bodies[name]
					b.position = Vector3(v[0], v[1], v[2])
					b.quaternion = Quaternion(v[3], v[4], v[5], v[6]).normalized()
		"terrain":
			_build_terrain(msg)
		"status":
			_update_status(msg)
		"points":
			_build_points(msg)
		"markers":
			_update_footholds(msg)


func _update_footholds(msg: Dictionary) -> void:
	# 디딜 곳 (월드 = ROS 좌표): 구는 다시 쓰고 모자라면 더 만든다. 선분은 매번 새로 그린다
	if foothold_root == null:
		foothold_root = Node3D.new()
		foothold_root.visible = clouds_visible
		world.add_child(foothold_root)
		foothold_lines = MeshInstance3D.new()
		var lm := StandardMaterial3D.new()
		lm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		lm.vertex_color_use_as_albedo = true
		lm.no_depth_test = true                  # 다리에 가려도 보이게
		foothold_lines.material_override = lm
		foothold_root.add_child(foothold_lines)
		if not hint.text.ends_with(L_HINT):
			hint.text = HINT + L_HINT
	var spheres: Array = msg["spheres"]
	while foothold_spheres.size() < spheres.size():
		var s := MeshInstance3D.new()
		var sm := SphereMesh.new()
		sm.radius = 0.5
		sm.height = 1.0
		sm.radial_segments = 12
		sm.rings = 6
		s.mesh = sm
		var m := StandardMaterial3D.new()
		m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		s.material_override = m
		foothold_root.add_child(s)
		foothold_spheres.append(s)
	for i in foothold_spheres.size():
		var s: MeshInstance3D = foothold_spheres[i]
		s.visible = i < spheres.size()
		if s.visible:
			var v: Array = spheres[i]
			s.position = Vector3(v[0], v[1], v[2])
			s.scale = Vector3.ONE * float(v[3])
			(s.material_override as StandardMaterial3D).albedo_color = Color(v[4], v[5], v[6], v[7])
	var lines: Array = msg["lines"]
	var im := ImmediateMesh.new()
	if lines.size() > 0:
		im.surface_begin(Mesh.PRIMITIVE_LINES)
		for v in lines:
			im.surface_set_color(Color(v[6], v[7], v[8], v[9]))
			im.surface_add_vertex(Vector3(v[0], v[1], v[2]))
			im.surface_add_vertex(Vector3(v[3], v[4], v[5]))
		im.surface_end()
	foothold_lines.mesh = im


func _build_points(msg: Dictionary) -> void:
	# 점군 (월드 = ROS 좌표). LiDAR는 작은 점, 알고리즘 지형 지도는 큰 점 (RViz의 Boxes 대신)
	var name: String = msg["name"]
	if not clouds.has(name):
		var mi := MeshInstance3D.new()
		var m := StandardMaterial3D.new()
		m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		m.vertex_color_use_as_albedo = true
		m.use_point_size = true
		m.point_size = 7.0 if msg["kind"] == "map" else 3.0
		mi.material_override = m
		mi.visible = clouds_visible
		world.add_child(mi)
		clouds[name] = mi
		hint.text = HINT + L_HINT
	var xyz := Marshalls.base64_to_raw(msg["xyz"]).to_float32_array()
	var rgb := Marshalls.base64_to_raw(msg["rgb"]).to_float32_array()
	var n: int = msg["n"]
	var mesh: ArrayMesh = ArrayMesh.new()
	if n > 0:
		var verts := PackedVector3Array()
		var cols := PackedColorArray()
		verts.resize(n)
		cols.resize(n)
		for i in n:
			verts[i] = Vector3(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2])
			cols[i] = Color(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2])
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = verts
		arrays[Mesh.ARRAY_COLOR] = cols
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	clouds[name].mesh = mesh


func _build_scene(msg: Dictionary) -> void:
	for b in bodies.values():
		b.queue_free()
	bodies.clear()
	var names: Array = msg["bodies"]
	follow_name = "base" if names.has("base") or names.is_empty() else str(names[0])
	for name in names:
		var n := Node3D.new()
		n.name = name
		world.add_child(n)
		bodies[name] = n
	for v in msg["visuals"]:
		if not bodies.has(v["body"]):
			continue
		var mi := MeshInstance3D.new()
		mi.mesh = _load_obj(v["mesh"])
		mi.position = Vector3(v["pos"][0], v["pos"][1], v["pos"][2])
		mi.quaternion = Quaternion(v["quat"][0], v["quat"][1], v["quat"][2], v["quat"][3]).normalized()
		var c: Array = v["rgba"]
		var m := _mat(Color(c[0], c[1], c[2], c[3]))
		m.roughness = 0.6
		mi.material_override = m
		bodies[v["body"]].add_child(mi)
	for rg in msg.get("soil_regions", []):           # 흙 지역 이름 (지역 색은 지형 꼭짓점 색으로 온다)
		var r: Array = rg["region"]
		var l := Label3D.new()
		l.text = rg["name"] if rg["name"] == rg["soil"] else "%s (%s)" % [rg["name"], rg["soil"]]
		l.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		l.font_size = 48
		l.pixel_size = 0.004
		l.modulate = Color(1.0, 0.85, 0.55)
		l.position = Vector3((r[0] + r[2]) * 0.5, clamp(1.5, r[1], r[3]), 0.2)
		world.add_child(l)


func _load_obj(path: String) -> ArrayMesh:
	# OBJ (v, f만): 원래 감김(오른손, 바깥이 반시계)으로 법선을 구하고, 삼각형 순서는 Godot 앞면(시계 방향)으로 뒤집는다
	if mesh_cache.has(path):
		return mesh_cache[path]
	var verts := PackedVector3Array()
	var idx := PackedInt32Array()
	var text := FileAccess.get_file_as_string(path)
	for line in text.split("\n", false):
		if line.begins_with("v "):
			var s := line.split(" ", false)
			verts.append(Vector3(float(s[1]), float(s[2]), float(s[3])))
		elif line.begins_with("f "):
			var s := line.split(" ", false)
			var f := []
			for k in range(1, s.size()):
				f.append(int(s[k].split("/")[0]) - 1)
			for k in range(1, f.size() - 1):                  # 다각형이면 부채꼴로 나눈다
				idx.append_array([f[0], f[k], f[k + 1]])
	var normals := PackedVector3Array()
	normals.resize(verts.size())
	for k in range(0, idx.size(), 3):
		var a := verts[idx[k]]
		var n := (verts[idx[k + 1]] - a).cross(verts[idx[k + 2]] - a)
		normals[idx[k]] += n
		normals[idx[k + 1]] += n
		normals[idx[k + 2]] += n
	for k in normals.size():
		normals[k] = normals[k].normalized()
	var flipped := PackedInt32Array()
	flipped.resize(idx.size())
	for k in range(0, idx.size(), 3):
		flipped[k] = idx[k]
		flipped[k + 1] = idx[k + 2]
		flipped[k + 2] = idx[k + 1]
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = flipped
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh_cache[path] = mesh
	return mesh


func _build_terrain(msg: Dictionary) -> void:
	# 표시 높이 격자 -> 메시 (ROS 좌표: x = x0 + j*dx, y = y0 + i*dy, z = 높이). 꼭짓점 색은 중계기가 계산
	var nr: int = msg["nrow"]
	var nc: int = msg["ncol"]
	var h := Marshalls.base64_to_raw(msg["h"]).to_float32_array()
	var rgb := Marshalls.base64_to_raw(msg["rgb"]).to_float32_array()
	var x0: float = msg["x0"]
	var y0: float = msg["y0"]
	var dx: float = msg["dx"]
	var dy: float = msg["dy"]
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	verts.resize(nr * nc)
	normals.resize(nr * nc)
	colors.resize(nr * nc)
	for i in nr:
		for j in nc:
			var k := i * nc + j
			verts[k] = Vector3(x0 + j * dx, y0 + i * dy, h[k])
			var hx: float = h[i * nc + mini(j + 1, nc - 1)] - h[i * nc + maxi(j - 1, 0)]
			var hy: float = h[mini(i + 1, nr - 1) * nc + j] - h[maxi(i - 1, 0) * nc + j]
			normals[k] = Vector3(-hx / (2.0 * dx), -hy / (2.0 * dy), 1.0).normalized()
			colors[k] = Color(rgb[3 * k], rgb[3 * k + 1], rgb[3 * k + 2])
	if terrain_idx_key != Vector2i(nr, nc):              # 격자 크기가 같으면 삼각형 목록을 다시 쓴다
		terrain_idx_key = Vector2i(nr, nc)
		terrain_idx = PackedInt32Array()
		terrain_idx.resize((nr - 1) * (nc - 1) * 6)
		var p := 0
		for i in nr - 1:
			for j in nc - 1:
				var a := i * nc + j
				# 위에서 볼 때 반시계 (a, a+1, a+nc+1), (a, a+nc+1, a+nc)를 Godot 앞면(시계)으로 뒤집는다
				terrain_idx[p] = a
				terrain_idx[p + 1] = a + nc + 1
				terrain_idx[p + 2] = a + 1
				terrain_idx[p + 3] = a
				terrain_idx[p + 4] = a + nc
				terrain_idx[p + 5] = a + nc + 1
				p += 6
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_COLOR] = colors
	arrays[Mesh.ARRAY_INDEX] = terrain_idx
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	terrain_mi.mesh = mesh
	if terrain_mi.material_override == null:
		var m := StandardMaterial3D.new()
		m.vertex_color_use_as_albedo = true
		m.roughness = 1.0
		terrain_mi.material_override = m


func _update_status(msg: Dictionary) -> void:
	label.text = "\n".join(msg["lines"])
	label.add_theme_color_override("font_color", Color(1.0, 0.4, 0.35) if msg["alert"] else Color(1, 1, 1))
	var f: Dictionary = msg.get("feet", {})
	for i in 4:
		var name: String = FOOT_NAMES[i]
		if f.has(name):
			var p: Array = f[name]["pos"]
			feet[i].visible = true
			feet[i].position = Vector3(p[0], p[1], p[2])
			feet[i].material_override.albedo_color = Color(0.2, 0.9, 0.3, 0.9) if f[name]["contact"] else Color(0.6, 0.6, 0.6, 0.5)
	if not msg.has("command"):            # 로봇이 아닌 출처: 경로와 명령 화살표는 그리지 않는다
		return
	var b: Array = msg["base_pos"]
	var bp := Vector3(b[0], b[1], b[2] - 0.22)
	if path_pts.is_empty() or path_pts[path_pts.size() - 1].distance_to(bp) > 0.03:
		path_pts.append(bp)
		if path_pts.size() > MAX_PATH:
			path_pts = path_pts.slice(path_pts.size() - MAX_PATH)
		if path_pts.size() >= 2:
			var im := ImmediateMesh.new()
			im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
			for q in path_pts:
				im.surface_add_vertex(q)
			im.surface_end()
			path_mi.mesh = im
	var yaw: float = msg["yaw"]
	var L: float = max(float(msg["command"]["vx"]), 0.02)
	var p0 := Vector3(b[0], b[1], b[2] + 0.15)
	var im2 := ImmediateMesh.new()
	im2.surface_begin(Mesh.PRIMITIVE_LINES)
	im2.surface_add_vertex(p0)
	im2.surface_add_vertex(p0 + L * Vector3(cos(yaw), sin(yaw), 0))
	im2.surface_end()
	cmd_mi.mesh = im2
