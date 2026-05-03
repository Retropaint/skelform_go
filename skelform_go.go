package skelform_go

import (
	"archive/zip"
	"encoding/json"
	"errors"
	"image"
	"image/png"
	"io"
	"math"
	"sort"
	"strconv"
	"strings"
	"time"
)

type Vec2 struct {
	X float32
	Y float32
}

func (v1 Vec2) Add(v2 Vec2) Vec2 {
	return Vec2{
		X: v1.X + v2.X,
		Y: v1.Y + v2.Y,
	}
}

func (v1 Vec2) Sub(v2 Vec2) Vec2 {
	return Vec2{
		X: v1.X - v2.X,
		Y: v1.Y - v2.Y,
	}
}

func (v1 Vec2) Mul(v2 Vec2) Vec2 {
	return Vec2{
		X: v1.X * v2.X,
		Y: v1.Y * v2.Y,
	}
}

func (v1 Vec2) Mulf(f float32) Vec2 {
	return Vec2{
		X: v1.X * f,
		Y: v1.Y * f,
	}
}

func magnitude(v1 Vec2) float32 {
	return float32(math.Sqrt(float64(v1.X*v1.X + v1.Y*v1.Y)))
}

func normalize(v1 Vec2) Vec2 {
	mag := magnitude(v1)
	if mag == 0 {
		return Vec2{X: 0, Y: 0}
	}
	return Vec2{X: v1.X / mag, Y: v1.Y / mag}
}

func rotate(point Vec2, rot float64) Vec2 {
	cos := float32(math.Cos(rot))
	sin := float32(math.Sin(rot))
	return Vec2{
		X: point.X*cos - point.Y*sin,
		Y: point.X*sin + point.Y*cos,
	}
}

type Keyframe struct {
	Frame        int
	Bone_id      int
	Element      string
	Value        float32
	Value_str    string
	Start_handle float32
	End_handle   float32
}

type Animation struct {
	Name      string
	Fps       int
	Keyframes []Keyframe
}

type Vertex struct {
	Pos      Vec2
	Init_pos Vec2
	Uv       Vec2
	Id       int
}

type BoneBindVert struct {
	Id     int
	Weight float32
}

type Bind struct {
	Bone_id int
	Is_path bool
	Verts   []BoneBindVert
}

type Bone struct {
	Id        int
	Name      string
	Parent_id int
	Tex       string
	Style_ids []int

	Ik_family_id  int
	Ik_constraint string
	Ik_mode       string
	Ik_target_id  int
	Ik_bone_ids   []int

	Has_physics            bool
	Phys_global_pos        Vec2
	Phys_pos_damping       float32
	Phys_pos_ratio         float32
	Phys_global_rot        float32
	Phys_global_orbit      float32
	Phys_global_orbit_diff float32
	Phys_global_orbit_vel  float32
	Phys_rot_damping       float32
	Phys_sway              float32
	Phys_rot_bounce        float32
	Phys_global_scale      Vec2
	Phys_scale_damping     float32
	Phys_scale_ratio       float32

	Rot    float32
	Scale  Vec2
	Pos    Vec2
	Pivot  Vec2
	Zindex float32

	Vertices []Vertex
	Indices  []int
	Binds    []Bind

	Init_rot           float32
	Init_scale         Vec2
	Init_pos           Vec2
	Init_ik_constraint string
}

type Texture struct {
	Name     string
	Size     Vec2
	Offset   Vec2
	AtlasIdx int
}

type Style struct {
	Id       int
	Name     string
	Textures []Texture
}

type Armature struct {
	Version           string
	Ik_root_ids       []int
	Texture_size      Vec2
	Bones             []Bone
	Constructed_bones []Bone
	Animations        []Animation
	Styles            []Style
}

func Load(path string) (Armature, []image.Image) {
	zip, _ := zip.OpenReader(path)

	defer zip.Close()

	var armature Armature
	var textures []image.Image

	for _, f := range zip.File {
		file, _ := f.Open()
		if f.Name == "armature.json" {
			bytes, _ := io.ReadAll(file)
			json.Unmarshal(bytes, &armature)
		} else if strings.Contains(f.Name, "atlas") {
			tex, _ := png.Decode(file)
			textures = append(textures, tex)
		}
	}

	return armature, textures
}

func Animate(armature *Armature, animations []Animation, frames []int, blendFrames []int) {
	for i := range animations {
		kf := animations[i].Keyframes
		bf := blendFrames[i]
		frame := frames[i]
		ikf := interpolateKeyframes
		for b := range armature.Bones {
			bone := &armature.Bones[b]
			ikf(&bone.Pos.X, "PositionX", kf, frame, bone.Id, bf)
			ikf(&bone.Pos.Y, "PositionY", kf, frame, bone.Id, bf)
			ikf(&bone.Rot, "Rotation", kf, frame, bone.Id, bf)
			ikf(&bone.Scale.X, "ScaleX", kf, frame, bone.Id, bf)
			ikf(&bone.Scale.Y, "ScaleY", kf, frame, bone.Id, bf)
			prev := getPrevKeyframe(kf, frame, "IkConstraint", bone.Id)
			if prev != -1 {
				bone.Ik_constraint = kf[prev].Value_str
			}
		}
	}

	for b := range armature.Bones {
		ResetBone(&armature.Bones[b], animations, frames[0], blendFrames[0])
	}
}

// Reset bones back to default states, if they haven't been animated.
// Must be called after `Animate()` with the same animations provided.
// `frame` must be first anim frame.
func ResetBone(bone *Bone, anims []Animation, frame int, blendFrames int) {
	resetBoneElement(&bone.Pos.X, bone.Init_pos.X, "PositionX", bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Pos.Y, bone.Init_pos.Y, "PositionY", bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Rot, bone.Init_rot, "Rotation", bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Scale.X, bone.Init_scale.X, "ScaleX", bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Scale.Y, bone.Init_scale.Y, "ScaleY", bone.Id, frame, blendFrames, anims)
	if shouldResetElement(anims, bone.Id, "IkConstraint") {
		bone.Ik_constraint = bone.Init_ik_constraint
	}
}

func shouldResetElement(anims []Animation, boneId int, el string) bool {
	for a := range anims {
		anim := &anims[a]
		for _, kf := range anim.Keyframes {
			if kf.Bone_id == boneId && kf.Element == el {
				return false
			}
		}
	}
	return true
}

func resetBoneElement(value *float32, init float32, el string, boneId int, frame int, blendFrames int, anims []Animation) {
	z := Vec2{0, 0}
	shouldReset := shouldResetElement(anims, boneId, el)
	if shouldReset {
		*value = interpolate(uint(frame), uint(blendFrames), *value, init, z, z)
	}
}

func resetInheritance(bones []Bone, constructedBones []Bone) {
	for b := range constructedBones {
		constBone := &constructedBones[b]
		bone := &bones[b]

		constBone.Pos = bone.Pos
		constBone.Scale = bone.Scale
		constBone.Rot = bone.Rot
	}
}

func inheritance(bones []Bone, ikRots map[uint]float32, armature_bones []Bone) []Bone {
	for b := range bones {
		bone := &bones[b]

		if bone.Parent_id != -1 {
			parent, _ := findBone(bones, bone.Parent_id)

			orbit_rot := bones[bones[b].Parent_id].Rot
			// apply orbital difference, if rotation resistance physics is active
			if len(armature_bones) > 0 && armature_bones[b].Phys_sway > 0. {
				orbit_rot -= armature_bones[b].Phys_global_orbit_diff
			}
			bone.Rot += orbit_rot

			bone.Scale = bone.Scale.Mul(parent.Scale)
			bone.Pos = bone.Pos.Mul(parent.Scale)

			bone.Pos = rotate(bone.Pos, float64(orbit_rot))

			bone.Pos = bone.Pos.Add(parent.Pos)
		}

		if rot, ok := ikRots[uint(b)]; ok {
			bones[b].Rot = rot
		}

		// apply physics, if armature_bones is provided
		if len(armature_bones) > 0 {
			if armature_bones[b].Phys_rot_damping > 0. {
				bones[b].Rot = armature_bones[b].Phys_global_rot
			}
			if armature_bones[b].Phys_pos_damping > 0. {
				bones[b].Pos = armature_bones[b].Phys_global_pos
			}
			if armature_bones[b].Phys_scale_damping > 0. {
				bones[b].Scale = armature_bones[b].Phys_global_scale
			}
		}
	}

	return bones
}

func simulatePhysics(armature *Armature) {
	for b := range armature.Bones {
		s := Vec2{X: 0.3, Y: 0.3}
		e := Vec2{X: 0.6, Y: 0.6}
		arm_bone := &armature.Bones[b]
		const_bone := &armature.Constructed_bones[b]
		prev_pos := arm_bone.Phys_global_pos

		// interpolate position
		if arm_bone.Phys_pos_damping > 0. || arm_bone.Phys_sway > 0. {
			phys_pos := &arm_bone.Phys_global_pos
			dampingX := arm_bone.Phys_pos_damping
			dampingY := arm_bone.Phys_pos_damping

			// ratio
			if arm_bone.Phys_pos_ratio < 0. {
				dampingY *= 1. - float32(math.Abs(float64(arm_bone.Phys_pos_ratio)))
			} else if arm_bone.Phys_pos_ratio > 0. {
				dampingX *= 1. - arm_bone.Phys_pos_ratio
			}

			phys_pos.X = interpolate(2, uint(dampingX), phys_pos.X, const_bone.Pos.X, s, e)
			phys_pos.Y = interpolate(2, uint(dampingY), phys_pos.Y, const_bone.Pos.Y, s, e)
		}

		// interpolate scale
		if arm_bone.Phys_scale_damping > 0. {
			phys_scale := &arm_bone.Phys_global_scale
			dampingX := arm_bone.Phys_scale_damping
			dampingY := arm_bone.Phys_scale_damping

			// ratio
			if arm_bone.Phys_scale_ratio < 0. {
				dampingY *= 1. - float32(math.Abs(float64(arm_bone.Phys_scale_ratio)))
			} else if arm_bone.Phys_scale_ratio > 0. {
				dampingX *= 1. - arm_bone.Phys_scale_ratio
			}

			phys_scale.X = interpolate(2, uint(dampingX), phys_scale.X, const_bone.Scale.X, s, e)
			phys_scale.Y = interpolate(2, uint(dampingY), phys_scale.Y, const_bone.Scale.Y, s, e)
		}

		// interpolate rotation
		if arm_bone.Phys_rot_damping > 0. {
			rot := shortest_angle_delta(arm_bone.Phys_global_rot, const_bone.Rot)
			arm_bone.Phys_global_rot += rot / arm_bone.Phys_rot_damping
		}

		// interpolate parent orbit (rot res, bounce, etc)
		var parent Bone
		has_parent := false
		for _, bone := range armature.Constructed_bones {
			if bone.Id == const_bone.Parent_id {
				parent = bone
				has_parent = true
				break
			}
		}
		if arm_bone.Phys_sway > 0. && has_parent {
			// interpolate to the angle difference between bone and parent
			diff := normalize(const_bone.Pos.Sub(parent.Pos))
			diff_angle := math.Atan2(float64(diff.Y), float64(diff.X))
			rest_rot := shortest_angle_delta(arm_bone.Phys_global_orbit, float32(diff_angle))
			// apply bounce
			if arm_bone.Phys_rot_bounce > 0. && arm_bone.Phys_rot_bounce <= 1. {
				rest_rot += arm_bone.Phys_global_orbit_vel / (2. - arm_bone.Phys_rot_bounce)
				arm_bone.Phys_global_orbit_vel = rest_rot
			}
			arm_bone.Phys_global_orbit += rest_rot / 10.

			// swing orbit based on position momentum
			vel := normalize(arm_bone.Phys_global_pos.Sub(prev_pos))
			angle := math.Atan2(float64(-vel.Y), float64(-vel.X))
			vel_rot := shortest_angle_delta(arm_bone.Phys_global_orbit, float32(angle))
			strength := magnitude(arm_bone.Phys_global_pos.Sub(prev_pos)) / 1000.
			arm_bone.Phys_global_orbit += vel_rot * strength * arm_bone.Phys_sway

			// apply difference in final angle and orbit
			arm_bone.Phys_global_orbit_diff = float32(diff_angle) - arm_bone.Phys_global_orbit
		}
	}
}

func shortest_angle_delta(fro float32, to float32) float32 {
	pi := 3.141592653589793
	tau := pi * 2.0
	delta := float64(to - fro)
	for delta > pi {
		delta -= tau
	}
	for delta < -pi {
		delta += tau
	}
	return float32(delta)
}

func interpolate(
	current uint,
	max uint,
	start_val,
	end_val float32,
	start_handle Vec2,
	end_handle Vec2,
) float32 {
	// snapping behavior for None transition preset
	if start_handle.Y == 999 && end_handle.Y == 999 {
		return start_val
	}
	if max == 0 || current >= max {
		return end_val
	}

	// solve for time (x axis) with Newton-Raphson
	initial := float32(current) / float32(max)
	t := initial
	for range 5 {
		x := cubic_bezier(t, start_handle.X, end_handle.X)
		dx := cubic_bezier_derivative(t, start_handle.X, end_handle.X)
		if math.Abs(float64(dx)) < 1e-5 {
			break
		}
		t -= (x - initial) / dx
		if t > 1 {
			t = 1
		} else if t < 0 {
			t = 0
		}
	}

	progress := cubic_bezier(t, start_handle.Y, end_handle.Y)
	return start_val + (end_val-start_val)*progress
}

func cubic_bezier(t float32, p1 float32, p2 float32) float32 {
	u := 1.0 - t
	return 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t
}

func cubic_bezier_derivative(t float32, p1 float32, p2 float32) float32 {
	u := 1.0 - t
	return 3.0*u*u*p1 + 6.0*u*t*(p2-p1) + 3.0*t*t*(1.0-p2)
}

func Construct(armature *Armature) {
	if len(armature.Constructed_bones) == 0 {
		for _, bone := range armature.Bones {
			armature.Constructed_bones = append(armature.Constructed_bones, bone)
		}
	} else {
		sort.Slice(armature.Constructed_bones, func(i, j int) bool {
			return armature.Constructed_bones[i].Id < armature.Constructed_bones[j].Id
		})
	}

	resetInheritance(armature.Bones, armature.Constructed_bones)
	inheritance(armature.Constructed_bones, make(map[uint]float32), []Bone{})

	ikRots := InverseKinematics(armature.Constructed_bones, armature.Ik_root_ids)

	resetInheritance(armature.Bones, armature.Constructed_bones)
	inheritance(armature.Constructed_bones, ikRots, []Bone{})

	simulatePhysics(armature)

	resetInheritance(armature.Bones, armature.Constructed_bones)
	inheritance(armature.Constructed_bones, ikRots, armature.Bones)

	ConstructVerts(armature.Constructed_bones)
}

func ConstructVerts(bones []Bone) {
	for b := range bones {
		bone := &bones[b]

		for v := range bone.Vertices {
			vert := &bone.Vertices[v]

			// reset vertex position
			vert.Pos = vert.Init_pos

			// inherit bone's properties into vertex
			vert.Pos = inheritVert(vert.Pos, *bone)
		}

		for bi, bind := range bone.Binds {
			if bind.Bone_id == -1 {
				continue
			}

			bindBone, _ := findBone(bones, bind.Bone_id)

			for _, bindVert := range bind.Verts {
				// non-pathing bind
				if !bind.Is_path {
					vert := &bone.Vertices[bindVert.Id]
					endPos := inheritVert(vert.Init_pos, bindBone).Sub(vert.Pos)
					vert.Pos = vert.Pos.Add(endPos.Mulf(bindVert.Weight))
					continue
				}

				// pathing bind

				// get previous and next bind bone (if they exist)
				prev := int(math.Max(float64(bi-1), 0))
				next := int(math.Min(float64(bi+1), float64(len(bone.Binds)-1)))
				prevBindBone, _ := findBone(bones, bone.Binds[prev].Bone_id)
				nextBindBone, _ := findBone(bones, bone.Binds[next].Bone_id)

				// get normal line along the 3 binds
				prevDir := bindBone.Pos.Sub(prevBindBone.Pos)
				nextDir := nextBindBone.Pos.Sub(bindBone.Pos)
				prevNormal := normalize(Vec2{-prevDir.Y, prevDir.X})
				nextNormal := normalize(Vec2{-nextDir.Y, nextDir.X})
				average := prevNormal.Add(nextNormal)
				normalAngle := math.Atan2(float64(average.Y), float64(average.X))

				// move vertex along the surface
				vert := &bone.Vertices[bindVert.Id]
				vert.Pos = vert.Init_pos.Add(bindBone.Pos)
				rotated := rotate(vert.Pos.Sub(bindBone.Pos), normalAngle)
				vert.Pos = bindBone.Pos.Add(rotated.Mulf(bindVert.Weight))
			}
		}
	}
}

func inheritVert(pos Vec2, bone Bone) Vec2 {
	pos = pos.Mul(bone.Scale)
	pos = rotate(pos, float64(bone.Rot))
	pos = pos.Add(bone.Pos)
	return pos
}

func SetupBoneTextures(bones []Bone, styles []Style) map[uint]Texture {
	finalTextures := make(map[uint]Texture)

	for _, bone := range bones {
		for _, style := range styles {
			found := false
			// find texture
			for _, tex := range style.Textures {
				if tex.Name == bone.Tex {
					found = true
					finalTextures[uint(bone.Id)] = tex
					found = true
					break
				}
			}
			if found {
				break
			}
		}
	}

	return finalTextures
}

func InverseKinematics(bones []Bone, ik_root_ids []int) map[uint]float32 {
	rotMap := make(map[uint]float32)

	for _, root_id := range ik_root_ids {
		family := bones[root_id]

		if family.Ik_target_id == -1 {
			continue
		}

		root := bones[family.Ik_bone_ids[0]].Pos
		target := bones[family.Ik_target_id].Pos

		switch family.Ik_mode {
		case "FABRIK":
			for range 10 {
				fabrik(family.Ik_bone_ids, bones, target, root)
			}
		case "Arc":
			arc_ik(family.Ik_bone_ids, bones, root, target)
		}

		tipPos := bones[family.Ik_bone_ids[len(family.Ik_bone_ids)-1]].Pos
		for i := len(family.Ik_bone_ids) - 1; i >= 0; i-- {
			bone := &bones[family.Ik_bone_ids[i]]
			if i == len(family.Ik_bone_ids)-1 {
				continue
			}

			dir := tipPos.Sub(bone.Pos)
			bones[family.Ik_bone_ids[i]].Rot = float32(math.Atan2(float64(dir.Y), float64(dir.X)))
			tipPos = bone.Pos
		}

		jointDir := normalize(bones[family.Ik_bone_ids[1]].Pos.Sub(bones[family.Ik_bone_ids[0]].Pos))
		baseDir := normalize(target.Sub(root))
		dir := jointDir.X*baseDir.Y - baseDir.X*jointDir.Y
		baseAngle := math.Atan2(float64(baseDir.Y), float64(baseDir.X))

		cw := family.Ik_constraint == "Clockwise" && dir > 0
		ccw := family.Ik_constraint == "CounterClockwise" && dir < 0
		if cw || ccw {
			for _, id := range family.Ik_bone_ids {
				bones[id].Rot = -bones[id].Rot + float32(baseAngle*2)
			}
		}

		for i, boneId := range family.Ik_bone_ids {
			if i == len(family.Ik_bone_ids)-1 {
				continue
			}
			rotMap[uint(boneId)] = bones[boneId].Rot
		}
	}

	return rotMap
}

func fabrik(bone_ids []int, bones []Bone, target Vec2, root Vec2) {
	// forward-reaching
	nextPos := target
	var nextLength float32 = 0.
	for i := len(bone_ids) - 1; i >= 0; i-- {
		bone := &bones[bone_ids[i]]

		lengthLine := Vec2{X: 0, Y: 0}
		if i != len(bone_ids)-1 {
			lengthLine = normalize(nextPos.Sub(bone.Pos)).Mulf(nextLength)
		}

		if i != 0 {
			nextBone := &bones[bone_ids[i-1]]
			nextLength = magnitude(bone.Pos.Sub(nextBone.Pos))
		}

		bone.Pos = nextPos.Sub(lengthLine)
		nextPos = bone.Pos
	}

	//backward-reaching
	prevPos := root
	var prevLength float32 = 0.
	for i := 0; i < len(bone_ids); i++ {
		bone := &bones[bone_ids[i]]

		lengthLine := Vec2{X: 0, Y: 0}
		if i != 0 {
			lengthLine = normalize(prevPos.Sub(bone.Pos)).Mulf(prevLength)
		}

		if i != len(bone_ids)-1 {
			nextBone := &bones[bone_ids[i+1]]
			prevLength = magnitude(bone.Pos.Sub(nextBone.Pos))
		}

		bone.Pos = prevPos.Sub(lengthLine)
		prevPos = bone.Pos
	}
}

func arc_ik(bone_ids []int, bones []Bone, root Vec2, target Vec2) {
	dist := []float32{0}
	maxLength := magnitude(bones[bone_ids[len(bone_ids)-1]].Pos.Sub(root))
	currLength := float32(0)
	for f := range bone_ids {
		if f == 0 {
			continue
		}
		length := magnitude(bones[bone_ids[f]].Pos.Sub(bones[bone_ids[f-1]].Pos))
		currLength += length
		dist = append(dist, float32(currLength)/float32(maxLength))
	}

	base := target.Sub(root)
	baseAngle := math.Atan2(float64(base.Y), float64(base.X))
	baseMag := math.Min(float64(magnitude(base)), float64(maxLength))
	peak := maxLength / float32(baseMag)
	valley := float32(baseMag) / maxLength

	for f := range bone_ids {
		if f == 0 {
			continue
		}

		angle := float32(math.Sin(float64(dist[f] * 3.14)))
		bones[bone_ids[f]].Pos = Vec2{
			X: bones[bone_ids[f]].Pos.X * valley,
			Y: root.Y + (1-peak)*angle*float32(baseMag),
		}

		rotated := rotate(bones[bone_ids[f]].Pos.Sub(root), baseAngle)
		bones[bone_ids[f]].Pos = rotated.Add(root)
	}
}

func findBone(bones []Bone, id int) (Bone, error) {
	for _, bone := range bones {
		if bone.Id == id {
			return bone, nil
		}
	}

	return bones[0], errors.New("Could not find bone of ID " + strconv.Itoa(id))
}

func getPrevKeyframe(keyframes []Keyframe, frame int, element string, boneId int) int {
	prevKf := -1
	for k, kf := range keyframes {
		if kf.Frame < frame && kf.Bone_id == boneId && kf.Element == element {
			prevKf = k
		}
	}
	return prevKf
}

func interpolateKeyframes(
	field *float32,
	element string,
	keyframes []Keyframe,
	frame int,
	boneId int,
	blendFrames int,
) {
	prevKf := getPrevKeyframe(keyframes, frame, element, boneId)
	nextKf := -1

	for k, kf := range keyframes {
		if kf.Frame >= frame && kf.Bone_id == boneId && kf.Element == element {
			nextKf = k
			break
		}
	}

	if prevKf == -1 {
		prevKf = nextKf
	} else if nextKf == -1 {
		nextKf = prevKf
	}

	if prevKf == -1 && nextKf == -1 {
		return
	}

	totalFrames := int(keyframes[nextKf].Frame - keyframes[prevKf].Frame)
	currentFrame := frame - int(keyframes[prevKf].Frame)

	z := Vec2{0, 0}
	result := interpolate(uint(currentFrame), uint(totalFrames), keyframes[prevKf].Value, keyframes[nextKf].Value, z, z)
	*field = interpolate(uint(currentFrame), uint(blendFrames), *field, result, z, z)
}

// Apply frame effects based on an animation.
func FormatFrame(animation Animation, frame int, reverse bool, loop bool) int {
	lastKf := len(animation.Keyframes) - 1
	lastFrame := animation.Keyframes[lastKf].Frame

	if loop {
		frame %= animation.Keyframes[lastKf].Frame + 1
	}

	if reverse {
		frame = lastFrame - frame
	}

	return frame
}

// Provide a frame of the animation based on time.
func TimeFrame(animation Animation, time time.Duration, reverse bool, loop bool) int {
	fps := animation.Fps

	frametime := 1. / float32(fps)
	frame := int(float32(time.Milliseconds()) / frametime / 1000)

	frame = FormatFrame(animation, frame, reverse, loop)

	return frame
}

func CheckBoneFlip(bone *Bone, scale Vec2) {
	either := scale.X < 0 || scale.Y < 0
	both := scale.X < 0 && scale.Y < 0
	if either && !both {
		bone.Rot = -bone.Rot
	}
}
