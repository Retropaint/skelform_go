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

func RotateVec2(point Vec2, rot float64) Vec2 {
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
	Start_handle Vec2
	End_handle   Vec2
	Next_kf      int
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

type BoneInverseKinematics struct {
	Constraint string
	Mode       string
	Target_id  int
	Bone_ids   []int
}

type Visuals struct {
	Tex         string
	Zindex      float32
	Vertices    []Vertex
	Indices     []int
	Binds       []Bind
	Pivot_pos   Vec2
	Pivot_rot   float32
	Pivot_scale Vec2
}

type Physics struct {
	Global_pos        Vec2
	Pos_damping       float32
	Pos_ratio         float32
	Global_rot        float32
	Global_orbit      float32
	Global_orbit_diff float32
	Global_orbit_vel  float32
	Rot_damping       float32
	Sway              float32
	Rot_bounce        float32
	Global_scale      Vec2
	Scale_damping     float32
	Scale_ratio       float32
}

type Bone struct {
	Id        int
	Name      string
	Parent_id int

	Rot    float32
	Scale  Vec2
	Pos    Vec2
	Hidden bool

	Ik_family_id int
	Visuals_id   int
	Physics_id   int

	Init_rot   float32
	Init_scale Vec2
	Init_pos   Vec2
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
	Version            string
	Texture_size       Vec2
	Bones              []Bone
	Constructed_bones  []Bone
	Animations         []Animation
	Styles             []Style
	Inverse_kinematics []BoneInverseKinematics
	Visuals            []Visuals
	Physics            []Physics
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

func Animate(armature *Armature, animations []Animation, frames []int, smoothFrames []int) {
	elementMap := make(map[int][]string)
	for a := range animations {
		for k := range animations[a].Keyframes {
			kf := animations[a].Keyframes[k]

			// only prev keyframes are considered
			if kf.Frame > frames[a] {
				break
			}

			if kf.Next_kf == -1 {
				kf.Next_kf = k
			}
			nextKf := animations[a].Keyframes[kf.Next_kf]

			// this is a redundant keyframe if the next one is also before this frame
			if nextKf.Frame < frames[a] && kf.Next_kf != k {
				continue
			}

			bone := &armature.Bones[kf.Bone_id]

			if kf.Element == "PositionX" {
				bone.Pos.X = interpolateKeyframes(bone.Pos.X, kf, nextKf, frames[a], smoothFrames[a])
			}
			if kf.Element == "PositionY" {
				bone.Pos.Y = interpolateKeyframes(bone.Pos.Y, kf, nextKf, frames[a], smoothFrames[a])
			}
			if kf.Element == "Rotation" {
				bone.Rot = interpolateKeyframes(bone.Rot, kf, nextKf, frames[a], smoothFrames[a])
			}
			if kf.Element == "ScaleX" {
				bone.Scale.X = interpolateKeyframes(bone.Scale.X, kf, nextKf, frames[a], smoothFrames[a])
			}
			if kf.Element == "ScaleY" {
				bone.Scale.Y = interpolateKeyframes(bone.Scale.Y, kf, nextKf, frames[a], smoothFrames[a])
			}
			if kf.Element == "Hidden" {
				bone.Hidden = kf.Value == 1
			}

			// add this bone and element to the element map.
			// will be used to check if bones should be reset later
			if _, has := elementMap[kf.Bone_id]; !has {
				elementMap[kf.Bone_id] = []string{}
			}
			hasEl := false
			for e := range elementMap[kf.Bone_id] {
				if elementMap[kf.Bone_id][e] == kf.Element {
					hasEl = true
					break
				}
			}
			if !hasEl {
				elementMap[kf.Bone_id] = append(elementMap[kf.Bone_id], kf.Element)
			}
		}
	}

	z := Vec2{0, 0}
	var elements []string
	for b := range armature.Bones {
		bone := &armature.Bones[b]
		elements = []string{}
		if _, has := elementMap[b]; has {
			elements = elementMap[b]
		}
		if !isAnimated("PositionX", elements) {
			bone.Pos.X = interpolate(uint(frames[0]), uint(smoothFrames[0]), bone.Pos.X, bone.Init_pos.X, z, z)
		}
		if !isAnimated("PositionY", elements) {
			bone.Pos.Y = interpolate(uint(frames[0]), uint(smoothFrames[0]), bone.Pos.Y, bone.Init_pos.Y, z, z)
		}
		if !isAnimated("Rotation", elements) {
			bone.Rot = interpolate(uint(frames[0]), uint(smoothFrames[0]), bone.Rot, bone.Init_rot, z, z)
		}
		if !isAnimated("ScaleX", elements) {
			bone.Scale.X = interpolate(uint(frames[0]), uint(smoothFrames[0]), bone.Scale.X, bone.Init_scale.X, z, z)
		}
		if !isAnimated("ScaleY", elements) {
			bone.Scale.Y = interpolate(uint(frames[0]), uint(smoothFrames[0]), bone.Scale.Y, bone.Init_scale.Y, z, z)
		}
	}

}

func isAnimated(target string, elements []string) bool {
	for _, str := range elements {
		if target == str {
			return true
		}
	}
	return false
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

func inheritance(bones []Bone, ikRots map[uint]float32, physics []Physics) []Bone {
	for b := range bones {
		bone := &bones[b]

		if bone.Parent_id != -1 {
			parent, _ := findBone(bones, bone.Parent_id)

			orbit_rot := bones[bones[b].Parent_id].Rot
			// apply orbital difference, if sway is active
			if len(physics) > 0 && bone.Physics_id != -1 && physics[bone.Physics_id].Sway > 0. {
				orbit_rot -= physics[bone.Physics_id].Global_orbit_diff
			}
			bone.Rot += orbit_rot

			bone.Scale = bone.Scale.Mul(parent.Scale)
			bone.Pos = bone.Pos.Mul(parent.Scale)

			bone.Pos = RotateVec2(bone.Pos, float64(orbit_rot))

			bone.Pos = bone.Pos.Add(parent.Pos)
		}

		if rot, ok := ikRots[uint(b)]; ok {
			bones[b].Rot = rot
		}

		// apply physics, if armature_bones is provided
		if len(physics) > 0 && bone.Physics_id != -1 {
			phys := physics[bone.Physics_id]
			if phys.Rot_damping > 0. {
				bones[b].Rot = phys.Global_rot
			}
			if phys.Pos_damping > 0. {
				bones[b].Pos = phys.Global_pos
			}
			if phys.Scale_damping > 0. {
				bones[b].Scale = phys.Global_scale
			}
		}
	}

	return bones
}

func simulatePhysics(constructedBones []Bone, physics []Physics) {
	for b := range constructedBones {
		constBone := &constructedBones[b]
		if constBone.Physics_id == -1 {
			continue
		}
		phys := &physics[constBone.Physics_id]

		s := Vec2{X: 0.3, Y: 0.3}
		e := Vec2{X: 0.6, Y: 0.6}
		prev_pos := phys.Global_pos

		// interpolate position
		if phys.Pos_damping > 0. || phys.Sway > 0. {
			dampingX := phys.Pos_damping
			dampingY := phys.Pos_damping

			// ratio
			if phys.Pos_ratio < 0. {
				dampingY *= 1. - float32(math.Abs(float64(phys.Pos_ratio)))
			} else if phys.Pos_ratio > 0. {
				dampingX *= 1. - phys.Pos_ratio
			}

			phys.Global_pos.X = interpolate(2, uint(dampingX), phys.Global_pos.X, constBone.Pos.X, s, e)
			phys.Global_pos.Y = interpolate(2, uint(dampingY), phys.Global_pos.Y, constBone.Pos.Y, s, e)
		}

		// interpolate scale
		if phys.Scale_damping > 0. {
			dampingX := phys.Scale_damping
			dampingY := phys.Scale_damping

			// ratio
			if phys.Scale_ratio < 0. {
				dampingY *= 1. - float32(math.Abs(float64(phys.Scale_ratio)))
			} else if phys.Scale_ratio > 0. {
				dampingX *= 1. - phys.Scale_ratio
			}

			phys.Global_scale.X = interpolate(2, uint(dampingX), phys.Global_scale.X, constBone.Scale.X, s, e)
			phys.Global_scale.Y = interpolate(2, uint(dampingY), phys.Global_scale.Y, constBone.Scale.Y, s, e)
		}

		// interpolate rotation
		if phys.Rot_damping > 0. {
			rot := shortest_angle_delta(phys.Global_rot, constBone.Rot)
			phys.Global_rot += rot / phys.Rot_damping
		}

		// interpolate parent orbit (rot res, bounce, etc)
		var parent Bone
		has_parent := false
		for _, bone := range constructedBones {
			if bone.Id == constBone.Parent_id {
				parent = bone
				has_parent = true
				break
			}
		}
		if phys.Sway > 0. && has_parent {
			// interpolate to the angle difference between bone and parent
			diff := normalize(constBone.Pos.Sub(parent.Pos))
			diff_angle := math.Atan2(float64(diff.Y), float64(diff.X))
			rest_rot := shortest_angle_delta(phys.Global_orbit, float32(diff_angle))
			// apply bounce
			if phys.Rot_bounce > 0. && phys.Rot_bounce <= 1. {
				rest_rot += phys.Global_orbit_vel / (2. - phys.Rot_bounce)
				phys.Global_orbit_vel = rest_rot
			}
			phys.Global_orbit += rest_rot / 10.

			// swing orbit based on position momentum
			vel := normalize(phys.Global_pos.Sub(prev_pos))
			angle := math.Atan2(float64(-vel.Y), float64(-vel.X))
			vel_rot := shortest_angle_delta(phys.Global_orbit, float32(angle))
			strength := magnitude(phys.Global_pos.Sub(prev_pos)) / 1000.
			phys.Global_orbit += vel_rot * strength * phys.Sway

			// apply difference in final angle and orbit
			phys.Global_orbit_diff = float32(diff_angle) - phys.Global_orbit
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

	// 1st inheritance pass
	resetInheritance(armature.Bones, armature.Constructed_bones)
	inheritance(armature.Constructed_bones, make(map[uint]float32), []Physics{})

	// 2nd inheritance pass with IK
	var ikRots map[uint]float32
	if len(armature.Inverse_kinematics) > 0 {
		ikRots = InverseKinematics(armature.Constructed_bones, armature.Inverse_kinematics)
		resetInheritance(armature.Bones, armature.Constructed_bones)
		inheritance(armature.Constructed_bones, ikRots, []Physics{})
	}

	// 3rd inheritance pass with physics
	if len(armature.Physics) > 0 {
		simulatePhysics(armature.Constructed_bones, armature.Physics)
		resetInheritance(armature.Bones, armature.Constructed_bones)
		inheritance(armature.Constructed_bones, ikRots, armature.Physics)
	}

	ConstructVerts(armature.Constructed_bones, armature.Visuals)
}

func ConstructVerts(bones []Bone, visuals []Visuals) {
	for b := range bones {
		bone := &bones[b]
		if bones[b].Visuals_id == -1 {
			continue
		}
		visual := visuals[bone.Visuals_id]

		for v := range visual.Vertices {
			vert := &visual.Vertices[v]

			// reset vertex position
			vert.Pos = vert.Init_pos

			// inherit bone's properties into vertex
			vert.Pos = inheritVert(vert.Pos, *bone, visual)
		}

		for bi, bind := range visual.Binds {
			if bind.Bone_id == -1 {
				continue
			}

			bindBone, _ := findBone(bones, bind.Bone_id)

			for _, bindVert := range bind.Verts {
				// non-pathing bind
				if !bind.Is_path {
					vert := &visual.Vertices[bindVert.Id]
					endPos := inheritVert(vert.Init_pos, bindBone, visual).Sub(vert.Pos)
					vert.Pos = vert.Pos.Add(endPos.Mulf(bindVert.Weight))
					continue
				}

				// pathing bind

				// get previous and next bind bone (if they exist)
				prev := int(math.Max(float64(bi-1), 0))
				next := int(math.Min(float64(bi+1), float64(len(visual.Binds)-1)))
				prevBindBone, _ := findBone(bones, visual.Binds[prev].Bone_id)
				nextBindBone, _ := findBone(bones, visual.Binds[next].Bone_id)

				// get normal line along the 3 binds
				prevDir := bindBone.Pos.Sub(prevBindBone.Pos)
				nextDir := nextBindBone.Pos.Sub(bindBone.Pos)
				prevNormal := normalize(Vec2{-prevDir.Y, prevDir.X})
				nextNormal := normalize(Vec2{-nextDir.Y, nextDir.X})
				average := prevNormal.Add(nextNormal)
				normalAngle := math.Atan2(float64(average.Y), float64(average.X))

				// move vertex along the surface
				vert := &visual.Vertices[bindVert.Id]
				vert.Pos = vert.Init_pos.Add(bindBone.Pos)
				rotated := RotateVec2(vert.Pos.Sub(bindBone.Pos), normalAngle)
				vert.Pos = bindBone.Pos.Add(rotated.Mulf(bindVert.Weight))
			}
		}
	}
}

func inheritVert(pos Vec2, bone Bone, visuals Visuals) Vec2 {
	pos = pos.Mul(bone.Scale).Mul(visuals.Pivot_scale)
	pos = RotateVec2(pos, float64(bone.Rot+visuals.Pivot_rot))
	pos = pos.Add(bone.Pos)
	return pos
}

func GetBoneTexture(texName string, styles []Style) (Texture, error) {
	for _, style := range styles {
		for _, tex := range style.Textures {
			if tex.Name == texName {
				return tex, nil
			}
		}
	}
	var texture Texture
	return texture, errors.New("test")
}

func InverseKinematics(bones []Bone, inverseKinematics []BoneInverseKinematics) map[uint]float32 {
	rotMap := make(map[uint]float32)

	for _, family := range inverseKinematics {
		if family.Target_id == -1 {
			continue
		}

		root := bones[family.Bone_ids[0]].Pos
		target := bones[family.Target_id].Pos

		switch family.Mode {
		case "FABRIK":
			for range 10 {
				fabrik(family.Bone_ids, bones, target, root)
			}
		case "Arc":
			arc_ik(family.Bone_ids, bones, root, target)
		}

		tipPos := bones[family.Bone_ids[len(family.Bone_ids)-1]].Pos
		for i := len(family.Bone_ids) - 1; i >= 0; i-- {
			bone := &bones[family.Bone_ids[i]]
			dir := tipPos.Sub(bone.Pos)
			bones[family.Bone_ids[i]].Rot = float32(math.Atan2(float64(dir.Y), float64(dir.X)))
			tipPos = bone.Pos
		}

		jointDir := normalize(bones[family.Bone_ids[1]].Pos.Sub(bones[family.Bone_ids[0]].Pos))
		baseDir := normalize(target.Sub(root))
		dir := jointDir.X*baseDir.Y - baseDir.X*jointDir.Y
		baseAngle := math.Atan2(float64(baseDir.Y), float64(baseDir.X))

		cw := family.Constraint == "Clockwise" && dir > 0
		ccw := family.Constraint == "CounterClockwise" && dir < 0
		if cw || ccw {
			for _, id := range family.Bone_ids {
				bones[id].Rot = -bones[id].Rot + float32(baseAngle*2)
			}
		}

		for i, boneId := range family.Bone_ids {
			if i == len(family.Bone_ids)-1 {
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

		rotated := RotateVec2(bones[bone_ids[f]].Pos.Sub(root), baseAngle)
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

func interpolateKeyframes(
	field float32, prevKf Keyframe, nextKf Keyframe, frame int, smoothFrame int,
) float32 {
	totalFrames := uint(nextKf.Frame - prevKf.Frame)
	currentFrame := uint(frame - prevKf.Frame)
	result := interpolate(currentFrame, totalFrames, prevKf.Value, nextKf.Value, nextKf.Start_handle, nextKf.End_handle)
	return interpolate(currentFrame, uint(smoothFrame), field, result, Vec2{X: 0, Y: 0}, Vec2{X: 0, Y: 0})
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

func IsFacingLeft(scale Vec2) bool {
	either := scale.X < 0 || scale.Y < 0
	both := scale.X < 0 && scale.Y < 0
	return either && !both
}
