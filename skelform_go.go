package skelform_go

import (
	"archive/zip"
	"encoding/json"
	"errors"
	"image"
	"image/png"
	"io"
	"math"
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
	Frame       int
	Bone_id     int
	Element_str string
	Element     int
	Value       float32
	Transition  string
}

type Animation struct {
	Name      string
	Fps       int
	Keyframes []Keyframe
}

type Vertex struct {
	Pos Vec2
	Uv  Vec2
	Id  int
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

	Ik_family_id      int
	Ik_constraint_str string
	Ik_constraint     int
	Ik_mode_str       string
	Ik_mode           int
	Ik_target_id      int
	Ik_bone_ids       []int

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
	Init_ik_constraint int
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
	Version      string
	Ik_root_ids  []int
	Texture_size Vec2
	Bones        []Bone
	Animations   []Animation
	Styles       []Style
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
			ikf(&bone.Pos.X, 0, kf, frame, bone.Id, bf)
			ikf(&bone.Pos.Y, 1, kf, frame, bone.Id, bf)
			ikf(&bone.Rot, 2, kf, frame, bone.Id, bf)
			ikf(&bone.Scale.X, 3, kf, frame, bone.Id, bf)
			ikf(&bone.Scale.Y, 4, kf, frame, bone.Id, bf)
			prev := getPrevKeyframe(kf, frame, 7, bone.Id)
			if prev != -1 {
				bone.Ik_constraint = int(kf[prev].Value)
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
	resetBoneElement(&bone.Pos.X, bone.Init_pos.X, 0, bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Pos.Y, bone.Init_pos.Y, 1, bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Rot, bone.Init_rot, 2, bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Scale.X, bone.Init_scale.X, 3, bone.Id, frame, blendFrames, anims)
	resetBoneElement(&bone.Scale.Y, bone.Init_scale.Y, 4, bone.Id, frame, blendFrames, anims)
	if shouldResetElement(anims, bone.Id, 7) {
		bone.Ik_constraint = bone.Init_ik_constraint
	}
}

func shouldResetElement(anims []Animation, boneId int, el int) bool {
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

func resetBoneElement(value *float32, init float32, el int, boneId int, frame int, blendFrames int, anims []Animation) {
	shouldReset := shouldResetElement(anims, boneId, el)
	if shouldReset {
		*value = interpolate(frame, blendFrames, *value, init)
	}
}

func Inheritance(bones []Bone, ikRots map[uint]float32) []Bone {
	for b := range bones {
		bone := &bones[b]

		if bone.Parent_id != -1 {
			parent, _ := findBone(bones, bone.Parent_id)

			bone.Rot += parent.Rot

			bone.Scale = bone.Scale.Mul(parent.Scale)
			bone.Pos = bone.Pos.Mul(parent.Scale)

			bone.Pos = rotate(bone.Pos, float64(parent.Rot))

			bone.Pos = bone.Pos.Add(parent.Pos)
		}

		if rot, ok := ikRots[uint(b)]; ok {
			bones[b].Rot = rot
		}
	}

	return bones
}

func Construct(armature *Armature) []Bone {
	var inheritedBones []Bone
	for _, bone := range armature.Bones {
		inheritedBones = append(inheritedBones, bone)
	}
	Inheritance(inheritedBones, make(map[uint]float32))
	ikRots := InverseKinematics(inheritedBones, armature.Ik_root_ids)

	var finalBones []Bone
	for _, bone := range armature.Bones {
		finalBones = append(finalBones, bone)
		finalBones[len(finalBones)-1].Vertices = nil
		for _, vert := range bone.Vertices {
			finalBones[len(finalBones)-1].Vertices = append(finalBones[len(finalBones)-1].Vertices, vert)
		}
	}
	Inheritance(finalBones, ikRots)
	ConstructVerts(finalBones)

	return finalBones
}

func ConstructVerts(bones []Bone) {
	for b := range bones {
		bone := &bones[b]

		var initVertPos []Vec2
		for _, vert := range bone.Vertices {
			initVertPos = append(initVertPos, vert.Pos)
		}

		for v := range bone.Vertices {
			vert := &bone.Vertices[v]
			vert.Pos = inheritVert(vert.Pos, *bone)
		}

		for bi, bind := range bone.Binds {
			if bind.Bone_id == -1 {
				continue
			}

			bindBone, _ := findBone(bones, bind.Bone_id)

			for _, bindVert := range bind.Verts {
				if !bind.Is_path {
					vert := &bone.Vertices[bindVert.Id]
					endPos := inheritVert(initVertPos[bindVert.Id], bindBone).Sub(vert.Pos)
					vert.Pos = vert.Pos.Add(endPos.Mulf(bindVert.Weight))
					continue
				}

				prev := int(math.Max(float64(bi-1), 0))
				next := int(math.Min(float64(bi+1), float64(len(bone.Binds)-1)))
				prevBindBone, _ := findBone(bones, bone.Binds[prev].Bone_id)
				nextBindBone, _ := findBone(bones, bone.Binds[next].Bone_id)

				prevDir := bindBone.Pos.Sub(prevBindBone.Pos)
				nextDir := nextBindBone.Pos.Sub(bindBone.Pos)
				prevNormal := normalize(Vec2{-prevDir.Y, prevDir.X})
				nextNormal := normalize(Vec2{-nextDir.Y, nextDir.X})
				average := prevNormal.Add(nextNormal)
				normalAngle := math.Atan2(float64(average.Y), float64(average.X))

				vert := &bone.Vertices[bindVert.Id]
				vert.Pos = initVertPos[bindVert.Id].Add(bindBone.Pos)
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
		case 0: // FABRIK
			for range 10 {
				fabrik(family.Ik_bone_ids, bones, target, root)
			}
		case 1: // Arc
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

		cw := family.Ik_constraint == 1 && dir > 0
		ccw := family.Ik_constraint == 2 && dir < 0
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

func getPrevKeyframe(keyframes []Keyframe, frame int, element int, boneId int) int {
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
	element int,
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

	result := interpolate(currentFrame, totalFrames, keyframes[prevKf].Value, keyframes[nextKf].Value)
	*field = interpolate(currentFrame, blendFrames, *field, result)
}

func interpolate(current int, max int, startValue float32, endValue float32) float32 {
	if max == 0 || current > max {
		return endValue
	}
	interp := float32(current) / float32(max)
	end := endValue - startValue
	return startValue + (end * interp)
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
