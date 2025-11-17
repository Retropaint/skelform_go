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
	Frame      int
	Bone_id    int
	Element    string
	Element_id int
	Value      float32
	Transition string
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
	Tex_idx   int
	Style_ids []int

	Rot    float32
	Scale  Vec2
	Pos    Vec2
	Pivot  Vec2
	Zindex float32

	Vertices []Vertex
	Indices  []int
	Binds    []Bind

	Parent_rot float32

	Init_rot   float32
	Init_scale Vec2
	Init_pos   Vec2
}

type Texture struct {
	Name   string
	Size   Vec2
	Offset Vec2
}

type Style struct {
	Id       int
	Name     string
	Textures []Texture
}

type IkFamily struct {
	Constraint string
	Target_id  int
	Bone_ids   []int
}

type Armature struct {
	Version      string
	Texture_size Vec2
	Bones        []Bone
	Animations   []Animation
	Styles       []Style
	Ik_families  []IkFamily
}

func Load(path string) (Armature, image.Image) {
	zip, _ := zip.OpenReader(path)

	defer zip.Close()

	var armature Armature
	var texture image.Image

	for _, f := range zip.File {
		file, _ := f.Open()
		if f.Name == "armature.json" {
			bytes, _ := io.ReadAll(file)
			json.Unmarshal(bytes, &armature)
		} else if f.Name == "textures.png" {
			texture, _ = png.Decode(file)
		}
	}

	return armature, texture
}

func Animate(bones []Bone, animation Animation, frame int, blendFrames int) {
	kf := animation.Keyframes
	bf := blendFrames
	ikf := interpolateKeyframes
	for b := range bones {
		bone := &bones[b]
		ikf(&bone.Rot, "Rotation", kf, frame, bone.Id, bf)
		ikf(&bone.Scale.X, "ScaleX", kf, frame, bone.Id, bf)
		ikf(&bone.Scale.Y, "ScaleY", kf, frame, bone.Id, bf)
		ikf(&bone.Pos.X, "PositionX", kf, frame, bone.Id, bf)
		ikf(&bone.Pos.Y, "PositionY", kf, frame, bone.Id, bf)
	}
}

// Reset bones back to default states, if they haven't been animated.
// Must be called after `Animate()` with the same animations provided.
// `frame` must be first anim frame.
func ResetBones(bones []Bone, anims []Animation, frame int, blendFrames int) {
	for b := range bones {
		bone := &bones[b]
		resetBoneElement(&bone.Pos.X, bone.Init_pos.X, 0, bone.Id, frame, blendFrames, anims)
		resetBoneElement(&bone.Pos.Y, bone.Init_pos.Y, 1, bone.Id, frame, blendFrames, anims)
		resetBoneElement(&bone.Rot, bone.Init_rot, 2, bone.Id, frame, blendFrames, anims)
		resetBoneElement(&bone.Scale.X, bone.Init_scale.X, 3, bone.Id, frame, blendFrames, anims)
		resetBoneElement(&bone.Scale.Y, bone.Init_scale.Y, 4, bone.Id, frame, blendFrames, anims)
	}
}

func resetBoneElement(value *float32, init float32, el int, bone_id int, frame int, blendFrames int, anims []Animation) {
	shouldReset := true
	for a := range anims {
		anim := &anims[a]
		for _, kf := range anim.Keyframes {
			if kf.Bone_id == bone_id && kf.Element_id == el {
				shouldReset = false
				break
			}
		}
		if !shouldReset {
			break
		}
	}
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

func InverseKinematics(bones []Bone, ikFamilies []IkFamily) map[uint]float32 {
	rotMap := make(map[uint]float32)

	for _, ikFamily := range ikFamilies {
		if ikFamily.Target_id == -1 {
			continue
		}

		startPos := bones[ikFamily.Bone_ids[0]].Pos

		target := bones[ikFamily.Target_id].Pos
		baseLine := normalize(target.Sub(startPos))
		baseAngle := math.Atan2(float64(baseLine.Y), float64(baseLine.X))

		// forward-reaching
		nextPos := bones[ikFamily.Target_id].Pos
		var nextLength float32 = 0.
		for i := len(ikFamily.Bone_ids) - 1; i >= 0; i-- {
			bone := &bones[ikFamily.Bone_ids[i]]

			lengthLine := Vec2{X: 0, Y: 0}
			if i != len(ikFamily.Bone_ids)-1 {
				lengthLine = normalize(nextPos.Sub(bone.Pos)).Mulf(nextLength)
			}

			if i != 0 {
				nextBone := &bones[ikFamily.Bone_ids[i-1]]
				nextLength = magnitude(bone.Pos.Sub(nextBone.Pos))
			}

			bone.Pos = nextPos.Sub(lengthLine)
			nextPos = bone.Pos
		}

		//backward-reaching
		prevPos := startPos
		var prevLength float32 = 0.
		for i := 0; i < len(ikFamily.Bone_ids); i++ {
			bone := &bones[ikFamily.Bone_ids[i]]

			lengthLine := Vec2{X: 0, Y: 0}
			if i != 0 {
				lengthLine = normalize(prevPos.Sub(bone.Pos)).Mulf(prevLength)
			}

			if i != len(ikFamily.Bone_ids)-1 {
				nextBone := &bones[ikFamily.Bone_ids[i+1]]
				prevLength = magnitude(bone.Pos.Sub(nextBone.Pos))
			}

			bone.Pos = prevPos.Sub(lengthLine)

			if i != 0 && i != len(ikFamily.Bone_ids)-1 && ikFamily.Constraint != "None" {
				jointLine := normalize(prevPos.Sub(bone.Pos))
				jointAngle := math.Atan2(float64(jointLine.Y), float64(jointLine.X)) - baseAngle

				var constraintMin float64
				var constraintMax float64
				if ikFamily.Constraint == "Clockwise" {
					constraintMin = -3.14
					constraintMax = 0
				} else {
					constraintMin = 0
					constraintMax = 3.14
				}

				if jointAngle > constraintMax || jointAngle < constraintMin {
					pushAngle := -jointAngle * 2
					newPoint := rotate(bone.Pos.Sub(prevPos), pushAngle)
					bone.Pos = newPoint.Add(prevPos)
				}
			}

			prevPos = bone.Pos
		}

		tipPos := bones[ikFamily.Bone_ids[len(ikFamily.Bone_ids)-1]].Pos
		for i := len(ikFamily.Bone_ids) - 1; i >= 0; i-- {
			bone := &bones[ikFamily.Bone_ids[i]]
			if i == len(ikFamily.Bone_ids)-1 {
				continue
			}

			dir := tipPos.Sub(bone.Pos)
			rot := math.Atan2(float64(dir.Y), float64(dir.X))
			tipPos = bone.Pos

			rotMap[uint(ikFamily.Bone_ids[i])] = float32(rot)
		}
	}

	return rotMap
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
	field *float32,
	element string,
	keyframes []Keyframe,
	frame int,
	boneId int,
	blendFrames int,
) {
	var prevKf Keyframe
	var nextKf Keyframe
	prevKf.Frame = -1
	nextKf.Frame = -1

	for _, kf := range keyframes {
		if kf.Frame < frame && kf.Bone_id == boneId && kf.Element == element {
			prevKf = kf
		}
	}

	for _, kf := range keyframes {
		if kf.Frame >= frame && kf.Bone_id == boneId && kf.Element == element {
			nextKf = kf
			break
		}
	}

	if prevKf.Frame == -1 {
		prevKf = nextKf
	} else if nextKf.Frame == -1 {
		nextKf = prevKf
	}

	if prevKf.Frame == -1 && nextKf.Frame == -1 {
		return
	}

	totalFrames := nextKf.Frame - prevKf.Frame
	currentFrame := frame - prevKf.Frame

	result := interpolate(currentFrame, totalFrames, prevKf.Value, nextKf.Value)
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
