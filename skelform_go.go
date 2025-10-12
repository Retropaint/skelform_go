package skelform_go

import (
	"archive/zip"
	"encoding/json"
	"errors"
	"image"
	"image/png"
	"io/ioutil"
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
	Element    string `json:"_element"`
	Element_id int
	Value      float32
	Transition string
}

type Animation struct {
	Name      string
	Fps       int
	Keyframes []Keyframe
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

	Parent_rot float32
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
	Bones       []Bone
	Animations  []Animation
	Styles      []Style
	Ik_families []IkFamily
}

type Root struct {
	Armature     Armature
	Texture_size Vec2
}

func Load(path string) (Root, image.Image) {
	zip, _ := zip.OpenReader(path)

	defer zip.Close()

	var root Root
	var texture image.Image

	for _, f := range zip.File {
		file, _ := f.Open()
		if f.Name == "armature.json" {
			bytes, _ := ioutil.ReadAll(file)
			json.Unmarshal(bytes, &root)
		} else if f.Name == "textures.png" {
			texture, _ = png.Decode(file)
		}
	}

	return root, texture
}

func Animate(bones []Bone, animation Animation, frame int, blendFrames int) []Bone {
	for b := range bones {
		bone := &bones[b]
		interpolateKeyframes(&bone.Rot, "Rotation", animation.Keyframes, frame, bone.Id, blendFrames)
		interpolateKeyframes(&bone.Scale.X, "ScaleX", animation.Keyframes, frame, bone.Id, blendFrames)
		interpolateKeyframes(&bone.Scale.Y, "ScaleY", animation.Keyframes, frame, bone.Id, blendFrames)
		interpolateKeyframes(&bone.Pos.X, "PositionX", animation.Keyframes, frame, bone.Id, blendFrames)
		interpolateKeyframes(&bone.Pos.Y, "PositionY", animation.Keyframes, frame, bone.Id, blendFrames)
	}

	return bones
}

func Inheritance(bones []Bone, ikRots map[uint]float32) []Bone {
	for b := range bones {
		bone := &bones[b]

		if bone.Parent_id != -1 {
			parent, _ := find_bone(bones, bone.Parent_id)

			bone.Rot += parent.Rot

			bone.Scale = bone.Scale.Mul(parent.Scale)
			bone.Pos = bone.Pos.Mul(parent.Scale)

			// rotate child such that it orbits parent
			x := bone.Pos.X
			y := bone.Pos.Y
			sin_parent_rot := float32(math.Sin(float64(parent.Rot)))
			cos_parent_rot := float32(math.Cos(float64(parent.Rot)))
			bone.Pos.X = x*cos_parent_rot - y*sin_parent_rot
			bone.Pos.Y = x*sin_parent_rot + y*cos_parent_rot

			bone.Pos = bone.Pos.Add(parent.Pos)
		}

		if rot, ok := ikRots[uint(b)]; ok {
			bones[b].Rot = rot
		}
	}

	return bones
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

func find_bone(bones []Bone, id int) (Bone, error) {
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
		frame %= animation.Keyframes[lastKf].Frame
	}

	if reverse {
		frame = lastFrame - frame
	}

	return frame
}

// Provide a frame of the animation based on time.
func TimeFrame(animation Animation, time time.Duration, reverse bool, loop bool) int {
	fps := animation.Fps

	var frametime float32 = 1 / float32(fps)
	frame := int(float32(time.Milliseconds()) / frametime / 1000)

	frame = FormatFrame(animation, frame, reverse, loop)

	return frame
}
