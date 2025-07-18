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
)

type Vec2 struct {
	X float32
	Y float32
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

type Bone struct {
	Id        int
	Name      string
	Parent_id int
	Tex_idx   int

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

type Armature struct {
	Bones      []Bone
	Animations []Animation
	Textures   []Texture
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

func Animate(armature Armature, anim_idx int, frame int) []Bone {
	var props []Bone

	// fmt.Println(len(armature.Animations[anim_idx].Keyframes));

	last_kf := len(armature.Animations[anim_idx].Keyframes) - 1
	frame %= armature.Animations[anim_idx].Keyframes[last_kf].Frame

	for _, bone := range armature.Bones {
		props = append(props, bone)

		prop := &props[len(props)-1]

		prop.Rot += interpolate(armature.Animations[anim_idx].Keyframes, frame, prop.Id, "Rotation", 0)
		prop.Scale.X *= interpolate(armature.Animations[anim_idx].Keyframes, frame, prop.Id, "ScaleX", 1)
		prop.Scale.Y *= interpolate(armature.Animations[anim_idx].Keyframes, frame, prop.Id, "ScaleY", 1)
		prop.Pos.X += interpolate(armature.Animations[anim_idx].Keyframes, frame, prop.Id, "PositionX", 0)
		prop.Pos.Y += interpolate(armature.Animations[anim_idx].Keyframes, frame, prop.Id, "PositionY", 0)

		if bone.Parent_id == -1 {
			continue
		}

		// inherit parent
		parent, _ := find_bone(props, bone.Parent_id)

		prop.Rot += parent.Rot
		prop.Scale.X *= parent.Scale.X
		prop.Scale.Y *= parent.Scale.Y
		prop.Pos.X *= parent.Scale.X
		prop.Pos.Y *= parent.Scale.Y

		x := prop.Pos.X
		y := prop.Pos.Y
		sin_parent_rot := float32(math.Sin(float64(parent.Rot)))
		cos_parent_rot := float32(math.Cos(float64(parent.Rot)))
		prop.Pos.X = x*cos_parent_rot - y*sin_parent_rot
		prop.Pos.Y = x*sin_parent_rot + y*cos_parent_rot

		prop.Pos.X += parent.Pos.X
		prop.Pos.Y += parent.Pos.Y
	}

	return props
}

func find_bone(bones []Bone, id int) (Bone, error) {
	for _, bone := range bones {
		if bone.Id == id {
			return bone, nil
		}
	}

	return bones[0], errors.New("Could not find bone of ID " + strconv.Itoa(id))
}

func interpolate(keyframes []Keyframe, frame int, bone_id int, element string, default_value float32) float32 {
	var prev_kf Keyframe
	var next_kf Keyframe
	prev_kf.Frame = -1
	next_kf.Frame = -1

	for _, kf := range keyframes {
		if kf.Frame >= frame {
			break
		} else if kf.Bone_id == bone_id && kf.Element == element {
			prev_kf = kf
		}
	}

	for _, kf := range keyframes {
		if kf.Frame >= frame && kf.Bone_id == bone_id && kf.Element == element {
			next_kf = kf
			break
		}
	}

	if prev_kf.Frame == -1 {
		prev_kf = next_kf
	} else if next_kf.Frame == -1 {
		next_kf = prev_kf
	}

	if prev_kf.Frame == -1 && next_kf.Frame == -1 {
		return default_value
	}

	total_frames := next_kf.Frame - prev_kf.Frame
	current_frame := frame - prev_kf.Frame
	if total_frames == 0 {
		return prev_kf.Value
	}

	// fmt.Println(current_frame, total_frames)

	interp := float32(current_frame) / float32(total_frames)
	start := prev_kf.Value
	end := next_kf.Value - prev_kf.Value
	result := start + (end * interp)
	return result
}

func Get_frame_by_time(anim Animation, unix_milli int64, reverse bool) int {
	fps := anim.Fps
	last_frame := anim.Keyframes[len(anim.Keyframes)-1].Frame

	var frametime float32 = 1 / float32(fps)
	frame := int(float32(unix_milli) / frametime / 1000)

	if reverse {
		frame = last_frame - frame
	}

	return frame
}
