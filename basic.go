package polygon

import "math"

type Rect struct {
	Left   float32
	Top    float32
	Right  float32
	Bottom float32
}

func (r Rect) Width() float32 {
	return r.Right - r.Left
}

func (r Rect) Height() float32 {
	return r.Bottom - r.Top
}

type Point struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
}

type Vector2F Point

func (v *Vector2F) Normalize() Vector2F {
	len := float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y)))
	return Vector2F{
		X: v.X / len,
		Y: v.Y / len,
	}
}

func (v *Vector2F) NormalizeInplace() {
	len := float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y)))
	v.X = v.X / len
	v.Y = v.Y / len
}