package polygon

import (
	"math"
	"reflect"

	log "github.com/sirupsen/logrus"
)

const (
	lineSidePositive = iota
	lineSideNegative
	lineCross
	lineCrossNoIntersect
)

// GridUnit when roi is larger,grid unit can also be adjust to be larger.
const GridUnit = 50

func sameSign(a, b float32) bool {
	return (a > 0 && b > 0) || (a < 0 && b < 0)
}

func lineLength(a, b Point) float32 {
	return float32(math.Sqrt(float64((b.X-a.X)*(b.X-a.X) + (b.Y-a.Y)*(b.Y-a.Y))))
}

func pointOnLine(point Point, line []Point) bool {
	linelen := lineLength(line[0], line[1])
	lenSum := lineLength(point, line[0]) + lineLength(point, line[1])
	return math.Abs(float64(lenSum-linelen)) < 1e-2
}

// pointOnSide return left hand side(<0), right hand side (>0), on line (=0)
func pointOnSide(line []Point, a Point) float32 {
	return CrossProduct2D(Vector2F{
		line[1].X - line[0].X,
		line[1].Y - line[0].Y,
	}, Vector2F{
		a.X - line[0].X,
		a.Y - line[0].Y,
	})
}

func CrossProduct2D(a, b Vector2F) float32 {
	return a.X*b.Y - a.Y*b.X
}

// RectContains determine if a point is on or inside the rect
func RectContains(rect Rect, point Point) bool {
	return rect.Left <= point.X && rect.Top <= point.Y && point.X <= rect.Right && point.Y <= rect.Bottom
}

// rectLineRelation tells the 2D position relation between a rect and a segment line
func rectLineRelation(rect Rect, line []Point) int {
	linelen := lineLength(line[0], line[1])
	l := Vector2F{
		(line[1].X - line[0].X) / linelen,
		(line[1].Y - line[0].Y) / linelen,
	}
	n := Vector2F{
		l.Y,
		-l.X,
	}
	cd := n.X*line[0].X + n.Y*line[0].Y
	corners := []Point{
		{rect.Left, rect.Top},
		{rect.Right, rect.Top},
		{rect.Right, rect.Bottom},
		{rect.Left, rect.Bottom},
	}
	var lastSign float32
	ds := make([]float32, 0, len(corners))
	sameSide := true
	for i, corner := range corners {
		ds = append(ds, n.X*corner.X+n.Y*corner.Y-cd)
		// if point on line,we make it lineCross
		if i < 1 || sameSign(ds[i], lastSign) {
			lastSign = ds[i]
		} else {
			sameSide = false
		}
	}
	if sameSide {
		if ds[0] > 0 {
			return lineSidePositive
		}
		return lineSideNegative
	} else if RectContains(rect, line[0]) || RectContains(rect, line[1]) {
		return lineCross
	}
	//seperate axis theorm
	var a, b, c, d float32
	if l.X > 0 {
		a, b = line[0].X, line[1].X
	} else {
		a, b = line[1].X, line[0].X
	}
	if l.Y > 0 {
		c, d = line[0].Y, line[1].Y
	} else {
		c, d = line[1].Y, line[0].Y
	}
	seperate := (rect.Left > b || rect.Right < a)
	seperate = seperate || (rect.Top > d || rect.Bottom < c)
	for i := 0; !seperate && i < len(ds); i++ {
		intersection := Point{
			corners[i].X - ds[i]*n.X,
			corners[i].Y - ds[i]*n.Y,
		}
		if pointOnLine(intersection, line) {
			seperate = false
		}
	}
	if seperate {
		return lineCrossNoIntersect
	}
	return lineCross
}

// Area : points is clock-wised open path of polygon vertices.Reference：http://mathworld.wolfram.com/PolygonArea.html
func Area(points []Point) float32 {
	if len(points) == 0 {
		return 0
	}
	var area float32
	points = append(points, points[0])
	for i := 0; i < len(points)-1; i++ {
		area += points[i].X*points[i+1].Y - points[i+1].X*points[i].Y
	}
	return 0.5 * area
}

// LineIntersect 判断两线段是否相交, 相交则返回交点
func LineIntersect(a, b []Point) (intersect *Point, t1 float32, t2 float32) {
	v1 := Vector2F{
		(a[1].X - a[0].X),
		(a[1].Y - a[0].Y),
	}
	v2 := Vector2F{
		(b[1].X - b[0].X),
		(b[1].Y - b[0].Y),
	}
	// v_1.x * t_1 - v_2.x * t_2 = b_0.x - a_0.x
	// v_1.y * t_1 - v_2.y * t_2 = b_0.y - a_0.y
	det := v1.X*(-v2.Y) - v1.Y*(-v2.X)
	if math.Abs(float64(det)) < 1e-3 {
		return nil, -1, -1
	}
	detT1 := (b[0].X-a[0].X)*(-v2.Y) - (b[0].Y-a[0].Y)*(-v2.X)
	detT2 := v1.X*(b[0].Y-a[0].Y) - v1.Y*(b[0].X-a[0].X)
	t1 = detT1 / det
	t2 = detT2 / det
	return &Point{
		X: a[0].X + t1*v1.X,
		Y: a[0].Y + t1*v1.Y,
	}, t1, t2
}

// ConvexPolyContains determine if a point is on (includeEdge=true) or inside the convex polygon
func ConvexPolyContains(polygon []Point, target Point, includeEdge bool) bool {
	var lastSign float32
	for i := 0; i < len(polygon); i++ {
		j := i + 1
		if j == len(polygon) {
			j = 0
		}
		vectorOut := Vector2F{
			X: polygon[i].X - target.X,
			Y: polygon[i].Y - target.Y,
		}
		vecotorSide := Vector2F{
			X: polygon[j].X - polygon[i].X,
			Y: polygon[j].Y - polygon[i].Y,
		}
		sign := CrossProduct2D(vectorOut, vecotorSide)
		if includeEdge && math.Abs(float64(sign)) < 1e-4 {
			continue
		}
		if math.Abs(float64(lastSign)) < 1e-4 || sameSign(lastSign, sign) {
			lastSign = sign
		} else {
			return false
		}
	}
	return true
}

// IsPointsConvex takes clock-wised point array as input
func IsPointsConvex(points []Point) bool {
	for i := 0; i < len(points); i++ {
		l := []Point{
			points[i],
			points[(i+1)%len(points)],
		}
		side := pointOnSide(l, points[0])
		for j := 0; j < len(points); j++ {
			newSide := pointOnSide(l, points[j])
			if (side > 0 && newSide < 0) || (side < 0 && newSide > 0) {
				return false
			}
			if side == 0 {
				side = newSide
			}
		}
	}
	return true
}

func PointDistance(a, b Point) float64 {
	return math.Sqrt(float64((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y)))
}

func ThreePointOnLine(a, b, c Point) bool {
	if a == b || b == c || a == c {
		return true
	}
	lenab := lineLength(a, b)
	lenbc := lineLength(b, c)
	v1 := Vector2F{
		(b.X - a.X) / lenab,
		(b.Y - a.Y) / lenab,
	}
	v2 := Vector2F{
		(c.X - b.X) / lenbc,
		(c.Y - b.Y) / lenbc,
	}
	return math.Abs(float64(v2.X-v1.X)) < 1e-3 && math.Abs(float64(v2.Y-v1.Y)) < 1e-3
}

// Polygon is defined as a clock-wised open path
type Polygon interface {
	GetPoints() []Point
	Contains(point Point) bool
	IsConvex() bool
	Area() float32
	GetConvexDecompositions() []Polygon
	IntersectArea(other Polygon) float32
}

func polygonContainsAll(polygon Polygon, testpoints []Point) bool {
	for _, p := range testpoints {
		if !polygon.Contains(p) {
			return false
		}
	}
	return true
}

func polygonContainsAny(polygon Polygon, testpoints []Point) bool {
	for _, p := range testpoints {
		if polygon.Contains(p) {
			return true
		}
	}
	return false
}

type PolygonBase struct {
	points []Point
	convex bool
	area   float32
}

func newPolygonBase(points []Point, convex bool) PolygonBase {
	area := Area(points)
	return PolygonBase{
		points: points,
		convex: convex,
		area:   area,
	}
}

func (pb *PolygonBase) GetPoints() []Point {
	return pb.points
}

func (pb *PolygonBase) IsConvex() bool {
	return pb.convex
}

func (pb *PolygonBase) Area() float32 {
	return pb.area
}

type Triangle struct {
	PolygonBase
}

func NewTriangle(points []Point) *Triangle {
	return &Triangle{
		newPolygonBase(points, true),
	}
}

func (t *Triangle) Contains(point Point) bool {
	return ConvexPolyContains(t.points, point, true)
}

func (t *Triangle) ContainsInside(point Point) bool {
	return ConvexPolyContains(t.points, point, false)
}

func (t *Triangle) GetConvexDecompositions() []Polygon {
	return []Polygon{t}
}

func (t *Triangle) IntersectArea(other Polygon) float32 {
	if IsPolygonNil(other) {
		return 0
	}
	return convexOverlap(t, other)
}

type Rectangle struct {
	PolygonBase
	rect Rect
}

func NewRectangle(points []Point) *Rectangle {
	if len(points) == 2 && points[1].X > points[0].X && points[1].Y > points[0].Y {
		corners := []Point{
			points[0],
			{points[1].X, points[0].Y},
			points[1],
			{points[0].X, points[1].Y},
		}
		return NewRectangle(corners)
	} else if len(points) == 4 {
		if points[0].X == points[3].X && points[0].Y == points[1].Y && points[1].X == points[2].X && points[2].Y == points[3].Y {
			rect := Rect{
				Left: math.MaxFloat32, Right: -1,
				Top: math.MaxFloat32, Bottom: -1,
			}
			for i := 0; i < len(points); i++ {
				if points[i].X < rect.Left {
					rect.Left = points[i].X
				} else if points[i].X > rect.Right {
					rect.Right = points[i].X
				}
				if points[i].Y < rect.Top {
					rect.Top = points[i].Y
				} else if points[i].Y > rect.Bottom {
					rect.Bottom = points[i].Y
				}
			}
			return &Rectangle{PolygonBase: newPolygonBase(points, true), rect: rect}
		}
	}
	return nil
}

func (r *Rectangle) Rect() Rect {
	return r.rect
}

func (r *Rectangle) Contains(point Point) bool {
	return !(point.X < r.rect.Left || point.X > r.rect.Right || point.Y < r.rect.Top || point.Y > r.rect.Bottom)
}

func (r *Rectangle) GetConvexDecompositions() []Polygon {
	return []Polygon{
		NewTriangle([]Point{{r.rect.Left, r.rect.Top}, {r.rect.Right, r.rect.Top}, {r.rect.Right, r.rect.Bottom}}),
		NewTriangle([]Point{{r.rect.Right, r.rect.Bottom}, {r.rect.Left, r.rect.Bottom}, {r.rect.Left, r.rect.Top}}),
	}
}

func (r *Rectangle) IntersectArea(other Polygon) float32 {
	if IsPolygonNil(other) {
		return 0
	}
	return convexOverlap(r, other)
}

// PolygonImpl is the implementation of general polygon
type PolygonImpl struct {
	PolygonBase
	normals   []Vector2F
	ds        []float32
	bounding  Rect
	grids     [][][]int
	triangles []*Triangle
}

func (p *PolygonImpl) Contains(point Point) bool {
	if point.X < p.bounding.Left || point.X > p.bounding.Right || point.Y < p.bounding.Top || point.Y > p.bounding.Bottom {
		return false
	}
	l := int32(point.X-p.bounding.Left) / GridUnit
	h := int32(point.Y-p.bounding.Top) / GridUnit
	if int(l) < len(p.grids) && p.grids[l] != nil && int(h) < len(p.grids[l]) && p.grids[l][h] != nil {
		ts := p.grids[l][h]
		for i := 0; i < len(ts); i++ {
			if p.triangles[ts[i]].Contains(point) {
				return true
			}
		}
	}
	return false
}

func (p *PolygonImpl) GetTriangles() []*Triangle {
	return p.triangles
}

func (p *PolygonImpl) GetConvexDecompositions() []Polygon {
	var o []Polygon
	for _, r := range p.GetTriangles() {
		o = append(o, r)
	}
	return o
}

func (p *PolygonImpl) init() bool {
	p.normals = make([]Vector2F, len(p.points))
	p.ds = make([]float32, len(p.points))
	p.bounding = Rect{
		Left: math.MaxFloat32, Right: -1,
		Top: math.MaxFloat32, Bottom: -1,
	}
	for i := 0; i < len(p.points); i++ {
		j := (i + 1) % len(p.points)
		linelen := lineLength(p.points[i], p.points[j])
		l := Vector2F{
			(p.points[j].X - p.points[i].X) / linelen,
			(p.points[j].Y - p.points[i].Y) / linelen,
		}
		p.normals[i] = Vector2F{
			l.Y,
			-l.X,
		}
		p.ds[i] = -(p.normals[i].X*p.points[i].X + p.normals[i].Y*p.points[i].Y)
		if p.points[i].X < p.bounding.Left {
			p.bounding.Left = p.points[i].X
		} else if p.points[i].X > p.bounding.Right {
			p.bounding.Right = p.points[i].X
		}
		if p.points[i].Y < p.bounding.Top {
			p.bounding.Top = p.points[i].Y
		} else if p.points[i].Y > p.bounding.Bottom {
			p.bounding.Bottom = p.points[i].Y
		}
	}
	p.convex = IsPointsConvex(p.points)
	idx := make([]int, len(p.points))
	for i := 0; i < len(p.points); i++ {
		idx[i] = i
	}
	//divide the polygon into triangles
	//assertion:
	// 1.a polygon is made up of at least one convex polygon
	// 2.a convex polygon can be devided into triangles
	//so,we have:
	edgeCnt := len(p.points)
	i, j, k := 0, 1, 2
	shiftRetry := 0
	normals := make([]Vector2F, len(p.normals))
	copy(normals, p.normals)
	ds := make([]float32, len(p.ds))
	copy(ds, p.ds)
	// TODO: optimize following with Delaunay Triangulation
	for edgeCnt > 3 {
		if ThreePointOnLine(p.points[i], p.points[j], p.points[k]) {
			//form a line
			idx[j] = -1
			j = k
			edgeCnt = edgeCnt - 1
			shiftRetry = 0
		} else {
			center := Point{
				X: (p.points[i].X + p.points[j].X + p.points[k].X) / 3,
				Y: (p.points[i].Y + p.points[j].Y + p.points[k].Y) / 3,
			}
			//for convex polygon, center should be on lineSideNegative
			d1 := normals[i].X*center.X + normals[i].Y*center.Y + ds[i]
			d2 := normals[j].X*center.X + normals[j].Y*center.Y + ds[j]
			if d1 < 0 && d2 < 0 {
				t := NewTriangle([]Point{
					p.points[i], p.points[j], p.points[k],
				})
				noContain := true
				for m := 0; m < len(idx); m++ {
					if m != i && m != j && m != k && idx[m] >= 0 && t.ContainsInside(p.points[idx[m]]) {
						noContain = false
						break
					}
				}
				if noContain {
					p.triangles = append(p.triangles, t)
					idx[j] = -1
					edgeCnt = edgeCnt - 1
					j = k
					//update normals
					linelen := lineLength(p.points[i], p.points[j])
					l := Vector2F{
						(p.points[j].X - p.points[i].X) / linelen,
						(p.points[j].Y - p.points[i].Y) / linelen,
					}
					normals[i] = Vector2F{
						l.Y,
						-l.X,
					}
					ds[i] = -(normals[i].X*p.points[i].X + normals[i].Y*p.points[i].Y)
					shiftRetry = 0
				} else {
					//shift
					i, j = j, k
					shiftRetry = shiftRetry + 1
				}
			} else {
				//shift
				i, j = j, k
				shiftRetry = shiftRetry + 1
			}
		}
		k = (k + 1) % len(p.points)
		for idx[k] < 0 && k < i {
			k = (k + 1) % len(p.points)
		}
		if shiftRetry >= edgeCnt || k == i {
			log.Errorf("should not happened,shiftRetry:%v,points:%v", shiftRetry, p.points)
			return false
		}
	}
	if !ThreePointOnLine(p.points[i], p.points[j], p.points[k]) {
		p.triangles = append(p.triangles, NewTriangle([]Point{
			p.points[i], p.points[j], p.points[k],
		}))
	}
	if len(p.triangles) == 0 {
		return false
	}
	// build index
	// TODO: optimize following indexes by only indexing the edge part and discretize the indexes
	w := int32(p.bounding.Right - p.bounding.Left)
	h := int32(p.bounding.Bottom - p.bounding.Top)
	wb := (w-1)/GridUnit + 1
	hb := (h-1)/GridUnit + 1
	p.grids = make([][][]int, wb)
	for i := 0; i < int(wb); i++ {
		p.grids[i] = make([][]int, hb)
		for j := 0; j < int(hb); j++ {
			rectPoly := NewRectangle([]Point{
				{X: p.bounding.Left + float32(i)*GridUnit, Y: p.bounding.Top + float32(j)*GridUnit},
				{X: p.bounding.Left + float32(i+1)*GridUnit, Y: p.bounding.Top + float32(j+1)*GridUnit},
			})

			m := make(map[int]bool)
			for k, t := range p.triangles {
				intersect := polygonContainsAny(t, rectPoly.GetPoints())
				tp := t.GetPoints()
				for g := 0; !intersect && g < len(tp); g++ {
					f := (g + 1) % len(tp)
					if rectLineRelation(rectPoly.rect, []Point{tp[g], tp[f]}) == lineCross {
						intersect = true
						break
					}
				}
				if intersect || polygonContainsAny(rectPoly, tp) {
					if _, ok := m[k]; !ok {
						m[k] = true
						p.grids[i][j] = append(p.grids[i][j], k)
					}
				}
			}
		}
	}
	return true
}

func (p *PolygonImpl) Grid(i, j int) *Rectangle {
	w := int32(p.bounding.Right - p.bounding.Left)
	h := int32(p.bounding.Bottom - p.bounding.Top)
	wb := (w-1)/GridUnit + 1
	hb := (h-1)/GridUnit + 1
	if i < int(wb) && j < int(hb) {
		return NewRectangle([]Point{
			{p.bounding.Left + float32(i)*GridUnit, p.bounding.Top + float32(j)*GridUnit},
			{p.bounding.Left + float32(i+1)*GridUnit, p.bounding.Top + float32(j+1)*GridUnit},
		})
	}
	return nil
}

func (p *PolygonImpl) IntersectArea(other Polygon) float32 {
	if IsPolygonNil(other) {
		return 0
	}
	return PolygonOverlap(p, other)
}

func NewPolygon(points []Point) Polygon {
	if len(points) < 2 {
		return nil
	} else if len(points) == 2 {
		return NewRectangle(points)
	} else if len(points) == 3 {
		return NewTriangle(points)
	}
	p := &PolygonImpl{
		PolygonBase: newPolygonBase(points, true),
	}
	if p.init() {
		return p
	}
	return nil
}

func IsPolygonNil(polygon Polygon) (r bool) {
	defer func() {
		if e := recover(); e != nil {
			r = true
		}
	}()
	if polygon == nil {
		return true

	}
	if reflect.ValueOf(polygon).IsNil() {
		return true
	}
	return false
}

// PolygonOverlap calculates intersection between two general polygon. Caller makes sure a, b is valid polygon.
func PolygonOverlap(a, b Polygon) float32 {
	if a.IsConvex() && b.IsConvex() {
		return convexOverlap(a, b)
	} else if a.IsConvex() {
		return convexOverlap(a, b)
	} else if b.IsConvex() {
		return convexOverlap(b, a)
	}
	var overlap float32
	as := a.GetConvexDecompositions()
	bs := b.GetConvexDecompositions()
	for i := 0; i < len(as); i++ {
		for j := 0; j < len(bs); j++ {
			area := convexOverlap(as[i], bs[j])
			overlap += area
		}
	}
	return overlap
}

// convexOverlap implements https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm. Caller makes sure clipper  and subject is valid polygon.
func convexOverlap(clipper, subject Polygon) float32 {
	aPoints := clipper.GetPoints()
	bPoints := subject.GetPoints()
	if len(aPoints) < 3 || len(bPoints) < 3 {
		return 0
	}
	aPoints = append(aPoints, aPoints[0])
	bPoints = append(bPoints, bPoints[0])
	cutPoints := make([]Point, len(bPoints))
	copy(cutPoints, bPoints)
	for i := 0; i < len(aPoints)-1; i++ {
		var newCuts []Point
		l1 := []Point{
			aPoints[i],
			aPoints[(i+1)%len(aPoints)],
		}
		curPoint := cutPoints[0]
		prevSide := pointOnSide(l1, curPoint)
		var nextSide float32
		for j := 0; j < len(cutPoints)-1; j++ {
			if prevSide >= 0 {
				newCuts = append(newCuts, cutPoints[j])
			}
			nextSide = pointOnSide(l1, cutPoints[j+1])
			if (nextSide < 0 && prevSide > 0) || (nextSide > 0 && prevSide < 0) {
				l2 := []Point{
					cutPoints[j],
					cutPoints[j+1],
				}
				intersect, _, _ := LineIntersect(l1, l2)
				newCuts = append(newCuts, *intersect)
			}
			prevSide = nextSide
		}
		if len(newCuts) == 0 {
			return 0
		}
		newCuts = append(newCuts, newCuts[0])
		cutPoints = newCuts
	}
	return Area(cutPoints)
}

func PolygonIOU(a, b Polygon) float32 {
	if IsPolygonNil(a) || IsPolygonNil(b) || len(a.GetPoints()) < 3 || len(a.GetPoints()) < 3 {
		return 0
	}
	overlap := PolygonOverlap(a, b)
	return overlap / (a.Area() + b.Area() - overlap)
}
