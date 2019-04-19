package polygon

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

type PointContainResult struct {
	point  Point
	result bool
}

type LineRelationResult struct {
	rect     []Point
	relation int
}

type PolyContainTestCase struct {
	testcase []Point
	result   []PointContainResult
}

type LineRelationTestCase struct {
	line   []Point
	result []LineRelationResult
}

func TestLineRelation(t *testing.T) {
	cases := []LineRelationTestCase{
		{
			line: []Point{{421.76096, 420.44562}, {372.51233, 21.372267}},
			result: []LineRelationResult{
				{
					rect:     []Point{{313.1293, 241.50015}, {378.28833, 241.50015}, {378.28833, 479.07062}, {313.1293, 479.07062}},
					relation: lineSidePositive,
				},
			},
		},
		{
			line: []Point{{264.57422, 252.84947}, {162.53148, 313.72778}},
			result: []LineRelationResult{
				{
					rect:     []Point{{335.34024, 46.14892}, {377.17014, 46.14892}, {377.17014, 479.96573}, {335.34024, 479.96573}},
					relation: lineCrossNoIntersect,
				},
			},
		},
		{
			line: []Point{{209.49667, 293.23285}, {431.2084, 398.03928}},
			result: []LineRelationResult{
				{
					rect:     []Point{{156.47978, 180.47937}, {258.69614, 180.47937}, {258.69614, 512.37646}, {156.47978, 512.37646}},
					relation: lineCross,
				},
			},
		},
		{
			line: []Point{{420.63766, 562.1564}, {367.3153, 505.08368}},
			result: []LineRelationResult{
				{
					rect:     []Point{{254.45914, 75.14796}, {370.7232, 75.14796}, {370.7232, 536.6}, {254.45914, 536.6}},
					relation: lineCross,
				},
			},
		},
		{
			line: []Point{{310.73267, 547.1488}, {76.076485, 550.04755}},
			result: []LineRelationResult{
				{
					rect:     []Point{{139.33293, 150.2615}, {433.72552, 150.2615}, {433.72552, 160.56137}, {139.33293, 160.56137}},
					relation: lineSideNegative,
				},
			},
		},
		{
			line: []Point{{310.80164, 234.88882}, {47.21902, 123.19179}},
			result: []LineRelationResult{
				{
					rect:     []Point{{145.89615, 302.1356}, {372.4525, 302.1356}, {372.4525, 590.88715}, {145.89615, 590.88715}},
					relation: lineSidePositive,
				},
			},
		},
	}
	for _, testcase := range cases {
		results := testcase.result
		for _, result := range results {
			rectangle := NewRectangle(result.rect)
			assert.True(t, rectangle != nil)
			relation := rectLineRelation(rectangle.rect, testcase.line)
			assert.True(t, relation == result.relation)
		}
	}
}

func TestPolygonIOU(t *testing.T) {
	polygonA := NewPolygon([]Point{
		{0, 0},
		{200, 0},
	})
	polygonB := NewPolygon([]Point{
		{100, 100},
		{300, 100},
		{300, 300},
		{100, 300},
	})
	assert.Equal(t, float32(0), PolygonIOU(polygonA, polygonB))
}

func TestInvalidPoly(t *testing.T) {
	assert.True(t, IsPolygonNil(NewPolygon([]Point{{100, 600}, {100, 1000}})), "invalid input for polygon,expect nil")
	assert.True(t, IsPolygonNil(NewPolygon([]Point{{100, 100}, {200, 100}, {300, 100}, {400, 100}})), "invalid input for polygon,expect nil")
}

func TestPolyContains(t *testing.T) {
	cases := []PolyContainTestCase{
		{
			testcase: []Point{{100, 100}, {200, 62}, {113, 150}},
			result: []PointContainResult{
				{Point{345.75494, 462.7914}, false},
				{Point{132.75758, 143.12341}, false},
				{Point{142.36473, 121.12864}, false},
				{Point{125.93778, 97.98476}, true},
			},
		},
		{
			testcase: []Point{{42.4, 30.1}, {105.2, 22.88}, {169.7, 24.7}, {193.75, 54.5}, {166.4, 107.2}, {105.57, 79.6}},
			result: []PointContainResult{
				{Point{262.9031, 458.09238}, false},
				{Point{111.4646, 61.95192}, true},
				{Point{327.38177, 553.6266}, false},
				{Point{109.725716, 56.109062}, true},
			},
		},
		{
			testcase: []Point{{42.4, 30.1}, {105.2, 22.88}, {169.7, 24.7}, {221.5, 49.97}, {170, 60}, {147.9, 79.609}, {166.4, 107.2}, {105.57, 79.6}},
			result: []PointContainResult{
				{Point{251.41632, 141.04443}, false},
				{Point{241.93634, 83.632225}, false},
				{Point{94.1042, 317.6373}, false},
				{Point{40.846085, 94.18137}, false},
				{Point{419.5006, 348.348}, false},
				{Point{143.07578, 21.594929}, false},
				{Point{211.66937, 544.63586}, false},
				{Point{149.82825, 128.71086}, false},
				{Point{180.42242, 39.68708}, true},
				{Point{141.27689, 82.26808}, true},
			},
		},
		{
			testcase: []Point{{106.9, 42.7}, {130.3, 12.9}, {148.1, 41.1}, {162.7, 25.3}, {170.9, 41.9}, {210, 20}, {212.1, 67.3}, {255.7, 66.7}, {255.3, 97.9}, {211.7, 97.7}, {212.3, 118.9}, {255.1, 118.3}, {254.5, 144.9},
				{90.5, 144.9}, {90.5, 121.7}, {40.7, 123.7}, {41.3, 80.7}, {90, 80}, {90, 60}, {32.1, 31.1}},
			result: []PointContainResult{
				{Point{251.41632, 141.04443}, true},
				{Point{241.93634, 83.632225}, true},
				{Point{94.1042, 317.6373}, false},
				{Point{40.846085, 94.18137}, false},
				{Point{419.5006, 348.348}, false},
				{Point{143.07578, 21.594929}, false},
				{Point{211.66937, 544.63586}, false},
				{Point{149.82825, 128.71086}, true},
				{Point{180.42242, 39.68708}, true},
				{Point{141.27689, 82.26808}, true},
			},
		},
		{
			testcase: []Point{{30, 100}, {60, 100}, {120, 100}, {300, 100}, {250, 150}, {200, 200}, {150, 150}, {100, 200}, {50, 130}},
			result: []PointContainResult{
				{Point{154.25655, 143.85406}, true},
				{Point{38.37799, 96.53387}, false},
				{Point{230.08603, 558.0324}, false},
				{Point{175.55606, 112.43701}, true},
				{Point{170.9, 41.9}, false},
				{Point{149.82825, 128.71086}, true},
				{Point{91.27964, 104.69871}, true},
				{Point{97.30802, 569.6271}, false},
				{Point{121.324234, 147.04175}, true},
			},
		},
		{
			testcase: []Point{{30, 100}, {165, 200}, {300, 100}, {300, 300}, {165, 200}, {30, 300}},
			result: []PointContainResult{
				{Point{449.6291, 139.66093}, false},
				{Point{68.81833, 241.0276}, true},
			},
		},
	}
	for _, testcase := range cases {
		polygon := NewPolygon(testcase.testcase)
		assert.True(t, polygon != nil)
		for _, result := range testcase.result {
			assert.True(t, result.result == polygon.Contains(result.point))
		}
	}
}

func TestLineIntersect(t *testing.T) {
	line0 := []Point{
		{X: 0, Y: 0},
		{X: 50, Y: 50},
	}
	line1 := []Point{
		{X: 50, Y: 0},
		{X: 0, Y: 50},
	}
	result, _, _ := LineIntersect(line0, line1)
	assert.NotNil(t, result)
	line0 = []Point{
		{X: 0, Y: 0},
		{X: 50, Y: 50},
	}
	line1 = []Point{
		{X: 100, Y: 0},
		{X: 0, Y: 100},
	}
	_, t1, t2 := LineIntersect(line0, line1)
	assert.True(t, t1 == 1 && t2 == 0.5)
	line0 = []Point{
		{X: 0, Y: 0},
		{X: 50, Y: 50},
	}
	line1 = []Point{
		{X: 10, Y: 0},
		{X: 60, Y: 50},
	}
	result, _, _ = LineIntersect(line0, line1)
	assert.Nil(t, result)
}

func TestPolygonOverlap(t *testing.T) {
	polygonA := NewPolygon([]Point{
		{0, 0},
		{200, 0},
		{200, 200},
		{0, 200},
	})
	polygonB := NewPolygon([]Point{
		{100, 100},
		{300, 100},
		{300, 300},
		{100, 300},
	})
	assert.Equal(t, float32(10000.0), PolygonOverlap(polygonA, polygonB))
	polygonA = NewPolygon([]Point{
		{0, 0},
		{200, 0},
		{200, 200},
		{0, 200},
	})
	polygonB = NewPolygon([]Point{
		{100, 100},
		{300, 100},
		{300, 300},
		{180, 300},
		{180, 150},
		{150, 150},
		{150, 300},
		{100, 300},
	})
	assert.Equal(t, float32(8500.0), PolygonOverlap(polygonA, polygonB))
}
