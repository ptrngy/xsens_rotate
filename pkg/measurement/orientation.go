package measurement

import (
	"math"
)

type EulerAngles struct {
	Roll  float64
	Pitch float64
	Yaw   float64
}

type Orientation struct {
	Q0 float64
	Q1 float64
	Q2 float64
	Q3 float64
}

func (o Orientation) GetAsEuler() EulerAngles {
	roll := math.Atan2(2*(o.Q0*o.Q1+o.Q2*o.Q3), 1-2*(o.Q1*o.Q1+o.Q2*o.Q2))
	pitch := math.Asin(2.0 * (o.Q0*o.Q2 - o.Q3*o.Q1))
	yaw := math.Atan2(2*(o.Q1*o.Q2+o.Q0*o.Q3), 1-2*(o.Q2*o.Q2+o.Q3*o.Q3))

	return EulerAngles{Roll: roll, Pitch: pitch, Yaw: yaw}
}
