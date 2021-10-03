package measurement

import (
	"math"
)

type Orientation struct {
	q0 float64
	q1 float64
	q2 float64
	q3 float64
}

func (o Orientation) getAsEuler() (float64, float64, float64) {
	roll := math.Atan2(2*(o.q0*o.q1+o.q2*o.q3), 1-2*(o.q1*o.q1+o.q2*o.q2))
	pitch := math.Asin(2.0 * (o.q0*o.q2 - o.q3*o.q1))
	yaw := math.Atan2(2*(o.q1*o.q2+o.q0*o.q3), 1-2*(o.q2*o.q2+o.q3*o.q3))

	return roll, pitch, yaw
}
