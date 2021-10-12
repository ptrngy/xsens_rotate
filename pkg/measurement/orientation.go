package measurement

import (
	"math"
)

type EulerAngles struct {
	Roll  float64
	Pitch float64
	Yaw   float64
}

type Quaternion struct {
	Q0 float64
	Q1 float64
	Q2 float64
	Q3 float64
}

func (q Quaternion) GetAsEuler() EulerAngles {
	roll := math.Atan2(2*(q.Q0*q.Q1+q.Q2*q.Q3), 1-2*(q.Q1*q.Q1+q.Q2*q.Q2))
	pitch := math.Asin(2.0 * (q.Q0*q.Q2 - q.Q3*q.Q1))
	yaw := math.Atan2(2*(q.Q1*q.Q2+q.Q0*q.Q3), 1-2*(q.Q2*q.Q2+q.Q3*q.Q3))

	return EulerAngles{Roll: roll, Pitch: pitch, Yaw: yaw}
}

func (q Quaternion) Update(q0, q1, q2, q3 float64) {
	q.Q0 = q0
	q.Q1 = q1
	q.Q2 = q2
	q.Q3 = q3
}

// SquareSum return the sum of squared components
func (q Quaternion) SquareSum() float64 {
	return math.Pow(q.Q0, 2) + math.Pow(q.Q1, 2) + math.Pow(q.Q2, 2) + math.Pow(q.Q3, 2)
}

// Scale is used to scale all components of a quaternion
func (q *Quaternion) Scale(factor float64) {
	q.Q0 = factor * q.Q0
	q.Q1 = factor * q.Q1
	q.Q2 = factor * q.Q2
	q.Q3 = factor * q.Q3
}
