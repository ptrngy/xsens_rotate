package imu

import (
	"math"

	"github.com/ptrngy/xsens_rotate/pkg/measurement"
)

// https://medium.com/@adrien.za/fast-inverse-square-root-in-go-and-javascript-for-fun-6b891e74e5a8
const magic64 = 0x5FE6EB50C7B537A9

type MadgwickAHRS struct {
	SamplingFrequency float64
	Quaternion        measurement.Quaternion
	BetaDef           float64
	Beta              float64
}

func NewMadgwickAHRS(samplingfreq, betadef float64) *MadgwickAHRS {
	m := MadgwickAHRS{
		SamplingFrequency: samplingfreq,
		BetaDef:           betadef,
		Beta:              betadef,
		Quaternion:        measurement.Quaternion{Q0: 1.0, Q1: 0.0, Q2: 0.0, Q3: 0.0},
	}

	return &m
}

//FastInvSqrt64 returns the inverse square root (quake heuristics) of a given number
func FastInvSqrt64(n float64) float64 {
	if n < 0 {
		return math.NaN()
	}
	n2, th := n*0.5, float64(1.5)
	b := math.Float64bits(n)
	b = magic64 - (b >> 1)
	f := math.Float64frombits(b)
	f *= th - (n2 * f * f)
	return f
}

// Update is used to update the quaternion if 9DOF is used
func (m *MadgwickAHRS) Update(gyro, accelero, magneto measurement.Vector3D) {
	if magneto.IsEmpty() {
		m.UpdateIMU(gyro, accelero)
		return
	}

	// Convert gyroscope degrees / sec to radians / sec
	gyro.Scale(0.0174533)

	// Rate of change of quaternion from gyroscope
	qDot := measurement.Quaternion{
		Q0: 0.5 * (-m.Quaternion.Q1*gyro.X - m.Quaternion.Q2*gyro.Y - m.Quaternion.Q3*gyro.Z),
		Q1: 0.5 * (m.Quaternion.Q0*gyro.X + m.Quaternion.Q2*gyro.Z - m.Quaternion.Q3*gyro.Y),
		Q2: 0.5 * (m.Quaternion.Q0*gyro.Y - m.Quaternion.Q1*gyro.Z + m.Quaternion.Q3*gyro.X),
		Q3: 0.5 * (m.Quaternion.Q0*gyro.Z + m.Quaternion.Q1*gyro.Y - m.Quaternion.Q2*gyro.X),
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !accelero.IsEmpty() {
		// Normalise accelerometer measurement
		recipNorm := FastInvSqrt64(accelero.SquareSum())
		accelero.Scale(recipNorm)

		// Normalise magnetometer measurement
		recipNorm = FastInvSqrt64(magneto.SquareSum())
		magneto.Scale(recipNorm)

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx := 2.0 * m.Quaternion.Q0 * magneto.X
		_2q0my := 2.0 * m.Quaternion.Q0 * magneto.Y
		_2q0mz := 2.0 * m.Quaternion.Q0 * magneto.Z
		_2q1mx := 2.0 * m.Quaternion.Q1 * magneto.X
		_2q0 := 2.0 * m.Quaternion.Q0
		_2q1 := 2.0 * m.Quaternion.Q1
		_2q2 := 2.0 * m.Quaternion.Q2
		_2q3 := 2.0 * m.Quaternion.Q3
		_2q0q2 := 2.0 * m.Quaternion.Q0 * m.Quaternion.Q2
		_2q2q3 := 2.0 * m.Quaternion.Q2 * m.Quaternion.Q3
		q0q0 := math.Pow(m.Quaternion.Q0, 2)
		q0q1 := m.Quaternion.Q0 * m.Quaternion.Q1
		q0q2 := m.Quaternion.Q0 * m.Quaternion.Q2
		q0q3 := m.Quaternion.Q0 * m.Quaternion.Q3
		q1q1 := math.Pow(m.Quaternion.Q1, 2)
		q1q2 := m.Quaternion.Q1 * m.Quaternion.Q2
		q1q3 := m.Quaternion.Q1 * m.Quaternion.Q3
		q2q2 := math.Pow(m.Quaternion.Q2, 2)
		q2q3 := m.Quaternion.Q2 * m.Quaternion.Q3
		q3q3 := math.Pow(m.Quaternion.Q3, 2)

		// Reference direction of Earth's magnetic field
		hx := magneto.X*q0q0 - _2q0my*m.Quaternion.Q3 + _2q0mz*m.Quaternion.Q2 + magneto.X*q1q1 + _2q1*magneto.Y*m.Quaternion.Q2 + _2q1*magneto.Z*m.Quaternion.Q3 - magneto.X*q2q2 - magneto.X*q3q3
		hy := _2q0mx*m.Quaternion.Q3 + magneto.Y*q0q0 - _2q0mz*m.Quaternion.Q1 + _2q1mx*m.Quaternion.Q2 - magneto.Y*q1q1 + magneto.Y*q2q2 + _2q2*magneto.Z*m.Quaternion.Q3 - magneto.Y*q3q3
		_2bx := math.Sqrt(hx*hx + hy*hy)
		_2bz := -_2q0mx*m.Quaternion.Q2 + _2q0my*m.Quaternion.Q1 + magneto.Z*q0q0 + _2q1mx*m.Quaternion.Q3 - magneto.Z*q1q1 + _2q2*magneto.Y*m.Quaternion.Q3 - magneto.Z*q2q2 + magneto.Z*q3q3
		_4bx := 2.0 * _2bx
		_4bz := 2.0 * _2bz

		// Gradient descent algorithm corrective step
		s0 := -_2q2*(2.0*q1q3-_2q0q2-accelero.X) + _2q1*(2.0*q0q1+_2q2q3-accelero.Y) - _2bz*m.Quaternion.Q2*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-magneto.X) + (-_2bx*m.Quaternion.Q3+_2bz*m.Quaternion.Q1)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-magneto.Y) + _2bx*m.Quaternion.Q2*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-magneto.Z)
		s1 := _2q3*(2.0*q1q3-_2q0q2-accelero.X) + _2q0*(2.0*q0q1+_2q2q3-accelero.Y) - 4.0*m.Quaternion.Q1*(1-2.0*q1q1-2.0*q2q2-accelero.Z) + _2bz*m.Quaternion.Q3*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-magneto.X) + (_2bx*m.Quaternion.Q2+_2bz*m.Quaternion.Q0)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-magneto.Y) + (_2bx*m.Quaternion.Q3-_4bz*m.Quaternion.Q1)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-magneto.Z)
		s2 := -_2q0*(2.0*q1q3-_2q0q2-accelero.X) + _2q3*(2.0*q0q1+_2q2q3-accelero.Y) - 4.0*m.Quaternion.Q2*(1-2.0*q1q1-2.0*q2q2-accelero.Z) + (-_4bx*m.Quaternion.Q2-_2bz*m.Quaternion.Q0)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-magneto.X) + (_2bx*m.Quaternion.Q1+_2bz*m.Quaternion.Q3)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-magneto.Y) + (_2bx*m.Quaternion.Q0-_4bz*m.Quaternion.Q2)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-magneto.Z)
		s3 := _2q1*(2.0*q1q3-_2q0q2-accelero.X) + _2q2*(2.0*q0q1+_2q2q3-accelero.Y) + (-_4bx*m.Quaternion.Q3+_2bz*m.Quaternion.Q1)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-magneto.X) + (-_2bx*m.Quaternion.Q0+_2bz*m.Quaternion.Q2)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-magneto.Y) + _2bx*m.Quaternion.Q1*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-magneto.Z)

		// Normalise step magnitude
		recipNorm = FastInvSqrt64(math.Pow(s0, 2) + math.Pow(s1, 2) + math.Pow(s2, 2) + math.Pow(s3, 2))
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		//Apply feedback step
		qDot.Q0 -= m.Beta * s0
		qDot.Q1 -= m.Beta * s1
		qDot.Q2 -= m.Beta * s2
		qDot.Q3 -= m.Beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	m.Quaternion.Q0 += qDot.Q0 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q1 += qDot.Q1 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q2 += qDot.Q2 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q3 += qDot.Q3 * (1.0 / m.SamplingFrequency)

	// Normalise quaternion
	recipNorm := FastInvSqrt64(m.Quaternion.SquareSum())
	m.Quaternion.Scale(recipNorm)
}

// UpdateIMU is used to update the quaternion if 6DOF is used
func (m *MadgwickAHRS) UpdateIMU(gyro, accelero measurement.Vector3D) {
	// Convert gyroscope degrees / sec to radians / sec
	gyro.Scale(0.0174533)

	// Rate of change of quaternion from gyroscope
	qDot := measurement.Quaternion{
		Q0: 0.5 * (-m.Quaternion.Q1*gyro.X - m.Quaternion.Q2*gyro.Y - m.Quaternion.Q3*gyro.Z),
		Q1: 0.5 * (m.Quaternion.Q0*gyro.X + m.Quaternion.Q2*gyro.Z - m.Quaternion.Q3*gyro.Y),
		Q2: 0.5 * (m.Quaternion.Q0*gyro.Y - m.Quaternion.Q1*gyro.Z + m.Quaternion.Q3*gyro.X),
		Q3: 0.5 * (m.Quaternion.Q0*gyro.Z + m.Quaternion.Q1*gyro.Y - m.Quaternion.Q2*gyro.X),
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !accelero.IsEmpty() {
		// Normalise accelerometer measurement
		recipNorm := FastInvSqrt64(accelero.SquareSum())
		accelero.Scale(recipNorm)

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 := 2.0 * m.Quaternion.Q0
		_2q1 := 2.0 * m.Quaternion.Q1
		_2q2 := 2.0 * m.Quaternion.Q2
		_2q3 := 2.0 * m.Quaternion.Q3
		_4q0 := 4.0 * m.Quaternion.Q0
		_4q1 := 4.0 * m.Quaternion.Q1
		_4q2 := 4.0 * m.Quaternion.Q2
		_8q1 := 8.0 * m.Quaternion.Q1
		_8q2 := 8.0 * m.Quaternion.Q2
		q0q0 := m.Quaternion.Q0 * m.Quaternion.Q0
		q1q1 := m.Quaternion.Q1 * m.Quaternion.Q1
		q2q2 := m.Quaternion.Q2 * m.Quaternion.Q2
		q3q3 := m.Quaternion.Q3 * m.Quaternion.Q3

		// Gradient decent algorithm corrective step
		s0 := _4q0*q2q2 + _2q2*accelero.X + _4q0*q1q1 - _2q1*accelero.Y
		s1 := _4q1*q3q3 - _2q3*accelero.X + 4.0*q0q0*m.Quaternion.Q1 - _2q0*accelero.Y - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*accelero.Z
		s2 := 4.0*q0q0*m.Quaternion.Q2 + _2q0*accelero.X + _4q2*q3q3 - _2q3*accelero.Y - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*accelero.Z
		s3 := 4.0*q1q1*m.Quaternion.Q3 - _2q1*accelero.X + 4.0*q2q2*m.Quaternion.Q3 - _2q2*accelero.Y

		// Normalise step magnitude
		recipNorm = FastInvSqrt64(math.Pow(s0, 2) + math.Pow(s1, 2) + math.Pow(s2, 2) + math.Pow(s3, 2))
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		//Apply feedback step
		qDot.Q0 -= m.Beta * s0
		qDot.Q1 -= m.Beta * s1
		qDot.Q2 -= m.Beta * s2
		qDot.Q3 -= m.Beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	m.Quaternion.Q0 += qDot.Q0 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q1 += qDot.Q1 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q2 += qDot.Q2 * (1.0 / m.SamplingFrequency)
	m.Quaternion.Q3 += qDot.Q3 * (1.0 / m.SamplingFrequency)

	// Normalise quaternion
	recipNorm := FastInvSqrt64(m.Quaternion.SquareSum())
	m.Quaternion.Scale(recipNorm)
}
