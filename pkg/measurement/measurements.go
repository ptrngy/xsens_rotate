package measurement

import (
	"math"
)

type Vector3D struct {
	X float64
	Y float64
	Z float64
}

// NewMagnetometer creates a new Magnetometer and returns its pointer.
func NewMagnetometer(x, y, z float64) *Vector3D {
	m := new(Vector3D)

	m.X = x
	m.Y = y
	m.Z = z

	return m
}

// GetRotated rotates the coords reading to magnetic north.
func (m *Vector3D) GetRotated(o Quaternion) (float64, float64, float64) {
	e := o.GetAsEuler()

	ny, nz := rotateX(m.Y, m.Z, e.Roll)
	nx, nz := rotateY(m.X, nz, e.Pitch)
	nx, ny = rotateZ(nx, ny, e.Yaw)

	return nx, ny, nz
}

// GetRotatedEuler rotates the coords reading to magnetic north based on given Euler angles.
func (m *Vector3D) GetRotatedEuler(e EulerAngles) (float64, float64, float64) {
	ny, nz := rotateX(m.Y, m.Z, e.Roll)
	nx, nz := rotateY(m.X, nz, e.Pitch)
	nx, ny = rotateZ(nx, ny, e.Yaw)

	return nx, ny, nz
}

// IsEmpty checks if given vector is (0.0, 0.0, 0.0)
func (m Vector3D) IsEmpty() bool {
	if m.X == 0.0 && m.Y == 0.0 && m.Z == 0.0 {
		return true
	}

	return false
}

// Scale is used to scale all dimension of a vector
func (m *Vector3D) Scale(factor float64) {
	m.X = factor * m.X
	m.Y = factor * m.Y
	m.Z = factor * m.Z
}

// SquareSum return the sum of squared components
func (m Vector3D) SquareSum() float64 {
	return math.Pow(m.X, 2) + math.Pow(m.Y, 2) + math.Pow(m.Z, 2)
}

// Rotates the point around the X axis by the given angle in radians.
func rotateX(y, z, rad float64) (float64, float64) {
	cosa := math.Cos(rad)
	sina := math.Sin(rad)
	ny := y*cosa - z*sina
	nz := y*sina + z*cosa

	return ny, nz
}

// Rotates the point around the Y axis by the given angle in radians.
func rotateY(x, z, rad float64) (float64, float64) {
	cosa := math.Cos(rad)
	sina := math.Sin(rad)
	nz := z*cosa - x*sina
	nx := z*sina + x*cosa

	return nx, nz
}

// Rotates the point around the Z axis by the given angle in radians.
func rotateZ(x, y, rad float64) (float64, float64) {
	cosa := math.Cos(rad)
	sina := math.Sin(rad)
	nx := x*cosa - y*sina
	ny := x*sina + y*cosa

	return nx, ny
}
