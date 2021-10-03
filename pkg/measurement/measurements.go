package measurement

import (
	"math"
)

type Magnetometer struct {
	X float64
	Y float64
	Z float64
}

// NewMagnetometer creates a new Magnetometer and returns its pointer.
func NewMagnetometer(x, y, z float64) *Magnetometer {
	m := new(Magnetometer)

	m.X = x
	m.Y = y
	m.Z = z

	return m
}

// GetRotated rotates the magnetic coords reading to magnetic north.
func (m *Magnetometer) GetRotated(o Orientation) (float64, float64, float64) {
	e := o.getAsEuler()

	ny, nz := rotateX(m.Y, m.Z, e.Roll)
	nx, nz := rotateY(m.X, nz, e.Pitch)
	nx, ny = rotateZ(nx, ny, e.Yaw)

	return nx, ny, nz
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
