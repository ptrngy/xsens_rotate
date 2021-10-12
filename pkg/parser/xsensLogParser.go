package parser

import (
	"encoding/csv"
	"errors"
	"fmt"
	"math"
	"os"
	"strconv"

	"github.com/ptrngy/xsens_rotate/pkg/measurement"
)

type XSensLogParser struct {
	Path           string
	Header         []string
	Accelero       []measurement.Vector3D
	Gyro           []measurement.Vector3D
	Magneto        []measurement.Vector3D
	EulerOri       []measurement.EulerAngles
	RotatedMagneto []measurement.Vector3D
}

// NewXSensLogParser is the constructor.
func NewXSensLogParser(path string) *XSensLogParser {
	x := XSensLogParser{
		Path:           path,
		Header:         make([]string, 0),
		Accelero:       make([]measurement.Vector3D, 0),
		Gyro:           make([]measurement.Vector3D, 0),
		Magneto:        make([]measurement.Vector3D, 0),
		EulerOri:       make([]measurement.EulerAngles, 0),
		RotatedMagneto: make([]measurement.Vector3D, 0),
	}

	return &x
}

// GetFloatVector3D returns the float64 representation of 3 consecutive values from XSens log
func GetFloatVector3D(chunks []string, startidx int) (measurement.Vector3D, error) {
	result := measurement.Vector3D{X: 0.0, Y: 0.0, Z: 0.0}

	for i := 0; i < 3; i++ {
		value, err := strconv.ParseFloat(chunks[startidx+i], 64)
		if err != nil {
			return result, err
		}

		switch i {
		case 0:
			result.X = value
		case 1:
			result.Y = value
		case 2:
			result.Z = value
		}
	}

	return result, nil
}

// GetFloatEuler returns the float64 representation of 3 consecutive values from XSens log
func GetFloatEuler(chunks []string, startidx int) (measurement.EulerAngles, error) {
	result := measurement.EulerAngles{Roll: 0.0, Pitch: 0.0, Yaw: 0.0}

	for i := 0; i < 3; i++ {
		value, err := strconv.ParseFloat(chunks[startidx+i], 64)
		if err != nil {
			return result, err
		}

		switch i {
		case 0:
			result.Roll = value * math.Pi / 180.0
		case 1:
			result.Pitch = value * math.Pi / 180.0
		case 2:
			result.Yaw = value * math.Pi / 180.0
		}
	}

	return result, nil
}

// GetFloatQuaternion returns the float64 representation of 3 consecutive values from XSens log
func GetFloatQuaternion(chunks []string, startidx int) (measurement.Quaternion, error) {
	result := measurement.Quaternion{Q0: 1.0, Q1: 0.0, Q2: 0.0, Q3: 0.0}

	for i := 0; i < 3; i++ {
		value, err := strconv.ParseFloat(chunks[startidx+i], 64)
		if err != nil {
			return result, err
		}

		switch i {
		case 0:
			result.Q0 = value
		case 1:
			result.Q1 = value
		case 2:
			result.Q2 = value
		case 3:
			result.Q3 = value
		}
	}

	return result, nil
}

// Parse is used to parse the given file.
func (x *XSensLogParser) Parse() (err error) {
	// Opening the file
	logfile, err := os.Open(x.Path)
	if err != nil {
		return err
	}

	defer func() {
		cerr := logfile.Close()
		if cerr != nil {
			err = fmt.Errorf("%w, %v", err, cerr)
		}
	}()

	reader := csv.NewReader(logfile)
	reader.Comma = '\t' // Use tab-delimited instead of comma

	reader.FieldsPerRecord = -1

	data, err := reader.ReadAll()
	if err != nil {
		return err
	}

	isHeader := true
	accStartIdx, gyrStartIdx, magStartIdx, eulerStartIdx := -1, -1, -1, -1

	for _, chunks := range data {
		if len(chunks) > 1 {
			if isHeader {
				x.Header = chunks

				magStartIdx = indexOf("Mag_X", x.Header)
				eulerStartIdx = indexOf("Roll", x.Header)
				accStartIdx = indexOf("Acc_X", x.Header)
				gyrStartIdx = indexOf("Gyr_X", x.Header)

				if magStartIdx == -1 || eulerStartIdx == -1 || accStartIdx == -1 || gyrStartIdx == -1 {
					err = errors.New("Required fields not found in file")
				}

				isHeader = false
			} else {
				if chunks[magStartIdx] != "" {
					a, err := GetFloatVector3D(chunks, accStartIdx)
					if err != nil {
						return err
					}
					x.Accelero = append(x.Accelero, a)

					g, err := GetFloatVector3D(chunks, gyrStartIdx)
					if err != nil {
						return err
					}
					x.Gyro = append(x.Gyro, g)

					m, err := GetFloatVector3D(chunks, magStartIdx)
					if err != nil {
						return err
					}
					x.Magneto = append(x.Magneto, m)

					e, err := GetFloatEuler(chunks, eulerStartIdx)
					if err != nil {
						return err
					}
					x.EulerOri = append(x.EulerOri, e)

					nx, ny, nz := m.GetRotatedEuler(e)
					x.RotatedMagneto = append(x.RotatedMagneto, measurement.Vector3D{X: nx, Y: ny, Z: nz})
				}
			}
		}
	}

	return err
}

func indexOf(element string, data []string) int {
	for k, v := range data {
		if element == v {
			return k
		}
	}
	return -1 //not found.
}
