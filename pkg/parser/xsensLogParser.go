package parser

import (
	"encoding/csv"
	"errors"
	"fmt"
	"os"
	"strconv"

	"github.com/ptrngy/xsens_rotate/pkg/measurement"
)

type XSensLogParser struct {
	Path           string
	Header         []string
	Magneto        []measurement.Magnetometer
	Ori            []measurement.Orientation
	EulerOri       []measurement.EulerAngles
	RotatedMagneto []measurement.Magnetometer
}

// NewXSensLogParser is the constructor.
func NewXSensLogParser(path string) *XSensLogParser {
	x := XSensLogParser{
		Path:           path,
		Header:         make([]string, 0),
		Magneto:        make([]measurement.Magnetometer, 0),
		Ori:            make([]measurement.Orientation, 0),
		EulerOri:       make([]measurement.EulerAngles, 0),
		RotatedMagneto: make([]measurement.Magnetometer, 0),
	}

	return &x
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
	magStartIdx, oriStartIdx, eulerStartIdx := -1, -1, -1

	for _, chunks := range data {
		if len(chunks) > 1 {
			if isHeader {
				x.Header = chunks

				magStartIdx = indexOf("Mag_X", x.Header)
				oriStartIdx = indexOf("Quat_q0", x.Header)
				eulerStartIdx = indexOf("Roll", x.Header)

				if magStartIdx == -1 || oriStartIdx == -1 || eulerStartIdx == -1 {
					err = errors.New("Required fields not found in file")
				}

				isHeader = false
			} else {
				if chunks[magStartIdx] != "" {
					mx, err := strconv.ParseFloat(chunks[magStartIdx], 64)
					if err != nil {
						return err
					}
					my, err := strconv.ParseFloat(chunks[magStartIdx+1], 64)
					if err != nil {
						return err
					}
					mz, err := strconv.ParseFloat(chunks[magStartIdx+2], 64)
					if err != nil {
						return err
					}

					m := measurement.Magnetometer{X: mx, Y: my, Z: mz}
					x.Magneto = append(x.Magneto, m)

					q0, err := strconv.ParseFloat(chunks[oriStartIdx], 64)
					if err != nil {
						return err
					}
					q1, err := strconv.ParseFloat(chunks[oriStartIdx+1], 64)
					if err != nil {
						return err
					}
					q2, err := strconv.ParseFloat(chunks[oriStartIdx+2], 64)
					if err != nil {
						return err
					}
					q3, err := strconv.ParseFloat(chunks[oriStartIdx+3], 64)
					if err != nil {
						return err
					}

					o := measurement.Orientation{Q0: q0, Q1: q1, Q2: q2, Q3: q3}
					x.Ori = append(x.Ori, o)

					roll, err := strconv.ParseFloat(chunks[eulerStartIdx], 64)
					if err != nil {
						return err
					}
					pitch, err := strconv.ParseFloat(chunks[eulerStartIdx+1], 64)
					if err != nil {
						return err
					}
					yaw, err := strconv.ParseFloat(chunks[eulerStartIdx+2], 64)
					if err != nil {
						return err
					}

					x.EulerOri = append(x.EulerOri, measurement.EulerAngles{Roll: roll, Pitch: pitch, Yaw: yaw})

					nx, ny, nz := m.GetRotated(o)
					x.RotatedMagneto = append(x.RotatedMagneto, measurement.Magnetometer{X: nx, Y: ny, Z: nz})
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
