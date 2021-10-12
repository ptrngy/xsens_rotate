package visualizer

import (
	"github.com/Arafatk/glot"
	"github.com/ptrngy/xsens_rotate/pkg/measurement"
	"github.com/ptrngy/xsens_rotate/pkg/parser"
)

type XSensVisualizer struct {
	Parser parser.XSensLogParser
}

func NewXSensVisualizer(parser parser.XSensLogParser) *XSensVisualizer {
	x := XSensVisualizer{
		Parser: parser,
	}

	return &x
}

func getVector3DAsPointGroup(slice []measurement.Vector3D) ([][]float64, [][]float64, [][]float64) {
	xresult := [][]float64{}
	yresult := [][]float64{}
	zresult := [][]float64{}

	indexes := make([]float64, 0)
	xvalues := make([]float64, 0)
	yvalues := make([]float64, 0)
	zvalues := make([]float64, 0)

	for i, v := range slice {
		indexes = append(indexes, float64(i))
		xvalues = append(xvalues, v.X)
		yvalues = append(yvalues, v.Y)
		zvalues = append(zvalues, v.Z)
	}

	xresult = append(xresult, indexes)
	xresult = append(xresult, xvalues)

	yresult = append(yresult, indexes)
	yresult = append(yresult, yvalues)

	zresult = append(zresult, indexes)
	zresult = append(zresult, zvalues)

	return xresult, yresult, zresult
}

func getEulerSliceAsPointGroup(slice []measurement.EulerAngles) ([][]float64, [][]float64, [][]float64) {
	xresult := [][]float64{}
	yresult := [][]float64{}
	zresult := [][]float64{}

	indexes := make([]float64, 0)
	xvalues := make([]float64, 0)
	yvalues := make([]float64, 0)
	zvalues := make([]float64, 0)

	for i, v := range slice {
		indexes = append(indexes, float64(i))
		xvalues = append(xvalues, v.Roll)
		yvalues = append(yvalues, v.Pitch)
		zvalues = append(zvalues, v.Yaw)
	}

	xresult = append(xresult, indexes)
	xresult = append(xresult, xvalues)

	yresult = append(yresult, indexes)
	yresult = append(yresult, yvalues)

	zresult = append(zresult, indexes)
	zresult = append(zresult, zvalues)

	return xresult, yresult, zresult
}

func plotVector3D(slice []measurement.Vector3D, name string) {
	dimensions := 2
	persist := false
	debug := false
	plot, _ := glot.NewPlot(dimensions, persist, debug)
	pointGroupName := "X"
	style := "lines"
	xpoints, ypoints, zpoints := getVector3DAsPointGroup(slice)
	plot.AddPointGroup(pointGroupName, style, xpoints)
	pointGroupName = "Y"
	plot.AddPointGroup(pointGroupName, style, ypoints)
	pointGroupName = "Z"
	plot.AddPointGroup(pointGroupName, style, zpoints)
	// A plot type used to make points/ curves and customize and save them as an image.
	plot.SetTitle("Magneto Plot")
	// Optional: Setting the title of the plot
	plot.SetXLabel("Sample")
	plot.SetYLabel("M")
	// Optional: Setting axis ranges
	plot.SavePlot("output/" + name + ".png")
}

func plotAngles(slice []measurement.EulerAngles, name string) {
	dimensions := 2
	persist := false
	debug := false
	plot, _ := glot.NewPlot(dimensions, persist, debug)
	pointGroupName := "Roll"
	style := "lines"
	xpoints, ypoints, zpoints := getEulerSliceAsPointGroup(slice)
	plot.AddPointGroup(pointGroupName, style, xpoints)
	pointGroupName = "Pitch"
	plot.AddPointGroup(pointGroupName, style, ypoints)
	pointGroupName = "Yaw"
	plot.AddPointGroup(pointGroupName, style, zpoints)
	// A plot type used to make points/ curves and customize and save them as an image.
	plot.SetTitle("Angle Plot")
	// Optional: Setting the title of the plot
	plot.SetXLabel("Sample")
	plot.SetYLabel("Degree")
	// Optional: Setting axis ranges
	plot.SavePlot("output/" + name + ".png")
}

func (x *XSensVisualizer) PlotBasics() {
	plotVector3D(x.Parser.Accelero, "accelero")
	plotVector3D(x.Parser.Gyro, "gyro")
	plotVector3D(x.Parser.Magneto, "magneto")
	plotVector3D(x.Parser.RotatedMagneto, "rotmagneto")
	plotAngles(x.Parser.EulerOri, "fromchip")
}
