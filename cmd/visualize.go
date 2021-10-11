package main

import (
	"flag"
	"fmt"
	"log"

	"github.com/Arafatk/glot"
	"github.com/ptrngy/xsens_rotate/pkg/measurement"
	"github.com/ptrngy/xsens_rotate/pkg/parser"
)

type config struct {
	Infile string
	Parser parser.XSensLogParser
}

var c config

func getMagnetoSliceAsPointGroup(slice []measurement.Magnetometer) ([][]float64, [][]float64, [][]float64) {
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

func getAngleSliceAsPointGroup(slice []measurement.EulerAngles) ([][]float64, [][]float64, [][]float64) {
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

func plotMagneto(slice []measurement.Magnetometer, name string) {
	dimensions := 2
	persist := false
	debug := false
	plot, _ := glot.NewPlot(dimensions, persist, debug)
	pointGroupName := "X"
	style := "lines"
	xpoints, ypoints, zpoints := getMagnetoSliceAsPointGroup(slice)
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
	xpoints, ypoints, zpoints := getAngleSliceAsPointGroup(slice)
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

func main() {
	flag.StringVar(&c.Infile, "input", "", "XSens log file to process. Extensions supported: .txt")
	flag.Parse()

	if c.Infile == "" {
		log.Fatalf("no log file defined")
	}

	c.Parser = *parser.NewXSensLogParser(c.Infile)

	err := c.Parser.Parse()
	if err != nil {
		log.Fatalf("unable to parse file: %s\n", err.Error())
	}

	fmt.Println("Processed ", len(c.Parser.Magneto), " measurements")

	plotMagneto(c.Parser.Magneto, "magneto")
	plotMagneto(c.Parser.RotatedMagneto, "rotmagneto")
	plotAngles(c.Parser.EulerFromQuat, "fromquaternion")
	plotAngles(c.Parser.EulerFromQuat, "fromchip")
}
