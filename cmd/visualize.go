package main

import (
	"flag"
	"fmt"
	"log"

	"github.com/ptrngy/xsens_rotate/pkg/parser"
	"github.com/ptrngy/xsens_rotate/pkg/visualizer"
)

type config struct {
	Infile     string
	Parser     parser.XSensLogParser
	Visualizer visualizer.XSensVisualizer
}

var c config

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

	c.Visualizer = *visualizer.NewXSensVisualizer(c.Parser)
	c.Visualizer.PlotBasics()
}
