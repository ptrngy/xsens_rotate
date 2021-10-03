# Go parameters
GOCMD=GOPRIVATE=$(GOPRIVATE) go
GOPRIVATE=github.org/ptrngy
GOBUILD=$(GOCMD) build
GOCLEAN=$(GOCMD) clean
GOTEST=$(GOCMD) test

all: test build
test: 
	$(GOTEST) -v ./pkg/...
build: 
	$(GOBUILD) -o ./bin/visualize ./cmd/visualize.go
coverage:
	$(GOCOV) ./...
