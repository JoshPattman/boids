package main

import (
	"github.com/gopxl/pixel"
)

type NeighbourInfo struct {
	Pos      pixel.Vec
	Fwd      pixel.Vec
	DeltaPos pixel.Vec
	Dist     float64
	Dir      pixel.Vec
}

type FlockingRule interface {
	Force(pos pixel.Vec, ns []NeighbourInfo) (force pixel.Vec, isActive bool) // ns are sorted, closest first
	Range() float64
}

type FlockingAlgorithm struct {
	rules    []FlockingRule
	weights  []float64
	maxRange float64
}

func NewFlockingAlgorithm() *FlockingAlgorithm {
	return &FlockingAlgorithm{
		rules:    make([]FlockingRule, 0),
		weights:  make([]float64, 0),
		maxRange: 0.0,
	}
}

func (f *FlockingAlgorithm) AddRule(rule FlockingRule, weight float64) {
	f.rules = append(f.rules, rule)
	f.weights = append(f.weights, weight)
	f.recalcMaxRange()
}

func (f *FlockingAlgorithm) Force(pos pixel.Vec, ns []NeighbourInfo) pixel.Vec {
	totalForce := pixel.ZV
	totalWeight := 0.0
	for ri, r := range f.rules {
		rf, ra := r.Force(pos, ns)
		if ra {
			totalForce = totalForce.Add(rf.Scaled(f.weights[ri]))
			totalWeight += f.weights[ri]
		}
	}
	if totalWeight == 0.0 {
		return pixel.ZV
	} else {
		return totalForce.Scaled(1.0 / totalWeight)
	}
}

func (f *FlockingAlgorithm) Range() float64 {
	return f.maxRange
}

func (f *FlockingAlgorithm) recalcMaxRange() {
	maxRange := 0.0
	for _, r := range f.rules {
		rng := r.Range()
		if rng > maxRange {
			maxRange = rng
		}
	}
	f.maxRange = maxRange
}
