package main

import "github.com/gopxl/pixel"

type SeparationRule struct {
	AvoidRange               float64
	DeactivateOnNoNeighbours bool
	OnlyUseClosest           bool
}

func (r *SeparationRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	totalDir := pixel.ZV
	for ni, n := range ns {
		if n.Dist > r.AvoidRange || ni > 0 && r.OnlyUseClosest {
			break
		}
		totalDir = totalDir.Add(n.Dir.Scaled(-1))
	}
	tdLen := totalDir.Len()
	if tdLen > 0 {
		return totalDir.Scaled(1.0 / tdLen), true
	} else {
		return pixel.ZV, !r.DeactivateOnNoNeighbours
	}
}

func (r *SeparationRule) Range() float64 { return r.AvoidRange }

type AlignmentRule struct {
	AlignRange               float64
	PerformNormalisation     bool // If this is on, the output will always be unit len. If it is off, it is between 0 and unit
	DeactivateOnNoNeighbours bool
}

func (r *AlignmentRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	totalFwd := pixel.ZV
	for _, n := range ns {
		if n.Dist > r.AlignRange {
			break
		}
		totalFwd = totalFwd.Add(n.Fwd)
	}
	tfLen := totalFwd.Len()
	if tfLen > 0 {
		if r.PerformNormalisation {
			return totalFwd.Scaled(1.0 / tfLen), true
		} else {
			return totalFwd.Scaled(1.0 / float64(len(ns))), true
		}
	} else {
		return pixel.ZV, !r.DeactivateOnNoNeighbours
	}
}

func (r *AlignmentRule) Range() float64 { return r.AlignRange }

type CohesionRule struct {
	CohereRange              float64
	DeactivateOnNoNeighbours bool
}

func (r *CohesionRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	totalDir := pixel.ZV
	for _, n := range ns {
		if n.Dist > r.CohereRange {
			break
		}
		totalDir = totalDir.Add(n.Dir)
	}
	tdLen := totalDir.Len()
	if tdLen > 0 {
		return totalDir.Scaled(1.0 / tdLen), true
	} else {
		return pixel.ZV, !r.DeactivateOnNoNeighbours
	}
}

func (r *CohesionRule) Range() float64 { return r.CohereRange }

type TargetingRule struct {
	TargetPos pixel.Vec
}

func (r *TargetingRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	return r.TargetPos.Sub(pos).Unit(), true
}

func (r *TargetingRule) Range() float64 { return 0 }

type AvoidanceRule struct {
	TargetPos  pixel.Vec
	TargetDist float64
}

func (r *AvoidanceRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	dist := r.TargetPos.Sub(pos).Len()
	if dist < r.TargetDist {
		return pos.Sub(r.TargetPos).Unit(), true
	}
	return pixel.ZV, false
}

func (r *AvoidanceRule) Range() float64 { return 0 }

type MultiAvoidanceRule struct {
	TargetPosses []pixel.Vec
	TargetDist   float64
}

func (r *MultiAvoidanceRule) Force(pos pixel.Vec, ns []NeighbourInfo) (pixel.Vec, bool) {
	totalDir := pixel.ZV
	for _, tp := range r.TargetPosses {
		dist := tp.Sub(pos).Len()
		if dist < r.TargetDist {
			totalDir = totalDir.Add(pos.Sub(tp).Unit())
		}
	}
	if totalDir.Len() > 0 {
		return totalDir.Unit(), true
	}
	return pixel.ZV, false
}

func (r *MultiAvoidanceRule) Range() float64 { return 0 }
