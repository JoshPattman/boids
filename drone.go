package main

import (
	"math"
	"sort"
	"sync"

	"github.com/gopxl/pixel"
)

type Drone struct {
	Pos          pixel.Vec
	Vel          pixel.Vec
	FlockingAlgo *FlockingAlgorithm
	MaxVel       float64
	MaxAcc       float64
}

func dronesToNeighbourInfo(d *Drone, f *Flock, maxRange float64) []NeighbourInfo {
	ns := make([]NeighbourInfo, 0)
	droneGidPos := ToGrid(d.Pos, f.GridCellWidth)
	gridRadius := roundUp(maxRange / f.GridCellWidth)
	for xo := -gridRadius; xo <= gridRadius; xo++ {
		for yo := -gridRadius; yo <= gridRadius; yo++ {
			gp := droneGidPos.Add(GridPos{xo, yo})
			if dis, ok := f.Grid[gp]; ok {
				for _, d2i := range dis {
					d2 := f.Drones[d2i]
					if d == d2 {
						continue
					}
					delta := d2.Pos.Sub(d.Pos)
					dist := delta.Len()
					if dist <= maxRange {
						dir := delta.Scaled(1.0 / dist)
						neighbourInfo := NeighbourInfo{
							Pos:      d2.Pos,
							Fwd:      d2.Vel.Unit(),
							DeltaPos: delta,
							Dist:     dist,
							Dir:      dir,
						}
						ns = append(ns, neighbourInfo)
					}
				}
			}
		}
	}
	sort.Slice(ns, func(i, j int) bool {
		return ns[i].Dist < ns[j].Dist
	})
	return ns
}

type GridPos struct {
	X int
	Y int
}

func roundUp(x float64) int {
	xr := math.Round(x)
	if xr < x {
		return int(xr) + 1
	} else {
		return int(xr)
	}
}

func round(x float64) int {
	return int(math.Round(x))
}

func ToGrid(v pixel.Vec, gridCellSize float64) GridPos {
	v2 := v.Scaled(1.0 / gridCellSize)
	return GridPos{
		X: round(v2.X),
		Y: round(v2.Y),
	}
}

func (g GridPos) Add(g2 GridPos) GridPos {
	return GridPos{g.X + g2.X, g.Y + g2.Y}
}

type Flock struct {
	Drones           []*Drone
	GridCellWidth    float64
	Grid             map[GridPos][]int
	Stage1Forces     []pixel.Vec
	Stage1UpdateChan chan int
	Stage2UpdateChan chan int
	ParrWg           *sync.WaitGroup
}

func flockWorker(f *Flock) {
	for {
		select {
		case di := <-f.Stage1UpdateChan:
			ns := dronesToNeighbourInfo(f.Drones[di], f, f.Drones[di].FlockingAlgo.Range())
			if len(ns) > 10 {
				ns = ns[:10]
			}
			force := f.Drones[di].FlockingAlgo.Force(f.Drones[di].Pos, ns)
			f.Stage1Forces[di] = force
			f.ParrWg.Done()
		case di := <-f.Stage2UpdateChan:
			d := f.Drones[di]
			d.Vel = d.Vel.Add(f.Stage1Forces[di].Scaled(d.MaxAcc / 60.0))
			if d.Vel.Len() > d.MaxVel {
				d.Vel = d.Vel.Unit().Scaled(d.MaxVel)
			}
			d.Pos = d.Pos.Add(d.Vel.Scaled(1.0 / 60.0))
			f.ParrWg.Done()
		}

	}
}

func NewFlock(ds []*Drone, workers int, gridWidth float64) *Flock {
	f := &Flock{
		Drones:           ds,
		GridCellWidth:    gridWidth,
		Stage1Forces:     make([]pixel.Vec, len(ds)),
		Stage1UpdateChan: make(chan int, len(ds)),
		Stage2UpdateChan: make(chan int, len(ds)),
		ParrWg:           &sync.WaitGroup{},
	}
	for i := 0; i < workers; i++ {
		// Workers to update drones
		for i := 0; i < workers; i++ {
			go flockWorker(f)
		}
	}
	return f
}

func (f *Flock) Update() {
	// Rebuild grid
	f.Grid = make(map[GridPos][]int)
	for di, d := range f.Drones {
		gridPos := ToGrid(d.Pos, f.GridCellWidth)
		if _, ok := f.Grid[gridPos]; ok {
			f.Grid[gridPos] = append(f.Grid[gridPos], di)
		} else {
			f.Grid[gridPos] = []int{di}
		}
	}
	// Update neighbours and forces
	f.ParrWg.Add(len(f.Drones))
	for i := range f.Drones {
		f.Stage1UpdateChan <- i
	}
	f.ParrWg.Wait()
	// Update kinematics
	f.ParrWg.Add(len(f.Drones))
	for i := range f.Drones {
		f.Stage2UpdateChan <- i
	}
	f.ParrWg.Wait()
}
