package main

import (
	"bytes"
	_ "embed"
	"flag"
	"fmt"
	"image/png"
	"math"
	"math/rand"
	"runtime"
	"time"

	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/pixelgl"
	"github.com/gopxl/pixel/text"
	"golang.org/x/image/font/basicfont"
)

//go:embed assets/fishies.png
var dronePng []byte

func main() {
	pixelgl.Run(run)
}

func run() {
	// Parse the args
	var numDrones int
	flag.IntVar(&numDrones, "n", 1000, "Number of drones to simulate")
	flag.Parse()

	// Start the window
	cfg := pixelgl.WindowConfig{
		Title:     "Flocking",
		Bounds:    pixel.R(0, 0, 1024, 1024),
		Maximized: true,
		Resizable: true,
		VSync:     true,
		Monitor:   pixelgl.Monitors()[0],
	}
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	// Create the preyFlocking algorithm
	preyFlocking := NewFlockingAlgorithm()
	preyFlocking.AddRule(&CohesionRule{
		CohereRange:              15,
		DeactivateOnNoNeighbours: false,
	}, 0.3)
	preyFlocking.AddRule(&SeparationRule{
		AvoidRange:               5,
		DeactivateOnNoNeighbours: true,
		OnlyUseClosest:           true,
	}, 2.0)
	preyFlocking.AddRule(&AlignmentRule{
		AlignRange:               10,
		PerformNormalisation:     false,
		DeactivateOnNoNeighbours: true,
	}, 1.0)
	preyFlocking.AddRule(&TargetingRule{
		TargetPos: pixel.ZV,
	}, 0.2)
	ar := &MultiAvoidanceRule{
		TargetPosses: []pixel.Vec{},
		TargetDist:   25,
	}
	preyFlocking.AddRule(ar, 2.0)

	predatorFlocking := NewFlockingAlgorithm()
	tr := &TargetingRule{}
	predatorFlocking.AddRule(tr, 0.3)
	predatorFlocking.AddRule(&CohesionRule{
		CohereRange:              15,
		DeactivateOnNoNeighbours: false,
	}, 0.3)
	predatorFlocking.AddRule(&SeparationRule{
		AvoidRange:               5,
		DeactivateOnNoNeighbours: true,
		OnlyUseClosest:           true,
	}, 2.0)
	predatorFlocking.AddRule(&AlignmentRule{
		AlignRange:               10,
		PerformNormalisation:     false,
		DeactivateOnNoNeighbours: true,
	}, 1.0)

	// Initialise the drones and spawn the workers
	preyDs := make([]*Drone, numDrones)
	for i := range preyDs {
		preyDs[i] = &Drone{
			Pos:          pixel.V(math.Sqrt(rand.Float64())*150, 0).Rotated(rand.Float64() * math.Pi * 2),
			Vel:          pixel.V(rand.Float64()*10, 0).Rotated(rand.Float64() * math.Pi * 2),
			MaxVel:       rand.Float64()*2.5 + 10,
			MaxAcc:       100,
			FlockingAlgo: preyFlocking,
		}
	}
	predDs := make([]*Drone, 15)
	for i := range predDs {
		predDs[i] = &Drone{
			Pos:          pixel.V(math.Sqrt(rand.Float64())*150, 0).Rotated(rand.Float64() * math.Pi * 2),
			Vel:          pixel.V(rand.Float64()*10, 0).Rotated(rand.Float64() * math.Pi * 2),
			MaxVel:       rand.Float64()*10.0 + 10,
			MaxAcc:       100,
			FlockingAlgo: predatorFlocking,
		}
	}
	preyFlock := NewFlock(preyDs, runtime.NumCPU(), 15)
	predatorFlock := NewFlock(predDs, runtime.NumCPU(), 15)

	// Stuff for drawing drones
	img, err := png.Decode(bytes.NewBuffer(dronePng))
	if err != nil {
		panic(err)
	}
	pic := pixel.PictureDataFromImage(img)
	//droneSprite := pixel.NewSprite(pic, pic.Bounds())
	droneBatch := pixel.NewBatch(&pixel.TrianglesData{}, pic)
	droneAnimator := NewAnimator(pic, 32,
		map[string]pixel.Vec{
			"swim.1": pixel.V(0, 0),
			"swim.2": pixel.V(1, 0),
			"swim.3": pixel.V(2, 0),
		},
		map[string][]string{
			"swim": {"swim.1", "swim.2", "swim.3"},
		},
		map[string]float64{
			"swim": 0.5,
		},
	)
	droneAnimator.Play("swim")

	// Frame counter stuff
	atlas := text.NewAtlas(basicfont.Face7x13, text.ASCII)
	txt := text.New(pixel.ZV, atlas)
	lastFrameTime := time.Now()
	counter := 0

	// Stuff for moving the camera
	pixelsPerMeter := 10.0
	metersOffset := pixel.ZV
	currentCameraVelocity := pixel.ZV
	currentCameraZoomVelocity := 1.0

	// Stuff for predator drone
	currentPredatorTarget := preyFlock.Drones[1]
	predatorTargetTimer := 0

	/*prof, _ := os.Create("profiling.prof")
	pprof.StartCPUProfile(prof)*/

	for !win.Closed() && !win.JustPressed(pixelgl.KeyEscape) {
		// Update window to get new keypresses and such
		win.Update()
		// Clear window background
		win.Clear(pixel.RGB(0, 0, 0))

		// Move the camera around
		zoomSpeed := 1.05
		moveSpeed := 80.0

		targetCameraZoomVelocity := 1.0
		if win.Pressed(pixelgl.KeyQ) {
			targetCameraZoomVelocity /= zoomSpeed
		} else if win.Pressed(pixelgl.KeyE) {
			targetCameraZoomVelocity *= zoomSpeed
		}
		currentCameraZoomVelocity = 0.9*currentCameraZoomVelocity + 0.1*targetCameraZoomVelocity
		pixelsPerMeter *= currentCameraZoomVelocity

		targetCameraVelocity := pixel.ZV
		if win.Pressed(pixelgl.KeyA) {
			targetCameraVelocity = targetCameraVelocity.Add(pixel.V(moveSpeed, 0))
		} else if win.Pressed(pixelgl.KeyD) {
			targetCameraVelocity = targetCameraVelocity.Add(pixel.V(-moveSpeed, 0))
		}

		if win.Pressed(pixelgl.KeyW) {
			targetCameraVelocity = targetCameraVelocity.Add(pixel.V(0, -moveSpeed))
		} else if win.Pressed(pixelgl.KeyS) {
			targetCameraVelocity = targetCameraVelocity.Add(pixel.V(0, moveSpeed))
		}

		currentCameraVelocity = pixel.Lerp(currentCameraVelocity, targetCameraVelocity, 0.1)
		metersOffset = metersOffset.Add(currentCameraVelocity.Scaled(1.0 / 60))

		// Update the frame counter
		if counter > 20 {
			newFrameTime := time.Now()
			deltaFrameTime := newFrameTime.Sub(lastFrameTime)
			lastFrameTime = newFrameTime
			deltaFrameSeconds := deltaFrameTime.Seconds() / 20
			txt.Clear()
			txt.WriteString(fmt.Sprintf("FPS %.1f | Boids %d", 1.0/deltaFrameSeconds, len(preyFlock.Drones)))
			counter = 0
		}
		counter++

		// Update the predator target
		predatorTargetTimer++
		if predatorTargetTimer > 240 {
			predatorTargetTimer = 0
			currentPredatorTarget = preyFlock.Drones[rand.Intn(len(preyFlock.Drones)-1)+1]
		}
		tr.TargetPos = currentPredatorTarget.Pos

		// Update the position to avoid
		ar.TargetPosses = []pixel.Vec{}
		for _, d := range predatorFlock.Drones {
			ar.TargetPosses = append(ar.TargetPosses, d.Pos)
		}
		ar.TargetPosses = append(ar.TargetPosses, win.MousePosition().Sub(win.Bounds().Center()).Scaled(1.0/pixelsPerMeter))

		// Update the drones (in parallell)
		preyFlock.Update()
		predatorFlock.Update()

		// Update the animators
		droneAnimator.Step(1.0 / 60)
		droneSprite := droneAnimator.CurrentSprite()

		// Draw the drones (in one batch for efficiency)
		droneBatch.Clear()
		for _, d := range preyFlock.Drones {
			drawMat := pixel.IM.Scaled(pixel.ZV, pixelsPerMeter*1.0/droneSprite.Frame().W())
			drawMat = drawMat.Rotated(pixel.ZV, d.Vel.Angle()-math.Pi/2)
			drawMat = drawMat.Moved(d.Pos.Add(metersOffset).Scaled(pixelsPerMeter)).Moved(win.Bounds().Center())
			droneSprite.Draw(droneBatch, drawMat)
		}
		for _, d := range predatorFlock.Drones {
			drawMat := pixel.IM.Scaled(pixel.ZV, pixelsPerMeter*1.0/droneSprite.Frame().W())
			drawMat = drawMat.Rotated(pixel.ZV, d.Vel.Angle()-math.Pi/2)
			drawMat = drawMat.Moved(d.Pos.Add(metersOffset).Scaled(pixelsPerMeter)).Moved(win.Bounds().Center())
			droneSprite.DrawColorMask(droneBatch, drawMat, pixel.RGB(1, 0, 0))
		}
		droneBatch.Draw(win)

		// Draw the frame counter text
		txt.Draw(win, pixel.IM)
	}

	//pprof.StopCPUProfile()
}
