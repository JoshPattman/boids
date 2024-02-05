package main

import (
	"github.com/gopxl/pixel"
)

type Animator struct {
	sprites          map[string]*pixel.Sprite
	animations       map[string][]string
	animationsTimes  map[string]float64
	currentTime      float64
	currentAnimation string
}

func NewAnimator(pic pixel.Picture, spriteWidth float64, spritesPosses map[string]pixel.Vec, animations map[string][]string, animationsTimes map[string]float64) *Animator {
	sprites := make(map[string]*pixel.Sprite)
	for name, pos := range spritesPosses {
		sprites[name] = pixel.NewSprite(pic, pixel.Rect{Min: pos.Scaled(spriteWidth), Max: pos.Add(pixel.V(1, 1)).Scaled(spriteWidth)})
	}
	return &Animator{
		sprites:         sprites,
		animations:      animations,
		currentTime:     0,
		animationsTimes: animationsTimes,
	}
}

func (a *Animator) Step(dt float64) {
	a.currentTime += dt / a.animationsTimes[a.currentAnimation]
}

func (a *Animator) CurrentSprite() *pixel.Sprite {
	ca := a.animations[a.currentAnimation]
	iF := a.currentTime * float64(len(ca))
	i := int(iF) % len(ca)
	return a.sprites[ca[i]]
}

func (a *Animator) Play(anim string) {
	a.currentAnimation = anim
	a.currentTime = 0
}
