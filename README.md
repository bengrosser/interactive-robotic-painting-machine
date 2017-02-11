## Interactive Robotic Painting Machine

[Interactive Robotic Painting Machine](http://bengrosser.com/projects/interactive-robotic-painting-machine/) is an artificially intelligent system that creates its own paintings using oil on canvas. While it does so, it listens to its environment and considers what it hears as input into the painting process.

### Dependencies 

This project requires:

* OSC.py
* pyevolve (with some mods)
* EMC2 (now called LinuxCNC)
* some kind of 3-axis machine

Not all of these things are provided here.

### Files

* pga.py --> the primary code for machine operation, sound analysis, etc.
* EMCsocket.py --> socket based communication between pga.py and EMC2
* Decoders.py --> bitstring chromosome description for the genetic algorithm
* arc.py --> arc drawing code

### Project Website

[Interactive Robotic Painting Machine](http://bengrosser.com/projects/interactive-robotic-painting-machine/) 

### Video Documentation

[Interactive Robotic Painting Machine (on Vimeo)](https://vimeo.com/23998286)

