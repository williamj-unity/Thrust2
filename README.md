
# Thrust 2

A space adventure. 

## Getting started

1) Pull this repo
2) Open at the repo root with Unity version 2020.1a1 or higher. 
3) Navigate to Assets > Scenes > ControlsPlayground
4) Click the play button at the top


### Controls:


W = add thrust in ship's forward direction  
A = add thrust in ship's left direction  
S = add thrust in ship's backward direction  
D = add thrust in ship's right direction  

Q = add rotational thrust left  
E = add rotational thrust right  

Z = Activate/Deactivate dampers (dampers will counter-act ship's forces making it easier to pilot, however, you may just want to coast.)  

Scroll wheel = zoom in and out on map  


## Short Term To-Dos:

- camera should animate/follow the ship
- camera should animate/zoom out as the ship increases velocity

## Bigger To-Dos:
- orbital mechanics for planet and ship interactions
- orbital mechanics for planet-planet, planet-moon, star-planet, intertactions???
	- maybe some way to "fake" this as the math and physics are overly complicated for capturing the feeling desired here
- start planet-side 2D sidescrolling ship controls
- start planet-side 2D sidescrolling character controls

## Biggest To-Dos:
- space to planet-side transistion
-- this is the biggest techinical challenge. How do we seamlessly load an entire 2D level? How do we seamlessly transtition to planet-side 2D sidescrolling controls? visually speaking, how do we transition seamlessly to the space view to this side scrolling view in a convincing way. it would be cool to see the horizon of the planet as the ship descends planet-side
