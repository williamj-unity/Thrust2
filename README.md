
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

# Grav Grid 
- start in the GravGridBuilder.cs file to see basic implementation of cloth mesh thingy. It's a bit of a mess due to a major feature sprint to get tessalation and the infinite grid working but the general job flow can be found in the Update loop.
