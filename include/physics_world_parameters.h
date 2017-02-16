#ifndef PHYSICS_WORLD_PARAMETERS_H
#define PHYSICS_WORLD_PARAMETERS_H

// World scaling for better precision, since we are dealing with small objects, scaling will be
// applied to collision shapes, gravity, and transforms in physics engine world. 
// When returning real world results, it will be rescaled back appropriately.

#define SCALING 100
#define GRAVITY_MAGNITUDE 9.807
#endif