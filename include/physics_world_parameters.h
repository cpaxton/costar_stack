#ifndef PHYSICS_WORLD_PARAMETERS_H
#define PHYSICS_WORLD_PARAMETERS_H

// World scaling for better precision, since we are dealing with small objects, scaling will be
// applied to collision shapes, gravity, and transforms in physics engine world. 
// When returning real world results, it will be rescaled back appropriately.

#define SCALING 100
#define GRAVITY_MAGNITUDE 9.807

enum SimulationMode {
	//  RUN THE SIMULATION UNTIL THE NUMBER OF SIMULATION STEP IS REACHED OR ALL OBJECT IS IN DEACTIVATED STATE
	BULLET_DEFAULT, 

	// RUN THE SIMULATION UNTIL SUPPORT GRAPH IS RETRIEVED (SECOND FRAME)
	RESET_VELOCITY_ON_EACH_FRAME, 

	// EVALUATE THE SUPPORT GRAPH AND THEN SET ALL OBJECT VELOCITY TO ZERO
	RUN_UNTIL_HAVE_SUPPORT_GRAPH
};
#endif