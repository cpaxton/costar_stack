#ifndef SCENE_PHYSICS_PENALTY_H
#define SCENE_PHYSICS_PENALTY_H

#include <vector>
#include <cmath>

#include <btBulletDynamicsCommon.h>

#include "physics_world_parameters.h"
#include "scene_physics_support.h"

struct MovementComponent
{
	btVector3 linear_;
	btVector3 angular_;

	void setValue(const btVector3 &linear, const btVector3 &angular )
	{
		this->linear_ = linear;
		this->angular_ = angular;
	}
};

struct ObjectPenaltyParameters
{
	btScalar maximum_angular_acceleration_;
	btScalar angular_acceleration_weight_, translational_acceleration_weight_;
	btScalar penetration_constant_;
	btScalar volume_;

	ObjectPenaltyParameters() : maximum_angular_acceleration_(0.), 
		angular_acceleration_weight_(1.), translational_acceleration_weight_(1.), volume_(0.)
	{}

};

double calculateStabilityPenalty(const MovementComponent &acceleration,
	const ObjectPenaltyParameters &penalty_params, const double &gravity_magnitude);

btScalar getObjectMaximumGravityTorqueLength(const btCollisionShape &object_shape);

btScalar getObjectMaximumAngularAcceleration(const btCollisionShape &object_shape, const btScalar &mass, const btVector3 &inertia);

btScalar getObjectSupportContribution(const scene_support_vertex_properties &support_graph_vertex);

btScalar getObjectCollisionPenalty(const scene_support_vertex_properties &support_graph_vertex);

btScalar dataProbabilityScale(const btScalar &hypothesis_confidence, const btScalar &max_confidence = 0.5);

#endif