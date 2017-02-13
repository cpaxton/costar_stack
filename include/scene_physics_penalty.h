#ifndef SCENE_PHYSICS_PENALTY_H
#define SCENE_PHYSICS_PENALTY_H

#include <vector>
#include <cmath>

#include <btBulletDynamicsCommon.h>

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
	double maximum_angular_acceleration_;
	double angular_acceleration_weight_, translational_acceleration_weight_;
	double penetration_constant_;
	double volume_;

	ObjectPenaltyParameters() : maximum_angular_acceleration_(0.), 
		angular_acceleration_weight_(1.), translational_acceleration_weight_(1.), volume_(0.)
	{}

};

double getTranslationalPenalty (const btVector3 &t_acceleration, const double &gravity_magnitude);

double getAngularPenalty(const btVector3 &rot_acceleration, const double &max_object_angular_acceleration);

double calculateStabilityPenalty(const MovementComponent &acceleration,
	const ObjectPenaltyParameters &penalty_params, const double &gravity_magnitude);

double getObjectMaximumAngularAcceleration(const btCollisionShape &object_shape);

#endif