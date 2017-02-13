#include "scene_physics_penalty.h"
#include <iostream>

double getTranslationalPenalty (const btVector3 &t_acceleration, const double &gravity_magnitude)
{
	// btVector3 t_acceleration = object.getInvMass() * object.getTotalForce();
	// std::cerr << "Lin: " << t_acceleration.x() << ", " << t_acceleration.y() << ", " << t_acceleration.z() << std::endl;
	return t_acceleration.norm() / gravity_magnitude;
}

double getAngularPenalty(const btVector3 &rot_acceleration, const double &max_object_angular_acceleration)
{
	// btVector3 rot_acceleration = object.getInvInertiaTensorWorld() * object.getTotalTorque();
	// btVector3 torque = object.getTotalTorque();
	// std::cerr << "Ang: " <<rot_acceleration.x() << ", " << rot_acceleration.y() << ", " << rot_acceleration.z() << std::endl;
	// std::cerr << rot_acceleration.norm() << " " << max_object_angular_acceleration << std::endl;
	return rot_acceleration.norm() / max_object_angular_acceleration;
}

double calculateStabilityPenalty(const MovementComponent &acceleration,
	const ObjectPenaltyParameters &penalty_params, const double &gravity_magnitude)
{
	double translation_penalty = getTranslationalPenalty(acceleration.linear_, gravity_magnitude),
		angular_penalty = getAngularPenalty(acceleration.angular_, penalty_params.maximum_angular_acceleration_);
	std::cerr << "lin penalty: " << translation_penalty << ", ang penalty: " << angular_penalty << std::endl;
	return exp(- (penalty_params.translational_acceleration_weight_ * translation_penalty + 
					penalty_params.angular_acceleration_weight_ * angular_penalty) );
}

double getObjectMaximumAngularAcceleration(const btCollisionShape &object_shape)
{
	// Calculate the maximum acceleration that can happen for a particular object under gravity
	std::vector<btVector3> object_corner_point_list;
	object_corner_point_list.reserve(150);

	// get all points in the compound object
	if (object_shape.isCompound())
	{
		const btCompoundShape * object_compound_shape = (btCompoundShape *)&object_shape;

		for (int i = 0; i < object_compound_shape->getNumChildShapes(); i++)
		{
			const btCollisionShape * child_shape = object_compound_shape->getChildShape(i);
			const btTransform child_transform = object_compound_shape->getChildTransform(i);
			if (child_shape->isConvex())
			{
				const btConvexHullShape * child_hull_shape = (btConvexHullShape *)child_shape;
				for (int pts = 0; pts < child_hull_shape->getNumPoints(); pts++)
				{
					btVector3 corner_points = child_hull_shape->getScaledPoint(pts);
					object_corner_point_list.push_back(child_transform * corner_points);
				}
			}
		}
	}

	// get the farthest distance from the object center of gravity
	// const btVector3 cog = object.getCenterOfMassPosition();
	const btVector3 cog(0.,0.,0.);
	btScalar max_dist_squared = 0;
	for (std::vector<btVector3>::iterator it = object_corner_point_list.begin(); it!= object_corner_point_list.end(); ++it)
	{
		btScalar point_distance_to_cog =  cog.distance2(*it);
		max_dist_squared = (point_distance_to_cog > max_dist_squared) ? point_distance_to_cog : max_dist_squared;
	}

	// calculate maximum angular acceleration under gravity
	// btVector3 gravity_forces = object.getGravity() / object.getInvMass();
	// double max_angular_acc = gravity_forces.norm() * sqrt(max_dist_squared);

	return sqrt(max_dist_squared);
}