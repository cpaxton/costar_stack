#include "scene_physics_penalty.h"

inline double logisticFunction(const double &steepness, const double &max_value, const double &midpoint, const double &x)
{
	return max_value / (1 + exp(-steepness * (x - midpoint)));
}

inline double stabilityPenaltyFormula(const double &a_t, const double &a_r, const double &w_t, const double &w_r)
{
	// use reversed logistic function for 
	return (logisticFunction(-40., 1., 0.10, a_t*w_t)) * (logisticFunction(-40., 1., 0.10, a_r*w_r));
}

double calculateStabilityPenalty(const MovementComponent &acceleration,
	const ObjectPenaltyParameters &penalty_params, const double &gravity_magnitude)
{
	double translation_penalty = acceleration.linear_.norm() / gravity_magnitude,
		angular_penalty = acceleration.angular_.norm() / penalty_params.maximum_angular_acceleration_;

	return stabilityPenaltyFormula(translation_penalty, angular_penalty, 
		penalty_params.translational_acceleration_weight_, penalty_params.angular_acceleration_weight_);
}

double calculateFrameTransitionPenalty(const btTransform &current_frame, const btTransform &prev_frame, 
	const btVector3 &gravity_direction, const btScalar &horizontal_weight, const btScalar &rotation_weight)
{
	btTransform transition = prev_frame.inverse() * current_frame;
	btScalar vertical_move = transition.getOrigin().dot(gravity_direction);
	btScalar horizontal_move = sqrt(transition.getOrigin().length2() - vertical_move * vertical_move);

	btScalar translation = (vertical_move + horizontal_move*horizontal_weight)/SCALING;
	btScalar rotation = transition.getRotation().getAngle();

	// use spring model for frame transition
	btScalar trans_force = 0.5 * translation * translation;
	btScalar rot_force = 0.5 * rotation_weight * rotation * rotation;
	// std::cerr << exp(-10000*trans_force) << ", " << exp(-10*rot_force) << std::endl;
	return exp(-10000*trans_force)*exp(-10*rot_force);
}

btScalar getObjectMaximumGravityTorqueLength(const btCollisionShape &object_shape)
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

	return sqrt(max_dist_squared);
}

btScalar getObjectMaximumAngularAcceleration(const btCollisionShape &object_shape, const btScalar &mass, const btVector3 &inertia)
{
	btScalar max_gravity_torque_length = getObjectMaximumGravityTorqueLength(object_shape);
	btScalar scaled_gravity_magnitude = SCALED_GRAVITY_MAGNITUDE;
	// Torque = I * alpha = (mg) * length => alpha = mgl / I
	// maximum gravity angular acceleration happened when object is supported on the point with highest distance from cog
	// and object orientation makes this distance vector perpendicular to the gravity direction

	btVector3 inv_maximum_gravity_angular_acceleration = inertia /(scaled_gravity_magnitude * max_gravity_torque_length * mass);
	return ( 1/inv_maximum_gravity_angular_acceleration.norm() );
}

btScalar getObjectSupportContribution(const scene_support_vertex_properties &support_graph_vertex)
{
	const double &support_value =  support_graph_vertex.support_contributions_;
	return logisticFunction(0.5, 1., 0., support_value);
}

btScalar getObjectCollisionPenalty(const scene_support_vertex_properties &support_graph_vertex)
{
	// TODO: Find a good parameter for the penetration depth
	const double &total_penetration_depth = support_graph_vertex.penetration_distance_;
	return logisticFunction(-1.5 , 1., 3., total_penetration_depth);
	// return 1.;
}

btScalar dataProbabilityScale(const btScalar &hypothesis_confidence, const btScalar &max_confidence)
{
	// Scaling for hypothesis confidence
	return logisticFunction(10, 1, 0.36, hypothesis_confidence/max_confidence);
}

