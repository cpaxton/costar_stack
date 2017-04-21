#ifndef UTILITY_ROS_H
#define UTILITY_ROS_H

#include <btBulletDynamicsCommon.h>
#include <tf/transform_datatypes.h>

// inline
// btTransform convertRosTFToBulletTF(const tf::StampedTransform & input)
// {
// 	double gl_matrix[15];
// 	input.getOpenGLMatrix(gl_matrix);
// 	btTransform bt;
// 	if (boost::is_same<double, btScalar>)
// 	{
// 		bt.setFromOpenGLMatrix(gl_matrix);
// 	}
// 	else
// 	{
// 		btScalar gl_matrix_f[15];
// 		for (int i = 0; i < 15; i++) gl_matrix_f[i] = btScalar(gl_matrix[i]);
// 		bt.setFromOpenGLMatrix(gl_matrix_f);
// 	}
// 	return bt;
// }

static
btTransform convertRosTFToBulletTF(const tf::Transform & input)
{
	double gl_matrix[16];
	input.getOpenGLMatrix(gl_matrix);
	btTransform bt;
#if defined(BT_USE_DOUBLE_PRECISION)
	bt.setFromOpenGLMatrix(gl_matrix);
#else
	btScalar gl_matrix_bt[16];
	for (int i = 0; i < 16; i++) gl_matrix_bt[i] = btScalar(gl_matrix[i]);
	bt.setFromOpenGLMatrix(gl_matrix_bt);
#endif
	// Scale the translation factor 
	bt.setOrigin(bt.getOrigin()*SCALING);
	return bt;
}

static
btTransform convertRosTFToBulletTF(const tf::StampedTransform & input)
{
	double gl_matrix[16];
	input.getOpenGLMatrix(gl_matrix);
	btTransform bt;
#if defined(BT_USE_DOUBLE_PRECISION)
	bt.setFromOpenGLMatrix(gl_matrix);
#else
	btScalar gl_matrix_bt[16];
	for (int i = 0; i < 16; i++) gl_matrix_bt[i] = btScalar(gl_matrix[i]);
	bt.setFromOpenGLMatrix(gl_matrix_bt);
#endif
	// Scale the translation factor 
	bt.setOrigin(bt.getOrigin()*SCALING);
	return bt;
}

static
tf::Transform convertBulletTFToRosTF(const btTransform & input)
{
	tf::Transform tf_transform;
	btScalar gl_matrix_bt[16];

	// Scale back the translation factor
	btTransform input_rescaled = input;
	input_rescaled.setOrigin(input_rescaled.getOrigin()/SCALING);
	input_rescaled.getOpenGLMatrix(gl_matrix_bt);

#if defined(BT_USE_DOUBLE_PRECISION)
	tf_transform.setFromOpenGLMatrix(gl_matrix_bt);
#else
	double gl_matrix[16];
	for (int i = 0; i < 16; i++) gl_matrix[i] = double(gl_matrix_bt[i]);
	tf_transform.setFromOpenGLMatrix(gl_matrix);
#endif
	return tf_transform;
}

#endif