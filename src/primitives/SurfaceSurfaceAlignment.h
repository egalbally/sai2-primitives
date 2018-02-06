/*
 * SurfaceSurfaceAlignment.h
 *
 *      This class creates a motion primitive for a surface surface alignment using a posori task and a joint task in its nullspace
 *      Assumes that the control frame origin is at the geometric center of the surface to align and the Z axis of the control frame is along the normal of the surface
 *      The robot is still free to move in the orthogonal direction to the surface as well as rotate around the normal
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_
#define SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_

#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

namespace Sai2Primitives
{

class SurfaceSurfaceAlignment
{
public:

	SurfaceSurfaceAlignment(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity(),
	                   const Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity());

	~SurfaceSurfaceAlignment();

	void updatePrimitiveModel();

	void computeTorques(Eigen::VectorXd& torques);

	void updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
								    const Eigen::Vector3d sensed_moment_sensor_frame);

	void enableGravComp();
	void disbleGravComp();

	void enableOrthogonalPosControl();
	void enableOrthogonalRotControl();

	Sai2Model::Sai2Model* _robot;
	std::string _link_name;
	Eigen::Affine3d _control_frame;
	Eigen::Affine3d _sensor_frame;
	Eigen::Affine3d _T_base_control;

	PosOriTask* _posori_task;
	JointTask* _joint_task;

	Eigen::Vector3d _desired_position;
	Eigen::Matrix3d _desired_orientation;

	Eigen::Vector3d _desired_velocity;
	Eigen::Vector3d _desired_angular_velocity;

	double _desired_normal_force;

protected:
	bool _gravity_compensation = false;


};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_ */