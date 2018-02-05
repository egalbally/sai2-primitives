/*
 * RedundantArmMotion.h
 *
 *      This class creates a motion primitive for a redundant arm using a posori task and a joint task in its nullspace
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_REDUNDANTARM_MOTION_H_
#define SAI2_PRIMITIVES_REDUNDANTARM_MOTION_H_

#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

namespace Sai2Primitives
{

class RedundantArmMotion
{
public:

	RedundantArmMotion(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity());

	RedundantArmMotion(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Vector3d pos_in_link,
	                   const Eigen::Matrix3d rot_in_link = Eigen::Matrix3d::Identity());

	void updatePrimitiveModel();

	void computeTorques(Eigen::VectorXd& torques);

	void enableGravComp();
	void disbleGravComp();

	Sai2Model::Sai2Model* _robot;
	std::string _link_name;
	Eigen::Affine3d _control_frame;

	PosOriTask* _posori_task;
	JointTask* _joint_task;

	Eigen::Vector3d _desired_position;
	Eigen::Matrix3d _desired_orientation;

	Eigen::Vector3d _desired_velocity;
	Eigen::Vector3d _desired_angular_velocity;

protected:
	bool _gravity_compensation = false;


};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_REDUNDANTARM_MOTION_PRIMITIVE_H_ */