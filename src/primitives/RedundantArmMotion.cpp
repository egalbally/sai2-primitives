/*
 * RedundantArmMotion.cpp
 *
 *      This class creates a motion primitive for a redundant arm using a posori task and a joint task in its nullspace
 *
 *      Author: Mikael Jorda
 */

#include "RedundantArmMotion.h"

namespace Sai2Primitives
{


RedundantArmMotion::RedundantArmMotion(Sai2Model::Sai2Model* robot,
				   const std::string link_name,
                   const Eigen::Affine3d control_frame) :
	RedundantArmMotion(robot, link_name, control_frame.translation(), control_frame.linear()) {}

RedundantArmMotion::RedundantArmMotion(Sai2Model::Sai2Model* robot,
				   const std::string link_name,
                   const Eigen::Vector3d pos_in_link,
                   const Eigen::Matrix3d rot_in_link)
{

	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	_posori_task = new PosOriTask(_robot, link_name, control_frame);
	_joint_task = new JointTask(_robot);

	_desired_position = _posori_task->_desired_position;
	_desired_orientation = _posori_task->_desired_orientation;

	_desired_velocity = _posori_task->_desired_velocity;
	_desired_angular_velocity = _posori_task->_desired_angular_velocity;

	// TODO make a nullspace criteria to avoid singularities and one to avoid obstacles
	_joint_task->_desired_position = _robot->_q;
	_joint_task->_desired_velocity.setZero(_robot->_dof);	
}

RedundantArmMotion::~RedundantArmMotion()
{
	delete _posori_task;
	delete _joint_task;
	_posori_task = NULL;
	_joint_task = NULL;
}

void RedundantArmMotion::updatePrimitiveModel(const Eigen::MatrixXd N_prec)
{
	_N_prec = N_prec;
	_posori_task->updateTaskModel(N_prec);
	if(_redundancy_handling)
	{
		_joint_task->updateTaskModel(_posori_task->_N);
		_N = Eigen::MatrixXd::Zero(_robot->dof() , _robot->dof());
	}
	else
	{
		_N = _posori_task->_N;
	}
}

void RedundantArmMotion::computeTorques(Eigen::VectorXd& torques)
{
	torques.setZero(_robot->_dof);

	_posori_task->_desired_position = _desired_position;
	_posori_task->_desired_orientation = _desired_orientation;
	_posori_task->_desired_velocity = _desired_velocity;
	_posori_task->_desired_angular_velocity = _desired_angular_velocity;

	Eigen::VectorXd posori_torques;
	Eigen::VectorXd joint_torques;
	Eigen::VectorXd gravity_torques;
	posori_torques.setZero(_robot->_dof);
	joint_torques.setZero(_robot->_dof);
	gravity_torques.setZero(_robot->_dof);

	_posori_task->computeTorques(posori_torques);
	_joint_task->computeTorques(joint_torques);

	if(_gravity_compensation)
	{
		_robot->gravityVector(gravity_torques);
	}
	if(!_redundancy_handling)
	{
		joint_torques.setZero(_robot->_dof);
	}

	torques = posori_torques + joint_torques + gravity_torques;
}

void RedundantArmMotion::enableGravComp()
{
	_gravity_compensation = true;
}

void RedundantArmMotion::disbleGravComp()
{
	_gravity_compensation = false;
}

void RedundantArmMotion::enableRedundancyHandling()
{
	_redundancy_handling = true;
}

void RedundantArmMotion::disableRedundancyHandling()
{
	_redundancy_handling = false;
}

} /* namespace Sai2Primitives */

