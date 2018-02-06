/*
 * SurfaceSurfaceAlignment.cpp
 *
 *      Author: Mikael Jorda
 */

#include "SurfaceSurfaceAlignment.h"

namespace Sai2Primitives
{


SurfaceSurfaceAlignment::SurfaceSurfaceAlignment(Sai2Model::Sai2Model* robot,
				   const std::string link_name,
                   const Eigen::Affine3d control_frame,
                   const Eigen::Affine3d sensor_frame)
{
	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;
	_sensor_frame = sensor_frame;
	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name, _control_frame.translation());
	_T_base_control = T_base_link * _control_frame;

	_posori_task = new PosOriTask(_robot, link_name, control_frame);
	_joint_task = new JointTask(_robot);

	_posori_task->setForceSensorFrame(link_name, sensor_frame);

	_posori_task->setForceAxis(Eigen::Vector3d(0.0,0.0,1.0));
	_posori_task->setAngularMotionAxis(Eigen::Vector3d(0.0,0.0,1.0));

	_posori_task->setClosedLoopMomentControl();

	_posori_task->_kp_moment = 1.0;
	_posori_task->_ki_moment = 0.5;
	_posori_task->_kv_moment = 10.0;

	_posori_task->_desired_moment = Eigen::Vector3d(0.0,0.0,0.0);

	_desired_position = _posori_task->_desired_position;
	_desired_orientation = _posori_task->_desired_orientation;

	_desired_velocity = _posori_task->_desired_velocity;
	_desired_angular_velocity = _posori_task->_desired_angular_velocity;

	_desired_normal_force = 10.0;

	// TODO make a nullspace criteria to avoid singularities and one to avoid obstacles
	_joint_task->_desired_position = _robot->_q;
	_joint_task->_desired_velocity.setZero(_robot->_dof);
}

SurfaceSurfaceAlignment::~SurfaceSurfaceAlignment()
{
	delete _posori_task;
	delete _joint_task;
	_posori_task = NULL;
	_joint_task = NULL;
}

void SurfaceSurfaceAlignment::updatePrimitiveModel()
{
	_posori_task->updateTaskModel(Eigen::MatrixXd::Identity(_robot->_dof,_robot->_dof));
	_joint_task->updateTaskModel(_posori_task->_N);

	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name, _control_frame.translation());
	_T_base_control = T_base_link * _control_frame;
}

void SurfaceSurfaceAlignment::computeTorques(Eigen::VectorXd& torques)
{
	torques.setZero(_robot->_dof);


	_posori_task->_desired_force = _T_base_control.rotation()*Eigen::Vector3d(0.0,0.0,_desired_normal_force);

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

	torques = posori_torques + joint_torques + gravity_torques;
}

void SurfaceSurfaceAlignment::updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
							    						 const Eigen::Vector3d sensed_moment_sensor_frame)
{
	_posori_task->updateSensedForceAndMoment(sensed_force_sensor_frame, sensed_moment_sensor_frame);
}

void SurfaceSurfaceAlignment::setDesiredNormalForce(const double force_value)
{
	_desired_normal_force = force_value;
}

void SurfaceSurfaceAlignment::enableGravComp()
{
	_gravity_compensation = true;
}

void SurfaceSurfaceAlignment::disbleGravComp()
{
	_gravity_compensation = false;
}

void SurfaceSurfaceAlignment::enableOrthogonalPosControl()
{
	// TODO : implement this
}

void SurfaceSurfaceAlignment::enableOrthogonalRotControl()
{
	// TODO : implement this
}

} /* namespace Sai2Primitives */

