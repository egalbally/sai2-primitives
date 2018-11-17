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
                   const Eigen::Affine3d sensor_frame) :
	SurfaceSurfaceAlignment(robot, link_name, control_frame.translation(), sensor_frame.translation(), control_frame.linear(), sensor_frame.linear()) {}

SurfaceSurfaceAlignment::SurfaceSurfaceAlignment(Sai2Model::Sai2Model* robot,
				   const std::string link_name,
                   const Eigen::Vector3d control_pos_in_link,
                   const Eigen::Vector3d sensor_pos_in_link,
                   const Eigen::Matrix3d control_rot_in_link,
                   const Eigen::Matrix3d sensor_rot_in_link)
{
	_robot = robot;
	_link_name = link_name;
	_control_frame = Eigen::Affine3d::Identity();
	_control_frame.translation() = control_pos_in_link;
	_control_frame.linear() = control_rot_in_link;
	_sensor_frame = Eigen::Affine3d::Identity();
	_sensor_frame.translation() = sensor_pos_in_link;
	_sensor_frame.linear() = sensor_rot_in_link;
	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name, _control_frame.translation());
	_T_base_control = T_base_link * _control_frame;

	_posori_task = new PosOriTask(_robot, link_name, control_pos_in_link, control_rot_in_link);
	_joint_task = new JointTask(_robot);

	_posori_task->setForceSensorFrame(link_name, _sensor_frame);

	Eigen::Vector3d localz;
	Eigen::Matrix3d R_base_link;
	_robot->rotation(R_base_link, _link_name);
	localz = R_base_link.col(2);
	_posori_task->setForceAxis(localz);
	_posori_task->setAngularMotionAxis(localz);
	_posori_task->_kp_moment = 2.5;
	_posori_task->_ki_moment = 1.5;
	_posori_task->_kv_moment = 5.0;

	_posori_task->setClosedLoopMomentControl();

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

void SurfaceSurfaceAlignment::updatePrimitiveModel(const Eigen::MatrixXd N_prec)
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

	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name, _control_frame.translation());
	_T_base_control = T_base_link * _control_frame;
}

void SurfaceSurfaceAlignment::updatePrimitiveModel()
{
	int dof = _robot->dof();
	Eigen::MatrixXd N = Eigen::MatrixXd::Identity(dof,dof);
	updatePrimitiveModel(N);
}

void SurfaceSurfaceAlignment::computeTorques(Eigen::VectorXd& torques)
{
	torques.setZero(_robot->_dof);

	Eigen::Vector3d localz;
	Eigen::Matrix3d R_base_link;
	_robot->rotation(R_base_link, _link_name);
	localz = R_base_link.col(2);
	_posori_task->updateForceAxis(localz);
	_posori_task->updateAngularMotionAxis(localz);

	_posori_task->_desired_force = _desired_normal_force * localz;

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
		joint_torques.setZero(_robot->dof());
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

void SurfaceSurfaceAlignment::enableRedundancyHandling()
{
	_redundancy_handling = true;
}

void SurfaceSurfaceAlignment::disableRedundancyHandling()
{
	_redundancy_handling = false;
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

