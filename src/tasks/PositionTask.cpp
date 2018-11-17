/*
 * PositionTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PositionTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


PositionTask::PositionTask(Sai2Model::Sai2Model* robot, std::string link_name, Eigen::Affine3d control_frame) :
	PositionTask(robot, link_name, control_frame.translation(), control_frame.linear()) {}

PositionTask::PositionTask(Sai2Model::Sai2Model* robot, std::string link_name, Eigen::Vector3d pos_in_link, Eigen::Matrix3d rot_in_link)
{

	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	int dof = _robot->_dof;

	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_robot->position(_goal_position, _link_name, _control_frame.translation());

	_max_velocity = 0.0;

	_current_velocity.setZero();
	_desired_velocity.setZero();
	_saturation_velocity.setZero();

	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;

	_task_force.setZero();
	_integrated_position_error.setZero();

	_jacobian.setZero(3,dof);
	_projected_jacobian.setZero(3,dof);
	_Lambda.setZero(3,3);
	_Jbar.setZero(dof,3);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;
}


void PositionTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in PositionTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in PositionTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->Jv(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);

}


void PositionTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_t_curr = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	else
	{
		_t_curr = std::chrono::high_resolution_clock::now();
	}
	_t_diff = _t_curr - _t_prev;

	// get position of control frame
	_robot->position(_current_position, _link_name, _control_frame.translation());

	// update integrated error for I term
	_integrated_position_error += (_current_position - _desired_position) * _t_diff.count();

	// update angular velocity for D term
	_current_velocity = _projected_jacobian * _robot->_dq;

	// update desired position if in velocity saturation mode
	if(_max_velocity > 0)
	{
		Eigen::Vector3d proxy_error = _goal_position - _desired_position;
		if( proxy_error.norm() > 0 && proxy_error.norm() > _max_velocity*_t_diff.count() )
		{
			_desired_position += proxy_error/proxy_error.norm() * _max_velocity * _t_diff.count(); 
		}
		else
		{
			_desired_position = _goal_position;
		}
		// _desired_velocity.setZero();
		if( proxy_error.norm() > 0 && proxy_error.norm() > 10 * _max_velocity*_t_diff.count() )
		{
			_desired_velocity = proxy_error/proxy_error.norm() * _max_velocity;
		}
		else
		{
			_desired_velocity.setZero();
		}
	}
	else
	{
		_desired_position = _goal_position;
	}

	// compute task force
	if(_velocity_saturation)
	{
		_desired_velocity = -_kp / _kv * (_current_position - _desired_position) - _ki/_kv * _integrated_position_error;
		for(int i=0; i<3; i++)
		{
			if(_desired_velocity(i) > _saturation_velocity(i))
			{
				_desired_velocity(i) = _saturation_velocity(i);
			}
			else if(_desired_velocity(i) < -_saturation_velocity(i))
			{
				_desired_velocity(i) = -_saturation_velocity(i);
			}
		}
		_task_force = _Lambda * (-_kv*(_current_velocity - _desired_velocity));
	}
	else
	{
		_task_force = _Lambda*(-_kp*(_current_position - _desired_position) - _kv*(_current_velocity - _desired_velocity ) - _ki * _integrated_position_error);
	}

	// compute task torques
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}

void PositionTask::reInitializeTask()
{
	int dof = _robot->_dof;

	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_robot->position(_goal_position, _link_name, _control_frame.translation());

	_current_velocity.setZero();
	_desired_velocity.setZero();
	_saturation_velocity.setZero();

	_integrated_position_error.setZero();

	_first_iteration = true;
}


void PositionTask::enableVelocitySaturation(const Eigen::Vector3d& saturation_velocity)
{
	_velocity_saturation = true;
	_saturation_velocity = saturation_velocity;
	for(int i=0; i<3; i++)
	{
		if(_saturation_velocity(i) < 0)
		{
			std::cout << "WARNING : saturation velocity " << i << " should be positive. Set to zero" << std::endl;
			_saturation_velocity(i) = 0;
		}
	}
}

void PositionTask::disableVelocitySaturation()
{
	_velocity_saturation = false;
	_saturation_velocity.setZero();
}

} /* namespace Sai2Primitives */

