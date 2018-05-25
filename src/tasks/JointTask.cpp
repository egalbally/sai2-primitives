/*
 * JointTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "JointTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


JointTask::JointTask(Sai2Model::Sai2Model* robot)
{
	_robot = robot;

	int dof = _robot->_dof;

	_current_position = _robot->_q;
	_desired_position = _robot->_q;

	_current_velocity.setZero(dof);
	_desired_velocity.setZero(dof);

	_kp = 50.0;
	_kv = 14.0;
	_ki = 0.0;

	_task_force.setZero(dof);
	_integrated_position_error.setZero(dof);

	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;
}


void JointTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in JointTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in JointTask::updateTaskModel\n");
	}

	_N_prec = N_prec;
}


void JointTask::computeTorques(Eigen::VectorXd& task_joint_torques)
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
	_current_position = _robot->_q;

	// update integrated error for I term
	_integrated_position_error += (_current_position - _desired_position) * _t_diff.count();

	// update angular velocity for D term
	_current_velocity = _robot->_dq;

	// compute task force
	_task_force = _robot->_M*(-_kp*(_current_position - _desired_position) - _kv*(_current_velocity - _desired_velocity ) - _ki * _integrated_position_error);
	// _task_force = (-_kp*(_current_position - _desired_position) - _kv*(_current_velocity - _desired_velocity ) - _ki * _integrated_position_error);

	// compute task torques
	task_joint_torques = _N_prec.transpose() * _task_force;

	// update previous time
	_t_prev = _t_curr;
}


} /* namespace Sai2Primitives */

