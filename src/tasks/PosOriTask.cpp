/*
 * PosOriTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PosOriTask.h"

#include <stdexcept>


namespace Sai2Primitives
{


PosOriTask::PosOriTask(Sai2Model::Sai2Model* robot, std::string link_name, Eigen::Affine3d control_frame) :
	PosOriTask(robot, link_name, control_frame.translation(), control_frame.linear()) {}

PosOriTask::PosOriTask(Sai2Model::Sai2Model* robot, std::string link_name, Eigen::Vector3d pos_in_link, Eigen::Matrix3d rot_in_link)
{

	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.linear() = rot_in_link;
	control_frame.translation() = pos_in_link;

	_robot = robot;
	_link_name = link_name;
	_control_frame = control_frame;

	int dof = _robot->_dof;

	// motion
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_robot->position(_goal_position, _link_name, _control_frame.translation());
	_robot->rotation(_current_orientation, _link_name);
	_robot->rotation(_desired_orientation, _link_name);

	_current_velocity.setZero();
	_desired_velocity.setZero();
	_current_angular_velocity.setZero();
	_desired_angular_velocity.setZero();
	_linear_saturation_velocity.setZero();
	_angular_saturation_velocity.setZero();

	_max_velocity = 0;

	_kp_pos = 50.0;
	_kv_pos = 14.0;
	_ki_pos = 0.0;
	_kp_ori = 50.0;
	_kv_ori = 14.0;
	_ki_ori = 0.0;

	_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_orientation_error.setZero();

	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_orientation = Eigen::Matrix3d::Identity();

	// force
	_T_control_to_sensor = Eigen::Affine3d::Identity();  

	_desired_force.setZero();
	_sensed_force.setZero();
	_desired_moment.setZero();
	_sensed_moment.setZero();

	_kp_force = 1.0;
	_kv_force = 10.0;
	_ki_force = 0.7;
	_kp_moment = 1.0;
	_kv_moment = 10.0;
	_ki_moment = 0.7;

	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();

	_sigma_force.setZero();
	_sigma_moment.setZero();

	_closed_loop_force_control = false;
	_closed_loop_moment_control = false;

	// model
	_task_force.setZero(6);

	_jacobian.setZero(6,dof);
	_projected_jacobian.setZero(6,dof);
	_Lambda.setZero(6,6);
	_Jbar.setZero(dof,6);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;
}


void PosOriTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in PosOriTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in PosOriTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->J_0(_jacobian, _link_name, _control_frame.translation());
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);

}


void PosOriTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;

	Eigen::Vector3d force_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d position_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d moment_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d orientation_related_force = Eigen::Vector3d::Zero();

	// update velocity and angular velocity for damping term
	_current_velocity = _projected_jacobian.block(0,0,3,_robot->_dof) * _robot->_dq;
	_current_angular_velocity = _projected_jacobian.block(3,0,3,_robot->_dof) * _robot->_dq;


	// force related terms
	if(_closed_loop_force_control)
	{
		// update the integrated error
		_integrated_force_error += (_sensed_force - _desired_force) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d force_feedback_term = - _kp_force * (_sensed_force - _desired_force) - _ki_force * _integrated_force_error - _kv_force * _current_velocity;

		// compute the final contribution
		force_related_force = _sigma_force * (_desired_force + force_feedback_term);
	}
	else
	{
		force_related_force = _sigma_force * _desired_force;
	}

	// moment related terms
	if(_closed_loop_moment_control)
	{
		// update the integrated error
		_integrated_moment_error += (_sensed_moment - _desired_moment) * _t_diff.count();

		// compute the feedback term
		Eigen::Vector3d moment_feedback_term = - _kp_moment * (_sensed_moment - _desired_moment) - _ki_moment * _integrated_moment_error - _kv_moment * _current_angular_velocity;

		// compute the final contribution
		moment_related_force = _sigma_moment * (_desired_moment + moment_feedback_term);
	}
	else
	{
		moment_related_force = _sigma_moment * _desired_moment;
	}

	// linear motion related terms

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
	
	// get curent position for P term
	_robot->position(_current_position, _link_name, _control_frame.translation());

	// update integrated error for I term
	_integrated_position_error += (_current_position - _desired_position) * _t_diff.count();

	// final contribution
	if(_velocity_saturation)
	{
		_desired_velocity = -_kp_pos / _kv_pos * (_current_position - _desired_position) - _ki_pos / _kv_pos * _integrated_position_error;
		for(int i=0; i<3; i++)
		{
			if(_desired_velocity(i) > _linear_saturation_velocity(i))
			{
				_desired_velocity(i) = _linear_saturation_velocity(i);
			}
			else if(_desired_velocity(i) < -_linear_saturation_velocity(i))
			{
				_desired_velocity(i) = -_linear_saturation_velocity(i);
			}
		}
		position_related_force = _sigma_position * (-_kv_pos*(_current_velocity - _desired_velocity));
	}
	else
	{
		position_related_force = _sigma_position*(-_kp_pos*(_current_position - _desired_position) - _kv_pos*(_current_velocity - _desired_velocity ) - _ki_pos * _integrated_position_error);
	}


	// angular motion related terms
	// get curent position and orientation error for P term
	_robot->rotation(_current_orientation, _link_name);
	_current_orientation = _current_orientation * _control_frame.linear(); // orientation of compliant frame in robot frame
	Sai2Model::orientationError(_orientation_error, _desired_orientation, _current_orientation);

	// update integrated error for I term
	_integrated_orientation_error += _orientation_error * _t_diff.count();

	// final contribution
	if(_velocity_saturation)
	{
		_desired_angular_velocity = -_kp_ori / _kv_ori * _orientation_error - _ki_ori / _kv_ori * _integrated_position_error;
		for(int i=0; i<3; i++)
		{
			if(_desired_angular_velocity(i) > _angular_saturation_velocity(i))
			{
				_desired_angular_velocity(i) = _angular_saturation_velocity(i);
			}
			else if(_desired_angular_velocity(i) < -_angular_saturation_velocity(i))
			{
				_desired_angular_velocity(i) = -_angular_saturation_velocity(i);
			}
		}
		orientation_related_force = _sigma_orientation * (-_kv_ori*(_current_angular_velocity - _desired_angular_velocity));
	}
	else
	{
		orientation_related_force = _sigma_orientation * ( -_kp_ori*_orientation_error - _kv_ori*(_current_angular_velocity - _desired_angular_velocity) - _ki_ori*_integrated_orientation_error);
	}

	// compute task force
	Eigen::VectorXd force_moment_contribution(6), position_orientation_contribution(6);
	force_moment_contribution.head(3) = force_related_force;
	force_moment_contribution.tail(3) = moment_related_force;

	position_orientation_contribution.head(3) = position_related_force;
	position_orientation_contribution.tail(3) = orientation_related_force;

	_task_force = _Lambda * position_orientation_contribution + force_moment_contribution;

	// compute task torques
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}

void PosOriTask::reInitializeTask()
{
	int dof = _robot->_dof;

	// motion
	_robot->position(_current_position, _link_name, _control_frame.translation());
	_robot->position(_desired_position, _link_name, _control_frame.translation());
	_robot->position(_goal_position, _link_name, _control_frame.translation());
	_robot->rotation(_current_orientation, _link_name);
	_robot->rotation(_desired_orientation, _link_name);

	_current_velocity.setZero();
	_desired_velocity.setZero();
	_current_angular_velocity.setZero();
	_desired_angular_velocity.setZero();
	_linear_saturation_velocity.setZero();
	_angular_saturation_velocity.setZero();

	_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_orientation_error.setZero();

	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_orientation = Eigen::Matrix3d::Identity();

	_desired_force.setZero();
	_sensed_force.setZero();
	_desired_moment.setZero();
	_sensed_moment.setZero();

	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();

	_sigma_force.setZero();
	_sigma_moment.setZero();

	_closed_loop_force_control = false;
	_closed_loop_moment_control = false;

	_first_iteration = true;	
}

void PosOriTask::enableVelocitySaturation(const Eigen::Vector3d& linear_saturation_velocity, const Eigen::Vector3d& angular_saturation_velocity)
{
	_velocity_saturation = true;
	_linear_saturation_velocity = linear_saturation_velocity;
	_angular_saturation_velocity = angular_saturation_velocity;
	for(int i=0; i<3; i++)
	{
		if(_linear_saturation_velocity(i) < 0)
		{
			std::cout << "WARNING : linear saturation velocity " << i << " should be positive. Set to zero" << std::endl;
			_linear_saturation_velocity(i) = 0;
		}
		if(_angular_saturation_velocity(i) < 0)
		{
			std::cout << "WARNING : angular saturation velocity " << i << " should be positive. Set to zero" << std::endl;
			_angular_saturation_velocity(i) = 0;
		}
	}
}

void PosOriTask::disableVelocitySaturation()
{
	_velocity_saturation = false;
	_linear_saturation_velocity.setZero();
	_angular_saturation_velocity.setZero();
}

void PosOriTask::setForceSensorFrame(const std::string link_name, const Eigen::Affine3d transformation_in_link)
{
	if(link_name != _link_name)
	{
		throw std::invalid_argument("The link to which is attached the sensor should be the same as the link to which is attached the control frame in PosOriTask::setForceSensorFrame\n");
	}
	_T_control_to_sensor = _control_frame.inverse() * transformation_in_link;
}

void PosOriTask::updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
							    		    const Eigen::Vector3d sensed_moment_sensor_frame)
{
	// find the transform from base frame to control frame
	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name);
	Eigen::Affine3d T_base_control = T_base_link * _control_frame;

	// find the resolved sensed force and moment in control frame
	_sensed_force = _T_control_to_sensor.rotation() * sensed_force_sensor_frame;
	_sensed_moment = _T_control_to_sensor.translation().cross(_sensed_force) + _T_control_to_sensor.rotation() * sensed_moment_sensor_frame;

	// rotate the quantities in base frame
	_sensed_force = T_base_control.rotation() * _sensed_force;
	_sensed_moment = T_base_control.rotation() * _sensed_moment;
}

void PosOriTask::setForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;

	resetIntegratorsLinear();
}

void PosOriTask::updateForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;
}

void PosOriTask::setLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;

	resetIntegratorsLinear();
}

void PosOriTask::updateLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;	
}

void PosOriTask::setFullForceControl()
{
	_sigma_force = Eigen::Matrix3d::Identity();
	_sigma_position.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setFullLinearMotionControl()
{
	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_force.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;

	resetIntegratorsAngular();
}

void PosOriTask::updateMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;	
}

void PosOriTask::setAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;

	resetIntegratorsAngular();
}

void PosOriTask::updateAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;
}

void PosOriTask::setFullMomentControl()
{
	_sigma_moment = Eigen::Matrix3d::Identity();
	_sigma_orientation.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setFullAngularMotionControl()
{
	_sigma_orientation = Eigen::Matrix3d::Identity();
	_sigma_moment.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setClosedLoopForceControl()
{
	_closed_loop_force_control = true;
	resetIntegratorsLinear();
}

void PosOriTask::setOpenLoopForceControl()
{
	_closed_loop_force_control = false;
}

void PosOriTask::setClosedLoopMomentControl()
{
	_closed_loop_moment_control = true;
	resetIntegratorsAngular();
}

void PosOriTask::setOpenLoopMomentControl()
{
	_closed_loop_moment_control = false;
}

void PosOriTask::resetIntegrators()
{
	_integrated_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();
	_first_iteration = true;	
}

void PosOriTask::resetIntegratorsLinear()
{
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
}

void PosOriTask::resetIntegratorsAngular()
{
	_integrated_orientation_error.setZero();
	_integrated_moment_error.setZero();
}


} /* namespace Sai2Primitives */

