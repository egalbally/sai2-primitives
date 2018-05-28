/*
 * DualArmObjectMotion.cpp
 *
 *      This class creates a motion primitive for dual arm system moving an object
 *
 *      Author: Mikael Jorda
 */

#include "DualArmObjectMotion.h"

namespace Sai2Primitives
{


DualArmObjectMotion::DualArmObjectMotion(Sai2Model::Sai2Model* robot,
				   const double object_mass,
				   const std::string end_effector_1,
				   const std::string end_effector_2,
                   const Eigen::Affine3d contact_frame_1,
                   const Eigen::Affine3d contact_frame_2
                   )
{
	_robot = robot;
	_object_mass = object_mass;
	_end_effector_1 = end_effector_1;
	_end_effector_2 = end_effector_2;
	_contact_frame_1 = contact_frame_1;
	_contact_frame_2 = contact_frame_2;

	_robot->addManipulationContact(_end_effector_1, _contact_frame_1.translation(), 2, _contact_frame_1.linear());
	_robot->addManipulationContact(_end_effector_2, _contact_frame_2.translation(), 2, _contact_frame_2.linear());

	_robot->updateModel();
	_robot->manipulationGraspMatrixAtGeometricCenter(_G, _Rg, _object_pos);
	_object_frame.linear() = _Rg;
	_object_frame.translation() = _object_pos;

	_current_position = _object_pos;
	_current_orientation = _Rg;
	_desired_internal_force = 10.0;
	_desired_position = _current_position;
	_desired_orientation = _current_orientation;

	_desired_velocity.setZero();
	_desired_angular_velocity.setZero();
	_current_velocity.setZero();
	_current_angular_velocity.setZero();

	Eigen::Affine3d T_0_l1;
	Eigen::Affine3d T_0_l2;
	_robot->transform(T_0_l1, _end_effector_1);
	_robot->transform(T_0_l2, _end_effector_2);
	_control_frame_1 = T_0_l1.inverse() * _object_frame;
	_control_frame_2 = T_0_l2.inverse() * _object_frame;

	_posori_task_1 = new PosOriTask(_robot, _end_effector_1, _control_frame_1);
	_posori_task_2 = new PosOriTask(_robot, _end_effector_2, _control_frame_2);

	_Lambda_object.setZero(6,6);
	_Lambda_object.block<3,3>(0,0) = _object_mass*Eigen::Matrix3d::Identity();
	
	_Lambda_augmented.setZero(6,6);
	_F_res.setZero(6);
	_F_int.setZero(6);

}

DualArmObjectMotion::~DualArmObjectMotion()
{
	delete _posori_task_1;
	delete _posori_task_2;
	_posori_task_1 = NULL;
	_posori_task_2 = NULL;
}

void DualArmObjectMotion::updatePrimitiveModel(const Eigen::MatrixXd N_prec)
{
	_robot->manipulationGraspMatrixAtGeometricCenter(_G, _Rg, _object_pos);
	_object_frame.linear() = _Rg;
	_object_frame.translation() = _object_pos;

	Eigen::Affine3d T_0_l1;
	Eigen::Affine3d T_0_l2;
	_robot->transform(T_0_l1, _end_effector_1);
	_robot->transform(T_0_l2, _end_effector_2);
	_control_frame_1 = T_0_l1.inverse() * _object_frame;
	_control_frame_2 = T_0_l2.inverse() * _object_frame;

	_posori_task_1->_control_frame = _control_frame_1;
	_posori_task_2->_control_frame = _control_frame_2;

	_N_prec = N_prec;
	_posori_task_1->updateTaskModel(N_prec);
	_posori_task_2->updateTaskModel(N_prec);

	_Lambda_augmented = _posori_task_1->_Lambda + _posori_task_2->_Lambda + _Lambda_object;

	_N = _posori_task_1->_N * _posori_task_2->_N;
}

void DualArmObjectMotion::computeTorques(Eigen::VectorXd& torques)
{
	torques.setZero(_robot->_dof);

	// current position and velocity
	_current_position = (_posori_task_1->_current_position + _posori_task_2->_current_position)/2.0;
	_current_velocity = (_posori_task_1->_current_velocity + _posori_task_2->_current_velocity)/2.0;
	_current_angular_velocity = (_posori_task_1->_current_angular_velocity + _posori_task_2->_current_angular_velocity)/2.0;
	_current_orientation = _posori_task_1->_current_orientation;

	Eigen::Vector3d orientation_error;
	Sai2Model::orientationError(orientation_error, _desired_orientation, _current_orientation);

	_F_res.head(3) = -_kp_pos * (_current_position - _desired_position) - _kv_pos * (_current_velocity - _desired_velocity);
	_F_res.tail(3) = -_kp_ori * (orientation_error) - _kv_pos * (_current_angular_velocity - _desired_angular_velocity);
	_F_res = _Lambda_augmented * _F_res;
	_F_res.head(3) -= _object_mass * _robot->_world_gravity;

	_F_int(0) = _desired_internal_force;

	Eigen::VectorXd F_tot = Eigen::VectorXd::Zero(12);
	F_tot.head(6) = _F_res;
	F_tot.tail(6) = _F_int;

	Eigen::VectorXd f_manipulators = _G.inverse() * F_tot;

	torques = _posori_task_1->_projected_jacobian.transpose() * f_manipulators.head(6) +
		_posori_task_2->_projected_jacobian.transpose() * f_manipulators.tail(6);

}

} /* namespace Sai2Primitives */

