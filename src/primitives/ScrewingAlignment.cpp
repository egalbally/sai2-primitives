/*
 * ScrewingAlignment.cpp
 *
 *      Author: Daniela Deschamps
 *		Modified from SurfaceSurfaceAlignment.cpp written by Mikael Jorda
 */

#include "ScrewingAlignment.h"

namespace Sai2Primitives
{


ScrewingAlignment::ScrewingAlignment(Sai2Model::Sai2Model* robot,
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


	Eigen::Vector3d localz;
	Eigen::Matrix3d R_base_link;
	_robot->rotation(R_base_link, _link_name);
	localz = R_base_link.col(2);
	_posori_task->setForceAxis(localz);
	_posori_task->setAngularMotionAxis(localz);

	_posori_task->setClosedLoopMomentControl();

	_posori_task->_kp_moment = 50.0;  	// originally 1.0
	// _posori_task->_ki_moment = 0.5;	  	// originally 0.5
	// _posori_task->_kv_moment = 10.0;	// originally 10.0

	_posori_task->_kp_force = 100.0; // added

	// _posori_task->_kp_pos = 100.0;  // added
	// _posori_task->_kv_pos = 20.0;	// added

	_posori_task->_kp_ori = 50.0; //modified, originally 50
	_posori_task->_kv_ori = 14.0; //modified, originally 14

	// _posori_task->_desired_moment = Eigen::Vector3d(0.0,0.0,0.0);

	_desired_position = _posori_task->_desired_position;
	_desired_orientation = _posori_task->_desired_orientation;

	_desired_velocity = _posori_task->_desired_velocity;
	_desired_angular_velocity = _posori_task->_desired_angular_velocity;

	_desired_normal_force = 10; // MODIFIED - Need to determine optimal force though

	// TODO make a nullspace criteria to avoid singularities and one to avoid obstacles
	// _joint_task->_desired_position = _robot->_q;
	// _joint_task->_desired_velocity.setZero(_robot->_dof);

	// _joint_task->_kp = 10.0; //modified, originally 10
	// _joint_task->_kv = 5.0;  //modified, originally 5
	std::cout << "task force at primitive creation : " << _posori_task->_task_force.transpose() << std::endl;
	std::cout << "kp force at primitive creation : " << _posori_task->_kp_force << std::endl;
}

ScrewingAlignment::~ScrewingAlignment()
{
	delete _posori_task;
	delete _joint_task;
	_posori_task = NULL;
	_joint_task = NULL;
}

void ScrewingAlignment::updatePrimitiveModel()
{
	_posori_task->updateTaskModel(Eigen::MatrixXd::Identity(_robot->_dof,_robot->_dof));
	_joint_task->updateTaskModel(_posori_task->_N);

	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name, _control_frame.translation());
	_T_base_control = T_base_link * _control_frame;
}

void ScrewingAlignment::computeTorques(Eigen::VectorXd& torques)
{
	torques.setZero(_robot->_dof);

	//
	Eigen::Vector3d localz;
	Eigen::Matrix3d R_base_link;
	_robot->rotation(R_base_link, _link_name);
	localz = R_base_link.col(2);
	_posori_task->updateForceAxis(localz);
	_posori_task->updateAngularMotionAxis(localz);

	Eigen::Vector3d globalz;
	globalz << 0,
			   0,
			   -1;

	_posori_task->_desired_force = _desired_normal_force * globalz; 

	_posori_task->_desired_moment = 0 * globalz; 

	
	// std::cout << "-------------" << std::endl;
	// std::cout << "desired force" << std::endl;
	// std::cout << _posori_task->_desired_force << std::endl; 
	// std::cout << "desired moment" << std::endl;
	// std::cout <<  << std::endl;



	/* localz shows the components of the ee position that are in the operational space z direction (does the operational space z direction shift with orientation of the plate in the surface alignment example????)
		the _posori_task->_desired_force splits the 5N desired normal force into components

	*/


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
	// torques = posori_torques + gravity_torques;
}

void ScrewingAlignment::updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
							    						 const Eigen::Vector3d sensed_moment_sensor_frame)
{
	_posori_task->updateSensedForceAndMoment(sensed_force_sensor_frame, sensed_moment_sensor_frame);
}

void ScrewingAlignment::setDesiredNormalForce(const double force_value)
{
	_desired_normal_force = force_value;
}

void ScrewingAlignment::enableGravComp()
{
	_gravity_compensation = true;
}

void ScrewingAlignment::disbleGravComp()
{
	_gravity_compensation = false;
}

void ScrewingAlignment::enableOrthogonalPosControl()
{
	// TODO : implement this
}

void ScrewingAlignment::enableOrthogonalRotControl()
{
	// TODO : implement this
}

} /* namespace Sai2Primitives */

