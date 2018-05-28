/*
 * DualArmObjectMotion.h
 *
 *      This class creates a motion primitive for dual arm system moving an object
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_DUALARMOBJECT_MOTION_H_
#define SAI2_PRIMITIVES_DUALARMOBJECT_MOTION_H_

#include "TemplatePrimitive.h"
#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

namespace Sai2Primitives
{

class DualArmObjectMotion : public TemplatePrimitive
{
public:

	/**
	 * @brief Constructor that takes the control frame in local link frame as an affine transform matrix
	 * 
	 * @param robot          robot model
	 * @param link_name      link to which are attached the control frame and the sensor frame (end effector link)
	 * @param control_frame  Transformation matrix of the control frame description in link frame
	 */
	DualArmObjectMotion(Sai2Model::Sai2Model* robot,
					   const double object_mass,
					   const std::string end_effector_1,
					   const std::string end_effector_2,
	                   const Eigen::Affine3d contact_frame_1 = Eigen::Affine3d::Identity(),
	                   const Eigen::Affine3d contact_frame_2 = Eigen::Affine3d::Identity()
	                   );
	
	/**
	 * @brief destructor
	 */
	~DualArmObjectMotion();

	/**
	 * @brief Updates the primitive model (dynamic quantities for op space and kinematics of the control frame position). 
	 * Call it after calling the dunction updateModel of the robot model
	 */
	virtual void updatePrimitiveModel(const Eigen::MatrixXd N_prec);

	/**
	 * @brief Computes the joint torques associated with the primitive
	 * 
	 * @param torques   Vector that will be populated by the joint torques
	 */
	virtual void computeTorques(Eigen::VectorXd& torques);

	std::string _end_effector_1;
	std::string _end_effector_2;
	Eigen::Affine3d _contact_frame_1; // in link frame
	Eigen::Affine3d _contact_frame_2; // in link frame
	Eigen::Affine3d _control_frame_1; // in link frame
	Eigen::Affine3d _control_frame_2; // in link frame

	PosOriTask* _posori_task_1;
	PosOriTask* _posori_task_2;

	Eigen::Vector3d _desired_position;
	Eigen::Matrix3d _desired_orientation;
	double _desired_internal_force;
	Eigen::Vector3d _current_position;
	Eigen::Matrix3d _current_orientation;

	Eigen::Vector3d _desired_velocity;
	Eigen::Vector3d _desired_angular_velocity;
	Eigen::Vector3d _current_velocity;
	Eigen::Vector3d _current_angular_velocity;

	double _kp_pos = 50.0;
	double _kv_pos = 14.0;
	double _kp_ori = 50.0;
	double _kv_ori = 14.0;

	double _object_mass;
	// TODO : handle object inertia
	// Eigen::Matrix3d _object_inertia; 
	Eigen::MatrixXd _Lambda_object;
	Eigen::MatrixXd _Lambda_augmented;
	Eigen::VectorXd _F_res;
	Eigen::VectorXd _F_int;

	Eigen::MatrixXd _G;
	Eigen::Matrix3d _Rg;            // in world frame
	Eigen::Vector3d _object_pos;    // in world frame
	Eigen::Affine3d _object_frame;
};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_REDUNDANTARM_MOTION_PRIMITIVE_H_ */