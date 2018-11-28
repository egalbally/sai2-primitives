/*
 * RedundantArmMotion.h
 *
 *      This class creates a motion primitive for a redundant arm using a posori task and a joint task in its nullspace
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_REDUNDANTARM_MOTION_H_
#define SAI2_PRIMITIVES_REDUNDANTARM_MOTION_H_

#include "TemplatePrimitive.h"
#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

namespace Sai2Primitives
{

class RedundantArmMotion : public TemplatePrimitive
{
public:

	/**
	 * @brief Constructor that takes the control frame in local link frame as an affine transform matrix
	 * 
	 * @param robot          robot model
	 * @param link_name      link to which are attached the control frame and the sensor frame (end effector link)
	 * @param control_frame  Transformation matrix of the control frame description in link frame
	 */
	RedundantArmMotion(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity());
	
	/**
	 * @brief Constructor that takes the control frame in local link frame as a position and a rotation
	 * 
	 * @param robot          robot model
	 * @param link_name      link to which are attached the control frame and the sensor frame (end effector link)
	 * @param control_frame  Position of the control frame in link frame
	 * @param control_frame  Rotation of the control frame in link frame
	 */
	RedundantArmMotion(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Vector3d pos_in_link,
	                   const Eigen::Matrix3d rot_in_link = Eigen::Matrix3d::Identity());

	/**
	 * @brief destructor
	 */
	~RedundantArmMotion();

	/**
	 * @brief Updates the primitive model in the nummspace of the previous primitives (dynamic quantities for op space and kinematics of the control frame position). 
	 * Call it after calling the dunction updateModel of the robot model
	 */
	virtual void updatePrimitiveModel(const Eigen::MatrixXd N_prec);

	/**
	 * @brief Updates the primitive model assuming it is the highest level (dynamic quantities for op space and kinematics of the control frame position). 
	 * Call it after calling the dunction updateModel of the robot model
	 */
	virtual void updatePrimitiveModel();

	/**
	 * @brief Computes the joint torques associated with the primitive
	 * 
	 * @param torques   Vector that will be populated by the joint torques
	 */
	virtual void computeTorques(Eigen::VectorXd& torques);

	/**
	 * @brief Enable the gravity compensation at the primitive level (disabled by default)
	 * @details Use with robots that do not handle their own gravity compensation
	 */
	void enableGravComp();

	/**
	 * @brief disable the gravity compensation at the primitive level (default behavior)
	 * @details For robots that handle their own gravity compensation
	 */
	void disbleGravComp();

	/**
	 * @brief Enable the redundancy handling at the primitive level (enabled by default by default)
	 * @details Use when controlling a single robot arm
	 */
	void enableRedundancyHandling();

	/**
	 * @brief Disable the redundancy handling at the primitive level (enabled by default by default)
	 * @details Use when controlling a multi-arm system or mobile manipulator
	 */
	void disableRedundancyHandling();

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
	bool _redundancy_handling = true;


};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_REDUNDANTARM_MOTION_PRIMITIVE_H_ */