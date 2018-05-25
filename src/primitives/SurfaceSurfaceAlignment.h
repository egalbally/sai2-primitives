/*
 * SurfaceSurfaceAlignment.h
 *
 *      This class creates a motion primitive for a surface surface alignment using a posori task and a joint task in its nullspace
 *      Assumes that the control frame origin is at the geometric center of the surface to align and the Z axis of the control frame is along the normal of the surface.
 *      The robot is still free to move in the orthogonal direction to the surface as well as rotate around the normal (TODO : not implemented yet)
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_
#define SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_

#include "TemplatePrimitive.h"
#include "tasks/PosOriTask.h"
#include "tasks/JointTask.h"

namespace Sai2Primitives
{

class SurfaceSurfaceAlignment : public TemplatePrimitive
{
public:

	/**
	 * @brief Constructor that needs the control frame and sensor frame in local link frame
	 * @details Assumes the control frame Z axis is in the direction of the surface normel on the robot side
	 * 
	 * @param robot          robot model
	 * @param link_name      link to which are attached the control frame and the sensor frame (end effector link)
	 * @param control_frame  Transformation matrix of the control frame description in link frame
	 * @param sensor_frame   Transformation matrix of the sensor frame in link frame
	 */
	SurfaceSurfaceAlignment(Sai2Model::Sai2Model* robot,
					   const std::string link_name,
	                   const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity(),
	                   const Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity());

	/**
	 * @brief Constructor that needs the control frame and sensor frame in local link frame
	 * @details Assumes the control frame Z axis is in the direction of the surface normel on the robot side
	 * 
	 * @param robot                       robot model
	 * @param link_name                   link to which are attached the control frame and the sensor frame (end effector link)
	 * @param control_pos_in_link         Position of the control frame origin in link frame
	 * @param sensor_pos_in_link          Position of the sensor frame origin in link frame
	 * @param control_rot_in_link         Orientation of the control frame origin in link frame
	 * @param sensor_rot_in_link          Orientation of the sensor frame origin in link frame
	 */
	SurfaceSurfaceAlignment(Sai2Model::Sai2Model* robot,
				   const std::string link_name,
                   const Eigen::Vector3d control_pos_in_link,
                   const Eigen::Vector3d sensor_pos_in_link,
                   const Eigen::Matrix3d control_rot_in_link = Eigen::Matrix3d::Identity(),
                   const Eigen::Matrix3d sensor_rot_in_link = Eigen::Matrix3d::Identity());

	/**
	 * @brief Destructor
	 */
	~SurfaceSurfaceAlignment();

	/**
	 * @brief Updates the primitive model (dynamic quantities for op space and kinematics of the control frame position). 
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
	 * @brief Updates the sensed force and moments from the force sensor. The quantities need to be expressed in sensor frame
	 * and they represent the force and moment applied by the robot to the environment (TODO : change this ?)
	 * 
	 * @param sensed_force_sensor_frame    the sensed force from the force sensor in sensor frame
	 * @param sensed_moment_sensor_frame   the sensed moment from the force sensor in sensor frame
	 */
	void updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
								    const Eigen::Vector3d sensed_moment_sensor_frame);

	/**
	 * @brief Sets the desired force in the normal direction in order not to loose contact
	 * Default value is 10 Newtons
	 * 
	 * @param force_value   The desired force value in Newtons
	 */
	void setDesiredNormalForce(const double force_value);

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
	 * @brief TODO : not implemented yet
	 */
	void enableOrthogonalPosControl();

	/**
	 * @brief TODO : not implemented yet
	 */
	void enableOrthogonalRotControl();

	std::string _link_name;
	Eigen::Affine3d _control_frame;
	Eigen::Affine3d _sensor_frame;
	Eigen::Affine3d _T_base_control;

	PosOriTask* _posori_task;
	JointTask* _joint_task;

	Eigen::Vector3d _desired_position;
	Eigen::Matrix3d _desired_orientation;

	Eigen::Vector3d _desired_velocity;
	Eigen::Vector3d _desired_angular_velocity;

	double _desired_normal_force;

protected:
	bool _gravity_compensation = false;


};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_SURFACE_SURFACE_ALIGNMENT_H_ */