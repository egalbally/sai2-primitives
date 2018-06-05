/*
 * SupportAndConstraintsTask.h
 *
 *      This class creates a task for an underactuated robot in contact with the environment (e.g. Humanoid robot)
 *		It allows to compute the support jacobian and controlled forces jacobian, as well as the projector on the nullspace 
 *		of the contact constraints taking into account the underactuation and the joint torques associated with the controlled internal and contact forces
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_SUPPORTANDCONSTRAINTS_TASK_H_
#define SAI2_PRIMITIVES_SUPPORTANDCONSTRAINTS_TASK_H_

#include "Sai2Model.h"
#include "TemplateTask.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

namespace Sai2Primitives
{

class SupportAndConstraintsTask : public TemplateTask
{
public:

	//------------------------------------------------
	// Constructors
	//------------------------------------------------

	/**
	 * @brief Constructor that takes an Affine3d matrix for definition of the control frame. Creates a full position controller by default.
	 * 
	 * @param robot                             A pointer to a Sai2Model object for the robot that is to be controlled	
	 * @param contact_link_names                A vector of link names for the different links in contact with the environment
	 * @param constrained_rotations             A vector of the number of constrained rotations for each contact. 0 for point contact, 1 for line contact, 2 for surface contact
	 * @param contact_frames                    A vector of contact frames corresponding to each link in contact with the environment. z must be aligned with constrained position direction.
	 */
	SupportAndConstraintsTask(Sai2Model::Sai2Model* robot, 
		            const std::vector<std::string> contact_link_names, 
		            const std::vector<int> constrained_rotations,
		            const std::vector<Eigen::Affine3d> contact_frames);

	/**
	 * @brief Constructor that takes an Affine3d matrix for definition of the control frame. Creates a full position controller by default.
	 * 
	 * @param robot                             A pointer to a Sai2Model object for the robot that is to be controlled	
	 * @param contact_link_names                A vector of link names for the different links in contact with the environment
	 * @param constrained_rotations             A vector of the number of constrained rotations for each contact. 0 for point contact, 1 for line contact, 2 for surface contact
	 * @param contact_frames                    A vector of contact frames corresponding to each link in contact with the environment. z must be aligned with constrained position direction.
	 * @param active_contacts                   A vector of 1 and 0. 1 if the contact dof is active (explicitely controlled) 0 if it is passive. the order is f1, f2, f3, ... m1, m2, m3. the order per force and moment is x, y, z (order of the grasp matrix in Sai2Model)
	 * @param uncontrolled_internal_forces      A vector of 1 and 0. 1 if the internal degree of freedom is uncontrolled, 0 otherwise. The order is tensions and moments (order of the grasp matrix in Sai2Model)
	 */
	SupportAndConstraintsTask(Sai2Model::Sai2Model* robot, 
		            const std::vector<std::string> contact_link_names, 
		            const std::vector<int> constrained_rotations,
		            const std::vector<Eigen::Affine3d> contact_frames,
		            const Eigen::VectorXd active_contacts,
		            const Eigen::VectorXd uncontrolled_internal_forces);


	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	virtual void updateTaskModel();

	/**
	 * @brief Computes the torques associated with this task.
	 * @details Computes the torques taking into account the last model update and updated values for the robot joint positions/velocities
	 * assumes the desired internal forces and active contact forces have been updated
	 * 
	 * @param task_joint_torques the vector to be filled with the new joint torques to apply for the task
	 */
	virtual void computeTorques(Eigen::VectorXd& task_joint_torques);

	//------------------------------------------------
	// Attributes
	//------------------------------------------------

	std::vector<Eigen::MatrixXd> _contact_jacobians;

	// grasp matrix related
	Eigen::MatrixXd _G, _G_permuted, _Gbar;
	Eigen::MatrixXd _P, _Q;
	Eigen::Matrix3d _Rg;
	Eigen::MatrixXd _G11, _G12, _G13, _G21, _G22, _G23;

	double _n_contacts;
	double _contact_dof;

	Eigen::VectorXd _active_contacts;
	Eigen::VectorXd _uncontrolled_internal_forces;

	double _n_support_dof = 6;
	double _n_ac_dof, _n_pc_dof;
	double _n_ic_dof, _n_iuc_dof;

	// model quantities
	Eigen::MatrixXd _J_contact;
	Eigen::MatrixXd _J_ac, _J_pc;
	Eigen::MatrixXd _Js, _Jf;
	Eigen::MatrixXd _N;
	Eigen::MatrixXd _Projector;


};


} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_SUPPORTANDCONSTRAINTS_TASK_H_ */
#endif