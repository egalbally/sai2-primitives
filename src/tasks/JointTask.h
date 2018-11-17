/*
 * JointTask.h
 *
 *      This class creates a joint controller for a robotic manipulator using dynamic decoupling and an underlying PID compensator.
 *      It requires a robot model parsed from a urdf file to a Sai2Model object.
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_PRIMITIVES_JOINT_TASK_H_
#define SAI2_PRIMITIVES_JOINT_TASK_H_

#include "Sai2Model.h"
#include "TemplateTask.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

namespace Sai2Primitives
{

class JointTask : public TemplateTask
{
public:

	/**
	 * @brief      Constructor
	 *
	 * @param      robot  A pointer to a Sai2Model object for the robot that is
	 *                    to be controlled
	 */
	JointTask(Sai2Model::Sai2Model* robot);

	/**
	 * @brief      update the task model (only _N_prec for a joint task)
	 *
	 * @param      N_prec  The nullspace matrix of all the higher priority
	 *                     tasks. If this is the highest priority task, use
	 *                     identity of size n*n where n in the number of DoF of
	 *                     the robot.
	 */
	virtual void updateTaskModel(const Eigen::MatrixXd N_prec);

	/**
	 * @brief      Computes the torques associated with this task.
	 * @details    Computes the torques taking into account the last model
	 *             update and updated values for the robot joint
	 *             positions/velocities assumes the desired position and
	 *             velocity has been updated
	 *
	 * @param      task_joint_torques  the vector to be filled with the new
	 *                                 joint torques to apply for the task
	 */
	virtual void computeTorques(Eigen::VectorXd& task_joint_torques);

	/**
	 * @brief      reinitializes the desired state to the current robot
	 *             configuration as well as the integrator terms
	 */
	void reInitializeTask();

	Eigen::VectorXd _current_position;
	Eigen::VectorXd _desired_position;
	Eigen::VectorXd _goal_position;

	Eigen::VectorXd _current_velocity;
	Eigen::VectorXd _desired_velocity;

	double _max_velocity;

	double _kp;
	double _kv;
	double _ki;

	Eigen::VectorXd _integrated_position_error;
};


} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_JOINT_TASK_H_ */
#endif