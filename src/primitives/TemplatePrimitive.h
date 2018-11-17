/*
 * TemplatePrimitive.h
 *
 *      Template for primitives in Sai2
 *
 *      Author: Mikael Jorda
 */

#ifndef SAI2_TEMPLATE_PRIMITIVE_H_
#define SAI2_TEMPLATE_PRIMITIVE_H_

#include "Sai2Model.h"

namespace Sai2Primitives
{

class TemplatePrimitive
{
public:

	/**
	 * @brief Updates the primitive model in the nummspace of the previous primitives (dynamic quantities for op space and kinematics of the control frame position). 
	 * Call it after calling the dunction updateModel of the robot model
	 */
	virtual void updatePrimitiveModel(const Eigen::MatrixXd N_prec) = 0;

	/**
	 * @brief Updates the primitive model assuming it is the highest level (dynamic quantities for op space and kinematics of the control frame position). 
	 * Call it after calling the dunction updateModel of the robot model
	 */
	virtual void updatePrimitiveModel() = 0;

	/**
	 * @brief Computes the joint torques associated with the primitive
	 * 
	 * @param torques   Vector that will be populated by the joint torques
	 */
	virtual void computeTorques(Eigen::VectorXd& torques) = 0;

	Sai2Model::Sai2Model* _robot;

	Eigen::MatrixXd _N_prec;
	Eigen::MatrixXd _N;

};


} /* namespace Sai2Primitives */

#endif /* SAI2_PRIMITIVES_REDUNDANTARM_MOTION_PRIMITIVE_H_ */