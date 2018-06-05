/*
 * SupportAndConstraintsTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "SupportAndConstraintsTask.h"

#include <stdexcept>


namespace Sai2Primitives
{

SupportAndConstraintsTask::SupportAndConstraintsTask(Sai2Model::Sai2Model* robot, 
		            const std::vector<std::string> contact_link_names, 
		            const std::vector<int> constrained_rotations,
		            const std::vector<Eigen::Affine3d> contact_frames)
{

	_robot = robot;

	if(contact_link_names.size() != constrained_rotations.size() or 
		contact_link_names.size() != contact_frames.size())
	{
		throw std::invalid_argument("Arguments inconsistents in SupportAndConstraintsTask");
	}

	_n_contacts = contact_link_names.size();
	_contact_dof = 0;
	for(int i = 0 ; i < contact_link_names.size() ; i++)
	{
		_robot->addEnvironmentalContact(contact_link_names[i], contact_frames[i].translation(), constrained_rotations[i], contact_frames[i].linear());
		_contact_dof += 3;
		if(constrained_rotations[i] > 0)
		{
			_contact_dof += 1 + constrained_rotations[i];
			_contact_jacobians.push_back(Eigen::MatrixXd::Zero(3,_robot->dof()));
		}
		else
		{
			_contact_jacobians.push_back(Eigen::MatrixXd::Zero(6, _robot->dof()));
		}
	}

	// grasp matrix related
	_G.setZero(_contact_dof, _contact_dof);
	_G_permuted.setZero(_contact_dof, _contact_dof);
	_Gbar.setZero(_contact_dof, _contact_dof);

	_P = Eigen::MatrixXd::Identity(_contact_dof, _contact_dof);
	_Q = Eigen::MatrixXd::Identity(_contact_dof, _contact_dof);
	_Rg = Eigen::Matrix3d::Identity();

	_active_contacts.setZero(_contact_dof);
	_uncontrolled_internal_forces.setZero(_contact_dof-6);

	_n_ac_dof = 0;
	_n_pc_dof = _contact_dof;
	_n_ic_dof = _contact_dof - _n_support_dof;
	_n_iuc_dof = 0;

	_G11 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_support_dof);
	_G12 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_ic_dof);
	_G13 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_iuc_dof);
	_G21 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_support_dof);
	_G22 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_ic_dof);
	_G23 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_iuc_dof);


	_J_contact.setZero(_contact_dof, _robot->dof());
	_J_ac.setZero(_n_ac_dof, _robot->dof());
	_J_pc.setZero(_n_pc_dof, _robot->dof());
	_Js.setZero(_n_support_dof, _robot->dof()); 
	_Jf.setZero(_n_ac_dof + _n_ic_dof, _robot->dof());
	_N_prec.setIdentity(_robot->dof(), _robot->dof());
	_N.setZero(_robot->dof(), _robot->dof());
	_Projector.setZero(_robot->dof() - _n_support_dof , _robot->dof());

}

SupportAndConstraintsTask::SupportAndConstraintsTask(Sai2Model::Sai2Model* robot, 
		            const std::vector<std::string> contact_link_names, 
		            const std::vector<int> constrained_rotations,
		            const std::vector<Eigen::Affine3d> contact_frames,
		            const Eigen::VectorXd active_contacts,
		            const Eigen::VectorXd uncontrolled_internal_forces)
{

	_robot = robot;

	if(contact_link_names.size() != constrained_rotations.size() or 
		contact_link_names.size() != contact_frames.size())
	{
		throw std::invalid_argument("Arguments inconsistents in SupportAndConstraintsTask");
	}

	_n_contacts = contact_link_names.size();
	_contact_dof = 0;
	for(int i = 0 ; i < contact_link_names.size() ; i++)
	{
		_robot->addEnvironmentalContact(contact_link_names[i], contact_frames[i].translation(), constrained_rotations[i], contact_frames[i].linear());
		_contact_dof += 3;
		if(constrained_rotations[i] > 0)
		{
			_contact_dof += 1 + constrained_rotations[i];
			_contact_jacobians.push_back(Eigen::MatrixXd::Zero(3,_robot->dof()));
		}
		else
		{
			_contact_jacobians.push_back(Eigen::MatrixXd::Zero(6, _robot->dof()));
		}
	}

	// grasp matrix related
	_G.setZero(_contact_dof, _contact_dof);
	_G_permuted.setZero(_contact_dof, _contact_dof);
	_Gbar.setZero(_contact_dof, _contact_dof);

	_P = Eigen::MatrixXd::Identity(_contact_dof, _contact_dof);
	_Q = Eigen::MatrixXd::Identity(_contact_dof, _contact_dof);
	_Rg = Eigen::Matrix3d::Identity();

	_active_contacts = active_contacts;
	_uncontrolled_internal_forces = uncontrolled_internal_forces;

	_n_ac_dof = _active_contacts.sum();
	_n_pc_dof = _contact_dof - _n_ac_dof;
	_n_iuc_dof = _uncontrolled_internal_forces.sum();
	_n_ic_dof = _contact_dof - _n_iuc_dof - _n_support_dof;

	_G11 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_support_dof);
	_G12 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_ic_dof);
	_G13 = Eigen::MatrixXd::Zero(_n_ac_dof,_n_iuc_dof);
	_G21 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_support_dof);
	_G22 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_ic_dof);
	_G23 = Eigen::MatrixXd::Zero(_n_pc_dof,_n_iuc_dof);

	_J_contact.setZero(_contact_dof, _robot->dof());
	_J_ac.setZero(_n_ac_dof, _robot->dof());
	_J_pc.setZero(_n_pc_dof, _robot->dof());
	_Js.setZero(_n_support_dof, _robot->dof()); 
	_Jf.setZero(_n_ac_dof + _n_ic_dof, _robot->dof());
	_N_prec.setIdentity(_robot->dof(), _robot->dof());
	_N.setZero(_robot->dof(), _robot->dof());
	_Projector.setZero(_robot->dof() - _n_support_dof , _robot->dof());

	// compute the projection matrices
	Eigen::MatrixXd P1 = Eigen::MatrixXd::Zero(_n_ac_dof, _contact_dof);
	int tmp_i = 0;
	for(int j=0; j<_contact_dof; j++)
	{
		if(_active_contacts(j) == 1)
		{
			P1(tmp_i, j) = 1;
			tmp_i++;
		}
	}

	Eigen::MatrixXd P2 = Eigen::MatrixXd::Zero(_n_pc_dof, _contact_dof);
	tmp_i = 0;
	for(int j=0; j<_contact_dof; j++)
	{
		if(_active_contacts(j) == 0)
		{
			P2(tmp_i, j) = 1;
			tmp_i++;
		}
	}

	_P.block(0,0,_n_ac_dof, _contact_dof) = P1;
	_P.block(_n_ac_dof,0,_n_pc_dof, _contact_dof) = P2;

	Eigen::MatrixXd Q1 = Eigen::MatrixXd::Zero(_n_ic_dof, _contact_dof-_n_support_dof);
	tmp_i = 0;
	for(int j=0; j<_contact_dof-_n_support_dof; j++)
	{
		if(_uncontrolled_internal_forces(j) == 0)
		{
			Q1(tmp_i, j) = 1;
			tmp_i++;
		}
	}

	Eigen::MatrixXd Q2 = Eigen::MatrixXd::Zero(_n_iuc_dof, _contact_dof-_n_support_dof);
	tmp_i = 0;
	for(int j=0; j<_contact_dof-_n_support_dof; j++)
	{
		if(_uncontrolled_internal_forces(j) == 1)
		{
			Q2(tmp_i, j) = 1;
			tmp_i++;
		}
	}
	_Q.block(_n_support_dof,_n_support_dof,_n_ic_dof,_contact_dof-_n_support_dof) = Q1;
	_Q.block(_n_support_dof+_n_ic_dof,_n_support_dof,_n_iuc_dof,_contact_dof-_n_support_dof) = Q2;

}


void SupportAndConstraintsTask::updateTaskModel()
{
	Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(3, _robot->dof());
	int j = 0;
	for(int i = 0; i < _robot->_environmental_contacts.size(); i++)
	{
		_robot->Jv(J_tmp, _robot->_environmental_contacts[i]._link_name, _robot->_environmental_contacts[i]._contact_position);
		_J_contact.block(3*i, 0, 3, _robot->dof()) = J_tmp;
		if(_robot->_environmental_contacts[i]._constrained_rotations > 0)
		{
			_robot->Jw(J_tmp, _robot->_environmental_contacts[i]._link_name);
			_J_contact.block(3*_n_contacts + 3*j, 0, 3, _robot->dof()) = J_tmp;
			j++;
		}
	}

	_J_contact = _P * _J_contact;

	_J_ac = _J_contact.block(0, 0, _n_ac_dof, _robot->dof());
	_J_pc = _J_contact.block(_n_ac_dof, 0, _n_pc_dof, _robot->dof());

	Eigen::Vector3d geometric_center;
	_robot->environmentalGraspMatrixAtGeometricCenter(_G, _Rg, geometric_center);
	// _G_permuted = _Q*_G*_P.transpose();
	_Gbar = _P * _G.inverse() * _Q.transpose();

	_G11 = _Gbar.block(0, 0, _n_ac_dof, _n_support_dof);
	_G12 = _Gbar.block(0, _n_support_dof, _n_ac_dof, _n_ic_dof);
	_G13 = _Gbar.block(0, _n_support_dof + _n_ic_dof, _n_ac_dof, _n_iuc_dof);
	_G21 = _Gbar.block(_n_ac_dof, 0, _n_pc_dof, _n_support_dof);
	_G22 = _Gbar.block(_n_ac_dof, _n_support_dof, _n_pc_dof, _n_ic_dof);
	_G23 = _Gbar.block(_n_ac_dof, _n_support_dof + _n_ic_dof, _n_pc_dof, _n_iuc_dof);

	Eigen::MatrixXd G13_inv = _G13.transpose()*(_G13 * _G13.transpose()).inverse();

	_Js = (_G21.transpose() - _G11.transpose()*G13_inv.transpose()*_G23.transpose())*_J_pc;

	// TODO : make member variables ?
	Eigen::MatrixXd Lambda_s = Eigen::MatrixXd::Zero(6 , 6);
	Eigen::MatrixXd Js_bar = Eigen::MatrixXd::Zero(_robot->dof() , 6);
	Eigen::MatrixXd Ns = Eigen::MatrixXd::Zero(_robot->dof() , _robot->dof());

	_robot->operationalSpaceMatrices(Lambda_s,Js_bar,Ns,_Js);
	_N_prec = Ns;

	Eigen::MatrixXd S = Eigen::MatrixXd::Zero(_robot->dof() - _n_support_dof , _robot->dof());
	S.block(0, 6, _robot->dof() - _n_support_dof , _robot->dof() - _n_support_dof) = Eigen::MatrixXd::Identity(_robot->dof() - _n_support_dof , _robot->dof() - _n_support_dof);

	Eigen::MatrixXd SN = S*Ns;
	Eigen::MatrixXd phi_star_inv;
	_robot->taskInertiaMatrixWithPseudoInv(phi_star_inv, SN);
	Eigen::MatrixXd SN_bar = _robot->_M_inv * (SN.transpose() * phi_star_inv);
	_Projector = SN_bar.transpose();
	
	// internal forces task
	_Jf.block(0,0,_n_ac_dof,_robot->dof()) = _J_ac + G13_inv.transpose()*_G23.transpose()*_J_pc;
	_Jf.block(_n_ac_dof,0,_n_ic_dof,_robot->dof()) = (_G22.transpose() - _G12.transpose()*G13_inv.transpose()*_G23.transpose())*_J_pc;
	_Jf = _Jf*_N_prec;
	Eigen::MatrixXd Lambda_f = Eigen::MatrixXd::Zero(_n_ac_dof + _n_ic_dof, _n_ac_dof + _n_ic_dof);
	Eigen::MatrixXd Jbar_f = Eigen::MatrixXd::Zero(_robot->dof(), _n_ac_dof + _n_ic_dof);
	Eigen::MatrixXd N_f = Eigen::MatrixXd::Zero(_robot->dof(), _robot->dof());
	_robot->operationalSpaceMatrices(Lambda_f,Jbar_f,N_f,_Jf,_N_prec);
	_N = N_f;

}


void SupportAndConstraintsTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

}

} /* namespace Sai2Primitives */

