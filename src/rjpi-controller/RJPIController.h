/*
 * KTSController.h
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#pragma once
#ifndef RJPICONTROLLER_H_
#define RJPICONTROLLER_H_

#include <rtt/TaskContext.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <XBotCoreModel.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jacobian.hpp>
#include <string>
#include "../kinematic-chain/ChainBase.h"
#include <Eigen/src/Core/DenseBase.h>

class RJPIController: public RTT::TaskContext, public ChainBase {
public:
	RJPIController(std::string const & name);
	virtual ~RJPIController();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void setPos(float x, float y, float z);
	void setPos_c(float x, float y, float z);
	void setStepSize(float step_size) {
		this->step_size = step_size;
	}

	// Declare input ports and their datatypes
	RTT::InputPort<Eigen::Matrix<float, 7, 7>> in_M_port;
	RTT::InputPort<Eigen::VectorXf> in_x_des_port;
	RTT::InputPort<Eigen::VectorXf> in_xd_des_port;

	RTT::FlowStatus in_M_flow;
	RTT::FlowStatus in_x_des_flow;
	RTT::FlowStatus in_xd_des_flow;

	Eigen::Matrix<float, 7, 7> in_M;
	Eigen::VectorXf in_x_des;
	Eigen::VectorXf in_xd_des;

	Eigen::Matrix<float, 6, 14> jac_c;
	Eigen::Matrix<float, 14, 14> P;
	Eigen::Matrix<float, 14, 14> Pd,R;
	Eigen::Matrix<float, 14, 14> M_c, N;
	Eigen::Matrix<float, 6, 14> jac_x;
	Eigen::Matrix<float, 6, 14> jacd_x;
	Eigen::Matrix<float, 6, 6> lambda_c, lambda_d;
	Eigen::Matrix<float, 6, 1> h_c;
	Eigen::Matrix<float, 6, 1> F, F_x;
	Eigen::Matrix<float, 6, 1> xdd_des;
	Eigen::Matrix<float, 14, 1> tau_0, tau_c;

	// Declare output ports and their datatypes
	RTT::OutputPort<rstrt::kinematics::JointAngles> out_angles_port;
	RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

	rstrt::kinematics::JointAngles out_angles_var;
	rstrt::dynamics::JointTorques out_torques_var;
	Eigen::Vector3f pos, pos_c;
	float step_size;
	Eigen::VectorXf quat_d, quat_c;
	float Kp, Dp, Ko, Do;
	float Kp_c, Dp_c, Ko_c, Do_c;
	ChainBase dual_arm_chain;
	bool constraint_on, simulation, traj_bool;
	Eigen::VectorXf force;
	void constraint_switch();

};

#endif /* RJPICONTROLLER_H_ */
