/*
 * KTSController.h
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#pragma once
#ifndef KINEMATIC_TASK_SPACE_KTSCONTROLLER_H_
#define KINEMATIC_TASK_SPACE_KTSCONTROLLER_H_

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

class KTSController: public RTT::TaskContext, public ChainBase {
public:
	KTSController(std::string const & name);
	virtual ~KTSController();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void setPos(float x, float y, float z);
	void setStepSize(float step_size){
		this->step_size = step_size;
	}

//	void setDOFSize(unsigned int DOFsize);
//
//	void setBaseAndTip(std::string base, std::string tip);
//
//	bool selectKinematicChain(const std::string& chainName);
//	bool loadURDFAndSRDF(const std::string &URDF_path,
//			const std::string &SRDF_path);

	// Declare input ports and their datatypes
//	RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

	// Declare output ports and their datatypes
	RTT::OutputPort<rstrt::kinematics::JointAngles> out_angles_port;
	RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

//	rstrt::robot::JointState in_robotstatus_var;
	rstrt::kinematics::JointAngles out_angles_var;
	rstrt::dynamics::JointTorques out_torques_var;
	Eigen::Vector3f pos;
	float step_size;
	Eigen::VectorXf quat_d;
	float K,D;
//	RTT::FlowStatus robot_state_flow;
//	unsigned int DOFsize;
//	boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
//	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;

//	bool _models_loaded;
//	XBot::XBotCoreModel xbot_model;
//	std::string xml_model, activeChain, base_name, tip_name;
//	KDLParser p;
//	KDL::Tree robot_tree;
//	KDL::Chain activeChain_KDL;
//	KDL::JntArrayVel jointStates_KDL;

};

#endif /* KINEMATIC_TASK_SPACE_KTSCONTROLLER_H_ */
