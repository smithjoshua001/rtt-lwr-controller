/*
 * ChainBase.h
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#ifndef KINEMATIC_CHAIN_CHAINBASE_H_
#define KINEMATIC_CHAIN_CHAINBASE_H_
#include <rtt/InputPort.hpp>
#include <rtt/Logger.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <XBotCoreModel.h>
// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/jacobian.hpp>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include "../KDLParser.hpp"
#include <string>
#include "../kukaLWRModel.h"


class ChainBase {
public:
	ChainBase(std::string name);
	virtual ~ChainBase();
	bool selectKinematicChain(const std::string& chainName);
	bool loadURDFAndSRDF(const std::string &URDF_path,
			const std::string &SRDF_path);
	void calculateKinematics(const rstrt::robot::JointState& jointState);
	void calculateKinematicsDynamics(const rstrt::robot::JointState& jointState);
	bool exists_test(const std::string& name);
	void setBaseAndTip(std::string base, std::string tip);
	void setDOF(unsigned int DOF);
	std::string getName();
	std::string name;
	bool _models_loaded;
	XBot::XBotCoreModel xbot_model;
	std::string xml_model, activeChain, base_name, tip_name;
	KDLParser p;
	KDL::Tree robot_tree;
	KDL::Chain activeChainBox_KDL,activeChain_KDL;
	KDL::Vector gravity_vector;
	boost::shared_ptr<KDL::ChainDynParam> id_solver;
	KDL::JntArrayVel jointStates_KDL;
	Eigen::VectorXf G, C, G_cf, C_cf;
	Eigen::MatrixXf M, jac, jacd, M_cf;
	KDL::JntArray G_kdl, C_kdl;
	KDL::JntSpaceInertiaMatrix M_kdl;
	KDL::Frame cart_pos;
	KDL::FrameVel cart_vel;
	KDL::Jacobian jac_kdl, jacd_kdl;
	RTT::InputPort<rstrt::robot::JointState> robot_state_port;
	RTT::FlowStatus robot_state_flow;
	rstrt::robot::JointState robot_state;
	unsigned int DOFsize;
	boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacDot_solver;
	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
	boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;
	kukaLWRModel model_cf;
	double M_cf_array[7][7], C_cf_array[7][7];
	
	KDL::Segment seg;
	KDL::Frame frame;

};

#endif /* KINEMATIC_CHAIN_CHAINBASE_H_ */
