/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>

#include <boost/shared_ptr.hpp>

#include "../KDLParser.hpp"

#include <srdfdom_advr/model.h>
#include <urdf/model.h>
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

// RST-RT
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

#include <string>
#include <rst-rt/geometry/Rotation.hpp>
#include <rst-rt/geometry/Translation.hpp>

class ControllerContained: public RTT::TaskContext {
public:
	ControllerContained(std::string const & name);

	// RTT::TaskContext methods that are needed for any standard component and
	// should be implemented by user
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void setPos(float x, float y, float z);
	void setGains(double p,double d, double np, double nd);

	bool exists_test(const std::string& name);
	bool selectKinematicChain(const std::string& chainName);
	bool loadURDFAndSRDF(const std::string &URDF_path,
			const std::string &SRDF_path);
	void calculateKinematicsDynamics(const rstrt::robot::JointState& jointState);
	void setBaseAndTip(std::string base, std::string tip);
	Eigen::VectorXf calcQuat(Eigen::MatrixXf rot);
	Eigen::MatrixXf calcMPI(Eigen::MatrixXf input);

private:
	bool _models_loaded;
	XBot::XBotCoreModel xbot_model;
	std::string xml_model, activeChain, base_name, tip_name;
	KDLParser p;
	KDL::Tree robot_tree;
	KDL::Chain activeChain_KDL;
	KDL::Vector gravity_vector;
	boost::shared_ptr<KDL::ChainDynParam> id_solver;

	KDL::JntArrayVel jointStates_KDL;
	Eigen::VectorXf G, C;
	Eigen::MatrixXf M, jac, jacd;
	KDL::JntArray G_kdl, C_kdl;
	KDL::JntSpaceInertiaMatrix M_kdl;
	KDL::Frame cart_pos;
	KDL::FrameVel cart_vel;
	KDL::Jacobian jac_kdl,jacd_kdl;
	Eigen::VectorXf quat_t1;

	Eigen::Vector3f pos;
	double P,D,NP,ND;
	bool nullspace;

	RTT::InputPort<rstrt::robot::JointState> robot_state_port;

	RTT::OutputPort<rstrt::dynamics::JointTorques> ojt_port;

	RTT::FlowStatus robot_state_flow;

	rstrt::robot::JointState robot_state;
	rstrt::dynamics::JointTorques jt_var;

	int DOFsize;

	boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jacDot_solver;
	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
	boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;

};

