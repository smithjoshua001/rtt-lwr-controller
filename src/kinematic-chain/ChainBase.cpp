/*
 * ChainBase.cpp
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#include "ChainBase.h"

ChainBase::ChainBase(std::string name) {
	_models_loaded = false;
	this->name = name;
	robot_state_port.setName("RobotIn");
	robot_state_port.doc("Robot Input");
	robot_state_flow = RTT::NoData;

}

ChainBase::~ChainBase() {
	// TODO Auto-generated destructor stub
}

bool ChainBase::exists_test(const std::string& name) {
	if (FILE *file = fopen(name.c_str(), "r")) {
		fclose(file);
		return true;
	} else {
		return false;
	}
}
bool ChainBase::selectKinematicChain(const std::string& chainName) {
	std::vector<std::string> enabled_joints_in_chain;
	xbot_model.get_enabled_joints_in_chain(chainName, enabled_joints_in_chain);

	RTT::log(RTT::Warning) << "Size of enabled joints: "
			<< enabled_joints_in_chain.size() << RTT::endlog();

	if (!p.initTreeAndChainFromURDFString(xml_model, base_name, tip_name,
			robot_tree, activeChain_KDL)) {
		RTT::log(RTT::Error) << "[ DLW " << this->getName()
				<< "] URDF could not be parsed !" << RTT::endlog();

		// TODO add proper error handling!
		return false;
	}
	_models_loaded = true;

	RTT::log(RTT::Info) << "[" << this->getName() << "] "
			<< "robot_tree joints: " << robot_tree.getNrOfJoints()
			<< ", robot_tree segments: " << robot_tree.getNrOfSegments()
			<< RTT::endlog();

	RTT::log(RTT::Info) << "[" << this->getName() << "] "
			<< " activeKDLChain joints: " << activeChain_KDL.getNrOfJoints()
			<< ", activeKDLChain segments: "
			<< activeChain_KDL.getNrOfSegments() << RTT::endlog();

//	if (activeChain_KDL.getNrOfJoints() != DOFsize) {
//		RTT::log(RTT::Info) << "DOFsize " << DOFsize
//				<< " is different from urdf model" << RTT::endlog();
//		assert(false); //TODO
//		return false;
//	}
	DOFsize = activeChain_KDL.getNrOfJoints();
	std::cout<<DOFsize<<"DOFS!!!!!!\n";

	jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(activeChain_KDL));
	jnt_to_jacDot_solver.reset(
			new KDL::ChainJntToJacDotSolver(activeChain_KDL));
	jnt_to_cart_pos_solver.reset(
			new KDL::ChainFkSolverPos_recursive(activeChain_KDL));
	jnt_to_cart_vel_solver.reset(
			new KDL::ChainFkSolverVel_recursive(activeChain_KDL));

	id_solver.reset(new KDL::ChainDynParam(activeChain_KDL, gravity_vector));

	jointStates_KDL.resize(DOFsize);

	G = Eigen::VectorXf(DOFsize, 1);
	C = Eigen::VectorXf(DOFsize, 1);

	M = Eigen::MatrixXf(DOFsize, DOFsize);

	M_kdl.resize(7);
	C_kdl.resize(7);
	G_kdl.resize(7);

	jac_kdl.resize(7);
	jacd_kdl.resize(7);
	jac.resize(6, DOFsize);
	jacd.resize(6, DOFsize);
	robot_state.angles.resize(DOFsize);
	robot_state.torques.resize(DOFsize);
	robot_state.velocities.resize(DOFsize);
	robot_state.angles.setZero();
	robot_state.torques.setZero();
	robot_state.velocities.setZero();
	return true;
}

bool ChainBase::loadURDFAndSRDF(const std::string &URDF_path,
		const std::string &SRDF_path) {
	if (!_models_loaded) {
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();
		RTT::log(RTT::Info)<<"CHECKING EXISTANCE OF URDF AND SRDF"<< RTT::endlog();
		assert(exists_test(_urdf_path) == true);
		assert(exists_test(_srdf_path) == true);
		RTT::log(RTT::Info)<<"URDF AND SRDF EXISTS"<< RTT::endlog();
		_models_loaded = xbot_model.init(_urdf_path, _srdf_path);
		RTT::log(RTT::Info)<<"XBOT INITIALISED"<< RTT::endlog();
		for (unsigned int i = 0; i < xbot_model.get_chain_names().size(); ++i) {
			std::vector<std::string> enabled_joints_in_chain_i;
			xbot_model.get_enabled_joints_in_chain(
					xbot_model.get_chain_names()[i], enabled_joints_in_chain_i);
			for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
				RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j]
						<< RTT::endlog();
		}
		xml_model = "";
		std::fstream xml_file(URDF_path.c_str(), std::fstream::in);
		if (xml_file.is_open()) {
			while (xml_file.good()) {
				std::string line;
				std::getline(xml_file, line);
				xml_model += (line + "\n");
			}
			xml_file.close();
		}
		activeChain = xbot_model.get_chain_names()[0];
		RTT::log(RTT::Warning) << "Set default chain: " << activeChain
				<< RTT::endlog();
		selectKinematicChain(activeChain);
	} else
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!"
				<< RTT::endlog();

	return _models_loaded;
}

void ChainBase::calculateKinematics(
		const rstrt::robot::JointState& jointState) {
	jointStates_KDL.q.data = jointState.angles.cast<double>();
	jointStates_KDL.qdot.data = jointState.velocities.cast<double>();
	jnt_to_jac_solver->JntToJac(jointStates_KDL.q, jac_kdl,
			activeChain_KDL.getNrOfSegments());
	jnt_to_cart_pos_solver->JntToCart(jointStates_KDL.q, cart_pos,
			activeChain_KDL.getNrOfSegments());
	jnt_to_cart_vel_solver->JntToCart(jointStates_KDL, cart_vel,
				activeChain_KDL.getNrOfSegments());
	jac = jac_kdl.data.cast<float>();

}
void ChainBase::calculateKinematicsDynamics(
		const rstrt::robot::JointState& jointState) {
	jointStates_KDL.q.data = jointState.angles.cast<double>();
	jointStates_KDL.qdot.data = jointState.velocities.cast<double>();
	/*id_solver->JntToGravity(jointStates_KDL.q, G_kdl);
	id_solver->JntToCoriolis(jointStates_KDL.q, jointStates_KDL.qdot, C_kdl);
	id_solver->JntToMass(jointStates_KDL.q, M_kdl);
	M = M_kdl.data.cast<float>();
	C = C_kdl.data.cast<float>();
	G = G_kdl.data.cast<float>();
	*/
	jnt_to_jac_solver->JntToJac(jointStates_KDL.q, jac_kdl,
			activeChain_KDL.getNrOfSegments());
	jnt_to_jacDot_solver->JntToJacDot(jointStates_KDL, jacd_kdl,
			activeChain_KDL.getNrOfSegments());

	jnt_to_cart_pos_solver->JntToCart(jointStates_KDL.q, cart_pos,
			activeChain_KDL.getNrOfSegments());
	jnt_to_cart_vel_solver->JntToCart(jointStates_KDL, cart_vel,
			activeChain_KDL.getNrOfSegments());
	jac = jac_kdl.data.cast<float>();
	jacd = jacd_kdl.data.cast<float>();
	
	model_cf.calc_inertia_matrix(M_cf_array,jointStates_KDL.q.data.data());
	M_cf=Eigen::Map<Eigen::MatrixXd>(*M_cf_array,7,7).cast<float>();
	model_cf.calc_coriolis_matrix(C_cf_array,jointStates_KDL.q.data.data(),jointStates_KDL.qdot.data.data());
        C_cf=(Eigen::Map<Eigen::MatrixXd>(*C_cf_array,7,7)*jointStates_KDL.qdot.data).cast<float>();

}
std::string ChainBase::getName() {
	return this->name;
}
void ChainBase::setDOF(unsigned int DOF) {
	this->DOFsize = DOF;
}

void ChainBase::setBaseAndTip(std::string base, std::string tip) {
	this->base_name = base;
	this->tip_name = tip;
}

