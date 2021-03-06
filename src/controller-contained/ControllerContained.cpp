/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#include "../controller-contained/ControllerContained.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file

ControllerContained::ControllerContained(
		std::string const & name) :
		RTT::TaskContext(name), _models_loaded(false), gravity_vector(0, 0,
				-9.81), DOFsize(7), robot_state(7) {
	this->addOperation("setBaseAndTip",
			&ControllerContained::setBaseAndTip, this,
			RTT::ClientThread).doc("Set base and tip of the kinematic chain");
	this->addOperation("loadURDFAndSRDF",
			&ControllerContained::loadURDFAndSRDF, this,
			RTT::ClientThread);
	this->addOperation("setGains", &ControllerContained::setGains,
			this, RTT::ClientThread);
	this->addOperation("setPos", &ControllerContained::setPos,
			this, RTT::ClientThread);

	robot_state_port.setName("robot_in");
	robot_state_port.doc("Robot Input");
	ports()->addPort(robot_state_port);
	robot_state_flow = RTT::NoData;
	robot_state.angles.setZero();
	robot_state.torques.setZero();
	robot_state.velocities.setZero();

	ojt_port.setName("torques_out");
	jt_var.torques.resize(DOFsize);
	jt_var.torques.setZero();
	ojt_port.setDataSample(jt_var);
	ports()->addPort(ojt_port);
	quat_t1.resize(4);
	properties()->addProperty("quat_x", quat_t1(0));
	properties()->addProperty("quat_y", quat_t1(1));
	properties()->addProperty("quat_z", quat_t1(2));
	properties()->addProperty("quat_w", quat_t1(3));
	nullspace = true;
	properties()->addProperty("nullspace_angles", nullspace);
}

bool ControllerContained::configureHook() {
	return true;
}
bool ControllerContained::startHook() {
	if (robot_state_port.connected() && ojt_port.connected()) {
		return true;
	}
	return false;

}
void ControllerContained::updateHook() {
	robot_state_flow = robot_state_port.read(robot_state);
	if (robot_state_flow != RTT::NewData) {
		return;
	}
	//calculate relevant kinematics and dynamics
	calculateKinematicsDynamics(robot_state);
	Eigen::VectorXf error_pos, error_vel, tacc, quat_t2, force, null, damping;
	Eigen::VectorXd quat_temp;
	Eigen::Quaternionf quat_t, quat;

	//Damping term for simulation! This may not be needed for the real robot! Don't use initially with real robot!
	damping.resize(7);
//	damping << 5, 5, 3, 3, 1, 1, 1;
	damping<<0,0,0,0,0,0,0;

	quat_t2.resize(4);

	quat_temp.resize(4);

	quat_temp.setZero();
	quat_t2.setZero();
	error_pos.resize(6);
	error_vel.resize(6);
	null.resize(7);
	null.setZero();
	tacc.resize(6);
	force.resize(6);
	error_pos.setZero();
	error_vel.setZero();
	tacc.setZero();
	force.setZero();

	//calculate error_positions and velocities
	error_pos(0) = pos(0) - cart_pos.p.x();
	error_pos(1) = pos(1) - cart_pos.p.y();
	error_pos(2) = pos(2) - cart_pos.p.z();
	error_vel(0) = -cart_vel.GetTwist().vel.x();
	error_vel(1) = -cart_vel.GetTwist().vel.y();
	error_vel(2) = -cart_vel.GetTwist().vel.z();
	error_vel(3) = -cart_vel.GetTwist().rot.x();
	error_vel(4) = -cart_vel.GetTwist().rot.y();
	error_vel(5) = -cart_vel.GetTwist().rot.z();

	//retrieve Quarternion from KDL
	cart_pos.M.GetQuaternion(quat_temp(0), quat_temp(1), quat_temp(2),
			quat_temp(3));
	quat_t2 = quat_temp.cast<float>();

	//make sure input quat is normalized
	quat_t1.normalize();

	//calculate input quat skew matrix
	Eigen::MatrixXf quatx(3, 3);
	quatx.setZero();
	quatx(1, 2) = -quat_t1(0);
	quatx(2, 1) = quat_t1(0);
	quatx(2, 0) = -quat_t1(1);
	quatx(0, 2) = quat_t1(1);
	quatx(0, 1) = -quat_t1(2);
	quatx(1, 0) = quat_t1(2);
	quatx.transposeInPlace();

	//calculate quaternion error
	Eigen::VectorXf delta_quat(3);
	delta_quat = (quat_t1(3) * quat_t2.head<3>())
			- (quat_t2(3) * quat_t1.head<3>()) - (quatx * quat_t2.head<3>());
	//put quaternion error into error_pos
	error_pos.tail<3>() = -delta_quat;

	//Multiply by position and damping gains
	error_pos *= P;
	error_vel *= D;
	tacc = error_pos + error_vel;

	//calculate relevant jacobians
	Eigen::MatrixXf jac_c = jac;
	jac_c.setZero();
	Eigen::MatrixXf jac_cd = jacd;
	jac_cd.setZero();
	Eigen::MatrixXf jac_c_MPI = calcMPI(jac_c);

	Eigen::MatrixXf P = Eigen::MatrixXf::Identity(7, 7) - (jac_c_MPI * jac_c);

	Eigen::MatrixXf M_c = (P * M) + Eigen::MatrixXf::Identity(7, 7) - P;
	Eigen::MatrixXf C_c = -jac_c_MPI * jac_cd;
	Eigen::MatrixXf Lambda_c_inv = jac * M_c.inverse() * P * jac.transpose();
	Eigen::MatrixXf Lambda_c = calcMPI(Lambda_c_inv);

	Eigen::MatrixXf Lambda_MPI = Lambda_c * jac * M_c.inverse() * P;

	// Normal inverse dynamics controller
	force = (((jac * M.inverse() * jac.transpose()).inverse()) * tacc)
			+ (((jac * M.inverse() * jac.transpose()).inverse()) * jac
					* M.inverse()
					* (C + G + (damping.cwiseProduct(robot_state.velocities))))
			- (((jac * M.inverse() * jac.transpose()).inverse())
					* (jacd * robot_state.velocities));

	//calculate nullspace
	Eigen::MatrixXf iden;
	iden.resize(7, 7);
	iden.setIdentity();
	Eigen::VectorXf angles = -robot_state.angles;
	if (nullspace)
		angles.tail<3>() << 0, 0, 0;
	//null = (NP * -robot_state.angles) + (ND * -robot_state.velocities);
//	for (unsigned int t = 0; t < 7; t++) {
//		if (robot_state.angles(t) > 2)
//			null(t) = robot_state - 2;
//
//	}
	//output torques
	jt_var.torques = (jac.transpose() * force)
			+ ((iden
					- (jac.transpose()
							* ((jac * M.inverse() * jac.transpose()).inverse()
									* jac * M.inverse()))) * null);
	//write to output torque port
	ojt_port.write(jt_var);

}
Eigen::VectorXf ControllerContained::calcQuat(
		Eigen::MatrixXf rot) {
	float m22 = rot(2, 2);
	float m00 = rot(0, 0);
	float m11 = rot(1, 1);
	float m01 = rot(0, 1);
	float m10 = rot(1, 0);
	float m20 = rot(2, 0);
	float m02 = rot(0, 2);
	float m21 = rot(2, 1);
	float m12 = rot(1, 2);
	float t = 0;
	Eigen::VectorXf q;
	q.resize(4);
	if (m22 < 0) {
		if (m00 > m11) {
			t = 1 + m00 - m11 - m22;
			q << t, m01 + m10, m20 + m02, m12 - m21;
		} else {
			t = 1 - m00 + m11 - m22;
			q << m01 + m10, t, m12 + m21, m20 - m02;
		}
	} else {
		if (m00 < -m11) {
			t = 1 - m00 - m11 + m22;
			q << m20 + m02, m12 + m21, t, m01 - m10;
		} else {
			t = 1 + m00 + m11 + m22;
			q << m12 - m21, m20 - m02, m01 - m10, t;
		}
	}
	q *= 0.5 / sqrt(t);
	return q;
}

void ControllerContained::stopHook() {
}
void ControllerContained::cleanupHook() {
}

bool ControllerContained::exists_test(const std::string& name) {
	if (FILE *file = fopen(name.c_str(), "r")) {
		fclose(file);
		return true;
	} else {
		return false;
	}
}
bool ControllerContained::selectKinematicChain(
		const std::string& chainName) {
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

	if (activeChain_KDL.getNrOfJoints() != DOFsize) {
		RTT::log(RTT::Info) << "DOFsize " << DOFsize
				<< " is different from urdf model" << RTT::endlog();
		assert(false); //TODO
		return false;
	}

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
	return true;
}

bool ControllerContained::loadURDFAndSRDF(
		const std::string &URDF_path, const std::string &SRDF_path) {
	if (!_models_loaded) {
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();
		assert(exists_test(_urdf_path) == true);
		assert(exists_test(_srdf_path) == true);
		_models_loaded = xbot_model.init(_urdf_path, _srdf_path);

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

void ControllerContained::calculateKinematicsDynamics(
		const rstrt::robot::JointState& jointState) {
	jointStates_KDL.q.data = jointState.angles.cast<double>();
	jointStates_KDL.qdot.data = jointState.velocities.cast<double>();
	id_solver->JntToGravity(jointStates_KDL.q, G_kdl);
	id_solver->JntToCoriolis(jointStates_KDL.q, jointStates_KDL.qdot, C_kdl);
	id_solver->JntToMass(jointStates_KDL.q, M_kdl);
	M = M_kdl.data.cast<float>();
	C = C_kdl.data.cast<float>();
	G = G_kdl.data.cast<float>();
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

}

void ControllerContained::setBaseAndTip(std::string base,
		std::string tip) {
	this->base_name = base;
	this->tip_name = tip;
}
void ControllerContained::setPos(float x, float y, float z) {
	pos(0) = x;
	pos(1) = y;
	pos(2) = z;
}

void ControllerContained::setGains(double p, double d, double np,
		double nd) {
	P = p;
	D = d;
	NP = np;
	ND = nd;
}

Eigen::MatrixXf ControllerContained::calcMPI(
		Eigen::MatrixXf input) {
	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(input.rows(), input.cols());
	svd_solver.compute(input, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values =
			svd_solver.singularValues();
	for (int i = 0; i < singular_values.size(); i++) {
		if (singular_values(i) < 1.e-06) {
			singular_values(i) = 0;
		} else {
			singular_values(i) = 1 / singular_values(i);
		}
	}
	return svd_solver.matrixV().leftCols(singular_values.size())
			* singular_values.asDiagonal()
			* svd_solver.matrixU().leftCols(singular_values.size()).transpose();

}
ORO_CREATE_COMPONENT_LIBRARY()
// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(ControllerContained)

