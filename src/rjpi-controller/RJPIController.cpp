/*
 * KTSController.cpp
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#include "RJPIController.h"
#include <rtt/Component.hpp>

RJPIController::RJPIController(std::string const & name) :
		RTT::TaskContext(name), ChainBase(name), dual_arm_chain(name + "_dual_") {
	this->addOperation("setBaseAndTip", &RJPIController::setBaseAndTip, this,
			RTT::ClientThread).doc("Set base and tip of the kinematic chain");
	this->addOperation("loadURDFAndSRDF", &RJPIController::loadURDFAndSRDF,
			this, RTT::ClientThread);
	this->addOperation("setPos", &RJPIController::setPos, this,
			RTT::ClientThread);
	this->addOperation("setPos_c", &RJPIController::setPos_c, this,
			RTT::ClientThread);
	this->addOperation("setStepSize", &RJPIController::setStepSize, this,
			RTT::ClientThread);
	this->addOperation("loadURDFAndSRDF_dual", &ChainBase::loadURDFAndSRDF,
			&dual_arm_chain, RTT::ClientThread);
	this->addOperation("setBaseAndTip_dual", &ChainBase::setBaseAndTip,
			&dual_arm_chain, RTT::ClientThread);
	step_size = 0.1;
	pos.setZero();
	quat_d.resize(4);
	quat_d.setZero();
	quat_c.resize(4);
	quat_c.setZero();
	jac_c.setZero();
	P.setZero();
	Pd.setZero();
	xdd_des.setZero();
	F_x.setZero();
	tau_c.setZero();
	tau_0.setZero();
	Kp = 10;
	Dp = 0.2;
	Ko = 10;
	Do = 0.2;
	Kp_c = 10;
		Dp_c = 0.2;
		Ko_c = 10;
		Do_c = 0.2;
	properties()->addProperty("quat_x", quat_d(0));
	properties()->addProperty("quat_y", quat_d(1));
	properties()->addProperty("quat_z", quat_d(2));
	properties()->addProperty("quat_w", quat_d(3));
	properties()->addProperty("quat_x_c", quat_c(0));
	properties()->addProperty("quat_y_c", quat_c(1));
	properties()->addProperty("quat_z_c", quat_c(2));
	properties()->addProperty("quat_w_c", quat_c(3));
	properties()->addProperty("Kp", Kp);
	properties()->addProperty("Dp", Dp);
	properties()->addProperty("Ko", Ko);
	properties()->addProperty("Do", Do);
	properties()->addProperty("Kp_c", Kp_c);
		properties()->addProperty("Dp_c", Dp_c);
		properties()->addProperty("Ko_c", Ko_c);
		properties()->addProperty("Do_c", Do_c);

	//ports()->addPort(robot_state_port);
	//out_angles_var.angles.resize(DOFsize);
	//out_angles_var.angles.setZero();
	//out_angles_port.setName("ja_out");
	//out_angles_port.setDataSample(out_angles_var);
	//ports()->addPort(out_angles_port);

}

RJPIController::~RJPIController() {

}

bool RJPIController::configureHook() {
	ports()->addPort(robot_state_port);
	out_angles_var.angles.resize(DOFsize);
	out_angles_var.angles.setZero();
	out_angles_port.setName("ja_out");
	out_angles_port.setDataSample(out_angles_var);
	ports()->addPort(out_angles_port);
	out_torques_var.torques.resize(DOFsize);
	out_torques_var.torques.setZero();
	out_torques_port.setName("jt_out");
	out_torques_port.setDataSample(out_torques_var);
	ports()->addPort(out_torques_port);
	in_M.setZero();
	in_M_port.setName("in_M");
	in_M_port.doc("input inertia matrix");
	in_M_flow = RTT::NoData;
	ports()->addPort(in_M_port);
	in_x_des.resize(6);
	in_x_des.setZero();
	in_x_des_port.setName("in_x_des");
	in_x_des_flow = RTT::NoData;
	ports()->addPort(in_x_des_port);
	in_xd_des.resize(6);
	in_xd_des.setZero();
	in_xd_des_port.setName("in_xd_des");
	in_xd_des_flow = RTT::NoData;
	ports()->addPort(in_xd_des_port);

	ports()->addPort(dual_arm_chain.robot_state_port);

	return true;

}
bool RJPIController::startHook() {
	return (robot_state_port.connected()
			&& dual_arm_chain.robot_state_port.connected()); //&& out_angles_port.connected());
	//&& in_M_port.connected());
}
void RJPIController::updateHook() {

	robot_state_flow = robot_state_port.read(robot_state);
	if (robot_state_flow != RTT::NewData) {
		return;
	}

	dual_arm_chain.robot_state_flow = dual_arm_chain.robot_state_port.read(
			dual_arm_chain.robot_state);
	if (dual_arm_chain.robot_state_flow != RTT::NewData) {
		return;
	}
	dual_arm_chain.robot_state.angles.head<7>().reverseInPlace();
	dual_arm_chain.robot_state.velocities.head<7>().reverseInPlace();

	in_M_flow = in_M_port.read(in_M);
//	if (in_M_flow != RTT::NewData) {
//		return;
//	}
	in_x_des_flow = in_x_des_port.read(in_x_des);
	in_xd_des_flow = in_xd_des_port.read(in_xd_des);

	RTT::log(RTT::Info) << name << " Start" << RTT::endlog();
//	calculateKinematics(robot_state);
	calculateKinematicsDynamics(robot_state);
	dual_arm_chain.calculateKinematicsDynamics(dual_arm_chain.robot_state);
	dual_arm_chain.robot_state.angles.head<7>().reverseInPlace();
		dual_arm_chain.robot_state.velocities.head<7>().reverseInPlace();
	//RTT::log(RTT::Info) << cart_pos.p << RTT::endlog();
	Eigen::VectorXd quat_temp(4);
	dual_arm_chain.cart_pos.M.GetQuaternion(quat_temp(0), quat_temp(1),
			quat_temp(2), quat_temp(3));
	Eigen::Matrix3f rot_mat_curr;
	rot_mat_curr(0, 0) = cart_pos.M.data[0];
	rot_mat_curr(0, 1) = cart_pos.M.data[1];
	rot_mat_curr(0, 2) = cart_pos.M.data[2];
	rot_mat_curr(1, 0) = cart_pos.M.data[3];
	rot_mat_curr(1, 1) = cart_pos.M.data[4];
	rot_mat_curr(1, 2) = cart_pos.M.data[5];
	rot_mat_curr(2, 0) = cart_pos.M.data[6];
	rot_mat_curr(2, 1) = cart_pos.M.data[7];
	rot_mat_curr(2, 2) = cart_pos.M.data[8];
	Eigen::Quaternionf eigen_quat(rot_mat_curr);
	//RTT::log(RTT::Info) << eigen_quat.vec() << ", " << quat_temp
	//<< "\nCOMPARISON TEST" << RTT::endlog();
//	//RTT::log(RTT::Info)<<eigen_quat(0)-quat_temp(0)<<","<<eigen_quat(1)-quat_temp(1)<<", "<<eigen_quat(2)-quat_temp(2)<<", "<<eigen_quat(3)-quat_temp(3)<<"\n QUATERNION ERROR"<<RTT::endlog();

	Eigen::VectorXf quat = quat_temp.cast<float>();
	quat_d.normalize();

	Eigen::VectorXf delta_quat(3);

	////RTT::log(RTT::Info)<<pos<<RTT::endlog();
	Eigen::VectorXf posError(6);
	if (in_x_des_flow == RTT::NoData) {
		posError[0] = pos[0] - cart_pos.p.data[0];
		posError[1] = pos[1] - cart_pos.p.data[1];
		posError[2] = pos[2] - cart_pos.p.data[2];
		KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
				quat_d(2), quat_d(3));
		Eigen::Matrix3f rot_mat;
		rot_mat(0, 0) = rot_KDL.data[0];
		rot_mat(0, 1) = rot_KDL.data[1];
		rot_mat(0, 2) = rot_KDL.data[2];
		rot_mat(1, 0) = rot_KDL.data[3];
		rot_mat(1, 1) = rot_KDL.data[4];
		rot_mat(1, 2) = rot_KDL.data[5];
		rot_mat(2, 0) = rot_KDL.data[6];
		rot_mat(2, 1) = rot_KDL.data[7];
		rot_mat(2, 2) = rot_KDL.data[8];
		Eigen::Matrix3f relative_rot = rot_mat.transpose() * rot_mat_curr;
		Eigen::Quaternionf relative_quat(relative_rot);
		delta_quat = relative_quat.vec().cast<float>();

		//RTT::log(RTT::Info) << rot_mat << RTT::endlog();
		//RTT::log(RTT::Info) << -rot_mat * delta_quat << ":delta_quat"
		//<< RTT::endlog();

		posError.tail<3>() = -rot_mat * delta_quat;
	} else {
		posError[0] = in_x_des[0] - cart_pos.p.data[0];
		posError[1] = in_x_des[1] - cart_pos.p.data[1];
		posError[2] = in_x_des[2] - cart_pos.p.data[2];
		KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
				quat_d(2), quat_d(3));
		Eigen::Matrix3f rot_mat;
		rot_mat(0, 0) = rot_KDL.data[0];
		rot_mat(0, 1) = rot_KDL.data[1];
		rot_mat(0, 2) = rot_KDL.data[2];
		rot_mat(1, 0) = rot_KDL.data[3];
		rot_mat(1, 1) = rot_KDL.data[4];
		rot_mat(1, 2) = rot_KDL.data[5];
		rot_mat(2, 0) = rot_KDL.data[6];
		rot_mat(2, 1) = rot_KDL.data[7];
		rot_mat(2, 2) = rot_KDL.data[8];
		Eigen::Matrix3f relative_rot = rot_mat.transpose() * rot_mat_curr;
		Eigen::Quaternionf relative_quat(relative_rot);
		delta_quat = relative_quat.vec().cast<float>();
		posError.tail<3>() = -rot_mat * delta_quat;
	}

	Eigen::MatrixXf in_M_Pinv = M_cf.inverse();
	posError.head<3>() *= Kp;
	posError.tail<3>() *= Ko;

	Eigen::VectorXf velError(6);
	if (in_xd_des_flow == RTT::NoData) {
		velError.head<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				cart_vel.GetTwist().vel.data).cast<float>() * -Dp;
		velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				cart_vel.GetTwist().rot.data).cast<float>() * -Do;
	} else {
		velError.head<3>() = ((Eigen::Map<Eigen::Array<double, 3, 1>>(
				cart_vel.GetTwist().vel.data).cast<float>()));
		velError.head<3>() = in_xd_des.head<3>() - velError.head<3>();
		velError.head<3>() *= Dp;
		velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				cart_vel.GetTwist().rot.data).cast<float>() * -Do;
	}

	Eigen::VectorXf xdd(6);

	xdd = posError + velError;
	dual_arm_chain.jointStates_KDL.q.data.head<7>().reverseInPlace();
	dual_arm_chain.jointStates_KDL.qdot.data.head<7>().reverseInPlace();

	model_cf.calc_inertia_matrix(M_cf_array,
			dual_arm_chain.jointStates_KDL.q.data.head<7>().data());
	M_cf = Eigen::Map<Eigen::MatrixXd>(*M_cf_array, 7, 7).cast<float>();
	model_cf.calc_coriolis_matrix(C_cf_array,
			dual_arm_chain.jointStates_KDL.q.data.head<7>().data(),
			dual_arm_chain.jointStates_KDL.qdot.data.head<7>().data());
	C_cf = (Eigen::Map<Eigen::MatrixXd>(*C_cf_array, 7, 7)
			* dual_arm_chain.jointStates_KDL.qdot.data.head<7>()).cast<float>();

	model_cf.calc_inertia_matrix(M_cf_array,
			dual_arm_chain.jointStates_KDL.q.data.tail<7>().data());
	Eigen::MatrixXf right_M_cf =
			Eigen::Map<Eigen::MatrixXd>(*M_cf_array, 7, 7).cast<float>();
	model_cf.calc_coriolis_matrix(C_cf_array,
			dual_arm_chain.jointStates_KDL.q.data.tail<7>().data(),
			dual_arm_chain.jointStates_KDL.qdot.data.tail<7>().data());
	Eigen::VectorXf right_C_cf = (Eigen::Map<Eigen::MatrixXd>(*C_cf_array, 7, 7)
			* dual_arm_chain.jointStates_KDL.qdot.data.tail<7>()).cast<float>();
	Eigen::MatrixXf dual_arm_M(14, 14);
	dual_arm_M.setZero();
	dual_arm_M.block<7, 7>(0, 0) = M_cf;
	dual_arm_M.block<7, 7>(7, 7) = right_M_cf;
	Eigen::VectorXf dual_arm_C(14);
	dual_arm_C.head<7>() = C_cf;
	dual_arm_C.tail<7>() = right_C_cf;
	Eigen::VectorXf dual_pos_error(12);
	dual_pos_error.head<6>() = posError;

//	Eigen::Matrix3f rot_mat_curr;
	rot_mat_curr(0, 0) = dual_arm_chain.cart_pos.M.data[0];
	rot_mat_curr(0, 1) = dual_arm_chain.cart_pos.M.data[1];
	rot_mat_curr(0, 2) = dual_arm_chain.cart_pos.M.data[2];
	rot_mat_curr(1, 0) = dual_arm_chain.cart_pos.M.data[3];
	rot_mat_curr(1, 1) = dual_arm_chain.cart_pos.M.data[4];
	rot_mat_curr(1, 2) = dual_arm_chain.cart_pos.M.data[5];
	rot_mat_curr(2, 0) = dual_arm_chain.cart_pos.M.data[6];
	rot_mat_curr(2, 1) = dual_arm_chain.cart_pos.M.data[7];
	rot_mat_curr(2, 2) = dual_arm_chain.cart_pos.M.data[8];
	eigen_quat = Eigen::Quaternionf(rot_mat_curr);

	posError[0] = pos_c[0] - dual_arm_chain.cart_pos.p.data[0];
	posError[1] = pos_c[1] - dual_arm_chain.cart_pos.p.data[1];
	posError[2] = pos_c[2] - dual_arm_chain.cart_pos.p.data[2];
	KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
			quat_d(2), quat_d(3));
	Eigen::Matrix3f rot_mat;
	rot_mat(0, 0) = rot_KDL.data[0];
	rot_mat(0, 1) = rot_KDL.data[1];
	rot_mat(0, 2) = rot_KDL.data[2];
	rot_mat(1, 0) = rot_KDL.data[3];
	rot_mat(1, 1) = rot_KDL.data[4];
	rot_mat(1, 2) = rot_KDL.data[5];
	rot_mat(2, 0) = rot_KDL.data[6];
	rot_mat(2, 1) = rot_KDL.data[7];
	rot_mat(2, 2) = rot_KDL.data[8];
	Eigen::Matrix3f relative_rot = rot_mat.transpose() * rot_mat_curr;
	Eigen::Quaternionf relative_quat(relative_rot);
	delta_quat = relative_quat.vec().cast<float>();
	posError.tail<3>() = -rot_mat * delta_quat;

	posError.head<3>() *= Kp_c;
		posError.tail<3>() *= Ko_c;
	dual_pos_error.tail<6>() = posError;

	Eigen::VectorXf dual_vel_error(12);
	dual_vel_error.head<6>() = velError;

	if (in_xd_des_flow == RTT::NoData) {
		velError.head<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				dual_arm_chain.cart_vel.GetTwist().vel.data).cast<float>()
				* -Dp_c;
		velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				dual_arm_chain.cart_vel.GetTwist().rot.data).cast<float>()
				* -Do_c;
	} else {
		velError.head<3>() = ((Eigen::Map<Eigen::Array<double, 3, 1>>(
				dual_arm_chain.cart_vel.GetTwist().vel.data).cast<float>()));
		velError.head<3>() = in_xd_des.head<3>() - velError.head<3>();
		velError.head<3>() *= Dp_c;
		velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				dual_arm_chain.cart_vel.GetTwist().rot.data).cast<float>()
				* -Do_c;
	}
	dual_vel_error.tail<6>() = velError;

	//RTT::log(RTT::Info) << quat_temp <<": QUATERNION VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << in_M<<":\n"<<M <<": IN_M VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << M_cf<<": M_cf VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << C_cf<<": C_cf VALUE" << RTT::endlog();

	//jac_c=vec*jac; FOR CALCULATING jac_c
	/*Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver_jac_c(jac_c.rows(), jac_c.cols());
	 svd_solver_jac_c.compute(jac_c, Eigen::ComputeFullU | Eigen::ComputeFullV);

	 Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values_jac_c =
	 svd_solver_jac_c.singularValues();
	 for (int i = 0; i < singular_values_jac_c.size(); i++) {
	 if (singular_values_jac_c(i) < 1.e-06) {
	 singular_values_jac_c(i) = 0;
	 } else {
	 singular_values_jac_c(i) = 1 / singular_values_jac_c(i);
	 }
	 }

	 Eigen::MatrixXf jac_c_Pinv = svd_solver_jac_c.matrixV().leftCols(
	 singular_values_jac_c.size()) * singular_values_jac_c.asDiagonal()
	 * svd_solver_jac_c.matrixU().leftCols(singular_values_jac_c.size()).transpose();
	 */
	P = Eigen::Matrix<float, 14, 14>::Identity();	//-(jac_c_Pinv*jac_c);
	M_c = (P * dual_arm_M) + Eigen::Matrix<float, 14, 14>::Identity() - P;
	jac_x.setZero();
	jacd_x.setZero();
//	jac_x.block<6, 7>(0, 0) = jac;
//	jacd_x.block<6, 7>(0, 0) = jacd;
	RTT::log(RTT::Info)<<dual_arm_chain.jac.rows()<<","<<dual_arm_chain.jac.cols()<<RTT::endlog();
	jac_x.block<6, 14>(0, 0) = dual_arm_chain.jac;
	jacd_x.block<6, 14>(0, 0) = dual_arm_chain.jacd;
	RTT::log(RTT::Info)<<"BEFORE:"<<jac_x.block<6, 7>(0, 0)<<RTT::endlog();
	//jac_x.block<6, 7>(0, 0).rowwise().reverseInPlace();
	//jacd_x.block<6, 7>(0, 0).rowwise().reverseInPlace();
	for(int i = 0; i<6; i++){
	  jac_x.block<1,7>(i,0).reverseInPlace();
	  jacd_x.block<1,7>(i,0).reverseInPlace();
	}
	RTT::log(RTT::Info)<<"AFTER:"<<jac_x.block<6, 7>(0, 0)<<RTT::endlog();

	lambda_c = (jac_x * (M_c.inverse()) * P * (jac_x.transpose())).inverse();
	h_c =
			(lambda_c * jac_x * M_c.inverse()
					* ((P * (dual_arm_C))
							- (Pd * dual_arm_chain.robot_state.velocities)))
					- (lambda_c * jacd_x * dual_arm_chain.robot_state.velocities);
	lambda_d = lambda_c;
//	F = h_c + lambda_c * xdd_des - lambda_c*lambda_d.inverse()*(velError+posError)+( lambda_c*lambda_d.inverse() - Eigen::Matrix<float,6,6>::Identity())*F_x;
	F = h_c + (lambda_c * xdd_des) - (-dual_vel_error.tail<6>() - dual_pos_error.tail<6>());
	RTT::log(RTT::Info)<<dual_vel_error<<", "<<dual_pos_error<<"\n";
	N = (Eigen::Matrix<float, 14, 14>::Identity()
			- (jac_x.transpose() * lambda_c * jac_x * M_c.inverse() * P));
	RTT::log(RTT::Info) << name << " F: " << F << RTT::endlog();
	RTT::log(RTT::Info) << name << " lambdaC norm: " << lambda_c.norm()
			<< RTT::endlog();;
	out_torques_var.torques = (P * jac_x.transpose() * F) + P * N * tau_0
			+ ((Eigen::Matrix<float, 14, 14>::Identity() - P) * tau_c);
//	out_torques_var.torques.head<7>().reverseInPlace();

//	out_torques_var.torques += P * N * tau_0
//			+ ((Eigen::Matrix<float, 14, 14>::Identity() - P) * tau_c);
	//out_torques_var.torques.setZero();
	out_torques_port.write(out_torques_var);
	RTT::log(RTT::Info) << name << " Stop" << RTT::endlog();

}
void RJPIController::stopHook() {

}
void RJPIController::cleanupHook() {

}

void RJPIController::setPos(float x, float y, float z) {
	//RTT::log(RTT::Info) << "SETTING POS" << x << ", " << y << ", " << z << "\n";
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
}
void RJPIController::setPos_c(float x, float y, float z) {
	//RTT::log(RTT::Info) << "SETTING POS" << x << ", " << y << ", " << z << "\n";
	pos_c[0] = x;
	pos_c[1] = y;
	pos_c[2] = z;
}
ORO_LIST_COMPONENT_TYPE(RJPIController)
