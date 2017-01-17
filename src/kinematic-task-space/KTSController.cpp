/*
 * KTSController.cpp
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#include "KTSController.h"
#include <rtt/Component.hpp>

KTSController::KTSController(std::string const & name) :
		RTT::TaskContext(name), ChainBase(name) {
	this->addOperation("setBaseAndTip", &KTSController::setBaseAndTip, this,
			RTT::ClientThread).doc("Set base and tip of the kinematic chain");
	this->addOperation("loadURDFAndSRDF", &KTSController::loadURDFAndSRDF, this,
			RTT::ClientThread);
	this->addOperation("setPos", &KTSController::setPos, this,
			RTT::ClientThread);
	this->addOperation("setStepSize", &KTSController::setStepSize, this,
			RTT::ClientThread);
	step_size = 0.1;
	pos.setZero();
	quat_d.resize(4);
	quat_d.setZero();
	Kp = 10;
	Dp = 0.2;
	Ko = 10;
	Do = 0.2;
	properties()->addProperty("quat_x", quat_d(0));
	properties()->addProperty("quat_y", quat_d(1));
	properties()->addProperty("quat_z", quat_d(2));
	properties()->addProperty("quat_w", quat_d(3));
	properties()->addProperty("Kp", Kp);
	properties()->addProperty("Dp", Dp);
	properties()->addProperty("Ko", Ko);
	properties()->addProperty("Do", Do);

	
	//ports()->addPort(robot_state_port);
	//out_angles_var.angles.resize(DOFsize);
	//out_angles_var.angles.setZero();
	//out_angles_port.setName("ja_out");
	//out_angles_port.setDataSample(out_angles_var);
	//ports()->addPort(out_angles_port);

}

KTSController::~KTSController() {

}

bool KTSController::configureHook() {
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
	return true;

}
bool KTSController::startHook() {
	return (robot_state_port.connected()); //&& out_angles_port.connected()
			//&& in_M_port.connected());
}
void KTSController::updateHook() {
	
	robot_state_flow = robot_state_port.read(robot_state);
	if (robot_state_flow != RTT::NewData) {
		return;
	}
	in_M_flow = in_M_port.read(in_M);
//	if (in_M_flow != RTT::NewData) {
//		return;
//	}
	in_x_des_flow = in_x_des_port.read(in_x_des);
	in_xd_des_flow = in_xd_des_port.read(in_xd_des);


	RTT::log(RTT::Info) <<name<<" Start" << RTT::endlog();
//	calculateKinematics(robot_state);
	calculateKinematicsDynamics(robot_state);
	//RTT::log(RTT::Info) << cart_pos.p << RTT::endlog();
	Eigen::VectorXd quat_temp(4);
	cart_pos.M.GetQuaternion(quat_temp(0), quat_temp(1), quat_temp(2),
			quat_temp(3));
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

	//calculate input quat skew matrix
	Eigen::MatrixXf quat_skew(3, 3);
	quat_skew.setZero();
	quat_skew(1, 2) = -quat_d(0);
	quat_skew(2, 1) = quat_d(0);
	quat_skew(2, 0) = -quat_d(1);
	quat_skew(0, 2) = quat_d(1);
	quat_skew(0, 1) = -quat_d(2);
	quat_skew(1, 0) = quat_d(2);
	//RTT::log(RTT::Info) << quat_skew << "\n :QUAT SKEW MATRIX" << RTT::endlog();
//	quat_skew.transposeInPlace();

	Eigen::VectorXf delta_quat(3);
	delta_quat = (quat_d(3) * quat.head<3>()) - (quat(3) * quat_d.head<3>())
			- (quat_skew * quat.head<3>());

	////RTT::log(RTT::Info)<<pos<<RTT::endlog();
	Eigen::VectorXf posError(6);
	if(true){
	posError[0] = pos[0] - cart_pos.p.data[0];
	posError[1] = pos[1] - cart_pos.p.data[1];
	posError[2] = pos[2] - cart_pos.p.data[2];
	KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
			quat_d(2), quat_d(3));
	Eigen::Matrix3f rot_mat;
//	rot_mat(0,0) = 1-2*pow(quat_d(1),2)-2*pow(quat_d(2),2);
//	rot_mat(0,1) = 2*quat_d(0)*quat_d(1) - 2*quat_d(2)*quat_d(3);
//	rot_mat(0,2) = 2*quat_d(0)*quat_d(2) + 2*quat_d(1)*quat_d(3);
//	rot_mat(1,0) = 2*quat_d(0)*quat_d(1) + 2*quat_d(2)*quat_d(3);
//	rot_mat(1,1) = 1-2*pow(quat_d(0),2)-2*pow(quat_d(2),2);
//	rot_mat(1,2) = 2*quat_d(1)*quat_d(2) - 2*quat_d(0)*quat_d(3);
//	rot_mat(2,0) = 2*quat_d(0)*quat_d(2) - 2*quat_d(1)*quat_d(3);
//	rot_mat(2,1) = 2*quat_d(2)*quat_d(1) + 2*quat_d(0)*quat_d(3);
//	rot_mat(2,2) = 1-2*pow(quat_d(0),2)-2*pow(quat_d(1),2);
//	rot_mat(0, 0) = 1 - 2 * pow(quat_d(1), 2) - 2 * pow(quat_d(2), 2);
//	rot_mat(0, 1) = 2 * quat_d(0) * quat_d(1) - 2 * quat_d(2) * quat_d(3);
//	rot_mat(0, 2) = 2 * quat_d(0) * quat_d(2) + 2 * quat_d(1) * quat_d(3);
//	rot_mat(1, 0) = 2 * quat_d(0) * quat_d(1) + 2 * quat_d(2) * quat_d(3);
//	rot_mat(1, 1) = 1 - 2 * pow(quat_d(0), 2) - 2 * pow(quat_d(2), 2);
//	rot_mat(1, 2) = 2 * quat_d(1) * quat_d(2) - 2 * quat_d(0) * quat_d(3);
//	rot_mat(2, 0) = 2 * quat_d(0) * quat_d(2) - 2 * quat_d(1) * quat_d(3);
//	rot_mat(2, 1) = 2 * quat_d(2) * quat_d(1) + 2 * quat_d(0) * quat_d(3);
//	rot_mat(2, 2) = 1 - 2 * pow(quat_d(0), 2) - 2 * pow(quat_d(1), 2);
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
	}else{
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
//		posError.head<3>() *= Kp;
//		posError.tail<3>() *= Ko;

//RTT::log(RTT::Info) << "POSERROR: " << posError << RTT::endlog();
	//if (posError.head<3>().norm() > step_size) {
//		posError.head<3>() /= (posError.head<3>().norm() / (float) step_size);
//	}

//	Eigen::MatrixXf jacResize(3, jac.cols());
//	jacResize = jac.block(0, 0, 3, jac.cols());
////RTT::log(RTT::Info)<<jacResize<<RTT::endlog();

	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(jac.rows(), jac.cols());
	svd_solver.compute(jac, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values =
			svd_solver.singularValues();
	for (int i = 0; i < singular_values.size(); i++) {
		if (singular_values(i) < 1e-04) {
			singular_values(i) = 0;
		} else {
			singular_values(i) = 1 / singular_values(i);
		}
	}

	Eigen::MatrixXf jacPinv = svd_solver.matrixV().leftCols(
			singular_values.size()) * singular_values.asDiagonal()
			* svd_solver.matrixU().leftCols(singular_values.size()).transpose();

	//Eigen::MatrixXf in_M_Pinv = M_cf.inverse();


	//out_angles_var.angles = robot_state.angles +(jac.transpose()*(jac*jac.transpose()).inverse())*posError;//+jacPinv*posError;
//	out_angles_var.angles[5] += 0.2 * 0.005;
	////RTT::log(RTT::Info)<<out_angles_var.angles<<RTT::endlog();
//	out_angles_port.write(out_angles_var);
//	//RTT::log(RTT::Info)
//			<< (jac.transpose()
//					* (jac * jac.transpose()).inverse()) * posError
//			<< ": JOINTPOS ERROR" << RTT::endlog();
	out_torques_var.torques = (Kp
			* (jacPinv * posError + ((Eigen::Matrix<float,7,7>::Identity()-jacPinv*jac)*-robot_state.angles)))
			- (Dp * (robot_state.velocities));
	//out_torques_var.torques.setZero();

//	//RTT::log(RTT::Info) << out_torques_var.torques << RTT::endlog();

//
//	Eigen::VectorXf velError(6);
//	if(in_xd_des_flow==RTT::NoData){
//	velError.head<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
//			cart_vel.GetTwist().vel.data).cast<float>() * -Dp;
//	velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
//			cart_vel.GetTwist().rot.data).cast<float>() * -Do;
//	}
//	else{
//	velError.head<3>() = ((Eigen::Map<Eigen::Array<double, 3, 1>>(
//			cart_vel.GetTwist().vel.data).cast<float>()));
//	velError.head<3>() = in_xd_des.head<3>()-velError.head<3>();
//	velError.head<3>() *= Dp;
//	velError.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
//			cart_vel.GetTwist().rot.data).cast<float>() * -Do;
//	}
//
//	Eigen::VectorXf xdd(6);
//
//	xdd = posError + velError;
//	Eigen::VectorXf damping(7);
//	damping << 5, 5, 3, 3, 1, 1, 1;
//
////	Eigen::MatrixXf jmjt = (jac * M.inverse() * jac.transpose()).inverse();
//	Eigen::MatrixXf jmjtinv = (jac * in_M_Pinv * jac.transpose());
//
//	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver2(jmjtinv.rows(), jmjtinv.cols());
//	svd_solver2.compute(jmjtinv, Eigen::ComputeFullU | Eigen::ComputeFullV);
//
//	Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values2 =
//			svd_solver2.singularValues();
//	for (int i = 0; i < singular_values2.size(); i++) {
//		if (singular_values2(i) < 1.e-06) {
//			singular_values2(i) = 0;
//		} else {
//			singular_values2(i) = 1 / singular_values2(i);
//		}
//	}
//
//	Eigen::MatrixXf jmjt = svd_solver2.matrixV().leftCols(
//			singular_values2.size()) * singular_values2.asDiagonal()
//			* svd_solver2.matrixU().leftCols(singular_values2.size()).transpose();
//
//	out_torques_var.torques = jac.transpose()
//			* ((jmjt * (xdd - jacd * robot_state.velocities))+jmjt*jac*in_M_Pinv*C_cf)
//			;//+ (robot_state.velocities.cwiseProduct(damping));
//
//	//RTT::log(RTT::Info) << quat_temp <<": QUATERNION VALUE" << RTT::endlog();
//	//RTT::log(RTT::Info) << in_M<<":\n"<<M <<": IN_M VALUE" << RTT::endlog();
//	//RTT::log(RTT::Info) << M_cf<<": M_cf VALUE" << RTT::endlog();
//	//RTT::log(RTT::Info) << C_cf<<": C_cf VALUE" << RTT::endlog();
//
//	//out_torques_var.torques.setZero();
	out_torques_port.write(out_torques_var);
//	RTT::log(RTT::Info) <<name<<" Stop" << RTT::endlog();

}
void KTSController::stopHook() {

}
void KTSController::cleanupHook() {

}

void KTSController::setPos(float x, float y, float z) {
	//RTT::log(RTT::Info) << "SETTING POS" << x << ", " << y << ", " << z << "\n";
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
}
ORO_LIST_COMPONENT_TYPE(KTSController)
