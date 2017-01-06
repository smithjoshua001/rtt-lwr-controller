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
	return true;

}
bool KTSController::startHook() {
	return (robot_state_port.connected() && out_angles_port.connected());
}
void KTSController::updateHook() {
	robot_state_flow = robot_state_port.read(robot_state);
	if (robot_state_flow != RTT::NewData) {
		return;
	}
	
	calculateKinematics(robot_state);
	RTT::log(RTT::Info)<<cart_pos.p<<RTT::endlog();
	//RTT::log(RTT::Info)<<pos<<RTT::endlog();
	Eigen::Vector3f posError;
	posError[0] = pos[0] - cart_pos.p.data[0];
	posError[1] = pos[1] - cart_pos.p.data[1];
	posError[2] = pos[2] - cart_pos.p.data[2];
	RTT::log(RTT::Info)<<"POSERROR: "<<posError<<RTT::endlog();
	if (posError.norm() > step_size) {
		posError /= (posError.norm() / (float) step_size);
	}
	
	Eigen::MatrixXf jacResize(3,jac.cols());
	jacResize = jac.block(0,0,3,jac.cols());
//RTT::log(RTT::Info)<<jacResize<<RTT::endlog();
	
	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(jacResize.rows(), jacResize.cols());
	svd_solver.compute(jacResize,Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values =
			svd_solver.singularValues();
	for (int i = 0; i < singular_values.size(); i++) {
		if (singular_values(i) < 1.e-06) {
			singular_values(i) = 0;
		} else {
			singular_values(i) = 1 / singular_values(i);
		}
	}

	Eigen::MatrixXf jacPinv= svd_solver.matrixV().leftCols(singular_values.size())
			* singular_values.asDiagonal()
			* svd_solver.matrixU().leftCols(singular_values.size()).transpose();
	out_angles_var.angles = robot_state.angles;//+jacPinv*posError;// (jacResize.transpose()*(jacResize*jacResize.transpose()).inverse())*posError;
        out_angles_var.angles[5]+=0.2*0.005;
	//RTT::log(RTT::Info)<<out_angles_var.angles<<RTT::endlog();
	//out_angles_port.write(out_angles_var);
	RTT::log(RTT::Info)<<(jacResize.transpose()*(jacResize*jacResize.transpose()).inverse())*posError<<": JOINTPOS ERROR"<<RTT::endlog();
	out_torques_var.torques = 200*((jacResize.transpose()*(jacResize*jacResize.transpose()).inverse())*posError)-2*(robot_state.velocities);
	//out_torques_var.torques.setZero();
	RTT::log(RTT::Info)<<out_torques_var.torques<<RTT::endlog();
	out_torques_port.write(out_torques_var);

}
void KTSController::stopHook() {

}
void KTSController::cleanupHook() {

}

void KTSController::setPos(float x, float y, float z) {
	std::cout<<"SETTING POS"<<x<<", "<<y<<", "<<z<<"\n";
	pos[0]=x;
	pos[1]=y;
	pos[2]=z;
}
ORO_LIST_COMPONENT_TYPE(KTSController)
