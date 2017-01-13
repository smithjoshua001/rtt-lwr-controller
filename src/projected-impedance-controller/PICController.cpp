/*
 * KTSController.cpp
 *
 *  Created on: 4 Jan 2017
 *      Author: joshua
 */

#include "PICController.h"
#include <rtt/Component.hpp>

PICController::PICController(std::string const & name) :
		RTT::TaskContext(name), ChainBase(name) {
	this->addOperation("setBaseAndTip", &PICController::setBaseAndTip, this,
			RTT::ClientThread).doc("Set base and tip of the kinematic chain");
	this->addOperation("loadURDFAndSRDF", &PICController::loadURDFAndSRDF, this,
			RTT::ClientThread);
	this->addOperation("setPos", &PICController::setPos, this,
			RTT::ClientThread);
	this->addOperation("setStepSize", &PICController::setStepSize, this,
			RTT::ClientThread);
	this->addOperation("constraint_switch", &PICController::constraint_switch, this,
				RTT::ClientThread);
	this->addOperation("setPOgains", &PICController::setPOgains, this,
				RTT::ClientThread);
	step_size = 0.1;
	pos.setZero();
	quat_d.resize(4);
	quat_d.setZero();
	jac_c.setZero();
	P.setZero();
	prevP.setZero();
	Pd.setZero();
	xdd_des.setZero();
	F_x.setZero();
	tau_c.setZero();
	tau_0.setZero();
	constraint_on = false;
	Kp = 10;
	Dp = 0.2;
	Ko = 10;
	Do = 0.2;
	Kop =0;
	Dop =0;
        properties()->addProperty("Kop", Kop);
	properties()->addProperty("Dop", Dop);
	properties()->addProperty("quat_x", quat_d(0));
	properties()->addProperty("quat_y", quat_d(1));
	properties()->addProperty("quat_z", quat_d(2));
	properties()->addProperty("quat_w", quat_d(3));
	properties()->addProperty("Kp", Kp);
	properties()->addProperty("Dp", Dp);
	properties()->addProperty("Ko", Ko);
	properties()->addProperty("Do", Do);
	properties()->addProperty("Kn", Kn);
	properties()->addProperty("Dn", Dn);
	properties()->addProperty("sim", simulation);
	traj_bool=false;
	properties()->addProperty("traj_bool", traj_bool);
	force.resize(6);
	force.setZero();
	properties()->addProperty("z_force",force(2));

	//ports()->addPort(robot_state_port);
	//out_angles_var.angles.resize(DOFsize);
	//out_angles_var.angles.setZero();
	//out_angles_port.setName("ja_out");
	//out_angles_port.setDataSample(out_angles_var);
	//ports()->addPort(out_angles_port);

}

PICController::~PICController() {

}

bool PICController::configureHook() {
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
	in_xdd_des.resize(6);
		in_xdd_des.setZero();
		in_xdd_des_port.setName("in_xdd_des");
		in_xdd_des_flow = RTT::NoData;
		ports()->addPort(in_xdd_des_port);
	return true;

}
bool PICController::startHook() {
	return (robot_state_port.connected()); //&& out_angles_port.connected());
	//&& in_M_port.connected());
}
void PICController::updateHook() {

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
	in_xdd_des_flow = in_xdd_des_port.read(in_xdd_des);

	RTT::log(RTT::Info) << name << " Start" << RTT::endlog();
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
	Eigen::Matrix3f rot_mat;
	if (in_x_des_flow == RTT::NoData|| !traj_bool) {
		posError[0] = pos[0] - cart_pos.p.data[0];
		posError[1] = pos[1] - cart_pos.p.data[1];
		posError[2] = pos[2] - cart_pos.p.data[2];
		KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
				quat_d(2), quat_d(3));
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
	} else {
		posError[0] = in_x_des[0] - cart_pos.p.data[0];
		posError[1] = in_x_des[1] - cart_pos.p.data[1];
		posError[2] = in_x_des[2] - cart_pos.p.data[2];
		KDL::Rotation rot_KDL = KDL::Rotation::Quaternion(quat_d(0), quat_d(1),
				quat_d(2), quat_d(3));
		
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

//	//RTT::log(RTT::Info) << "POSERROR: " << posError << RTT::endlog();
	//if (posError.head<3>().norm() > step_size) {
//		posError.head<3>() /= (posError.head<3>().norm() / (float) step_size);
//	}

//	Eigen::MatrixXf jacResize(3, jac.cols());
//	jacResize = jac.block(0, 0, 3, jac.cols());
////RTT::log(RTT::Info)<<jacResize<<RTT::endlog();

	/*Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(M_cf.rows(), M_cf.cols());
	 svd_solver.compute(M_cf, Eigen::ComputeFullU | Eigen::ComputeFullV);

	 Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values =
	 svd_solver.singularValues();
	 for (int i = 0; i < singular_values.size(); i++) {
	 if (singular_values(i) < 1.e-06) {
	 singular_values(i) = 0;
	 } else {
	 singular_values(i) = 1 / singular_values(i);
	 }
	 }

	 Eigen::MatrixXf in_M_Pinv = svd_solver.matrixV().leftCols(
	 singular_values.size()) * singular_values.asDiagonal()
	 * svd_solver.matrixU().leftCols(singular_values.size()).transpose();
	 */
	Eigen::MatrixXf in_M_Pinv(7,7);
	if(simulation){
		in_M_Pinv= M.inverse();
	}else{
	in_M_Pinv= M_cf.inverse();
	}
//	out_angles_var.angles = robot_state.angles;	//+jacPinv*posError;// (jacResize.transpose()*(jacResize*jacResize.transpose()).inverse())*posError;
//	out_angles_var.angles[5] += 0.2 * 0.005;
	////RTT::log(RTT::Info)<<out_angles_var.angles<<RTT::endlog();
	//out_angles_port.write(out_angles_var);
//	//RTT::log(RTT::Info)
//			<< (jac.transpose()
//					* (jac * jac.transpose()).inverse()) * posError
//			<< ": JOINTPOS ERROR" << RTT::endlog();
//	out_torques_var.torques = K
//			* ((jac.transpose() * (jac * jac.transpose()).inverse()) * posError)
//			- D * (robot_state.velocities);
	//out_torques_var.torques.setZero();

//	//RTT::log(RTT::Info) << out_torques_var.torques << RTT::endlog();
	posError.head<3>() *= Kp;
	posError.tail<3>() *= Ko;

	Eigen::VectorXf velError(6);
	if (in_xd_des_flow == RTT::NoData || !traj_bool) {
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
	Eigen::VectorXf damping(7);
	damping << 5, 5, 3, 3, 1, 1, 1;

//	Eigen::MatrixXf jmjt = (jac * M.inverse() * jac.transpose()).inverse();
//	Eigen::MatrixXf jmjtinv = (jac * in_M_Pinv * jac.transpose());
//
//	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver2(jmjtinv.rows(),
//			jmjtinv.cols());
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
//	Eigen::MatrixXf jmjt =
//			svd_solver2.matrixV().leftCols(singular_values2.size())
//					* singular_values2.asDiagonal()
//					* svd_solver2.matrixU().leftCols(singular_values2.size()).transpose();
//
//	out_torques_var.torques = jac.transpose()
//			* ((jmjt * (xdd - jacd * robot_state.velocities))
//					+ jmjt * jac * in_M_Pinv * C_cf);//+ (robot_state.velocities.cwiseProduct(damping));

	//RTT::log(RTT::Info) << quat_temp <<": QUATERNION VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << in_M<<":\n"<<M <<": IN_M VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << M_cf<<": M_cf VALUE" << RTT::endlog();
	//RTT::log(RTT::Info) << C_cf<<": C_cf VALUE" << RTT::endlog();

	//jac_c=vec*jac; FOR CALCULATING jac_c
	jac_x = jac;
	jacd_x = jacd;
	jac_c.setZero();
	tau_c.setZero();
	Eigen::VectorXf POerror(6);
	POerror.setZero();
	Eigen::VectorXf VOerror(6);
	VOerror.setZero();
	if (constraint_on) {
		jac_c = jac;
		jac_c.row(0).setZero();
		jac_c.row(1).setZero();
		jac_c.row(3).setZero();
		jac_c.row(4).setZero();
		jac_c.row(5).setZero();
		jac_x.row(2).setZero();
		//jac_x.row(3).setZero();
		//jac_x.row(4).setZero();
		jacd_x.row(2).setZero();
		//jacd_x.row(3).setZero();
		//jacd_x.row(4).setZero();
		/*POerror.tail<3>() = -rot_mat * delta_quat;
		POerror *= Kop;
		VOerror.tail<3>() = Eigen::Map<Eigen::Array<double, 3, 1>>(
				cart_vel.GetTwist().rot.data).cast<float>() * -Dop;+POerror+VOerror*/
		tau_c = jac.transpose()*(force);
	}

	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver_jac_c(jac_c.rows(),
			jac_c.cols());
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

	Eigen::MatrixXf jac_c_Pinv =
			svd_solver_jac_c.matrixV().leftCols(singular_values_jac_c.size())
					* singular_values_jac_c.asDiagonal()
					* svd_solver_jac_c.matrixU().leftCols(
							singular_values_jac_c.size()).transpose();

	P = Eigen::Matrix<float, 7, 7>::Identity() - (jac_c_Pinv * jac_c);
	end_time = (1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));
	if(constraint_on){
	     if(prevP.isZero()){
		start_time = (1E-9*RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()));
		prevP = P;
	     }
             Pd = (P - prevP)*(end_time-start_time);
             start_time=end_time;
	}
	if(simulation){
		M_c = (P * M) + Eigen::Matrix<float, 7, 7>::Identity() - P;
	}else{
	M_c = (P * M_cf) + Eigen::Matrix<float, 7, 7>::Identity() - P;
	}
	//RTT::log(RTT::Error)<<name<<" jac_c_Pinv: "<<jac_c_Pinv<<RTT::endlog();
	//RTT::log(RTT::Error)<<name<<" P: "<<P<<RTT::endlog();
	lambda_c = (jac_x * (M_c.inverse()) * P * (jac_x.transpose()));
	Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver_lambda_c(lambda_c.rows(),
			lambda_c.cols());
		svd_solver_lambda_c.compute(lambda_c, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::JacobiSVD<Eigen::MatrixXf>::SingularValuesType singular_values_lambda_c =
				svd_solver_lambda_c.singularValues();
		for (int i = 0; i < singular_values_lambda_c.size(); i++) {
			if (singular_values_lambda_c(i) < 1.e-06) {
				singular_values_lambda_c(i) = 0;
			} else {
				singular_values_lambda_c(i) = 1 / singular_values_lambda_c(i);
			}
		}

		lambda_c =
				svd_solver_lambda_c.matrixV().leftCols(singular_values_lambda_c.size())
						* singular_values_lambda_c.asDiagonal()
						* svd_solver_lambda_c.matrixU().leftCols(
								singular_values_lambda_c.size()).transpose();

	//RTT::log(RTT::Error)<<name<<" lambda_c: "<<lambda_c<<RTT::endlog();
	//RTT::log(RTT::Error)<<name<<" jac_x: "<<jac_x<<RTT::endlog();
	//RTT::log(RTT::Error)<<name<<" M_c: "<<M_c<<RTT::endlog();
	//RTT::log(RTT::Error)<<name<<" lambda_c_inv: "<<(jac_x * (M_c.inverse()) * P * (jac_x.transpose()))<<RTT::endlog();
	h_c = (lambda_c * jac_x * M_c.inverse()
			* ((P * (C_cf)) - (Pd * robot_state.velocities)))
			- (lambda_c * jacd_x * robot_state.velocities);

	lambda_d = lambda_c;
//	F = h_c + lambda_c * xdd_des - lambda_c*lambda_d.inverse()*(velError+posError)+( lambda_c*lambda_d.inverse() - Eigen::Matrix<float,6,6>::Identity())*F_x;
	F = h_c + (lambda_c * in_xdd_des) - (-velError - posError);
	N = (Eigen::Matrix<float, 7, 7>::Identity()
			- (jac_x.transpose() * lambda_c * jac_x * M_c.inverse() * P));
	tau_0 = Kn*(-robot_state.angles)+Dn*(robot_state.velocities);
	out_torques_var.torques = P * jac_x.transpose() * F + P * N * tau_0
			+ ((Eigen::Matrix<float, 7, 7>::Identity() - P) * tau_c);
	//out_torques_var.torques.setZero();
	out_torques_port.write(out_torques_var);
	RTT::log(RTT::Info) << name << " Stop" << RTT::endlog();

}
void PICController::constraint_switch() {
	constraint_on = !constraint_on;
}
void PICController::stopHook() {

}
void PICController::cleanupHook() {

}
void PICController::setPOgains(float k, float d){
     this->Kop = k;
     this->Dop = d;
}

void PICController::setPos(float x, float y, float z) {
	//RTT::log(RTT::Info) << "SETTING POS" << x << ", " << y << ", " << z << "\n";
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
}
ORO_LIST_COMPONENT_TYPE(PICController)
