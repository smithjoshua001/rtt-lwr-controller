/** @file kukaLWRModel.h
 *
 * @author	Maxime Adjigble
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KUKA_KUKALWR_MODEL_H_
#define _GOLEM_DEVICE_KUKA_KUKALWR_MODEL_H_

enum {
     NUMJ_MOD = 7
};

enum {
     NUMJ_ROB = 7
};


class kukaLWRModel
{
    public:
        kukaLWRModel();
        virtual ~kukaLWRModel();

		void calc_dkm(double dkm[16],double q[NUMJ_MOD]);

		void calc_jacobian(double jacobian[6][NUMJ_MOD],double q[NUMJ_MOD]);

		void calc_jacobiand(double jacobiand[6][NUMJ_MOD],double q[NUMJ_MOD],double qd[NUMJ_MOD]);

        /*void calc_residual(double resvec[2*NUMJ_MOD], const double q[NUMJ_MOD],
                           const double dq[NUMJ_MOD], const double dqp[NUMJ_MOD],
                           const double ddqp[NUMJ_MOD], const double tau[NUMJ_MOD]);*/

        /*void calc_jacobian(double resmat[2*NUMJ_MOD][2*NUMJ_MOD], const double cj,
                           const double q[NUMJ_MOD], const double dq[NUMJ_MOD],
                           const double dqp[NUMJ_MOD], const double ddqp[NUMJ_MOD]);*/

        void calc_inertia_matrix(double resmat[NUMJ_MOD][NUMJ_MOD],
                                 const double q[NUMJ_MOD]);

		void calc_coriolis_matrix(double resmat[NUMJ_MOD][NUMJ_MOD],
                            const double q[NUMJ_MOD], const double dq[NUMJ_MOD]);

        void calc_gravity_torque_vector(double resvec[NUMJ_MOD],
                                        const double q[NUMJ_MOD]);

        void calc_friction_torque_vector(double resvec[NUMJ_MOD],
                                         const double dq[NUMJ_MOD]);

    private:
		//Denhavit Hartenberg parameters
		float DH_r1;
		float DH_r3;
		float DH_r5;
		float DH_r7;

    protected:
};


#endif
