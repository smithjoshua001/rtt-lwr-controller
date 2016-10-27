/* Author: Milad Malekzadeh
 * Date:   16 August 2016
 *
 * Description:
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>

//#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
//#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/geometry/Rotation.hpp>
#include <rst-rt/geometry/Translation.hpp>

class TrajectoryGen: public RTT::TaskContext {
public:
    TrajectoryGen(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    // call before preparePorts()
	void setTranslationOnly(const bool translationOnly);
    void setRadius(float r);
    void preparePorts();
    double getSimulationTime();

    void getPosition(double time, Eigen::VectorXf & ret);
    void getVelocity(double time, Eigen::VectorXf & ret);
    void getAcceleration(double time, Eigen::VectorXf & ret);

    void getPositionTranslation(double time, Eigen::VectorXf & ret);
    void getVelocityTranslation(double time, Eigen::VectorXf & ret);
    void getAccelerationTranslation(double time, Eigen::VectorXf & ret);

    void getPositionOrientation(double time, Eigen::VectorXf & ret);
    void getVelocityOrientation(double time, Eigen::VectorXf & ret);
    void getAccelerationOrientation(double time, Eigen::VectorXf & ret);

    void setCenter(double x, double y,double z);

private:
    // Declare input ports and their datatypes

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpacePosition_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceVelocity_port;
    RTT::OutputPort<Eigen::VectorXf> out_desiredTaskSpaceAcceleration_port;

    // Data flow:
    RTT::FlowStatus out_desiredTaskSpacePosition_flow;
    RTT::FlowStatus out_desiredTaskSpaceVelocity_flow;
    RTT::FlowStatus out_desiredTaskSpaceAcceleration_flow;

    // variables
    Eigen::VectorXf out_desiredTaskSpacePosition_var;
    Eigen::VectorXf out_desiredTaskSpaceVelocity_var;
    Eigen::VectorXf out_desiredTaskSpaceAcceleration_var;
    bool portsArePrepared;
    bool sendTranslationOnly;
    unsigned int TaskSpaceDimension;
    float radius;
    RTT::os::TimeService::ticks start_ticks;
    double start_time, current_time, time_diff;
    double _timescale;
    Eigen::MatrixXf BoardRot;
    Eigen::VectorXf BoardTransl, TipOrientation;
    Eigen::VectorXf tmp;
};

