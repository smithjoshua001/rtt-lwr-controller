/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>

class JointPositionCtrl: public RTT::TaskContext {
public:
    JointPositionCtrl(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsizeAndGains(unsigned int DOFsize, float gainP, float gainD);
    void setGains(float gainP, float gainD);
    bool setDesiredJointAngles(rstrt::kinematics::JointAngles & desJointAngles);
    bool setDesiredJointVelocities(rstrt::kinematics::JointVelocities & desJointVelocities);
    void preparePorts();
    void computeJointTorques(rstrt::robot::JointState const & jointState,
                             rstrt::kinematics::JointAngles const & desJointAngles,
                             rstrt::kinematics::JointVelocities const & desJointVelocities,
                             rstrt::dynamics::JointTorques & jointTorques);
    void printCurrentState();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::dynamics::JointTorques out_torques_var;
    unsigned int DOFsize;
    float gainP, gainD;
    rstrt::kinematics::JointAngles desJointAngles;
    rstrt::kinematics::JointVelocities desJointVelocities;
    bool portsArePrepared;
};

