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
#include <boost/lexical_cast.hpp>

class TorqueTransition: public RTT::TaskContext {
public:
    TorqueTransition(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsizeAndTransitionTime(unsigned int DOFsize, float transitionTime);
    void preparePorts();
    double getSimulationTime();
    void printCurrentState();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torquesA_port;
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torquesB_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_torquesA_flow;
    RTT::FlowStatus in_torquesB_flow;

    // variables
    rstrt::dynamics::JointTorques in_torquesA_var;
    rstrt::dynamics::JointTorques in_torquesB_var;
    rstrt::dynamics::JointTorques out_torques_var;
    unsigned int DOFsize;
    double transitionTime;
    double startTime;
    bool portsArePrepared;
};

