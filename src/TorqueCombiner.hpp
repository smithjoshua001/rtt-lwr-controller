/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/dynamics/JointTorques.hpp>
#include <boost/lexical_cast.hpp>

class TorqueCombiner: public RTT::TaskContext {
public:
    TorqueCombiner(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void addChainDOFsize(unsigned int ChainDOFsize);
    void preparePorts(std::string prefix);
    void printCurrentState();

private:
    // Declare input ports and their datatypes
    std::vector<boost::shared_ptr< RTT::InputPort<rstrt::dynamics::JointTorques> > > in_robotstatus_ports;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_robotstatus_port;

    // Data flow:
    std::vector<RTT::FlowStatus> in_robotstatus_flow;

    // variables
    std::vector<rstrt::dynamics::JointTorques> in_robotstatus_var;
    rstrt::dynamics::JointTorques out_robotstatus_var;
    unsigned int DOFsize;
    unsigned int numInputPorts;
    std::vector<unsigned int> numChainDOFs;
    bool portsArePrepared;
};

