/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "TorqueTransition.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TorqueTransition::TorqueTransition(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsizeAndTransitionTime", &TorqueTransition::setDOFsizeAndTransitionTime, this).doc("set DOF size and transition time");
    addOperation("toggle", &TorqueTransition::toggle, this).doc("toggle output");
    addOperation("printCurrentState", &TorqueTransition::printCurrentState, this).doc("printCurrentState");
    properties()->addProperty("toggle_bool", toggle_bool);
    //other stuff
    startTime = 0;
    portsArePrepared = false;
    toggle_bool = false;
}

bool TorqueTransition::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion
	RTT::log(RTT::Info)<<"HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<RTT::endlog();
        return true;
}

bool TorqueTransition::startHook() {
    // this method starts the component
    startTime = this->getSimulationTime();
    if (!in_torquesA_port.connected() || !in_torquesB_port.connected() || !out_torques_port.connected())
           return false;
       else
    return true;
}

void TorqueTransition::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_torquesA_port.connected() && in_torquesB_port.connected()) {
        // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
        in_torquesA_flow = in_torquesA_port.read(in_torquesA_var);
        in_torquesB_flow = in_torquesB_port.read(in_torquesB_var);
    } else {
        // handle the situation
    }

//    RTT::log(RTT::Info)<<"DEBUG1"<<RTT::endlog();
    // you can handle cases when there is no new data.
    if ( (in_torquesA_flow == RTT::NewData || in_torquesA_flow == RTT::OldData) && !toggle_bool){
            
//	RTT::log(RTT::Info)<<"DEBUG2"<<RTT::endlog();
        assert(in_torquesA_var.torques.rows()==DOFsize);
        assert(in_torquesA_var.torques.cols()==1);
        assert(in_torquesA_var.torques.rows()==DOFsize);
        assert(in_torquesA_var.torques.cols()==1);
	out_torques_var.torques = in_torquesA_var.torques;
        //hard jump
        //if (this->getSimulationTime() - startTime < transitionTime){
       
    } else if ((in_torquesB_flow == RTT::NewData || in_torquesB_flow == RTT::OldData)&& toggle_bool) {	RTT::log(RTT::Info)<<"DEBUG3"<<RTT::endlog();
	 out_torques_var.torques = in_torquesB_var.torques;
    } else if ((in_torquesA_flow == RTT::NoData) &&!toggle_bool) {
//	RTT::log(RTT::Info)<<"DEBUG5"<<RTT::endlog();
        out_torques_var.torques.setZero();
    } else if ((in_torquesB_flow == RTT::NoData) && toggle_bool){
//	RTT::log(RTT::Info)<<"DEBUG6"<<RTT::endlog();
	out_torques_var.torques.setZero();
    }
    else {
        // there should be something really wrong!
    }

    out_torques_port.write(out_torques_var);
}

void TorqueTransition::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TorqueTransition::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void TorqueTransition::setDOFsizeAndTransitionTime(unsigned int DOFsize, float transitionTime){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->transitionTime = transitionTime;
    this->preparePorts();
}

void TorqueTransition::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_torquesA_port");
        ports()->removePort("in_torquesB_port");
        ports()->removePort("out_torques_port");
    }

    //prepare input
    in_torquesA_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torquesA_port.setName("in_torquesA_port");
    in_torquesA_port.doc("Input port for reading torquesA values");
    ports()->addPort(in_torquesA_port);
    in_torquesA_flow = RTT::NoData;

    in_torquesB_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torquesB_port.setName("in_torquesB_port");
    in_torquesB_port.doc("Input port for reading torquesB values");
    ports()->addPort(in_torquesB_port);
    in_torquesB_flow = RTT::NoData;

    //prepare output
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    portsArePrepared = true;
}

double TorqueTransition::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void TorqueTransition::printCurrentState(){
    std::cout << "############## TorqueTransition State begin " << std::endl;
    std::cout << " time " << boost::lexical_cast<std::string>(this->getSimulationTime() - startTime) << std::endl;
    std::cout << " in_torquesA_var " << in_torquesA_var.torques << std::endl;
    std::cout << " in_torquesB_var " << in_torquesB_var.torques << std::endl;
    std::cout << " out_torques_var " << out_torques_var.torques << std::endl;
    std::cout << "############## TorqueTransition State end " << std::endl;
}
void TorqueTransition::toggle(){
   toggle_bool = !toggle_bool;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TorqueTransition)
