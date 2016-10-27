/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "JointPositionCtrl.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


JointPositionCtrl::JointPositionCtrl(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsizeAndGains", &JointPositionCtrl::setDOFsizeAndGains, this).doc("set DOF size and Gains");
    addOperation("setGains", &JointPositionCtrl::setGains, this).doc("set Gains");
    addOperation("setDesiredJointAngles", &JointPositionCtrl::setDesiredJointAngles, this).doc("set desired joint angles");
    addOperation("setDesiredJointVelocities", &JointPositionCtrl::setDesiredJointVelocities, this).doc("set desired joint velocities");
    addOperation("computeJointTorques", &JointPositionCtrl::computeJointTorques, this).doc("compute joint torques");
    addOperation("printCurrentState", &JointPositionCtrl::printCurrentState, this).doc("print current state");

    //other stuff
    portsArePrepared = false;
}

bool JointPositionCtrl::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion

        return true;
}

bool JointPositionCtrl::startHook() {
    // this method starts the component
	 if (!in_robotstatus_port.connected() || !out_torques_port.connected())
	        return false;
	    else
    return true;
}

void JointPositionCtrl::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if ((in_robotstatus_flow == RTT::NewData || in_robotstatus_flow == RTT::OldData)) {

        assert(in_robotstatus_var.angles.rows()==DOFsize);
        assert(in_robotstatus_var.angles.cols()==1);
        assert(in_robotstatus_var.velocities.rows()==DOFsize);
        assert(in_robotstatus_var.velocities.cols()==1);
        assert(in_robotstatus_var.torques.rows()==DOFsize);
        assert(in_robotstatus_var.torques.cols()==1);

        assert(desJointAngles.angles.rows()==DOFsize);
        assert(desJointAngles.angles.cols()==1);

        assert(desJointVelocities.velocities.rows()==DOFsize);
        assert(desJointVelocities.velocities.cols()==1);
        this->computeJointTorques(in_robotstatus_var, desJointAngles, desJointVelocities, out_torques_var);
    } else if ((in_robotstatus_flow == RTT::NoData)) {
        out_torques_var.torques.setZero();
    } else {
        // there should be something really wrong!
    }

    out_torques_port.write(out_torques_var);
}

void JointPositionCtrl::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void JointPositionCtrl::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void JointPositionCtrl::setDOFsizeAndGains(unsigned int DOFsize, float gainP, float gainD){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->desJointAngles = rstrt::kinematics::JointAngles(DOFsize);
    this->desJointAngles.angles.setZero();
    this->desJointVelocities = rstrt::kinematics::JointVelocities(DOFsize);
    this->desJointVelocities.velocities.setZero();
    this->setGains(gainP, gainD);
    this->preparePorts();
}

void JointPositionCtrl::setGains(float gainP, float gainD){
    assert(gainP > 0);
    assert(gainD > 0);
    this->gainP = gainP;
    this->gainD = gainD;
}

bool JointPositionCtrl::setDesiredJointAngles(rstrt::kinematics::JointAngles & desJointAngles){
    if (portsArePrepared){
        if(desJointAngles.angles.size() != DOFsize){
            return false;
        }
        this->desJointAngles.angles = desJointAngles.angles;
        return true;
    }
    else{
        return false;
    }
}

bool JointPositionCtrl::setDesiredJointVelocities(rstrt::kinematics::JointVelocities & desJointVelocities){
    if (portsArePrepared){
        if(desJointVelocities.velocities.size() != DOFsize){
            return false;
        }
        this->desJointVelocities.velocities = desJointVelocities.velocities;
        return true;
    }
    else{
        return false;
    }
}

void JointPositionCtrl::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_torques_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    portsArePrepared = true;
}

void JointPositionCtrl::computeJointTorques(rstrt::robot::JointState const & jointState,
                                            rstrt::kinematics::JointAngles const & desJointAngles,
                                            rstrt::kinematics::JointVelocities const & desJointVelocities,
                                            rstrt::dynamics::JointTorques & jointTorques) {
    jointTorques.torques = gainP * (desJointAngles.angles - jointState.angles) + gainD * (desJointVelocities.velocities - jointState.velocities);
}

void JointPositionCtrl::printCurrentState(){
    std::cout << "############## JointPositionCtrl State begin " << std::endl;
    std::cout << " angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " desJointAngles " << desJointAngles.angles << std::endl;
    std::cout << " desJointVelocities " << desJointVelocities.velocities << std::endl;
    std::cout << " jointTorques " << out_torques_var.torques << std::endl;
    std::cout << "############## JointPositionCtrl State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(JointPositionCtrl)
