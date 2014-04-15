/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Gennaro Raiola, Karsten Knese
 *   inspired on the sot_pr2 class written by Thomas Moulard, available here https://github.com/laas/sot_pr2.
 */

# include "sot_controller/sot_device.h"

# include <dynamic-graph/factory.h>
# include <dynamic-graph/command-setter.h>
# include <dynamic-graph/debug.h>
# include "soth/debug.hpp"
# include <sot/core/exception-factory.hh>
# include <sot/core/exception-feature.hh>
# include <dynamic-graph/all-commands.h>

#include <dynamic-graph/pool.h>



// Activate timing functions
# define TIMING
# ifdef TIMING
ros::Time start;
ros::Time end;
bool started = false;

void start_timing(){
    if (!started){
        start = ros::Time::now();
        started= true;
    }
}
void start_reset(){
    started = false;
}

// return duration in u-seconds
long end_timing(){
    end = ros::Time::now();
    ros::Duration tdiff = end - start;
    return tdiff.toNSec();
}
# endif


using sot_controller::SotDevice;
using dynamicgraph::sot::ExceptionFactory;

const std::string SotDevice::CLASS_NAME = "SotDevice";

SotDevice::SotDevice(const std::string& entityName, const std::size_t num_increments = 1):
    dynamicgraph::sot::Device(entityName),
    velocitySOUT("SotDevice("+entityName+")::output(vector)::velocity"),
    dtSOUT("SotDevice("+entityName+")::output(double)::dt"),
    status_(false),
    period_(0.001),
    num_increments_(num_increments),
    self_collision_(false),
    sleepTime_(0),
    debugCounter(0),
    r_(100)
{
    // Register signals into the entity.

    signalRegistration(velocitySOUT);
    signalRegistration(dtSOUT);
}

SotDevice::~SotDevice(){}

bool SotDevice::init(int jointsSize)
{
    // Read state from dynamic graph and use it to check the sizes
    int t = stateSOUT.getTime();
    maal::boost::Vector state = stateSOUT.access(t);
    const int internalStateSize = state.size();
    const int externalStateSize = jointsSize + offset_;

    sotDEBUG(25) << "stateSOUT.access ("<< t << ") = " << state << std::endl;
    sotDEBUG(25) << "state size from dynamic graph = " << internalStateSize << std::endl;
    sotDEBUG(25) << "state size from the controller (plus the ff pose) = " << externalStateSize << std::endl;

    if (internalStateSize == externalStateSize){
        // Initialize the shared variables to zero
        // We are not in real time yet
        shared_position_.resize(internalStateSize);
        shared_velocity_.resize(internalStateSize);
        // This vector is used for the self collision check
        jointPositions_.resize(jointsSize);
        state_size_ = internalStateSize;
        return true;
    }
    else {
        std::stringstream err;
        err << "The robot state in dynamic graph has size ["<< internalStateSize
            <<"], it is different from the state size in the controller ["
           << externalStateSize <<"].";
        throw std::range_error(err.str());
        return false;
    }
}

void SotDevice::starting(const stdVector_t &initPos,const stdVector_t &initVel){
    // Initialize the shared variable with motor positions and velocities
    // Note: Now the velocities are zero because the device and the controller are working with only positions.
    // Now we are in real time
    setSharedState(initPos,initVel);
    // Set the device internal variable
    setState(shared_position_);
}

void SotDevice::startThread(){
    setKillSignal(false);
    thread_ = boost::thread(&SotDevice::update, this);
    start_timing();
}

void SotDevice::stopThread(){
    // Interrupt the thread, we don't care anymore about the device execution
    thread_.interrupt();
}

void SotDevice::setSharedState(ml::Vector const &inputPosition,ml::Vector const &inputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_state_);
    // inputPosition and inputVelocity will always contain the ffpose
    shared_position_ = inputPosition;
    shared_velocity_ = inputVelocity;
}

void SotDevice::setSharedState(stdVector_t const &inputPosition,stdVector_t const &inputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_state_,boost::defer_lock);
    guard.lock();
    // inputPosition and inputVelocity contain the ffpose
    if(inputPosition.size() == state_size_ && inputVelocity.size() == state_size_){
        for (int i = 0; i<state_size_; ++i){
            shared_position_(i) = inputPosition[i];
            shared_velocity_(i) = inputVelocity[i];
        }
    }
    // inputPosition and inputVelocity don't contain the ffpose
    else
        for (int i = 0; i<state_size_-offset_; ++i){
            shared_position_(i+offset_) = inputPosition[i];
            shared_velocity_(i+offset_) = inputVelocity[i];
        }
    guard.unlock();
}

bool SotDevice::getSharedState(stdVector_t &outputPosition, stdVector_t &outputVelocity){
    boost::unique_lock<mutex_t> guard(mtx_state_,boost::defer_lock);

    if(guard.try_lock()){
        // outputPosition and outputVelocity don't contain the ffpose
        for (int i = 0; i<state_size_-offset_; ++i){
            outputPosition[i] = shared_position_(i+offset_);
            outputVelocity[i] = shared_velocity_(i+offset_);
        }
        guard.unlock();
        return true;
    }
    else
        return false;
}

// Thread 1 update loop, running and computing SoT
void SotDevice::update(){
    while(!getKillSignal()){

        r_.reset();
        computeNewState();
        setDeviceStatus(false);

//        debugCounter++;
//        sleepTime_ = end_timing();
//        //        std::cout << "time so far: " << sleepTime_ << std::endl;
//        if (sleepTime_ > 1000000000){
//            //std::cout << "time elapsed: " << sleepTime_ << " num_inc: " << debugCounter << std::endl;
//            sleepTime_ = 0.0;
//            debugCounter = 0;
//            start_reset();
//            start_timing();
//        }
        r_.sleep();
    }
}

void SotDevice::computeNewState() {

    boost::unique_lock<mutex_t> guard(mtx_run_,boost::defer_lock);
    try{
        guard.lock();
        increment(period_);

        // Export the old control value and dt as dynamic graph signals
        dtSOUT.setConstant(period_);
        guard.unlock();

        control_ = controlSIN.accessCopy();

        velocitySOUT.setConstant(control_);
        velocitySOUT.setTime(controlSIN.getTime());
        dtSOUT.setTime(controlSIN.getTime());
    }
    catch(boost::thread_resource_error &e){
        ROS_ERROR_STREAM("exception at SotDevice::computeNewState at guard.try_lock()" << e.what());
    }
    catch(dynamicgraph::sot::ExceptionFeature &fe){
        ROS_ERROR_STREAM("Featureexception at SotDevice::computeNewState" << fe.getMessage() << "::"<<fe.getCode());
    }
    catch (std::exception const &e)
    {
        ROS_ERROR_STREAM("signal computation error" << e.what());
    }
    catch(int n){
        ROS_ERROR_STREAM("iter > 1000 in HCOD.cpp " << n);
        return;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("unknown exception at SotDevice::computeNewState. ");
    }

    setSharedState(state_,control_);
}

// triggered from the outside, in this case sot_controller which enables the actual computation of the sot
void SotDevice::runDevice(const ros::Duration& period) {

    // check if already triggered
    boost::unique_lock<mutex_t> guard(mtx_run_, boost::defer_lock);
    if(guard.try_lock()){
        period_ = period.toSec() / num_increments_; //Set the internal period
        setDeviceStatus(true);
        //guard.unlock(); get unlocked after the scope anyways
    }
}


bool SotDevice::getDeviceStatus() {
    return status_;
}

void SotDevice::setDeviceStatus(const bool status) {
    status_ = status;
}

bool SotDevice::getKillSignal() {
    return killThread_;
}

void SotDevice::setKillSignal(const bool kill) {
    killThread_ = kill;
}



