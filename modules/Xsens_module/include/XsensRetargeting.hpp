// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XSENSRETARGETING_H
#define XSENSRETARGETING_H

// std
#include <cmath>
#include <memory>

// YARP
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <hde/msgs/HumanState.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include <chrono>
#include <yarp/os/Clock.h>
// iDynTree
#include <iDynTree/Core/Transform.h>
//#include <RetargetingController.hpp>

class mapJoints
{
public:
    std::string name;
    int index;
};

class XsensRetargeting : public yarp::os::RFModule
{
private:
    /** An implementation class for spepcific functionalities required in this module. */
    class impl;
    std::unique_ptr<impl> pImpl;
    /** Minimum jerk trajectory smoother for the desired whole body joints */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_WBTrajectorySmoother{nullptr};
    /** target (robot) joint values (raw amd smoothed values) */
    yarp::sig::Vector m_jointValues, m_smoothedJointValues;
    /** CoM joint values coming from human-state-provider */
    yarp::sig::Vector m_CoMValues;
    std::vector<std::string>
        m_humanJointsListName; // the order of joints list arrived from human state provider is
                               // different from the one we want to send to the controller

    /** Port used to retrieve the human whole body joint pose. */
    yarp::os::BufferedPort<hde::msgs::HumanState> m_wholeBodyHumanJointsPort;

    /** Port used to provide the smoothed joint pose to the controller. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_wholeBodyHumanSmoothedJointsPort;
    /** Port used to provide the human CoM position to the controller.  */
    yarp::os::BufferedPort<yarp::sig::Vector> m_HumanCoMPort;

    double m_dT; /**< Module period. */
    bool m_useXsens; /**< True if the Xsens is used in the retargeting */

    std::vector<std::string>
        m_robotJointsListNames; /**< Vector containing the name of the controlled joints.*/
    size_t m_actuatedDOFs; /**< Number of the actuated DoF */

    std::vector<unsigned> m_humanToRobotMap;

    bool m_firstIteration;
    double m_jointDiffThreshold;

    /* do smoothing of the joint values */
    bool m_useSmoothing;

public:
    XsensRetargeting();
    ~XsensRetargeting();

    bool getJointValues();

    bool getSmoothedJointValues(yarp::sig::Vector& smoothedJointValues);

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() final;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() final;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) final;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() final;
};

#endif // WHOLEBODYRETARGETING_H
