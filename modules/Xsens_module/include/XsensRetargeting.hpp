#ifndef XSENSRETARGETING_H
#define XSENSRETARGETING_H

// std
#include <memory>
#include <cmath>

// YARP
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/Clock.h>
#include <chrono>
// iDynTree
#include <iDynTree/Core/Transform.h>
//#include <RetargetingController.hpp>

class mapJoints{
public:
    std::string name;
    int index;

};

class XsensRetargeting: public yarp::os::RFModule
{
private:
    /** Minimum jerk trajectory smoother for the desired whole body joints */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_WBTrajectorySmoother{nullptr};
    yarp::sig::Vector m_jointValues, m_smoothedJointValues;
    std::vector<std::string> m_humanJointsListName; // the order of joints list arrived from human state provider is different from the one we want to send to the controller

    /** Port used to retrieve the human whole body joint pose. */
    yarp::os::BufferedPort<yarp::os::Bottle> m_wholeBodyHumanJointsPort;

    /** Port used to retrieve the human whole body joint pose. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_wholeBodyHumanSmoothedJointsPort;

    double m_dT; /**< Module period. */
    bool m_useXsens;/**< True if the Xsens is used in the retargeting */

    std::vector<std::string> m_robotJointsListNames; /**< Vector containing the name of the controlled joints.*/
    size_t m_actuatedDOFs; /**< Number of the actuated DoF */

    std::vector<unsigned> m_humanToRobotMap;

//   std::chrono::milliseconds m_tick,m_tock;
   bool m_firstIteration;
   double m_jointDiffThreshold;

public:
   XsensRetargeting();
   ~XsensRetargeting();
   /*
    * Configure the whole body retargeting retargeting.
    * @param config reference to a resource finder object.
    * @return true in case of success and false otherwise
    */
   bool configure(const yarp::os::Searchable& config, const std::string& name);

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

//    std::vector<double> split(const std::string& str, const std::string& delim);

};


#endif // WHOLEBODYRETARGETING_H
