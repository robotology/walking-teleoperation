#ifndef WHOLEBODYRETARGETING_H
#define WHOLEBODYRETARGETING_H

// std
#include <memory>

// YARP
#include <yarp/os/Bottle.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>


// iDynTree
#include <iDynTree/Core/Transform.h>
#include <RetargetingController.hpp>

class WholeBodyRetargeting : public RetargetingController
{
private:
    /** Minimum jerk trajectory smoother for the desired whole body joints */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_WBTrajectorySmoother{nullptr};
    yarp::sig::Vector m_jointValues;
    std::vector<std::string> m_humanJointsListName; // the order of joints list arrived from human state provider is different from the one we want to send to the controller


public:
   WholeBodyRetargeting();
   ~WholeBodyRetargeting() override;
   /**
    * Configure the whole body retargeting retargeting.
    * @param config reference to a resource finder object.
    * @return true in case of success and false otherwise
    */
   bool configure(const yarp::os::Searchable& config, const std::string& name) override;

   bool setJointValues(const yarp::sig::Vector& jointValues);

    bool getSmoothedJointValues(yarp::sig::Vector&  smoothedJointValues);

};


#endif // WHOLEBODYRETARGETING_H
