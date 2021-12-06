/**
 * @file ControlHelper.hpp
 * @authors Kourosh Darvish <kourosh.darvish@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */
#ifndef CONTROL_HELPER_HPP
#define CONTROL_HELPER_HPP

// std
#include <vector>
// Eigen
#include <Eigen/Dense>
// yarp
#include <yarp/sig/Vector.h>

namespace HapticGlove
{
namespace CtrlHelper
{
// types
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Eigen_Mat;

// functions
Eigen::Map<Eigen::VectorXd> toEigenVector(std::vector<double>& vec);

Eigen::Map<Eigen::VectorXd> toEigenVector(yarp::sig::Vector& vec);

void toStdVector(Eigen::VectorXd& vecEigen, std::vector<double>& vecStd);

void toStdVector(yarp::sig::Vector& vecYarp, std::vector<double>& vecStd);

void toYarpVector(std::vector<double>& vecStd, yarp::sig::Vector& vecYarp);

void toYarpVector(Eigen::VectorXd& vecEigen, yarp::sig::Vector& vecYarp);

struct Data;

} // namespace CtrlHelper
} // namespace HapticGlove

struct HapticGlove::CtrlHelper::Data
{
    std::vector<double> axisValueReferencesStd; /** Reference axis value in radiant */
    Eigen::VectorXd axisValueReferencesEigen; /** Reference axis value in radiant */
    yarp::sig::Vector axisValueReferencesYarp; /** Reference axis value in radiant */

    std::vector<double> axisValueFeedbacksStd; /** Feedback axis value in radiant */
    Eigen::VectorXd axisValueFeedbacksEigen; /** Feedback axis value in radiant */
    yarp::sig::Vector axisValueFeedbacksYarp; /** feedback axis value in radiant */

    std::vector<double> jointValueReferencesStd; /** Reference joint values in radiant */
    Eigen::VectorXd jointValueReferencesEigen; /** Reference joint values in radiant */
    yarp::sig::Vector jointValueReferencesYarp; /** Reference joint values in radiant */

    std::vector<double> jointValuesExpectedStd; /** Expected joint values in radiant */
    Eigen::VectorXd jointValuesExpectedEigen; /** Expected joint values in radiant */
    yarp::sig::Vector jointValuesExpectedYarp; /** Expected joint values in radiant */

    std::vector<double> jointValueFeedbacksStd; /** Feedback joint values in radiant */
    Eigen::VectorXd jointValueFeedbacksEigen; /** Feedback joint values in radiant */
    yarp::sig::Vector jointValueFeedbacksYarp; /** Feedback joint values in radiant */

    std::vector<double> motorCurrentFeedbacksStd; /** motor current feedbacks*/
    Eigen::VectorXd motorCurrentFeedbacksEigen; /** motor current feedbacks*/
    yarp::sig::Vector motorCurrentFeedbacksYarp; /** motor current feedbacks*/

    std::vector<double> motorCurrentReferencesStd; /** motor current references*/
    Eigen::VectorXd motorCurrentReferencesEigen; /** motor current references*/
    yarp::sig::Vector motorCurrentReferencesYarp; /** motor current references*/

    std::vector<double> motorPwmFeedbacksStd; /** motor PWM feedbacks*/
    Eigen::VectorXd motorPwmFeedbacksEigen; /** motor PWM feedbacks*/
    yarp::sig::Vector motorPwmFeedbacksYarp; /** motor PWM feedbacks*/

    std::vector<double> motorPwmReferencesStd; /** motor PWM references*/
    Eigen::VectorXd motorPwmReferencesEigen; /** motor PWM references*/
    yarp::sig::Vector motorPwmReferencesYarp; /** motor PWM references*/

    std::vector<double> motorPidOutputsStd; /** motor PID outputs*/
    Eigen::VectorXd motorPidOutputsEigen; /** motor PID outputs*/
    yarp::sig::Vector motorPidOutputsYarp; /** motor PID outputs*/
};

#endif // CONTROL_HELPER_HPP
