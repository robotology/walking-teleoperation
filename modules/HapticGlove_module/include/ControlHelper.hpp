// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef CONTROL_HELPER_HPP
#define CONTROL_HELPER_HPP

// std
#include <iostream>
#include <vector>

// Eigen
#include <Eigen/Dense>
// yarp
#include <yarp/sig/Vector.h>

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define EIGEN_MALLOC_ALLOWED Eigen::internal::set_is_malloc_allowed(true);
#define EIGEN_MALLOC_NOT_ALLOWED Eigen::internal::set_is_malloc_allowed(false);
#else
#define EIGEN_MALLOC_ALLOWED
#define EIGEN_MALLOC_NOT_ALLOWED
#endif

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

// template <typename DynamicEigenMatrix, typename DynamicEigenVector>
// bool push_back_row(DynamicEigenMatrix& m, const DynamicEigenVector& values);

template <typename DynamicEigenMatrix, typename DynamicEigenVector>
bool push_back_row(DynamicEigenMatrix& m, const DynamicEigenVector& values)
{
    if (m.size() != 0)
    {
        if (m.cols() != values.cols())
        {

            std::cerr << "[push_back_row]: the number of columns in matrix and vactor are not "
                         "equal; m.cols(): "
                      << m.cols() << ", values.cols():" << values.cols() << std::endl;
            return false;
        }
    }
    Eigen::Index row = m.rows();
    m.conservativeResize(row + 1, values.cols());
    for (int i = 0; i < values.cols(); i++)
        m(row, i) = values(0, i);
    return true;
};

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
