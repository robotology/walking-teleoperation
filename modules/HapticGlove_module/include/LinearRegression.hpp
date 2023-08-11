// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef LINEAR_REGRESSION_HPP
#define LINEAR_REGRESSION_HPP

#include <ControlHelper.hpp>
#include <iostream>
namespace HapticGlove
{
class LinearRegression;
} // namespace HapticGlove

/**
 * LinearRegression Class useful to learn a linear relationship between inputs and outputs.
 */
class HapticGlove::LinearRegression
{

public:
    LinearRegression();
    ~LinearRegression();

    bool Initialize();

    bool LearnOneShot(const CtrlHelper::Eigen_Mat& inputData,
                      const CtrlHelper::Eigen_Mat& outputData,
                      CtrlHelper::Eigen_Mat& tetha);

    bool LearnOneShotMatrix(const CtrlHelper::Eigen_Mat& inputData,
                            const CtrlHelper::Eigen_Mat& ouputData,
                            CtrlHelper::Eigen_Mat& theta); // A size: <n,m>

    bool LearnIncrementally();
};

#endif // LINEAR_REGRESSION_HPP
