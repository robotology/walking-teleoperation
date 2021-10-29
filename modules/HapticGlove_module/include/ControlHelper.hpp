#ifndef CONTROLHELPER_HPP
#define CONTROLHELPER_HPP

// std
#include <vector>
// Eigen
#include <Eigen/Dense>

namespace HapticGlove
{
namespace CtrlHelper
{
// types
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Eigen_Mat;

// functions
Eigen::Map<Eigen::VectorXd> toEigen(std::vector<double>& vec);

void toStd(Eigen::VectorXd& vecEigen, std::vector<double>& vecStd);
} // namespace CtrlHelper
} // namespace HapticGlove

#endif // CONTROLHELPER_HPP
