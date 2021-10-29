// teleoperation
#include <ControlHelper.hpp>

using namespace HapticGlove;

Eigen::Map<Eigen::VectorXd> CtrlHelper::toEigen(std::vector<double>& vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

void CtrlHelper::toStd(Eigen::VectorXd& vecEigen, std::vector<double>& vecStd)
{
    if (vecStd.size() != vecEigen.size())
    {
        vecStd.resize(vecEigen.size());
    }
    Eigen::VectorXd::Map(&vecStd[0], vecEigen.size()) = vecEigen;
}
