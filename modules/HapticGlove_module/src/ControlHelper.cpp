// teleoperation
#include <ControlHelper.hpp>

using namespace HapticGlove;

Eigen::Map<Eigen::VectorXd> CtrlHelper::toEigenVector(std::vector<double>& vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

Eigen::Map<Eigen::VectorXd> CtrlHelper::toEigenVector(yarp::sig::Vector& vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

void CtrlHelper::toStdVector(Eigen::VectorXd& vecEigen, std::vector<double>& vecStd)
{
    if (vecStd.size() != vecEigen.size())
    {
        vecStd.resize(vecEigen.size());
    }
    Eigen::VectorXd::Map(&vecStd[0], vecEigen.size()) = vecEigen;
}

void CtrlHelper::toStdVector(yarp::sig::Vector& vecYarp, std::vector<double>& vecStd)
{
    if (vecStd.size() != vecYarp.size())
    {
        vecStd.resize(vecYarp.size());
    }
    CtrlHelper::toEigenVector(vecStd) = CtrlHelper::toEigenVector(vecYarp);
}

void CtrlHelper::toYarpVector(Eigen::VectorXd& vecEigen, yarp::sig::Vector& vecYarp)
{
    if (vecYarp.size() != vecEigen.size())
    {
        vecYarp.resize(vecEigen.size());
    }
    Eigen::VectorXd::Map(&vecYarp[0], vecEigen.size()) = vecEigen;
}

void CtrlHelper::toYarpVector(std::vector<double>& vecStd, yarp::sig::Vector& vecYarp)
{
    if (vecYarp.size() != vecStd.size())
    {
        vecYarp.resize(vecStd.size());
    }
    CtrlHelper::toEigenVector(vecYarp) = CtrlHelper::toEigenVector(vecStd);
}
