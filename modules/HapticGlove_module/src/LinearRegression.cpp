#include <LinearRegression.hpp>

using namespace HapticGlove;

LinearRegression::LinearRegression()
{
}

LinearRegression::~LinearRegression()
{
}

bool LinearRegression::Initialize()
{

    return true;
}

bool LinearRegression::LearnOneShot(const CtrlHelper::Eigen_Mat& inputData,
                                    const CtrlHelper::Eigen_Mat& ouputData,
                                    CtrlHelper::Eigen_Mat& tetha)
{

    // TODEL
    std::cout << "[LinearRegression::LearnOneShot()]" << std::endl;

    CtrlHelper::Eigen_Mat xT_x = (inputData.transpose() * inputData);
    std::cout << "xT_x: " << xT_x.rows() << xT_x.cols() << std::endl;
    std::cout << "xT_x.determinant(): " << xT_x.determinant() << std::endl;
    Eigen::FullPivLU<CtrlHelper::Eigen_Mat> lu(xT_x);
    std::cout << "lu.rank(): " << lu.rank() << std::endl;
    // TODEL

    CtrlHelper::Eigen_Mat coeff
        = ((inputData.transpose() * inputData).inverse()) * inputData.transpose(); // m X o

    tetha = coeff * ouputData; // m X 1

    return true;
}

bool LinearRegression::LearnOneShotMatrix(const CtrlHelper::Eigen_Mat& inputData,
                                          const CtrlHelper::Eigen_Mat& ouputData,
                                          CtrlHelper::Eigen_Mat& tetha)
{

    tetha.resize(0, 0);
    for (size_t i = 0; i < ouputData.cols(); i++)
    {
        CtrlHelper::Eigen_Mat tetha_i;
        LearnOneShot(inputData, ouputData.col(i), tetha_i);
        push_back_row(tetha, tetha_i.transpose());
    }

    return true;
}

bool LinearRegression::LearnIncrementally()
{
    return true;
}
