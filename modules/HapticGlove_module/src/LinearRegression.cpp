#include <LinearRegression.hpp>


LinearRegression::LinearRegression(){

}

LinearRegression::~LinearRegression(){

}

bool LinearRegression::Initialize(){

    return true;
}

bool LinearRegression::LearnOneShot(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&  inputData,
                                    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&  ouputData,
                                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& tetha){

    // TODEL
    std::cout << "[LinearRegression::LearnOneShot()]"<<std::endl;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> xT_x
        = (inputData.transpose() * inputData);
    std::cout << "xT_x: " << xT_x.rows() << xT_x.cols()<<std::endl;
    std::cout  << "xT_x.determinant(): " << xT_x.determinant()<<std::endl;
    Eigen::FullPivLU<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> lu(
        xT_x);
    std::cout << "lu.rank(): " << lu.rank()<<std::endl;
    // TODEL


    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> coeff
        = ((inputData.transpose() * inputData).inverse()) * inputData.transpose(); // m X o

        tetha = coeff * ouputData; // m X 1

    return true;
}

bool LinearRegression::LearnOneShotMatrix(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&  inputData,
                                    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>&  ouputData,
                                    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& tetha){

    tetha.resize(0,0);
    for (size_t i = 0; i < ouputData.cols(); i++)
    {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tetha_i;
        LearnOneShot(inputData, ouputData.col(i), tetha_i);
        push_back_row(tetha, tetha_i.transpose());
    }

    return true;
}

bool LinearRegression::LearnIncrementally(){

    return true;
}

