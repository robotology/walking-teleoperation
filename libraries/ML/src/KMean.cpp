#include <walking-teleoperation/ML/KMean.h>
#include <cstdlib>


WalkingTeleoperation::MachineLearning::KMean::KMean()
{
}

WalkingTeleoperation::MachineLearning::KMean::~KMean()
{
}

double WalkingTeleoperation::MachineLearning::KMean::distance(double op1, double op2)
{
    return std::abs(op1 - op2);
}


