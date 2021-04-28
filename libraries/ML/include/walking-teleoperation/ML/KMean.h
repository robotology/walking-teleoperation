#ifndef KMEAN_H
#define KMEAN_H

/**
 * \ingroup LibTemplateCMake_namespace
 *
 * LibTemplateCMake namespace.
 */
namespace WalkingTeleoperation
{

namespace MachineLearning
{

class KMean
{
public:
    /**
     * Constructor
     */
    KMean();

    /**
     * Destructor
     */
    virtual ~KMean();

    /**
     * A method that computes the distance
     */
    virtual double distance(double op1, double op2);
}; //class KMean

} // namespace MachineLearning

} // namespace WalkingTeleoperation
#endif /* KMEAN_H */
