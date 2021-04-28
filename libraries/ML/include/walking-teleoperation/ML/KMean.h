#ifndef KMEAN_H
#define KMEAN_H

#include <vector>
#include <string>
#include <memory>

/**
 * \ingroup LibTemplateCMake_namespace
 *
 * LibTemplateCMake namespace.
 */
namespace WalkingTeleoperation
{

namespace MachineLearning
{

class Cluster
{
public:

    std::vector<double> m_centroid;
    size_t m_noFeatures;
    std::string m_name;
    /**
     * Constructor
     */
    Cluster();

    /**
     * Constructor by initial centroid value of the cluster
     */
    Cluster(std::vector<double> centroid, std::string name);

    /**
     * Destructor
     */
    virtual ~Cluster();

    /**
     * A method to set/update the centroid value/vector of the cluster and the name of the cluster
     */
    virtual void updateCentroid(std::vector<double> centroid, std::string name);

    /**
     * A method to set/update the centroid value/vector of the cluster
     */
    virtual void updateCentroid(std::vector<double> centroid);

    /**
     * A method to get the centroid value/vector of the cluster
     */
    virtual std::vector<double> getCentroid();

    /**
     * A method to get the centroid value/vector of the cluster
     */
    virtual void getCentroid(std::vector<double> centroid);

    /**
     * A method to get the number of features of the cluster
     */
    virtual void getNoFeatures(size_t noFeatures);


    /**
     * A method to get the number of features of the cluster
     */
    virtual size_t getNoFeatures();


    /**
     * A method to get the name of the cluster
     */
    virtual std::string getName();

};

class KMean
{
private:
    std::vector<Cluster> m_clusters;

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
     * A method to add a new cluster to the list of clusters
     */
    virtual void addCluster(std::vector<double> centroid, std::string name);

    /**
     * A method that update the cluster centroid associated with cluster Name
     */
    virtual bool updateCluster(std::vector<double> centroid, std::string clusterName);

    /**
     * A method that update the cluster centroid associated with cluster number
     */
    virtual bool updateCluster(std::vector<double> centroid, size_t clusterNumber);

    /**
     * A method that computes the euclidean distance between two numbers
     */
    virtual double  euclideanDistance(double val1, double val2);


    /**
     * A method that computes the euclidean distance between two double vectors
     */
    virtual double  euclideanDistance(std::vector<double> vec1, std::vector<double> vec2);

    /**
     * A method that returns the cluster number associated with the input feature vector
     */
    virtual size_t  infereClusterNumber(std::vector<double> input);


    /**
     * A method that returns the cluster name
     */
    virtual std::string  infereClusterName(std::vector<double> input);

    /**
     * A method that returns bool if the input vector is in the cluster name
     */
    virtual bool  isInputInClusterName(std::vector<double> input, std::string clusterName);



}; //class KMean

} // namespace MachineLearning

} // namespace WalkingTeleoperation
#endif /* KMEAN_H */
