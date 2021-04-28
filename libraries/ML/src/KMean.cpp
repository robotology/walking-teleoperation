#include <walking-teleoperation/ML/KMean.h>
#include <cstdlib>
#include <algorithm>    // std::transform
#include <numeric>      // std::accumulate
#include <cmath>        // std::sqrt
#include <iostream>
#include <limits>



double square(double n) {return n*n;}

double mse(double n1, double n2) {return square(n1-n2);}


WalkingTeleoperation::MachineLearning::Cluster::Cluster()
{
    m_name="";
}

WalkingTeleoperation::MachineLearning::Cluster::Cluster(std::vector<double> centroid, std::string name="")
{
     m_centroid=centroid;
     m_name= name;
     m_noFeatures= m_centroid.size();
}

WalkingTeleoperation::MachineLearning::Cluster::~Cluster()
{
}

void WalkingTeleoperation::MachineLearning::Cluster::updateCentroid(std::vector<double> centroid, std::string name)
{
    m_centroid=centroid;
    m_name= name;
    m_noFeatures= m_centroid.size();
}

void WalkingTeleoperation::MachineLearning::Cluster::updateCentroid(std::vector<double> centroid)
{
    m_centroid=centroid;
    m_noFeatures= m_centroid.size();
}

std::vector<double> WalkingTeleoperation::MachineLearning::Cluster::getCentroid()
{
    return m_centroid;
}

void WalkingTeleoperation::MachineLearning::Cluster::getCentroid(std::vector<double> centroid)
{
    centroid=m_centroid;
}


void WalkingTeleoperation::MachineLearning::Cluster::getNoFeatures(size_t noFeatures)
{
    noFeatures=m_noFeatures;
}

size_t WalkingTeleoperation::MachineLearning::Cluster::getNoFeatures()
{
    return m_noFeatures;
}

std::string WalkingTeleoperation::MachineLearning::Cluster::getName()
{
    return m_name;
}


//
WalkingTeleoperation::MachineLearning::KMean::KMean()
{
}

WalkingTeleoperation::MachineLearning::KMean::~KMean()
{
}

double WalkingTeleoperation::MachineLearning::KMean::euclideanDistance(double val1, double val2)
{
    return std::abs(val1 - val2);
}

double WalkingTeleoperation::MachineLearning::KMean::euclideanDistance(std::vector<double> vec1, std::vector<double> vec2)
{
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), vec1.begin(), mse);
    return std::sqrt(std::accumulate(vec1.begin(), vec1.end(), 0.0));
}

void WalkingTeleoperation::MachineLearning::KMean::addCluster(std::vector<double> centroid, std::string name)
{
    Cluster newCluster(centroid, name);
    m_clusters.push_back(newCluster);
}

bool WalkingTeleoperation::MachineLearning::KMean::updateCluster(std::vector<double> centroid, std::string clusterName)
{
    bool success=false;
    for(auto&& cluster: m_clusters)
    {
        if (cluster.getName()==clusterName)
        {
            cluster.updateCentroid(centroid);
            success=true;
            break;
        }
    }
    return success;
}

bool WalkingTeleoperation::MachineLearning::KMean::updateCluster(std::vector<double> centroid, size_t clusterNumber)
{
    if (clusterNumber>=m_clusters.size())
    {
        std::cout<<"[Mean::updateCluster] the clusterNumber is larger than the cluster size"<<std::endl;
        return false;
    }
    m_clusters[clusterNumber].updateCentroid(centroid);
    return true;
}

size_t  WalkingTeleoperation::MachineLearning::KMean::infereClusterNumber(std::vector<double> input)
{
    double minDistance= std::numeric_limits<double>::max();
    double distance;
    size_t minIndex, index = 0;

    for(auto&& cluster: m_clusters)
    {
        distance= euclideanDistance(cluster.getCentroid(), input);
        if( distance< minDistance)
        {
            minDistance= distance;
            minIndex=index;
        }
        index++;
    }
    return minIndex;
}

std::string  WalkingTeleoperation::MachineLearning::KMean::infereClusterName(std::vector<double> input)
{
    size_t minIndex= infereClusterNumber(input);
    return m_clusters[minIndex].getName();
}

bool  WalkingTeleoperation::MachineLearning::KMean::isInputInClusterName(std::vector<double> input, std::string clusterName)
{
    return infereClusterName(input)==clusterName;
}
