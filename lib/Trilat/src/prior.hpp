#pragma once

#include <Eigen.h>
#include <Eigen/LU>


class PriorTrilat {
public:
    PriorTrilat(Eigen::MatrixX3d&& anchor_locations, double xVariance, double yVariance);
    PriorTrilat(Eigen::MatrixX3d& anchor_locations, double xVariance, double yVariance);

    void Update(const Eigen::VectorXd& distances);

    const Eigen::Vector3d& Read();

    // It's useful if you want to use a seperate method for calculating the Z axis. For example if the anchors are in the same plane
    void CalculateZ(const Eigen::VectorXd& anchor_distances);      

    double CalculateRMS(const Eigen::VectorXd& anchor_distances);      

    Eigen::Vector2d CalculatePrior(const Eigen::Vector2d& currMesure, const Eigen::Vector2d& priorMesure);  
    
private:
    void CalculatePositionMatrix(double xVariance, double yVariance);
    void CalculateKValues();
    void CalculatePriorMatrices(Eigen::MatrixXd&& A, double xVariance, double yVariance);

    double CalculateKValue(size_t index);

    static constexpr size_t GetDimensions();

private:
    // Matrix of anchor locations
    Eigen::MatrixX3d m_AnchorLocations;         
    // Matrix for calculating the tag final position
    Eigen::MatrixXd m_PseudoInverseA;
    // K values
    Eigen::VectorXd m_k;
    // B values
    Eigen::VectorXd m_b;
    // Tag position
    Eigen::Vector3d m_TagPosition;

    // Prior matrices
    // Eq: ((Ecoords^-1 + Eprior^-1)^-1) * (Ecoords^-1 * (currMesure) + Eprior^-1 * (priorMesure))
    Eigen::Matrix2d m_PriorCoords;              // Ecoords^-1
    Eigen::Matrix2d m_PriorCovariance;          // Eprior^-1
    Eigen::Matrix2d m_PriorFixedTerm;           // ((Ecoords^-1 + Eprior^-1)^-1)
};