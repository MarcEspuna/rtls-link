#include <iostream>
#include "prior.hpp"

#include <Eigen/QR>

#include <vector>


// Create here function to get distances to anchors using Eigen
static Eigen::VectorXd calculate_anchor_distances(const Eigen::Vector3d& pos, const Eigen::MatrixX3d& anchor_locations)
{
    Eigen::VectorXd distances(anchor_locations.rows());
    for (int i = 0; i < anchor_locations.rows(); i++)
    {
        distances(i) = (anchor_locations.row(i) - pos.transpose()).norm();
    }
    return distances;
}

PriorTrilat::PriorTrilat(Eigen::MatrixX3d &&anchor_locations, double xVariance, double yVariance)
    : m_AnchorLocations(std::move(anchor_locations))
{
    CalculatePositionMatrix(xVariance, yVariance);
    CalculateKValues();
}

PriorTrilat::PriorTrilat(Eigen::MatrixX3d &anchor_locations, double xVariance, double yVariance)
    : m_AnchorLocations(std::move(anchor_locations))
{
    CalculatePositionMatrix(xVariance, yVariance);
    CalculateKValues();
}

void PriorTrilat::Update(const Eigen::VectorXd &distances)
{
    m_b = Eigen::VectorXd::Zero(m_AnchorLocations.rows()-1);
    
    // Recalculate the B matrix
    for (int i = 1; i < m_AnchorLocations.rows(); i++)
    {
        m_b(i-1) = distances(0) * distances(0) - distances(i) * distances(i) - m_k(0) + m_k(i);
    }

    // move to class atributtes so it doesn't get created and destroyed every time
    Eigen::VectorXd posn = m_PseudoInverseA * m_b;

    for (int i = 0; i < GetDimensions(); i++)
    {
        m_TagPosition(i) = posn(i) * 0.5;
    }
}

const Eigen::Vector3d& PriorTrilat::Read()
{
    return m_TagPosition;
}


void PriorTrilat::CalculateZ(const Eigen::VectorXd& anchor_distances)
{
    assert(anchor_distances.size() == m_AnchorLocations.rows() && "Anchor distances must be the same size as the anchor locations");
    // From the x and y values calculated on the m_TagPosition, and the distance from each anchor calculate the z value
    // and keep the one that has the overall lowest relative error

    // Get the norm of the x and y values
    std::vector<double> Zs;        // Store the various z values
    Zs.reserve(m_AnchorLocations.rows());
    for (int i = 0; i < m_AnchorLocations.rows(); i++)
    {
        // Put the current tag position z coordinate and anchor z coordinate to the same plane for the calculation 
        m_TagPosition(2) = m_AnchorLocations(i, 2);                                 // Set the z value to the current anchor z value
        double norm = (m_AnchorLocations.row(i)-m_TagPosition.transpose()).norm();  // Calculate the norm of the x and y values
        
        double z = std::sqrt(std::pow(anchor_distances(i),2) - norm * norm);        // Calculate the z value
        if (std::isnan(z)) {
            Zs.push_back(0.0);
        } else {
            Zs.push_back(m_TagPosition(2) + z);                                         // Store the z value
        }
    }

    // Substract z results between them to find the one with the lowest error
    // Find the z value with the lowest relative error
    double best_z = 0.0;

    // @Todo: Consider using mean or even calculate RMS and take the best one
    // Get the median of Zs
    std::sort(Zs.data(), Zs.data() + Zs.size());
    best_z = Zs[Zs.size()/2]; 
    m_TagPosition(2) = best_z;
}


double PriorTrilat::CalculateRMS(const Eigen::VectorXd& anchor_distances)
{
    // Calculate distances to anchors from calculated tag position
    Eigen::VectorXd trilat_distances = calculate_anchor_distances(m_TagPosition, m_AnchorLocations);

    // Calculate the sum of the abs of the differences
    double sum = 0.0;
    for (int i = 0; i < anchor_distances.size(); i++)
    {
        sum += std::abs(anchor_distances(i) - trilat_distances(i));
    }

    return sum / anchor_distances.size();
}

Eigen::Vector2d PriorTrilat::CalculatePrior(const Eigen::Vector2d& currMesure, const Eigen::Vector2d& priorMesure)
{
    // Print all the matrices
    std::cout << "PriorCoords:\n" << m_PriorCoords << std::endl;
    std::cout << "PriorCovariance:\n" << m_PriorCovariance << std::endl;
    std::cout << "PriorFixedTerm:\n" << m_PriorFixedTerm << std::endl;

    return m_PriorFixedTerm * (m_PriorCoords * currMesure + m_PriorCovariance * priorMesure);
}

void PriorTrilat::CalculatePositionMatrix(double xVariance, double yVariance)
{
    assert(m_AnchorLocations.rows() >= 3 && "At least 3 anchors are needed for trilateration");
    assert(m_AnchorLocations.cols() >= 2 && "Anchor locations must be at least 2D");

    static constexpr uint32_t N_DIMENSIONS = 2;
    // For simplicity, we suppose that we have more measurements than unknowns for now  
    Eigen::MatrixXd A(m_AnchorLocations.rows()-1, N_DIMENSIONS);

    // Fill the matrix A
    for (int i = 0; i < m_AnchorLocations.rows()-1; i++)
    {
        for (int j = 0; j < N_DIMENSIONS; j++)
        {
            double n0 = m_AnchorLocations(0, j);            // Always the first anchor
            double n1 = m_AnchorLocations(i+1, j);
            std::cout << "n0: " << n0 << " n1: " << n1 << std::endl;
            A(i, j) = n1 - n0;
        }
    }
    // Calculate the pseudo inverse of A
    m_PseudoInverseA = (A.transpose() * A).inverse() * A.transpose();

    // Calculate the prior matrix
    CalculatePriorMatrices(std::move(A), xVariance, yVariance);
}


void PriorTrilat::CalculateKValues()
{
    // Calculate the k values
    m_k.resize(m_AnchorLocations.rows());
    for (uint32_t i = 0; i < m_AnchorLocations.rows(); i++)
    {
        m_k(i) = CalculateKValue(i);
    }
}

void PriorTrilat::CalculatePriorMatrices(Eigen::MatrixXd&& A, double xVariance, double yVariance)
{
    A = A * 2;
    m_PriorCoords = (A.transpose() * A);
    m_PriorCovariance = Eigen::Matrix2d::Identity(2,2);
    m_PriorCovariance(0,0) = xVariance;
    m_PriorCovariance(1,1) = yVariance;

    // Calculate the inverse of the prior matrices
    m_PriorCovariance = m_PriorCovariance.inverse().eval();
    // m_PriorCoords = m_PriorCoords.inverse().eval();

    // Calculate the first constant term for prior calculation
    m_PriorFixedTerm = (m_PriorCoords + m_PriorCovariance).inverse().eval();
}

/**
 * Calculate the k value for the given index
 * @param index The index of the anchor
 * @return The k value
 * @note The k value is calculated as the sum of the squares of the anchor coordinates of the given index
*/
double PriorTrilat::CalculateKValue(size_t index)
{
    double kn = std::pow(m_AnchorLocations(index, 0), 2.0);
    for (uint32_t j = 1; j < GetDimensions(); j++)
    {
        kn +=  std::pow(m_AnchorLocations(index, j), 2.0);
    } 
    return kn;
}


constexpr size_t PriorTrilat::GetDimensions()
{
    return 2;
}
