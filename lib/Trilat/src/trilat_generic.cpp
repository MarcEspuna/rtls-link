#include <iostream>

#include "trilat_generic.hpp"

#include <vector>

namespace Trilat {

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

template<Mode mode>
Trilat<mode>::Trilat(const Eigen::MatrixX3d &&anchor_locations)
    : ITrilat(std::move(anchor_locations))
{
    CalculatePositionMatrix();
    CalculateKValues();
}

template<Mode mode>
Trilat<mode>::Trilat(const Eigen::MatrixX3d &anchor_locations)
    : ITrilat(anchor_locations)
{
    CalculatePositionMatrix();
    CalculateKValues();
}

template <Mode mode>
void Trilat<mode>::Update(const Eigen::VectorXd &distances)
{
    m_b = Eigen::VectorXd::Zero(m_AnchorLocations.rows()-1);
    
    // Recalculate the B matrix
    std::cout << "Recalculating B matrix" << std::endl;
    for (int i = 1; i < m_AnchorLocations.rows(); i++)
    {
        m_b(i-1) = distances(0) * distances(0) - distances(i) * distances(i) - m_k(0) + m_k(i);
    }
    std::cout << "B matrix: " << m_b << std::endl;
    // move to class atributtes so it doesn't get created and destroyed every time
    Eigen::VectorXd posn = m_PseudoInverseA * m_b;
    std::cout << "posn" << posn << std::endl;       // The issue is here
    for (int i = 0; i < posn.size(); i++)
    {
        m_TagPosition(i) = posn(i) * 0.5;           // The issue is here
    }
    
    if constexpr (mode == Mode::k2D)
    {
        std::cout << "2D trilat" << std::endl;
        CalculateZ(distances);
    } else {
        std::cout << "3D trilat" << std::endl;
    }

    m_RMS = CalculateRMS(distances);
}


template <Mode mode>
void Trilat<mode>::CalculateZ(const Eigen::VectorXd& anchor_distances)
{
    assert(anchor_distances.size() == m_AnchorLocations.rows() && "Anchor distances must be the same size as the anchor locations");
    // From the x and y values calculated on the m_TagPosition, and the distance from each anchor calculate the z value
    // and keep the one that has the overall lowest relative error

    // Get the norm of the x and y values
    std::vector<double> Zs;        // Store the various z values
    Zs.reserve(m_AnchorLocations.rows());
    double square_sum = 0.0;
    for (int i = 0; i < m_AnchorLocations.rows(); i++)
    {
        // Put the current tag position z coordinate and anchor z coordinate to the same plane for the calculation 
        m_TagPosition(2) = m_AnchorLocations(i, 2);                                 // Set the z value to the current anchor z value
        double norm = (m_AnchorLocations.row(i)-m_TagPosition.transpose()).norm();  // Calculate the norm of the x and y values        
        square_sum += std::pow(anchor_distances(i),2) - norm * norm;        // Calculate the z value
    }
    double z = std::sqrt(square_sum / m_AnchorLocations.rows());
    if (!std::isnan(z)) {  
        m_TagPosition(2) = m_TagPosition(2) + z;                                         // Store the z value
    } 
}

template <Mode mode>
double Trilat<mode>::CalculateRMS(const Eigen::VectorXd& anchor_distances)
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

template<Mode mode>
void Trilat<mode>::CalculatePositionMatrix()
{
    if constexpr (mode == Mode::k2D) {
        assert(m_AnchorLocations.rows() >= 3 && "At least 3 anchors are needed for trilateration");
        assert(m_AnchorLocations.cols() >= 2 && "Anchor locations must be at least 2D");
    } else {
        assert(m_AnchorLocations.rows() >= 4 && "At least 4 anchors are needed for trilateration");
        assert(m_AnchorLocations.cols() >= 3 && "Anchor locations must be at least 3D");
    }

    static constexpr uint32_t N_DIMENSIONS = GetDimensions();
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
    // Deside between more mesurments than unknowns or equal formulas.
    if (m_AnchorLocations.rows() > N_DIMENSIONS + 1)
    {
        m_PseudoInverseA = (A.transpose() * A).inverse() * A.transpose();
        std::cout << "Pseudo inverse A(OD): " << m_PseudoInverseA << std::endl;
    } else {
        m_PseudoInverseA = A.inverse();
        std::cout << "Pseudo inverse A(D): " << m_PseudoInverseA << std::endl;
    }
}

template <Mode mode>
void Trilat<mode>::CalculateKValues()
{
    // Calculate the k values
    m_k.resize(m_AnchorLocations.rows());
    for (uint32_t i = 0; i < m_AnchorLocations.rows(); i++)
    {
        m_k(i) = CalculateKValue(i);
    }
}

/**
 * Calculate the k value for the given index
 * @param index The index of the anchor
 * @return The k value
 * @note The k value is calculated as the sum of the squares of the anchor coordinates of the given index
*/
template <Mode mode>
double Trilat<mode>::CalculateKValue(size_t index)
{
    double kn = std::pow(m_AnchorLocations(index, 0), 2.0);
    for (uint32_t j = 1; j < GetDimensions(); j++)
    {
        kn +=  std::pow(m_AnchorLocations(index, j), 2.0);
    } 
    return kn;
}

template <Mode mode>
constexpr size_t Trilat<mode>::GetDimensions()
{
    return mode == Mode::k2D ? 2 : 3;
}

// Explicit template instantiation
template class Trilat<Mode::k2D>;
template class Trilat<Mode::k3D>;

} // namespace Trilat