#include "trilat.hpp"

Trilat3D::Trilat3D(const std::array<std::array<double, 3>, 4>& anchor_matrix)
 : m_Ainv(CalculateAInv(anchor_matrix))
{

}

void Trilat3D::Reset(const std::array<std::array<double, 3>, 4> &anchor_matrix)
{
    m_Ainv = CalculateAInv(anchor_matrix);
}

void Trilat3D::Update(const std::array<double , N_ANCHORS>& distances)
{
    // set up least squares equation
    for (int i = 1; i < 4; i++) {
        m_b[i - 1] = distances[0] * distances[0] - distances[i] * distances[i] + m_k[i] - m_k[0];
    }

    // solve:  2 A x posn = b
    double posn2[3];
    MAT_DOT_VEC_3X3(posn2, m_Ainv, m_b);
    // copy to global current_tag_position[]
    for (int i = 0; i < 3; i++) {
        m_tag_position[i] = posn2[i] * 0.5; //remove factor of 2
    } 
}

std::array<double, 3> Trilat3D::Read()
{
    return m_tag_position;
}

std::array<double, 3> Trilat3D::Error()
{
  //rms error in measured versus calculated distances
  // float x[3] = {0}, rmse = 0.0, dc = 0.0;
  // for (i = 0; i < N_ANCHORS; i++) {
  //   x[0] = anchor_matrix[i][0] - m_tag_position[0];
  //   x[1] = anchor_matrix[i][1] - m_tag_position[1];
  //   x[2] = anchor_matrix[i][2] - m_tag_position[2];
  //   dc = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  //   rmse += (d[i] - dc) * (d[i] - dc);
  // }
  // current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS)); //copy to global
  // return current_distance_rmse;
  return {}; 
}

bool Trilat3D::IsSingular() const
{
    return m_is_singular;
}

constexpr std::array<std::array<double, 3>, 3> Trilat3D::CalculateAInv(const std::array<std::array<double, 3>, 4> &anchor_matrix)
{
    std::array<std::array<double, 3>,3> Ainv = {};

    /*
    double x[N_ANCHORS], y[N_ANCHORS], z[N_ANCHORS];     //intermediate vectors
    double A[3][3];                                      //the A matrix for system of equations to solve

    for (uint32_t i = 0; i < N_ANCHORS; i++) {
        x[i] = anchor_matrix[i][0];
        y[i] = anchor_matrix[i][1];
        z[i] = anchor_matrix[i][2];
        m_k[i] = x[i] * x[i] + y[i] * y[i] + z[i] * z[i];   // Precalculate k values as well
    }

    // set up the A matrix
    for (uint32_t i = 1; i < N_ANCHORS; i++) {
        A[i - 1][0] = x[i] - x[0];
        A[i - 1][1] = y[i] - y[0];
        A[i - 1][2] = z[i] - z[0];
    }

    double det;
    DETERMINANT_3X3 (det, A);

    if (fabs(det) < 1.0e-4) {
        m_is_singular = true;
        std::cerr << "***Singular matrix, check anchor coordinates***" << std::endl;
    } else {
        m_is_singular = false;
    }

    det = 1.0 / det;
    SCALE_ADJOINT_3X3 (Ainv, det, A); 
    */
    return Ainv;
}