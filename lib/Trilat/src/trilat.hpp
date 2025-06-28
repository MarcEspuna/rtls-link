#pragma once

#include <array>
#include <iostream>
#include "m33v3.h"   //matrix and vector macro library, all loops unrolled

class Trilat3D
{
public:
    static constexpr uint32_t N_ANCHORS = 4;


    Trilat3D(const std::array<std::array<double, 3>,4>& anchor_matrix);

    /**
     * @brief Reset the trilateration algorithm with new anchor coordinates
    */
    void Reset(const std::array<std::array<double, 3>,4>& anchor_matrix);

    /**
     * @brief Update the trilateration algorithm with new distance measurements
    */
    void Update(const std::array<double, N_ANCHORS>& d);

    /**
     * @brief Get the current position of the trilateration algorithm
    */
    std::array<double, 3> Read();

    /**
     * @brief Get error rates of the trilateration algorithm
    */
    std::array<double, 3> Error();

    /**
     * @brief Check if the trilateration matrix of anchors is singular
    */
    bool IsSingular() const;


private:
    constexpr std::array<std::array<double, 3>, 3> CalculateAInv(const std::array<std::array<double, 3>,4>& anchor_matrix);

private:
    std::array<std::array<double, 3>,3> m_Ainv;
    std::array<double,3> m_tag_position;

    std::array<double, N_ANCHORS> m_k; //these are calculated only once
    std::array<double, 3> m_b; //these are calculated every time

    bool m_is_singular;

};