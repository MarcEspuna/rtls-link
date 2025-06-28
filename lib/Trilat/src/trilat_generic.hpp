#pragma once

#include <Eigen.h>
#include <Eigen/QR>
#include <Eigen/LU>

#include <etl/variant.h>
#include <etl/visitor.h>

#include "trilat_base.hpp"

/**
 * @brief Trilateration algorithm for 2D and 3D
 * 
 * @note  For the 2D mode, it is recommended that the anchors are in the same plane or close to it in order to get best results BECAUSE we are not 
 *        taking into account the z values in the k calculation!! That's why!
 * 
 * @todo  1. Add logic for choosing values that have slowest relative error
 *        2. Add error handling 
 *        3. Add z calculation in k values in the 2D case
 *        4. Optimize Eigen memory usage. Try to minimize continuous allocation and deallocation of memory during noraml runtime and only dynamically allocate
 *           during initialization. 
 */
namespace Trilat {
    
    enum class Mode : uint8_t {
        k2D = 0,
        k3D
    };

    template<Mode mode>
    class Trilat : public ITrilat {
    public:
        Trilat(const Eigen::MatrixX3d&& anchor_locations);
        Trilat(const Eigen::MatrixX3d& anchor_locations);

        virtual void Update(const Eigen::VectorXd& distances) override;
  
    private:
        void CalculatePositionMatrix();
        void CalculateKValues();
        double CalculateKValue(size_t index);
        double CalculateRMS(const Eigen::VectorXd& anchor_distances);
        void CalculateZ(const Eigen::VectorXd& anchor_distances);

        static constexpr size_t GetDimensions();
    };
}

using Trilat2D = Trilat::Trilat<Trilat::Mode::k2D>;
using Trilat3D = Trilat::Trilat<Trilat::Mode::k3D>;
using TrilatEmpty = Trilat::ITrilat;

using TrilatStorage = etl::variant<Trilat2D, Trilat3D, Trilat::ITrilat>;

/**
 * Helper template for retrieving the interface from a variant
 */
template<typename TInterf, typename TVariant>
static TInterf& GetInterface(TVariant& variant)
{
  return *etl::visit([](auto& arg) -> TInterf* { return static_cast<TInterf*>(&arg); }, variant);
}

