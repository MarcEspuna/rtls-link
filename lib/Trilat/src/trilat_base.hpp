#pragma once

#include <Eigen.h>


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
 * 
 *       Make the overdetermined or determined system desicion within the library
 * 
 *  
 */
namespace Trilat {
    
    class ITrilat {
    public:
        ITrilat() = default;
        ITrilat(const Eigen::MatrixX3d&& anchor_locations) 
            : m_AnchorLocations(anchor_locations) {}
        ITrilat(const Eigen::MatrixX3d& anchor_locations) 
            : m_AnchorLocations(anchor_locations) {}
        
        /**
         * @brief Process the new distances samples
         * 
         * @param distances The distances from the tag to the anchors
         */
        virtual void Update(const Eigen::VectorXd& distances) {};
        
        /**
         * @brief Get the last calculated 3D tag position
         * 
         * @return Last calculated 3D tag position
         */
        const Eigen::Vector3d& Read() { return m_TagPosition; };  
        
        /**
         * @brief Calculate the Z axis of the tag position
         * 
         * @param anchor_distances The distances from the tag to the anchors
         */
        double ReadRMS() {return m_RMS; };        
        
    protected:
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
        // Cached RMS 
        double m_RMS = 0.0;
    };






}




