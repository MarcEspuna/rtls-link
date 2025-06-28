#include <Eigen.h>
#include <gtest/gtest.h>

#include "tdoa_newton_raphson.hpp"

#include <vector>
#include <random>
#include <iostream>

class TDOANewtonRaphsonTest : public ::testing::Test {
protected:
    std::default_random_engine generator;
    
    void SetUp() override {
        std::cout << "\n=== TDOA Newton-Raphson Algorithm Test Suite ===\n" << std::endl;
    }
    
    // Helper function to generate random anchor positions
    tdoa_estimator::PosMatrix generateAnchorPositions(int numAnchors, double spaceSize = 10.0) {
        std::uniform_real_distribution<double> distribution(-spaceSize, spaceSize);
        tdoa_estimator::PosMatrix anchors(numAnchors, 3);
        
        for(int i = 0; i < numAnchors; ++i) {
            for(int j = 0; j < 3; ++j) {
                anchors(i, j) = distribution(generator);
            }
        }
        return anchors;
    }
    
    // Calculate true TDOAs
    std::tuple<tdoa_estimator::PosMatrix, tdoa_estimator::PosMatrix, tdoa_estimator::DynVector> calculateTrueTDOAs(
        const tdoa_estimator::PosMatrix& anchors,
        const std::vector<std::pair<int, int>>& measurementAnchorIds,
        const tdoa_estimator::DynVector& truePosition) {
        
        const int numMeasurements = measurementAnchorIds.size();
        tdoa_estimator::PosMatrix anchorPositionsLeft(numMeasurements, 3);
        tdoa_estimator::PosMatrix anchorPositionsRight(numMeasurements, 3);
        tdoa_estimator::DynVector tdoas(numMeasurements);
        
        for(int i = 0; i < numMeasurements; ++i) {
            anchorPositionsLeft.row(i) = anchors.row(measurementAnchorIds[i].first);
            anchorPositionsRight.row(i) = anchors.row(measurementAnchorIds[i].second);
            
            double distanceLeft = (anchorPositionsLeft.row(i).transpose() - truePosition).norm();
            double distanceRight = (anchorPositionsRight.row(i).transpose() - truePosition).norm();
            tdoas(i) = distanceLeft - distanceRight;
        }
        
        return {anchorPositionsLeft, anchorPositionsRight, tdoas};
    }
    
    // Calculate initial guess
    tdoa_estimator::DynVector calculateInitialGuess(const tdoa_estimator::PosMatrix& anchors) {
        // Manual calculation of min/max coordinates to avoid colwise().redux() issues
        if (anchors.rows() == 0) {
            return tdoa_estimator::DynVector::Zero(3);
        }

        tdoa_estimator::DynVector minCoords = anchors.row(0).transpose();
        tdoa_estimator::DynVector maxCoords = anchors.row(0).transpose();

        for (int i = 1; i < anchors.rows(); ++i) {
            for (int j = 0; j < 3; ++j) { // Assuming 3D
                if (anchors(i, j) < minCoords(j)) {
                    minCoords(j) = anchors(i, j);
                }
                if (anchors(i, j) > maxCoords(j)) {
                    maxCoords(j) = anchors(i, j);
                }
            }
        }

        tdoa_estimator::DynVector sumCoords = minCoords + maxCoords;        
        tdoa_estimator::DynVector initialGuess = sumCoords / 2.0;         
        
        return initialGuess;
    }
    
    // Add noise to TDOAs
    tdoa_estimator::DynVector addNoiseToTDOAs(const tdoa_estimator::DynVector& tdoas, double noiseStd) {
        std::normal_distribution<double> distribution(0.0, noiseStd);
        tdoa_estimator::DynVector noisyTdoas = tdoas;
        
        for(int i = 0; i < tdoas.size(); ++i) {
            noisyTdoas(i) += distribution(generator);
        }
        return noisyTdoas;
    }
};

TEST_F(TDOANewtonRaphsonTest, PerfectConditions) {
    std::cout << "\nTesting under perfect conditions..." << std::endl;
    
    // True position
    tdoa_estimator::DynVector truePosition(3);
    truePosition << 2.0, 1.0, 2.5;
    
    // Anchor positions
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0, -5.0, 0.0,
               5.0, -5.0, 0.0,
               -5.0, 5.0, 0.0,
               5.0, 5.0, 1.5,
               2.5, 2.5, 0.0;
    
    // Measurement pairs
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };
    
    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
    
    tdoa_estimator::DynVector initialGuess = calculateInitialGuess(anchors);
    
    tdoa_estimator::DynVector estimatedPosition = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);
    
    double error = (estimatedPosition - truePosition).norm();
    
    std::cout << "True position: " << truePosition.transpose() << std::endl;
    std::cout << "Estimated position: " << estimatedPosition.transpose() << std::endl;
    std::cout << "Position error: " << error << " units" << std::endl;
    
    EXPECT_NEAR((estimatedPosition - truePosition).norm(), 0.0, 0.001);
}

TEST_F(TDOANewtonRaphsonTest, WithNoise) {
    std::cout << "\nTesting with noisy measurements..." << std::endl;
    
    tdoa_estimator::DynVector truePosition(3);
    truePosition << 2.0, -3.0, 4.2;
    
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0, -5.0, 2.0,
               5.0, -5.0, 0.0,
               -5.0, 5.0, 0.0,
               5.0, 5.0, 2.0,
               2.5, 2.5, 0.0;
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };
    
    std::vector<double> noiseLevels = {0.01, 0.05, 0.1, 0.5};
    
    for(double noiseStd : noiseLevels) {
        auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
            calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
        
        tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
        tdoa_estimator::DynVector initialGuess = tdoa_estimator::DynVector::Zero(3);
        
        tdoa_estimator::DynVector estimatedPosition = tdoa_estimator::newtonRaphson(
            anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuess, 10);
        
        double error = (estimatedPosition - truePosition).norm();
        
        std::cout << "\nNoise level (std): " << noiseStd << std::endl;
        std::cout << "True position: " << truePosition.transpose() << std::endl;
        std::cout << "Estimated position: " << estimatedPosition.transpose() << std::endl;
        std::cout << "Position error: " << error << " units" << std::endl;
        
        EXPECT_LT(error, noiseStd * 3);
    }
}

TEST_F(TDOANewtonRaphsonTest, DifferentAnchorConfigurations) {
    std::cout << "\nTesting different anchor configurations..." << std::endl;
    
    tdoa_estimator::DynVector truePosition(3);
    truePosition << 2.0, 3.0, 1.0;
    
    std::vector<int> numAnchorsToTest = {5, 8, 10};
    
    for(int numAnchors : numAnchorsToTest) {
        std::cout << "\nNumber of anchors: " << numAnchors << std::endl;
        
        tdoa_estimator::PosMatrix anchors = generateAnchorPositions(numAnchors);
        
        std::vector<std::pair<int, int>> measurementAnchorIds;
        for(int i = 0; i < numAnchors; ++i) {
            measurementAnchorIds.push_back({i, (i + 1) % numAnchors});
        }
        
        auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
            calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
        
        tdoa_estimator::DynVector initialGuess = calculateInitialGuess(anchors);
        
        tdoa_estimator::DynVector estimatedPosition = tdoa_estimator::newtonRaphson(
            anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);
        
        double error = (estimatedPosition - truePosition).norm();
        
        std::cout << "True position: " << truePosition.transpose() << std::endl;
        std::cout << "Estimated position: " << estimatedPosition.transpose() << std::endl;
        std::cout << "Position error: " << error << " units" << std::endl;
        
        EXPECT_LT(error, 0.1);
    }
}

TEST_F(TDOANewtonRaphsonTest, RealAnchorLayout) {
    
    std::cout << "\nTesting real anchor layout..." << std::endl;
    
    tdoa_estimator::PosMatrix anchors(5, 3);

    anchors << 0.0, 0.0, 1.8,
               5.5, 0.0, 1.8,
               0.0, 2.5, 1.8,
               5.5, 2.5, 1.8,
               2.7, 0.0, 1.1;
    
    // Measurement pairs
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {1, 2}, {2, 3}, {0, 1}, {4, 0}, {3, 4}, {1, 4}
    };
    
    tdoa_estimator::DynVector truePosition;
    truePosition << 3, 1, 2; 

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition);
    
    tdoa_estimator::DynVector initialGuess = calculateInitialGuess(anchors);
    
    tdoa_estimator::DynVector estimatedPosition = tdoa_estimator::newtonRaphson(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuess, 10);
    
    double error = (estimatedPosition - truePosition).norm();
    
    std::cout << "True position: " << truePosition.transpose() << std::endl;
    std::cout << "Estimated position: " << estimatedPosition.transpose() << std::endl;
    std::cout << "Position error: " << error << " units" << std::endl;
    
    EXPECT_NEAR((estimatedPosition - truePosition).norm(), 0.0, 0.001);
}

/**
 * Original Data: 
 * tdoas << -0.655085,
 *       0.706425,
 *      0.6933,
 *      1.17087,
 *      -1.89361,
 *      -0.0140753;
 */

TEST_F(TDOANewtonRaphsonTest, RealDateTest) {
    
    std::cout << "\nTesting with real measurements..." << std::endl;
    
    tdoa_estimator::PosMatrix anchors(5, 3);

    anchors << 0.0, 0.0, 1.8,
               5.5, 0.0, 1.8,
               0.0, 2.5, 1.8,
               5.5, 2.5, 1.8,
               2.7, 0.0, 1.1;
    
    tdoa_estimator::PosMatrix left;
    tdoa_estimator::PosMatrix right;
    tdoa_estimator::DynVector tdoas;

    left << 5.5,   0, 1.8,
            0, 2.5, 1.8,
            0,   0, 1.8,
            2.7,   0, 1.1,
            5.5, 2.5, 1.8,
            5.5,   0, 1.8;

    right <<  0, 2.5, 1.8,
              5.5, 2.5, 1.8,
              5.5,   0, 1.8,
              0,   0, 1.8,
              2.7,   0, 1.1,
              5.5, 2.5, 1.8;
    
    tdoas << -0.655085,
            0.706425,
            0.6933,
            1.17087,
            -1.89361,
            -0.0140753;
    
    tdoa_estimator::DynVector initial_guess;

    initial_guess << 2.0, 1.2, 1.5;

    tdoa_estimator::DynVector estimatedPosition = tdoa_estimator::newtonRaphson(
        left, right, tdoas, initial_guess, 10);
        
    std::cout << "Estimated position: " << estimatedPosition.transpose() << std::endl;
}

TEST_F(TDOANewtonRaphsonTest, PerfectConditions2D) {
    std::cout << "\nTesting 2D estimation under perfect conditions..." << std::endl;
    
    // True position (3D)
    tdoa_estimator::DynVector truePosition3D(3);
    truePosition3D << 2.0, 1.0, 2.5; // Z = 2.5
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // Anchor positions (3D)
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0, -5.0, 0.0,
               5.0, -5.0, 0.0,
               -5.0, 5.0, 0.0,
               5.0, 5.0, 1.5,
               2.5, 2.5, 0.0;
    
    // Measurement pairs
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };
    
    // Calculate TDOAs using the full 3D positions
    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    // Initial guess (2D) - using the center of the XY projection of anchors
    auto anchorsXY = anchors.leftCols<2>(); 
    
    // Calculate min/max for XY (returns 1x2 row vector expressions)
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();

    // Calculate initial guess (add 1x2 vectors, transpose to 2x1, divide)
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    // Fixed Z for the 2D algorithm - let's use the true Z for this perfect case
    double fixedZ = truePosition3D(2);
    
    // Run the 2D Newton-Raphson
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Fixed Z used: " << fixedZ << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    
    // Expect the 2D error to be very small
    EXPECT_NEAR(errorXY, 0.0, 0.001);
}

TEST_F(TDOANewtonRaphsonTest, WithNoise2D) {
    std::cout << "\nTesting 2D estimation with noisy measurements..." << std::endl;
    
    // True position (3D)
    tdoa_estimator::DynVector truePosition3D(3);
    truePosition3D << 2.0, -3.0, 4.2; // Z = 4.2
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // Anchor positions (3D)
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0, -5.0, 2.0,
               5.0, -5.0, 0.0,
               -5.0, 5.0, 0.0,
               5.0, 5.0, 2.0,
               2.5, 2.5, 0.0;
    
    // Measurement pairs
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };
    
    // Noise level (10cm standard deviation)
    double noiseStd = 0.1;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
    
    // Initial guess (2D) - using the center of the XY projection of anchors
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    // Fixed Z for the 2D algorithm - use true Z for this test
    double fixedZ = truePosition3D(2); 
    
    // Run the 2D Newton-Raphson
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "\nNoise level (std): " << noiseStd << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Fixed Z used: " << fixedZ << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    
    // Expect the 2D error to be reasonably low, maybe within 3*noiseStd?
    EXPECT_LT(errorXY, noiseStd * 5); // Allow slightly more margin for 2D projection
}

TEST_F(TDOANewtonRaphsonTest, CoplanarAnchorsDifferentZ_2D) {
    std::cout << "\nTesting 2D estimation with coplanar anchors and different tag Z..." << std::endl;
    
    // True position (3D)
    tdoa_estimator::DynVector truePosition3D(3);
    truePosition3D << 1.5, 2.5, 1.5; // Tag at Z=1.5
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // Anchor positions (3D) - All at Z=0
    tdoa_estimator::PosMatrix anchors(5, 3);
    anchors << -5.0, -5.0, 0.0,
               5.0, -5.0, 0.0,
               -5.0, 5.0, 0.0,
               5.0, 5.0, 0.0,
               0.0, 0.0, 0.0; // Center anchor also at Z=0
    
    // Measurement pairs
    std::vector<std::pair<int, int>> measurementAnchorIds = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 0}
    };
    
    // Noise level (10cm standard deviation)
    double noiseStd = 0.1;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
    
    // Initial guess (2D)
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    // Fixed Z for the 2D algorithm - Use the true Z of the tag (1.5)
    double fixedZ = truePosition3D(2); 
    
    // Run the 2D Newton-Raphson
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "\nNoise level (std): " << noiseStd << std::endl;
    std::cout << "Anchors Z: 0.0, True Tag Z: " << truePosition3D(2) << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Fixed Z used for estimation: " << fixedZ << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    
    // Expect the 2D error to still be reasonably low
    EXPECT_LT(errorXY, noiseStd * 6); // Maybe slightly more margin due to geometry
}

// --- 4 Anchor Tests --- 

TEST_F(TDOANewtonRaphsonTest, PerfectConditions4Anchors2D) {
    std::cout << "\nTesting 2D estimation (4 Anchors) under perfect conditions..." << std::endl;
    
    tdoa_estimator::DynVector truePosition3D(3); truePosition3D << 2.0, 1.0, 2.5; 
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // 4 Anchor positions (Square, non-coplanar)
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -5.0, -5.0, 0.0,
               5.0, -5.0, 1.0, // Different Z
               5.0,  5.0, 0.0,
              -5.0,  5.0, 1.0; // Different Z
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    
    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    double fixedZ = truePosition3D(2);
    
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, tdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    EXPECT_NEAR(errorXY, 0.0, 0.001);
}

TEST_F(TDOANewtonRaphsonTest, WithNoise4Anchors2D) {
    std::cout << "\nTesting 2D estimation (4 Anchors) with noisy measurements..." << std::endl;
    
    tdoa_estimator::DynVector truePosition3D(3); truePosition3D << 2.0, -3.0, 4.2;
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // 4 Anchor positions (Square, non-coplanar)
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -5.0, -5.0, 2.0,
               5.0, -5.0, 0.0,
               5.0,  5.0, 2.0, 
              -5.0,  5.0, 0.0;
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    double noiseStd = 0.1;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
    
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    double fixedZ = truePosition3D(2); 
    
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "\nNoise level (std): " << noiseStd << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    EXPECT_LT(errorXY, noiseStd * 6); // Expect slightly higher error possible
}

TEST_F(TDOANewtonRaphsonTest, CoplanarAnchorsDifferentZ4Anchors2D) {
    std::cout << "\nTesting 2D estimation (4 Anchors) with coplanar anchors and different tag Z..." << std::endl;
    
    tdoa_estimator::DynVector truePosition3D(3); truePosition3D << 1.5, 2.5, 1.5; // Tag at Z=1.5
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // 4 Anchor positions (Square, All at Z=0)
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -5.0, -5.0, 0.0,
               5.0, -5.0, 0.0,
               5.0,  5.0, 0.0,
              -5.0,  5.0, 0.0;
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    double noiseStd = 0.1;

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
    
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    double fixedZ = truePosition3D(2); 
    
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "\nNoise level (std): " << noiseStd << std::endl;
    std::cout << "Anchors Z: 0.0, True Tag Z: " << truePosition3D(2) << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    EXPECT_LT(errorXY, noiseStd * 7); // Allow potentially more error with 4 coplanar anchors
}

TEST_F(TDOANewtonRaphsonTest, WithNoiseLevels4AnchorsRectangular2D) {
    std::cout << "\nTesting 2D estimation (4 Anchors, Rectangular) with varying noise levels..." << std::endl;
    
    tdoa_estimator::DynVector truePosition3D(3); truePosition3D << 2.0, -1.0, 3.0;
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // 4 Anchor positions (10x5 Rectangle, non-coplanar)
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -5.0, -2.5, 2.0,
               5.0, -2.5, 0.0,
               5.0,  2.5, 2.0,
              -5.0,  2.5, 0.0;
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    std::vector<double> noiseLevels = {0.01, 0.05, 0.1, 0.25, 0.5};

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);

    std::cout << "Anchor Layout: Rectangular (10x5)" << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << std::endl;
    
    for(double noiseStd : noiseLevels) {
        tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
        
        auto anchorsXY = anchors.leftCols<2>(); 
        auto minCoordsXY = anchorsXY.colwise().minCoeff();
        auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
        tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
        
        double fixedZ = truePosition3D(2); 
        
        tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
            anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
        
        double errorXY = (estimatedPositionXY - truePosition2D).norm();
        
        std::cout << "\n  Noise level (std): " << noiseStd << std::endl;
        std::cout << "  Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
        std::cout << "  Position error (XY): " << errorXY << " units" << std::endl;
        // Expect error to increase with noise, allow more margin for higher noise
        EXPECT_LT(errorXY, noiseStd * 10); 
    }
}

TEST_F(TDOANewtonRaphsonTest, OutsideBounds4AnchorsRectangular2D) {
    std::cout << "\nTesting 2D estimation (4 Anchors, Rectangular) with tag outside bounds..." << std::endl;
    
    // True position (3D) - Outside the X range [-5, 5]
    tdoa_estimator::DynVector truePosition3D(3); truePosition3D << 6.0, 1.0, 3.0; 
    tdoa_estimator::PosVector2D truePosition2D = truePosition3D.head<2>();
    
    // 4 Anchor positions (10x5 Rectangle, non-coplanar)
    tdoa_estimator::PosMatrix anchors(4, 3);
    anchors << -5.0, -2.5, 2.0,
               5.0, -2.5, 0.0,
               5.0,  2.5, 2.0,
              -5.0,  2.5, 0.0;
    
    std::vector<std::pair<int, int>> measurementAnchorIds = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    double noiseStd = 0.1; // Keep 10cm noise

    auto [anchorPositionsLeft, anchorPositionsRight, tdoas] = 
        calculateTrueTDOAs(anchors, measurementAnchorIds, truePosition3D);
    
    tdoa_estimator::DynVector noisyTdoas = addNoiseToTDOAs(tdoas, noiseStd);
    
    auto anchorsXY = anchors.leftCols<2>(); 
    auto minCoordsXY = anchorsXY.colwise().minCoeff();
    auto maxCoordsXY = anchorsXY.colwise().maxCoeff();
    // Use center as initial guess, even though tag is outside
    tdoa_estimator::PosVector2D initialGuessXY = (minCoordsXY + maxCoordsXY).transpose() / 2.0;
    
    double fixedZ = truePosition3D(2); 
    
    tdoa_estimator::PosVector2D estimatedPositionXY = tdoa_estimator::newtonRaphson2D(
        anchorPositionsLeft, anchorPositionsRight, noisyTdoas, initialGuessXY, fixedZ, 10);
    
    double errorXY = (estimatedPositionXY - truePosition2D).norm();
    
    std::cout << "\nNoise level (std): " << noiseStd << std::endl;
    std::cout << "Anchor Layout: Rectangular (10x5)" << std::endl;
    std::cout << "True position (XY): " << truePosition2D.transpose() << " (Outside bounds)" << std::endl;
    std::cout << "Estimated position (XY): " << estimatedPositionXY.transpose() << std::endl;
    std::cout << "Position error (XY): " << errorXY << " units" << std::endl;
    // Expect potentially larger error when outside bounds
    EXPECT_LT(errorXY, noiseStd * 15); 
}

#ifndef ARDUINO

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}

#else // ARDUINO

#include <Arduino.h> // Required for setup() and loop()

void setup() {
  // Initialize Google Test (using dummy arguments for Arduino)
  // Serial.begin(115200); // Optional: Initialize Serial for output
  int argc = 1;
  char *argv[] = { (char *)"dummy" }; // Cast to char* as argv elements are char*
  ::testing::InitGoogleTest(&argc, argv);

  // Run tests
  RUN_ALL_TESTS();
}

void loop() {
  // Empty loop for Arduino - tests run once in setup
  delay(100); // Small delay to prevent busy-waiting if loop runs
}

#endif // ARDUINO