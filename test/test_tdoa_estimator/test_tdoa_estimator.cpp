#include <gtest/gtest.h>
#include "tdoa_estimator.hpp"

class TDoAEstimatorTest : public ::testing::Test {
protected:
    const double cube_size = 10.0;
    Eigen::MatrixXd anchor_positions_3d;
    Eigen::MatrixXd anchor_positions_2d;
    Eigen::Vector3d true_position_3d;
    Eigen::Vector2d true_position_2d;
    std::default_random_engine generator;
    std::unique_ptr<TDoAEstimator> estimator_3d;
    std::unique_ptr<TDoAEstimator> estimator_2d;


    Eigen::MatrixXd anchor_positions_real_world;
    std::unique_ptr<TDoAEstimator> estimator_2d_real_world; 

    void SetUp() override {
        // Initialize 3D anchor positions (8 corners of a cube)
        anchor_positions_3d = Eigen::MatrixXd::Zero(8, 3);
        anchor_positions_3d << 0, 0, 0,
                             10, 0, 0,
                             0, 10, 0,
                             10, 10, 0,
                             0, 0, 10,
                             10, 0, 10,
                             0, 10, 10,
                             10, 10, 10;

        // Initialize 2D anchor positions (4 corners of a square)
        anchor_positions_2d = Eigen::MatrixXd::Zero(4, 3);
        anchor_positions_2d << -10, 0, 0,
                             0, -10, 0,
                             10, 10, 0,
                             -10, -10, 0;

        anchor_positions_real_world = Eigen::MatrixXd::Zero(4, 3);
        anchor_positions_real_world << 0, 0, 0,
                             4.2, 0, 0,
                             0, 2.5, 0,
                             4.2, 2.5, 0;

        // Set true positions
        true_position_3d << 3.0, 4.0, 5.0;
        true_position_2d << 3.0, 4.0;
        
        // Create estimators
        estimator_3d = std::make_unique<TDoAEstimator>(anchor_positions_3d, TDoAEstimator::Mode::MODE_3D);
        estimator_2d = std::make_unique<TDoAEstimator>(anchor_positions_2d, TDoAEstimator::Mode::MODE_2D);
        estimator_2d_real_world = std::make_unique<TDoAEstimator>(anchor_positions_real_world, TDoAEstimator::Mode::MODE_2D);
    }
};

// TEST_F(TDoAEstimatorTest, Position3DCalculationTest) {
//     std::vector<double> noise_levels = {0.0, 0.1, 0.5};
//     
//     for (double noise_std : noise_levels) {
//         Eigen::VectorXd tdoa_measurements = TDoAEstimator::simulateTDoAMeasurements(
//             true_position_3d, anchor_positions_3d, noise_std, generator);
//         
//         std::vector<Eigen::Vector3d> initial_guesses = {
//             Eigen::Vector3d(5.0, 5.0, 5.0),
//             Eigen::Vector3d(0.0, 0.0, 0.0),
//             Eigen::Vector3d(10.0, 10.0, 10.0)
//         };
// 
//         double best_error = std::numeric_limits<double>::infinity();
//         Eigen::Vector3d best_position;
//         TDoAEstimator::SolutionMetrics best_metrics;
// 
//         for (const auto& guess : initial_guesses) {
//             auto [position, metrics] = estimator_3d->calculatePosition3D(tdoa_measurements, guess);
//             double error = (position - true_position_3d).norm();
// 
//             if (metrics.confidence_score > 1.0 / (1.0 + best_error)) {
//                 best_error = error;
//                 best_position = position;
//                 best_metrics = metrics;
//             }
//         }
// 
//         std::cout << "\n3D Test - Noise level: " << noise_std << std::endl;
//         std::cout << "True position: " << true_position_3d.transpose() << std::endl;
//         std::cout << "Calculated position: " << best_position.transpose() << std::endl;
//         std::cout << "Position error: " << best_error << " meters" << std::endl;
// 
//         double max_allowed_error = noise_std * 2 + 0.1;
//         EXPECT_LT(best_error, max_allowed_error);
//     }
// }

TEST_F(TDoAEstimatorTest, Position2DCalculationTest) {
    std::vector<double> noise_levels = {0.0, 0.1, 0.5};
    
    // Define the anchor pairs we want to simulate measurements for
    // This simulates measurements between all anchors and anchor 0
    std::vector<std::pair<int, int>> anchor_pairs = {
        {1, 0},
        {2, 0},
        {3, 0}
    };
    
    for (double noise_std : noise_levels) {
        // Create 3D position with z=0 for simulation
        Eigen::Vector3d true_position_3d_sim(true_position_2d.x(), true_position_2d.y(), 0.0);
        
        auto tdoa_measurements = TDoAEstimator::simulateTDoAMeasurements(
            true_position_3d_sim, 
            anchor_positions_2d, 
            anchor_pairs,
            noise_std, 
            generator);

        std::vector<Eigen::Vector2d> initial_guesses = {
            Eigen::Vector2d(5.0, 5.0),
            Eigen::Vector2d(0.0, 0.0),
            Eigen::Vector2d(10.0, 10.0)
        };

        double best_error = std::numeric_limits<double>::infinity();
        Eigen::Vector2d best_position;
        TDoAEstimator::SolutionMetrics best_metrics;

        for (const auto& guess : initial_guesses) {
            auto [position, metrics] = estimator_2d->calculatePosition2D(tdoa_measurements, guess);
            double error = (position - true_position_2d).norm();

            if (metrics.confidence_score > 1.0 / (1.0 + best_error)) {
                best_error = error;
                best_position = position;
                best_metrics = metrics;
            }
        }

        std::cout << "\n2D Test - Noise level: " << noise_std << std::endl;
        std::cout << "True position: " << true_position_2d.transpose() << std::endl;
        std::cout << "Calculated position: " << best_position.transpose() << std::endl;
        std::cout << "Position error: " << best_error << " meters" << std::endl;
        
        // Print the simulated measurements for debugging
        std::cout << "Simulated TDoA measurements:" << std::endl;
        for (const auto& meas : tdoa_measurements) {
            std::cout << "A: " << meas.anchor_a << ", B: " << meas.anchor_b 
                     << " -> " << meas.tdoa << std::endl;
        }

        double max_allowed_error = noise_std * 2 + 0.1;
        EXPECT_LT(best_error, max_allowed_error);
    }
}

// You might also want to add additional test cases specifically for different anchor pair combinations:
TEST_F(TDoAEstimatorTest, Position2DWithVariedAnchorPairsTest) {
    double noise_std = 0.1;
    
    // Test different combinations of anchor pairs
    std::vector<std::vector<std::pair<int, int>>> test_cases = {
        // Case 1: Sequential pairs
        {{0, 1}, {1, 2}, {2, 3}},
        // Case 2: All relative to anchor 1
        {{1, 0}, {1, 2}, {1, 3}},
        // Case 3: Mixed pairs
        {{0, 2}, {1, 3}, {2, 0}},
        // Case 4: Redundant measurements
        {{0, 1}, {1, 0}, {1, 2}, {2, 1}}
    };
    
    for (const auto& anchor_pairs : test_cases) {
        Eigen::Vector3d true_position_3d_sim(true_position_2d.x(), true_position_2d.y(), 0.0);
        
        auto tdoa_measurements = TDoAEstimator::simulateTDoAMeasurements(
            true_position_3d_sim,
            anchor_positions_2d,
            anchor_pairs,
            noise_std,
            generator);
            
        auto [position, metrics] = estimator_2d->calculatePosition2D(
            tdoa_measurements,
            Eigen::Vector2d(5.0, 5.0));  // Use middle of space as initial guess
            
        double error = (position - true_position_2d).norm();
        
        std::cout << "\nTest with " << anchor_pairs.size() << " anchor pairs:" << std::endl;
        for (const auto& pair : anchor_pairs) {
            std::cout << "Pair: " << pair.first << "-" << pair.second << std::endl;
        }
        std::cout << "Position error: " << error << " meters" << std::endl;
        
        EXPECT_LT(error, 0.3);  // Adjust threshold as needed
    }
}


TEST_F(TDoAEstimatorTest, SpecificTDoAPatternTest) {
    // Test with real measurement pattern
    std::vector<std::pair<int, int>> anchor_pairs = {
        {3, 0},  // A: 3, B: 0
        {0, 1},  // A: 0, B: 1
        {2, 3},  // A: 2, B: 3
        {1, 2}   // A: 1, B: 2
    };
    
    double noise_std = 0.1;  // Adjust based on your system's noise level
    
    // Create test positions to verify
    std::vector<Eigen::Vector3d> test_positions = {
        Eigen::Vector3d(2.0, 2.0, 0.0),
        Eigen::Vector3d(5.0, 5.0, 0.0),
        Eigen::Vector3d(3.0, 4.0, 0.0)
    };
    
    for (const auto& true_position : test_positions) {
        // Simulate measurements with the specific anchor pair pattern
        auto tdoa_measurements = TDoAEstimator::simulateTDoAMeasurements(
            true_position,
            anchor_positions_2d,
            anchor_pairs,
            noise_std,
            generator);
            
        // Print the simulated measurements for verification
        std::cout << "\nSimulated measurements for position " 
                  << true_position.transpose() << ":" << std::endl;
        for (const auto& meas : tdoa_measurements) {
            std::cout << "A: " << meas.anchor_a << ", B: " << meas.anchor_b 
                     << " -> " << meas.tdoa << std::endl;
        }
        
        // Try different initial guesses
        std::vector<Eigen::Vector2d> initial_guesses = {
            Eigen::Vector2d(0.0, 0.0),
            Eigen::Vector2d(5.0, 5.0),
            Eigen::Vector2d(true_position.x() + 1.0, true_position.y() + 1.0)  // Near truth
        };
        
        double best_error = std::numeric_limits<double>::infinity();
        Eigen::Vector2d best_position;
        
        for (const auto& guess : initial_guesses) {
            auto [position, metrics] = estimator_2d->calculatePosition2D(
                tdoa_measurements, 
                guess,
                0.01,  // Lower learning rate for stability
                0.7    // Moderate momentum
            );
            
            double error = (position - true_position.head<2>()).norm();
            
            if (error < best_error) {
                best_error = error;
                best_position = position;
            }
            
            // Print iteration results
            std::cout << "Initial guess: " << guess.transpose() << std::endl;
            std::cout << "Calculated position: " << position.transpose() << std::endl;
            std::cout << "Error: " << error << " meters" << std::endl;
        }
        
        // Verify results
        std::cout << "\nBest result:" << std::endl;
        std::cout << "True position: " << true_position.head<2>().transpose() << std::endl;
        std::cout << "Best calculated position: " << best_position.transpose() << std::endl;
        std::cout << "Final error: " << best_error << " meters" << std::endl;
        
        // Test assertion - adjust threshold based on your accuracy requirements
        EXPECT_LT(best_error, 0.5) << "Position error too large: " << best_error << " meters";
    }
}

// Add a specific test case with your exact measurement values
TEST_F(TDoAEstimatorTest, RealMeasurementPatternTest) {
    etl::vector<TDoAEstimator::TDoAMeasurement, TDoAEstimator::kMaxTdoaSamples> measurements = {
        {3, 0, 1.440372},   // A: 3, B: 0
        {0, 1, -1.163557},  // A: 0, B: 1
        {2, 3, -2.101910},  // A: 2, B: 3
        {1, 2, 1.815713}    // A: 1, B: 2
    };
    
    // Try several initial guesses
    std::vector<Eigen::Vector2d> initial_guesses = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.5, 2.5),
        Eigen::Vector2d(5.0, 5.0)
    };
    
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_position;
    TDoAEstimator::SolutionMetrics best_metrics;
    
    for (const auto& guess : initial_guesses) {
        auto [position, metrics] = estimator_2d_real_world->calculatePosition2D(
            measurements,
            guess,
            0.01,  // Lower learning rate
            0.7    // Moderate momentum
        );
        
        std::cout << "\nInitial guess: " << guess.transpose() << std::endl;
        std::cout << "Calculated position: " << position.transpose() << std::endl;
        std::cout << "Error metric: " << metrics.final_error << std::endl;
        
        if (metrics.final_error < best_metrics.final_error) {
            best_position = position;
            best_metrics = metrics;
        }
    }
    
    std::cout << "\nBest solution found:" << std::endl;
    std::cout << "Position: " << best_position.transpose() << std::endl;
    std::cout << "Final error: " << best_metrics.final_error << std::endl;
    
    // We can't check against true position since we don't have it,
    // but we can verify the solution is reasonable
    EXPECT_LT(best_metrics.final_error, 1) 
        << "Solution error too large: " << best_metrics.final_error;
}


// Add a specific test case with your exact measurement values
TEST_F(TDoAEstimatorTest, RealMeasurementPatternSimulatorTest) {

     std::vector<std::pair<int, int>> anchor_pairs = {
        {3, 0},  // A: 3, B: 0
        {0, 1},  // A: 0, B: 1
        {2, 3},  // A: 2, B: 3
        {1, 2}   // A: 1, B: 2
    };
    Eigen::Vector3d true_position = Eigen::Vector3d(2.0, 0.8, 0.0);
    auto tdoa_measurements = TDoAEstimator::simulateTDoAMeasurements(
        true_position,
        anchor_positions_real_world,
        anchor_pairs,
        0.1,
        generator);
        
    // Print the simulated measurements for verification
    std::cout << "\nSimulated measurements for position " 
                << true_position.transpose() << ":" << std::endl;
    for (const auto& meas : tdoa_measurements) {
        std::cout << "A: " << meas.anchor_a << ", B: " << meas.anchor_b 
                    << " -> " << meas.tdoa << std::endl;
    }

    // Try several initial guesses
    std::vector<Eigen::Vector2d> initial_guesses = {
        Eigen::Vector2d(0.0, 0.0),
    };
    
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_position;
    TDoAEstimator::SolutionMetrics best_metrics;
    
    for (const auto& guess : initial_guesses) {
        auto [position, metrics] = estimator_2d_real_world->calculatePosition2D(
            tdoa_measurements,
            guess,
            0.01,  // Lower learning rate
            0.7    // Moderate momentum
        );
        
        std::cout << "\nInitial guess: " << guess.transpose() << std::endl;
        std::cout << "Calculated position: " << position.transpose() << std::endl;
        std::cout << "Error metric: " << metrics.final_error << std::endl;
        
        if (metrics.final_error < best_metrics.final_error) {
            best_position = position;
            best_metrics = metrics;
        }
    }
    
    std::cout << "\nBest solution found:" << std::endl;
    std::cout << "Position: " << best_position.transpose() << std::endl;
    std::cout << "Final error: " << best_metrics.final_error << std::endl;
    
    // We can't check against true position since we don't have it,
    // but we can verify the solution is reasonable
    EXPECT_LT(best_metrics.final_error, 1) 
        << "Solution error too large: " << best_metrics.final_error;
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}