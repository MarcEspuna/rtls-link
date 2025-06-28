#pragma once

#include <Eigen.h>
#include <random>
#include <cmath>
#include <etl/vector.h>

class TDoAEstimator {
public:
    static constexpr uint32_t kMaxTdoaSamples = 32;
    static constexpr uint32_t kMaxAnchors = 8;

    struct SolutionMetrics {
        double rmse;
        double gdop;
        double residuals_std;
        double max_residual;
        double final_error;
        double confidence_score;
    };

    enum class Mode {
        MODE_2D,
        MODE_3D
    };

    struct TDoAMeasurement {
        int anchor_a;
        int anchor_b;
        double tdoa;
        uint64_t timestamp;
    };

    TDoAEstimator(const Eigen::MatrixXd& anchor_positions, Mode mode = Mode::MODE_3D)
        : anchor_positions_(anchor_positions), mode_(mode) {
        if (mode == Mode::MODE_2D && anchor_positions.rows() != 4) {
            throw std::invalid_argument("2D mode requires exactly 4 anchors");
        }
        if (mode == Mode::MODE_3D && anchor_positions.rows() != 8) {
            throw std::invalid_argument("3D mode requires exactly 8 anchors");
        }

        // Create polygon from anchor positions
        polygon_.reserve(anchor_positions_.rows());
        for (int i = 0; i < anchor_positions_.rows(); ++i) {
            polygon_.push_back(anchor_positions_.row(i).head(2).transpose());
        }
    }

    // Main calculation functions for 2D and 3D
    std::pair<Eigen::Vector2d, SolutionMetrics> calculatePosition2D(
        const etl::vector<TDoAMeasurement, kMaxTdoaSamples>& tdoa_measurements,
        const Eigen::Vector2d& initial_guess,
        const double learning_rate = 0.05,      // Old value 0.02
        const double momentum = 0.8,            // Old value 0.05     
        const int max_iterations = 500) const;  // Old value 1000 or 2000

    std::pair<Eigen::Vector3d, SolutionMetrics> calculatePosition3D(
        const Eigen::VectorXd& tdoa_measurements,
        const Eigen::Vector3d& initial_guess,
        const double learning_rate = 0.02,
        const double momentum = 0.95,
        const int max_iterations = 1000) const;


    bool isPointInsideAnchorBoundary(const Eigen::Vector2d& point) const;

    // Helper functions
    static double calculateGDOP2D(const Eigen::Vector2d& position, const Eigen::MatrixXd& anchors);
    static double calculateGDOP3D(const Eigen::Vector3d& position, const Eigen::MatrixXd& anchors);
    
    // For testing/simulation purposes
    static etl::vector<TDoAEstimator::TDoAMeasurement, kMaxTdoaSamples> simulateTDoAMeasurements(
        const Eigen::Vector3d& position,
        const Eigen::MatrixXd& anchor_positions,
        const std::vector<std::pair<int, int>>& anchor_pairs,
        double noise_std,
        std::default_random_engine& generator);

private:
    Eigen::MatrixXd anchor_positions_;
    Mode mode_;
    etl::vector<Eigen::Vector2d, kMaxAnchors> polygon_;


    // Helper function to calculate 3D distances even in 2D mode
    Eigen::VectorXd calculate3DDistances(const Eigen::Vector2d& position2d, double z_height = 0.0) const;
    Eigen::VectorXd calculate3DDistances(const Eigen::Vector3d& position) const;
};