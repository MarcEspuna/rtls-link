#include "config/features.hpp"  // MUST be first project include

#ifdef USE_UWB_MODE_TDOA_TAG

#include "logging/logging.hpp"

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
#include "tag/dynamicAnchorPositions.hpp"
#endif

#include <Eigen.h>

#include <algorithm>
#include <iostream>
#include <optional>
#include <array>

#include <etl/set.h>

#include "tdoa_newton_raphson.hpp"

#include "FunctionalInterrupt.h"

#include "tag/tdoa_tag_algorithm.hpp"

#include "uwb_tdoa_tag.hpp"

#include "scheduler.hpp"
#include "app.hpp"
#include "bsp/board.hpp"
#include "uwb_frontend_littlefs.hpp"

static void txCallback(dwDevice_t *dev);
static void rxCallback(dwDevice_t *dev);
static void rxTimeoutCallback(dwDevice_t *dev);
static void rxFailedCallback(dwDevice_t *dev);

// Helper function to get DW1000 mode array by index
static const uint8_t* getDwModeByIndex(uint8_t idx) {
    switch(idx) {
        case 0: return MODE_SHORTDATA_FAST_ACCURACY;   // 6.8Mb/s, 64MHz PRF, 128 preamble (DEFAULT)
        case 1: return MODE_LONGDATA_FAST_ACCURACY;    // 6.8Mb/s, 64MHz PRF, 1024 preamble
        case 2: return MODE_SHORTDATA_FAST_LOWPOWER;   // 6.8Mb/s, 16MHz PRF, 128 preamble
        case 3: return MODE_LONGDATA_FAST_LOWPOWER;    // 6.8Mb/s, 16MHz PRF, 1024 preamble
        case 4: return MODE_SHORTDATA_MID_ACCURACY;    // 850kb/s, 64MHz PRF, 128 preamble
        case 5: return MODE_LONGDATA_MID_ACCURACY;     // 850kb/s, 64MHz PRF, 1024 preamble
        case 6: return MODE_LONGDATA_RANGE_ACCURACY;   // 110kb/s, 64MHz PRF, 2048 preamble
        case 7: return MODE_LONGDATA_RANGE_LOWPOWER;   // 110kb/s, 16MHz PRF, 2048 preamble
        default: return MODE_SHORTDATA_FAST_ACCURACY;
    }
}

// Helper function to check if mode uses 64MHz PRF (for preamble code selection)
static bool isDwMode64MHzPRF(uint8_t idx) {
    // Modes 2, 3, 7 use 16MHz PRF; others use 64MHz PRF
    return (idx != 2 && idx != 3 && idx != 7);
}

// Helper function to apply TX power settings
static void applyTxPower(dwDevice_t* dev, uint8_t powerLevel, uint8_t smartPowerEnable) {
    if (smartPowerEnable) {
        dwUseSmartPower(dev, true);
    } else {
        dwUseSmartPower(dev, false);
        switch(powerLevel) {
            case 0: // Low power
                dwSetTxPower(dev, 0x07070707ul);
                break;
            case 1: // Medium-low
                dwSetTxPower(dev, 0x0F0F0F0Ful);
                break;
            case 2: // Medium-high
                dwSetTxPower(dev, 0x17171717ul);
                break;
            case 3: // High (large power) - default
            default:
                dwEnableLargePower(dev);
                break;
        }
    }
}

static void estimatorCallback(tdoaMeasurement_t* tdoa);
Eigen::MatrixXd spanToMatrix(etl::span<const UWBAnchorParam> data, int rows);
static void estimatorProcess();

static etl::vector<tdoa_estimator::TDoAMeasurement, 64> measurements;
static SemaphoreHandle_t measurements_mtx = xSemaphoreCreateMutex();
static etl::array<UWBAnchorParam, 8> anchor_positions;

static constexpr uint32_t kNumberMeasurements = 64;     // Preferably to be multiple of the number of anchors

static bool isr_flag = false;

// Cache for anchors_seen (used when mutex is unavailable)
static uint8_t cached_anchors_seen = 0;

#if TDOA_STATS_LOGGING == ENABLE
static uint32_t stats_samples_sent = 0;
static uint32_t stats_samples_rejected = 0;
static uint32_t stats_reject_rmse = 0;          // Rejected due to RMSE > threshold
static uint32_t stats_reject_nan = 0;           // Rejected due to NaN in estimate
static uint32_t stats_reject_insufficient = 0;  // Rejected due to < MIN_MEASUREMENTS
static uint32_t stats_stale_removed = 0;        // Number of stale measurements removed
static uint32_t stats_min_meas_count = UINT32_MAX;  // Min measurements seen this period
static uint32_t stats_max_meas_count = 0;       // Max measurements seen this period
static uint64_t stats_last_log_time_ms = 0;
static constexpr uint64_t STATS_LOG_INTERVAL_MS = 1000;
#endif

// Position logging with timed interval
static uint64_t position_last_log_time_ms = 0;
static constexpr uint64_t POSITION_LOG_INTERVAL_MS = 500;  // Log position every 500ms

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
// Static member definitions for dynamic anchor positioning
DynamicAnchorPositionCalculator UWBTagTDoA::s_dynamicCalc;
bool UWBTagTDoA::s_useDynamicPositions = false;
uint32_t UWBTagTDoA::s_lastPositionUpdate = 0;

// Interval for dynamic position updates (ms)
static constexpr uint32_t DYNAMIC_POS_UPDATE_INTERVAL_MS = 200;
#endif

static StaticTaskHolder<etl::delegate<void()>, 16384> pos_estimator_task = {
  "PosEstimatorTask",
  150,              // 150Hz
  1,                // Priority
  etl::delegate<void()>::create<&estimatorProcess>(),
    {},
    {}
};


UWBTagTDoA::UWBTagTDoA(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors)
    : UWBBackend(front, uwb_config)
{
    // NOTE: Look into short data fast accuracy...
    // Using a lambda to attach the class method as an interrupt handler
    LOG_INFO("--- UWB Tag TDOA Mode ---");
    vTaskDelay(pdMS_TO_TICKS(300));

    // Fill in anchor positions lookup table
    for (uint32_t i = 0; i < anchors.size(); i++) {
        uint32_t index_to_copy = anchors[i].shortAddr[0] - '0';
        if (index_to_copy < anchor_positions.size()) {
            anchor_positions[index_to_copy] = anchors[i];
        } 
    }

#ifdef USE_BEACON_PROTOCOL
    // --- Echo anchor positions to the App (for beacon protocol) ---
    etl::array<double, 12> anchors_to_echo = {};
    // Fill the anchor positions from id 0 up to 3 (max 4 anchors for echoing)
    for (uint32_t i = 0; i < anchor_positions.size() && i < anchors_to_echo.size()/3 ; i++) {
        anchors_to_echo[i*3] = anchor_positions[i].x;
        anchors_to_echo[i*3 + 1] = anchor_positions[i].y;
        anchors_to_echo[i*3 + 2] = anchor_positions[i].z;
    }
    // --- Logging ---
    LOG_INFO("Echoing Anchor Positions:");
    for (uint32_t i = 0; i < anchors_to_echo.size() / 3; ++i) {
        LOG_INFO("  Anchor %u: X=%.2f, Y=%.2f, Z=%.2f",
                 i, anchors_to_echo[i*3], anchors_to_echo[i*3+1], anchors_to_echo[i*3+2]);
    }
    // ---------------
    App::AnchorsToEcho(anchors_to_echo);
#endif
    // --------------------------------------

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000
    if (result != 0) {
        LOG_WARN("DW1000 configuration failed, devid: %u", static_cast<uint32_t>(result));
    }
    LOG_INFO("DW1000 Configured");
    vTaskDelay(pdMS_TO_TICKS(300));

    dwEnableAllLeds(&m_Device);

    dwTime_t delay = {.full = 0};       
    dwSetAntenaDelay(&m_Device, delay); // hmmm but why?
    dwAttachSentHandler(&m_Device, txCallback);
    dwAttachReceivedHandler(&m_Device, rxCallback);
    dwAttachReceiveTimeoutHandler(&m_Device, rxTimeoutCallback);
    dwAttachReceiveFailedHandler(&m_Device, rxFailedCallback);
    dwNewConfiguration(&m_Device);
    dwSetDefaults(&m_Device);

    // Get UWB radio settings from parameters
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    const uint8_t* dwMode = getDwModeByIndex(uwbParams.dwMode);
    dwEnableMode(&m_Device, dwMode);
    dwSetChannel(&m_Device, uwbParams.channel);
    // Select preamble code based on PRF (16MHz vs 64MHz)
    if (isDwMode64MHzPRF(uwbParams.dwMode)) {
        dwSetPreambleCode(&m_Device, PREAMBLE_CODE_64MHZ_9);
    } else {
        dwSetPreambleCode(&m_Device, PREAMBLE_CODE_16MHZ_4);
    }

    // Apply TX power settings
    applyTxPower(&m_Device, uwbParams.txPowerLevel, uwbParams.smartPowerEnable);

    dwSetReceiveWaitTimeout(&m_Device, 10000);

    dwCommitConfiguration(&m_Device);

    uint32_t dev_id = dwGetDeviceId(&m_Device);
    LOG_INFO("Initialized TDoA Tag: 0x%08X", dev_id);

    // Init the tdoa anchor algorithm
    uwbTdoa2TagAlgorithm.init(&m_Device, estimatorCallback);

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Initialize dynamic anchor positioning if enabled
    s_useDynamicPositions = (uwbParams.dynamicAnchorPosEnabled != 0);
    if (s_useDynamicPositions) {
        DynamicAnchorConfig dynamicConfig = {
            .layout = uwbParams.anchorLayout,
            .anchorCount = 4,  // Rectangular layout uses 4 anchors
            .anchorHeight = uwbParams.anchorHeight,
            .avgSampleCount = uwbParams.distanceAvgSamples,
            .lockedMask = uwbParams.anchorPosLocked
        };
        s_dynamicCalc.init(dynamicConfig);
        s_lastPositionUpdate = 0;

        // Register callback to receive inter-anchor distances
        uwbTdoa2TagSetDistanceCallback(&UWBTagTDoA::onInterAnchorDistance);

        LOG_INFO("Dynamic anchor positioning enabled (layout=%d, height=%.2f, samples=%d)",
                 uwbParams.anchorLayout, uwbParams.anchorHeight, uwbParams.distanceAvgSamples);
    }
#endif

    attachInterrupt(digitalPinToInterrupt(uwb_config.pins.int_pin), 
    [this]() { 
        this->InterruptHandler(); 
        }, 
    RISING);

    measurements.reserve(64);

    // Any on event needed? 
    uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);

    // Start the pos estimator task
    Scheduler::scheduler.CreateStaticTask(pos_estimator_task);
}


void UWBTagTDoA::Update()
{
    if (isr_flag) {
        dwHandleInterrupt(&m_Device);
        isr_flag = false;
    }

    // Call libdw1000 loop
    TagTDoADispatcher dispatcher(this);
    dispatcher.Dispatch(static_cast<libDw1000::IsrFlags>(m_DwData.interrupt_flags));

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
    // Periodically update anchor positions from dynamic calculation
    maybeUpdateDynamicPositions();
#endif
}

uint8_t UWBTagTDoA::GetAnchorsSeenCount()
{
    // Try to acquire mutex with small timeout (10ms) - non-blocking would often fail
    // since the estimator holds the mutex frequently
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Collect unique anchor IDs using etl::set for arbitrary uint8_t IDs
        etl::set<uint8_t, 16> seen_anchors;  // Max 16 unique anchors expected
        for (const auto& m : measurements) {
            seen_anchors.insert(static_cast<uint8_t>(m.anchor_a));
            seen_anchors.insert(static_cast<uint8_t>(m.anchor_b));
        }
        cached_anchors_seen = static_cast<uint8_t>(seen_anchors.size());
        xSemaphoreGive(measurements_mtx);
    }
    // Return cached value (either fresh or from last successful read)
    return cached_anchors_seen;
}

uint32_t UWBTagTDoA::GetNumberOfConnectedDevices()
{
    // Use the thread-safe anchors_seen count for consistency
    return GetAnchorsSeenCount();
}

template<libDw1000::IsrFlags TFlags>
void UWBTagTDoA::OnEvent()
{
    if constexpr (TFlags == libDw1000::RX_DONE) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventPacketReceived);
    } else if constexpr (TFlags == libDw1000::TX_DONE) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventPacketSent);
    } else if constexpr (TFlags == libDw1000::RX_TIMEOUT) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveTimeout);
    } else if constexpr (TFlags == libDw1000::RX_FAILED) {
        uwbTdoa2TagAlgorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveFailed);
    }
    m_DwData.interrupt_flags &= ~TFlags;  // Clear the specific flag at the end
}

/* **** libdw1000 **** */
void UWBTagTDoA::InterruptHandler()
{
    // Call libdw1000 interrupt handler
    isr_flag = true;
}

/* TODO: Move to FreeRTOS notifications */
static void txCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::TX_DONE;
}

static void rxCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_DONE;
}

static void rxTimeoutCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_TIMEOUT;
}

static void rxFailedCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_FAILED;
}

// TODO: Profile this callback. What's the update rate?
static void estimatorCallback(tdoaMeasurement_t* tdoa)
{
    if (xSemaphoreTake(measurements_mtx, portMAX_DELAY) == pdTRUE) {
        auto it = std::find_if(measurements.begin(), measurements.end(),
            [&tdoa](const tdoa_estimator::TDoAMeasurement& m) {
                return (m.anchor_a == tdoa->anchorIdA && m.anchor_b == tdoa->anchorIdB) ||
                        (m.anchor_a == tdoa->anchorIdB && m.anchor_b == tdoa->anchorIdA);
            });

        if (it != measurements.end()) {
            // Use raw measurement - ArduPilot's EKF handles filtering
            float distanceDiff = tdoa->distanceDiff;
            if (tdoa->anchorIdA == it->anchor_b) {  // If it's a reverse measurement, change sign
                distanceDiff = -distanceDiff;
            }

            it->tdoa = distanceDiff;
            it->timestamp = millis();
        } else {
            measurements.push_back({tdoa->anchorIdA, tdoa->anchorIdB, tdoa->distanceDiff, millis()});
        }
        xSemaphoreGive(measurements_mtx);
    }
}

// Maybe notify uppon update or if a buffer is full? TODO: Better flow control
static void estimatorProcess() {
    static tdoa_estimator::PosMatrix anchors_left;
    static tdoa_estimator::PosMatrix anchors_right;
    static tdoa_estimator::DynVector tdoas;
    static tdoa_estimator::DynVector last_position(3); // Stores the full 3D position state
    static bool first_estimation = true;
    static uint32_t anchor_id_range_sample = 0;

    // Configuration Flag: Set to true for 2D, false for 3D
    static constexpr bool USE_2D_ESTIMATOR = true; 
    static constexpr double ASSUMED_TAG_Z = 0.0; // Used for initial Z in 2D mode
    static constexpr int NUM_ITERATIONS = 10;
    static constexpr size_t MIN_MEASUREMENTS = 4; // Require 4 measurements for robustness in both modes

    if (xSemaphoreTake(measurements_mtx, portMAX_DELAY) == pdTRUE) {
        uint64_t current_time = millis(); // Get time at the start after mutex
        
#if TDOA_STATS_LOGGING == ENABLE
        size_t meas_count_before = measurements.size();
#endif
        
        // Remove stale measurements (older than 350ms). TODO: Make this parameter configurable in the future.
        // NOTE: For some reason sometimes we are getting measurements that are older than 350ms. 
        measurements.erase(
            std::remove_if(measurements.begin(), measurements.end(),
                [current_time](const tdoa_estimator::TDoAMeasurement& m) {
                    return (current_time - m.timestamp) > 350;  // 200ms is the timeout for a single measurement
                }),
            measurements.end()
        );

#if TDOA_STATS_LOGGING == ENABLE
        size_t meas_count_after = measurements.size();
        stats_stale_removed += (meas_count_before - meas_count_after);
        // Track min/max measurement counts
        if (meas_count_after < stats_min_meas_count) stats_min_meas_count = meas_count_after;
        if (meas_count_after > stats_max_meas_count) stats_max_meas_count = meas_count_after;
#endif

        // --- Optional Debug Prints ---
        // printf("Measurements: \n");
        // for (uint32_t i = 0; i < measurements.size(); i++) {
        //     printf("%f, ID1: %d, ID2: %d\n", measurements[i].tdoa, measurements[i].anchor_a, measurements[i].anchor_b);
        // }
        // printf("Anchor positions: \n");
        // for (uint32_t i = 0; i < anchor_positions.size(); i++) {
        //     printf("%f, %f, %f, ID: %d: \n", anchor_positions[i].x, anchor_positions[i].y, anchor_positions[i].z, static_cast<int>(anchor_positions[i].shortAddr[0] - '0')); 
        // }
        // ---------------------------

        // Only proceed if we have enough measurements
        if (measurements.size() >= MIN_MEASUREMENTS) {  
            // Resize matrices based on number of measurements
            anchors_left.resize(measurements.size(), 3);
            anchors_right.resize(measurements.size(), 3);
            tdoas.resize(measurements.size());

            // Fill matrices with current measurements
            for (size_t i = 0; i < measurements.size(); ++i) {
                const auto& meas = measurements[i];
                
                // Get anchor positions from the configuration lookup table
                anchors_left.row(i) << anchor_positions[meas.anchor_a].x,
                                        anchor_positions[meas.anchor_a].y,
                                        anchor_positions[meas.anchor_a].z;
                
                anchors_right.row(i) << anchor_positions[meas.anchor_b].x,
                                        anchor_positions[meas.anchor_b].y,
                                        anchor_positions[meas.anchor_b].z;
                
                // Newton-Raphson expects Left - Right difference.
                // Measurement tdoa = distanceDiff = Right - Left (from tdoaMeasurement_t definition)
                tdoas(i) = -meas.tdoa; 
            }

            // Initial Guess Calculation (only on first valid estimation)
            if (first_estimation) {
                tdoa_estimator::DynVector avg_pos = tdoa_estimator::DynVector::Zero(3);
                 for(size_t i = 0; i < measurements.size(); ++i) {
                    avg_pos += anchors_left.row(i).transpose();
                    avg_pos += anchors_right.row(i).transpose();
                }
                avg_pos /= (2.0 * measurements.size());
                last_position = avg_pos; // Use average X, Y, Z

                if (USE_2D_ESTIMATOR) { // If starting in 2D mode, override Z
                    last_position(2) = ASSUMED_TAG_Z;
                }
                first_estimation = false;
                LOG_INFO("Position estimator initialized at [%.2f, %.2f, %.2f]",
                         last_position(0), last_position(1), last_position(2));
            }

            // --- Run Estimator --- 
            uint64_t estimation_start_time = millis();
            tdoa_estimator::DynVector current_estimate_3d = last_position; // Use last state as starting point
            bool is_valid_estimate = false;
            double solution_rmse = 0.0;

            // Covariance to pass to App layer
            std::optional<std::array<float, 6>> position_covariance = std::nullopt;

            // Get configurable parameters
            const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
            const double rmseThreshold = static_cast<double>(uwbParams.rmseThreshold);
            const bool enableCovMatrix = uwbParams.enableCovMatrix != 0;

            if (USE_2D_ESTIMATOR) {
                // Prepare inputs for 2D estimator
                tdoa_estimator::PosVector2D initial_guess_2d = current_estimate_3d.head<2>();
                // Use the Z component from the current 3D state as the fixed Z for this iteration
                double fixed_z_for_estimation = current_estimate_3d(2);

                // Run 2D Newton-Raphson
                tdoa_estimator::SolverResult2D result = tdoa_estimator::newtonRaphson2D(
                    anchors_left,
                    anchors_right,
                    tdoas,
                    initial_guess_2d,
                    fixed_z_for_estimation,
                    NUM_ITERATIONS,
                    1e-4,  // convergenceThreshold (default)
                    rmseThreshold
                );

                if (result.valid) {
                    // Update only X and Y components of the 3D state vector
                    current_estimate_3d.head<2>() = result.position;
                    // Z component remains unchanged from the previous state in 2D mode
                    is_valid_estimate = true;
                    solution_rmse = result.rmse;

                    // Extract covariance if valid and enabled
                    if (enableCovMatrix && result.covarianceValid) {
                        position_covariance = std::array<float, 6>{
                            static_cast<float>(result.positionCovariance(0, 0)),  // var_x
                            static_cast<float>(result.positionCovariance(0, 1)),  // cov_xy
                            0.0f,                                                   // cov_xz (no Z correlation in 2D)
                            static_cast<float>(result.positionCovariance(1, 1)),  // var_y
                            0.0f,                                                   // cov_yz (no Z correlation in 2D)
                            100.0f                                                  // var_z (large uncertainty, Z is fixed)
                        };
                    }
                }

                uint64_t estimation_end_time = millis();
                int duration = estimation_end_time - estimation_start_time;
                // Serial.printf("Estimated position (2D): [%.2f, %.2f], RMSE: %.3f, it took %d ms\n\r",
                //              current_estimate_3d(0), current_estimate_3d(1), result.rmse, duration);

            } else { // Use 3D Estimator
                // Use the full 3D vector as the initial guess
                tdoa_estimator::DynVector initial_guess_3d = current_estimate_3d;

                // Run 3D Newton-Raphson
                tdoa_estimator::SolverResult result = tdoa_estimator::newtonRaphson(
                    anchors_left,
                    anchors_right,
                    tdoas,
                    initial_guess_3d,
                    NUM_ITERATIONS,
                    1e-4,  // convergenceThreshold (default)
                    rmseThreshold
                );

                if (result.valid) {
                    // Update the full 3D state vector
                    current_estimate_3d = result.position;
                    is_valid_estimate = true;
                    solution_rmse = result.rmse;

                    // Extract covariance if valid and enabled
                    if (enableCovMatrix && result.covarianceValid) {
                        position_covariance = std::array<float, 6>{
                            static_cast<float>(result.positionCovariance(0, 0)),  // var_x
                            static_cast<float>(result.positionCovariance(0, 1)),  // cov_xy
                            static_cast<float>(result.positionCovariance(0, 2)),  // cov_xz
                            static_cast<float>(result.positionCovariance(1, 1)),  // var_y
                            static_cast<float>(result.positionCovariance(1, 2)),  // cov_yz
                            static_cast<float>(result.positionCovariance(2, 2))   // var_z
                        };
                    }
                }

                uint64_t estimation_end_time = millis();
                int duration = estimation_end_time - estimation_start_time;
                // Serial.printf("Estimated position (3D): [%.2f, %.2f, %.2f], RMSE: %.3f, it took %d ms\n",
                //              current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), result.rmse, duration);
            }

            // --- Send Data to Application ---
            bool has_nan = current_estimate_3d.hasNaN();
            if (is_valid_estimate && !has_nan) { // Only send if the estimate is valid
                App::SendSample(current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), position_covariance);

                // Update the persistent state for the next iteration (Warm Start)
                last_position = current_estimate_3d;

#if TDOA_STATS_LOGGING == ENABLE
                stats_samples_sent++;
#endif
            } else {
#if TDOA_STATS_LOGGING == ENABLE
                stats_samples_rejected++;
                // Track rejection reason
                if (has_nan) {
                    stats_reject_nan++;
                } else if (!is_valid_estimate) {
                    stats_reject_rmse++;  // RMSE was too high in solver
                }
#endif
            }

            // --- Position Logging (timed interval) - logs regardless of validity ---
            uint64_t now_position_log = millis();
            if (now_position_log - position_last_log_time_ms >= POSITION_LOG_INTERVAL_MS) {
                const char* valid_str = (is_valid_estimate && !current_estimate_3d.hasNaN()) ? "OK" : "INVALID";
                LOG_DEBUG("Position: X=%.2f Y=%.2f Z=%.2f RMSE=%.3fm [%s]",
                          current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), solution_rmse, valid_str);
                position_last_log_time_ms = now_position_log;
            }
            // ------------------------------------------

        } else {
            // Not enough measurements to run estimator
#if TDOA_STATS_LOGGING == ENABLE
            stats_samples_rejected++;
            stats_reject_insufficient++;
#endif
        }

#if TDOA_STATS_LOGGING == ENABLE
        uint64_t now_ms = millis();
        if (now_ms - stats_last_log_time_ms >= STATS_LOG_INTERVAL_MS) {
            LOG_DEBUG("Stats: Sent=%u Rej=%u (RMSE=%u NaN=%u Insuff=%u) Meas=[%u-%u] Stale=%u",
                      stats_samples_sent, stats_samples_rejected,
                      stats_reject_rmse, stats_reject_nan, stats_reject_insufficient,
                      (stats_min_meas_count == UINT32_MAX) ? 0 : stats_min_meas_count,
                      stats_max_meas_count,
                      stats_stale_removed);
            // Reset all counters
            stats_samples_sent = 0;
            stats_samples_rejected = 0;
            stats_reject_rmse = 0;
            stats_reject_nan = 0;
            stats_reject_insufficient = 0;
            stats_stale_removed = 0;
            stats_min_meas_count = UINT32_MAX;
            stats_max_meas_count = 0;
            stats_last_log_time_ms = now_ms;
        }
#endif

        xSemaphoreGive(measurements_mtx);
    }
}

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
// Include for DynamicAnchorTelemetry definition
#include "wifi/wifi_discovery.hpp"

bool UWBTagTDoA::IsDynamicPositioningEnabled() {
    return s_useDynamicPositions;
}

uint8_t UWBTagTDoA::GetDynamicAnchorPositions(DynamicAnchorTelemetry* out, uint8_t maxCount) {
    if (!s_useDynamicPositions || out == nullptr || maxCount == 0) {
        return 0;
    }

    // Try to acquire mutex with short timeout (10ms) to avoid blocking discovery
    if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;  // Mutex unavailable, skip this update
    }

    // Copy positions from anchor_positions array
    uint8_t count = 0;
    for (uint8_t i = 0; i < 4 && count < maxCount; i++) {
        out[count].id = i;
        out[count].x = anchor_positions[i].x;
        out[count].y = anchor_positions[i].y;
        out[count].z = anchor_positions[i].z;
        count++;
    }

    xSemaphoreGive(measurements_mtx);
    return count;
}

void UWBTagTDoA::onInterAnchorDistance(uint8_t fromAnchor, uint8_t toAnchor, uint16_t distanceTimestampUnits, uint16_t fromAntennaDelay) {
    if (!s_useDynamicPositions) {
        return;
    }

    // Look up the "to" anchor's antenna delay from previously received packets
    uint16_t toAntennaDelay = uwbTdoa2TagGetAnchorAntennaDelay(toAnchor);

    // Correct TWR distance: subtract both endpoints' antenna delays
    // The raw TWR distance includes uncorrected antenna delays from both anchors
    // because TDoA anchors set dwSetAntenaDelay(0) in the DW1000
    int32_t corrected = static_cast<int32_t>(distanceTimestampUnits)
                      - static_cast<int32_t>(fromAntennaDelay)
                      - static_cast<int32_t>(toAntennaDelay);
    if (corrected < 0) corrected = 0;

    // Convert corrected DW1000 timestamp units to meters
    float distanceMeters = static_cast<float>(corrected) * DW1000_TIME_TO_METERS;

    // Update the calculator with the corrected distance measurement
    s_dynamicCalc.updateDistance(fromAnchor, toAnchor, distanceMeters);
}

void UWBTagTDoA::maybeUpdateDynamicPositions() {
    if (!s_useDynamicPositions) {
        return;
    }

    // Throttle updates to avoid excessive computation
    uint32_t now = millis();
    if ((now - s_lastPositionUpdate) < DYNAMIC_POS_UPDATE_INTERVAL_MS) {
        return;
    }

    // Check if we have enough data to calculate positions
    if (!s_dynamicCalc.canCalculate()) {
        return;
    }

    // Calculate new positions
    point_t newPositions[4];
    if (s_dynamicCalc.calculatePositions(newPositions, 4)) {
        // CRITICAL: Lock mutex before updating shared anchor_positions
        // This prevents race conditions with estimatorProcess() which reads these values
        if (xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(50)) == pdTRUE) {
            for (int i = 0; i < 4; i++) {
                anchor_positions[i].x = newPositions[i].x;
                anchor_positions[i].y = newPositions[i].y;
                anchor_positions[i].z = newPositions[i].z;
            }
            xSemaphoreGive(measurements_mtx);
        }

        s_lastPositionUpdate = now;

        // Log the updated positions periodically
        static uint32_t lastLogTime = 0;
        if ((now - lastLogTime) >= 2000) {  // Log every 2 seconds
            LOG_DEBUG("Dynamic Anchors: A0(%.2f,%.2f) A1(%.2f,%.2f) A2(%.2f,%.2f) A3(%.2f,%.2f)",
                      anchor_positions[0].x, anchor_positions[0].y,
                      anchor_positions[1].x, anchor_positions[1].y,
                      anchor_positions[2].x, anchor_positions[2].y,
                      anchor_positions[3].x, anchor_positions[3].y);
            lastLogTime = now;
        }
    }
}
#endif // USE_DYNAMIC_ANCHOR_POSITIONS

#endif // USE_UWB_MODE_TDOA_TAG