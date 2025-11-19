#include <Eigen.h>

#include <iostream>

#include "tdoa_newton_raphson.hpp"

#include "FunctionalInterrupt.h"

#include "tag/tdoa_tag_algorithm.hpp"

#include "uwb_tdoa_tag.hpp"

#include "scheduler.hpp"
#include "app.hpp"

static void txCallback(dwDevice_t *dev);
static void rxCallback(dwDevice_t *dev);
static void rxTimeoutCallback(dwDevice_t *dev);
static void rxFailedCallback(dwDevice_t *dev);

static void estimatorCallback(tdoaMeasurement_t* tdoa);
Eigen::MatrixXd spanToMatrix(etl::span<const UWBAnchorParam> data, int rows);
static void estimatorProcess();

static etl::vector<tdoa_estimator::TDoAMeasurement, 10> measurements;
static SemaphoreHandle_t measurements_mtx = xSemaphoreCreateMutex();
static etl::array<UWBAnchorParam, 8> anchor_positions;

static constexpr uint32_t kNumberMeasurements = 20;     // Preferably to be multiple of the number of anchors

static bool isr_flag = false;

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
    Serial.println("--- UWB Tag TDOA ---\n\r");
    vTaskDelay(pdMS_TO_TICKS(300));

    // Fill in anchor positions lookup table
    for (uint32_t i = 0; i < anchors.size(); i++) {
        uint32_t index_to_copy = anchors[i].shortAddr[0] - '0';
        if (index_to_copy < anchor_positions.size()) {
            anchor_positions[index_to_copy] = anchors[i];
        } 
    }

    // --- Echo anchor positions to the App --- 
    etl::array<double, 12> anchors_to_echo = {};
    // Fill the anchor positions from id 0 up to 3 (max 4 anchors for echoing)
    for (uint32_t i = 0; i < anchor_positions.size() && i < anchors_to_echo.size()/3 ; i++) {
        anchors_to_echo[i*3] = anchor_positions[i].x;
        anchors_to_echo[i*3 + 1] = anchor_positions[i].y;
        anchors_to_echo[i*3 + 2] = anchor_positions[i].z;
    }
    // --- Logging --- 
    Serial.println("[App LOG] Echoing Anchor Positions:");
    for (uint32_t i = 0; i < anchors_to_echo.size() / 3; ++i) {
        Serial.printf("  Anchor %d: X=%.2f, Y=%.2f, Z=%.2f\n", 
                      i, anchors_to_echo[i*3], anchors_to_echo[i*3+1], anchors_to_echo[i*3+2]);
    }
    // ---------------
    App::AnchorsToEcho(anchors_to_echo);
    // --------------------------------------

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000    
    if (result != 0) {
        Serial.print("DW1000 configuration failed, devid: ");
        Serial.println(static_cast<uint32_t>(result));
    }
    Serial.print("DW1000 Configured\n\r");
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

    const uint8_t* mode = MODE_SHORTDATA_FAST_ACCURACY;
    dwEnableMode(&m_Device, mode);
    dwSetChannel(&m_Device, CHANNEL_2);
    dwSetPreambleCode(&m_Device, PREAMBLE_CODE_64MHZ_9);

    dwEnableLargePower(&m_Device);
    dwUseSmartPower(&m_Device, false);
    // dwSetTxPower(&m_Device, 0x1F1F1F1Ful);

    dwSetReceiveWaitTimeout(&m_Device, 10000);

    dwCommitConfiguration(&m_Device);

    Serial.print("Initialized TDoA Tag: ");
    uint32_t dev_id = dwGetDeviceId(&m_Device);
    Serial.println(dev_id, HEX);
    Serial.println("\n\r");

    // Init the tdoa anchor algorithm
    uwbTdoa2TagAlgorithm.init(&m_Device, estimatorCallback); 

    attachInterrupt(digitalPinToInterrupt(uwb_config.pins.int_pin), 
    [this]() { 
        this->InterruptHandler(); 
        }, 
    RISING);

    measurements.reserve(10);

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
}

uint32_t UWBTagTDoA::GetNumberOfConnectedDevices()
{
    return measurements.size(); // Not quite right since there can be redundant measurements but OK for now
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
            // Update running average
            const float alpha = 0.5f;  // Exponential moving average weight (legacy 0.3)
            float distanceDiff = tdoa->distanceDiff;
            if (tdoa->anchorIdA == it->anchor_b) {  // If it's a reverse measurement, change sign
                distanceDiff = -distanceDiff;
            }

            it->tdoa = alpha * distanceDiff + (1 - alpha) * it->tdoa;
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
        
        // Remove stale measurements (older than 1 second)
        measurements.erase(
            std::remove_if(measurements.begin(), measurements.end(),
                [current_time](const tdoa_estimator::TDoAMeasurement& m) {
                    return (current_time - m.timestamp) > 1000;
                }),
            measurements.end()
        );

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
                Serial.println("Initialized position estimate.");
            }

            // --- Run Estimator --- 
            uint64_t estimation_start_time = millis();
            tdoa_estimator::DynVector current_estimate_3d = last_position; // Use last state as starting point

            if (USE_2D_ESTIMATOR) {
                // Prepare inputs for 2D estimator
                tdoa_estimator::PosVector2D initial_guess_2d = current_estimate_3d.head<2>();
                // Use the Z component from the current 3D state as the fixed Z for this iteration
                double fixed_z_for_estimation = current_estimate_3d(2);

                // Run 2D Newton-Raphson
                tdoa_estimator::PosVector2D new_position_2d = tdoa_estimator::newtonRaphson2D(
                    anchors_left,
                    anchors_right,
                    tdoas,
                    initial_guess_2d,
                    fixed_z_for_estimation,
                    NUM_ITERATIONS
                );

                // Update only X and Y components of the 3D state vector
                current_estimate_3d.head<2>() = new_position_2d;
                // Z component remains unchanged from the previous state in 2D mode

                uint64_t estimation_end_time = millis();
                int duration = estimation_end_time - estimation_start_time;
                // Serial.printf("Estimated position (2D): [%.2f, %.2f], Fixed Z: %.2f, it took %d ms\n\r",
                //              current_estimate_3d(0), current_estimate_3d(1), fixed_z_for_estimation, duration);

            } else { // Use 3D Estimator
                // Use the full 3D vector as the initial guess
                tdoa_estimator::DynVector initial_guess_3d = current_estimate_3d;

                // Run 3D Newton-Raphson
                tdoa_estimator::DynVector new_position_3d = tdoa_estimator::newtonRaphson(
                    anchors_left,
                    anchors_right,
                    tdoas,
                    initial_guess_3d,
                    NUM_ITERATIONS
                );

                // Update the full 3D state vector
                current_estimate_3d = new_position_3d;

                uint64_t estimation_end_time = millis();
                int duration = estimation_end_time - estimation_start_time;
                // Serial.printf("Estimated position (3D): [%.2f, %.2f, %.2f], it took %d ms\n",
                //              current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), duration);
            }

            // --- Send Data to Application --- 
            if (!current_estimate_3d.hasNaN()) { // Only send if the estimate is valid
                App::SendSample(current_estimate_3d(0), current_estimate_3d(1), current_estimate_3d(2), 20); 
            }

            // Update the persistent state for the next iteration
            // TODO: Add more robust sanity checks before updating (e.g., check for large jumps)
            if (!current_estimate_3d.hasNaN()) { // Basic sanity check
                last_position = current_estimate_3d;
            }

        } else {
            // Optional: Print message if not enough measurements
            // Serial.printf("Waiting for measurements (have %d, need %d)\n", measurements.size(), MIN_MEASUREMENTS);
        }
        
        xSemaphoreGive(measurements_mtx);
    }
}