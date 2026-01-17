#include "config/features.hpp"  // MUST be first project include

#ifdef USE_UWB_MODE_TWR_TAG

#include <Eigen.h>

#include <iostream>

#include <SPI.h>
#include <DW1000Ranging.h>
#include <DW1000.h>
#include <DW1000Time.h>


#include <etl/vector.h>
#include <etl/variant.h>
#include <etl/visitor.h>

#include "trilat_generic.hpp"

#include "uwb_tag.hpp"

#include "app.hpp"

#include "wifi/wifi_frontend_littlefs.hpp"

static constexpr uint32_t ANCHOR_DISTANCE_EXPIRED = 300;   //measurements older than this are ignore (milliseconds)

static void newRange();
static void newDevice(DW1000Device *device);
static void inactiveDevice(DW1000Device *device);

// leftmost two bytes below will become the "short address"
// TODO: Add device short address on UWBConfig struct and perform corresponding configuration
static char tag_addr[] = "7F:00:22:EA:82:60:3B:9C";
static Eigen::Vector3d current_tag_position; //tag current position (meters with respect to origin anchor)

static TrilatStorage trilat_storage = Trilat::ITrilat();    // Empty trilat storage
static Eigen::MatrixX3d anchor_coordinates; //matrix for least squares solution
static Eigen::VectorXd last_anchor_distance(UWBParams::maxAnchorCount); //distances from tag to anchors
static etl::vector<uint32_t, UWBParams::maxAnchorCount> last_anchor_update = {}; //millis() value last time anchor was seen
static uint8_t anchorCount = 0;

// @todo: make a calculation of each anchor to anchor distance for each anchor and set that maximum range. 
// We will then have a calculated maximum range for each anchor.
static float max_allawed_range = 0.0f;
static float max_allowed_range_added_ratio = 1.3f;

static uint32_t last_sample_timestamp = 0;
static uint32_t last_pos_timestamp = 0;

static constexpr uint16_t max_rms__cm = 150;       // Maximum RMS value in cm. If value is higher, discard the measurement

static bool started = false;

// Kalman parameters for now (Move them to parameter config)
static constexpr float e_mea_x = 0.5; // Measurement noise for x: Tshe measurements from your sensor could deviate from the true value by a standard deviation of 1.0 units due to noise.
static constexpr float e_est_x = 1.0; // Estimation error for x: Initial estimate of the state could be off by a standard deviation of 1.0 units from the true state.
static constexpr float q_x = 0.1;     // Process noise for x: 

static constexpr float e_mea_y = 1.0; // Measurement noise for y
static constexpr float e_est_y = 1.0; // Estimation error for y
static constexpr float q_y = 0.1;     // Process noise for y

static SimpleKalmanFilter kFilterX(e_mea_x, e_est_x, q_x);
static SimpleKalmanFilter kFilterY(e_mea_y, e_est_y, q_y);  

// Ranges Kalman filter
SimpleKalmanFilter range_filters[4] = {
  {e_mea_x, e_est_x, q_x},
  {e_mea_x, e_est_x, q_x},
  {e_mea_x, e_est_x, q_x},
  {e_mea_x, e_est_x, q_x}
};

static Trilat::ITrilat& GetTrilat();
static void CreateTrilat(const Eigen::MatrixX3d& anchors);

UWBTag::UWBTag(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors)
    : UWBBackend(front, uwb_config)
{
  /**
   * @brief Reserve memory for trilat based on anchor data
   * 
   * For now we ignore anchor ids and just use LSB of short address as anchor id. We expect 1 through 4 and up to 7.
   */

  const bsp::UWBPinout& pinout = uwb_config.pins; 
  DW1000Ranging.initCommunication(pinout.reset_pin, pinout.spi_cs_pin, pinout.int_pin); //Reset, CS, IRQ pin

  // Here transform anchor data to anchor distance matrix and reserve data for tag to anchor distance  vector.
  anchorCount = anchors.size(); // Number of anchors
  anchorCount = min(anchorCount, UWBParams::maxAnchorCount); // Limit the number of anchors to the maximum number of anchors
  anchor_coordinates.resize(anchorCount, 3); // Resize the matrix to fit the data
  last_anchor_update.reserve(anchorCount);
  last_anchor_distance.resize(anchorCount);

  printf("anchor count: %d\n", anchorCount); 

  for(int i = 0; i < anchorCount; ++i) {
      // Assign the x, y, z coordinates to the matrix
      anchor_coordinates(i, 0) = anchors[i].x;
      anchor_coordinates(i, 1) = anchors[i].y;
      anchor_coordinates(i, 2) = anchors[i].z;
      max_allawed_range = max(anchors[i].x, max_allawed_range * max_allowed_range_added_ratio);
      max_allawed_range = max(anchors[i].y, max_allawed_range * max_allowed_range_added_ratio);
      last_anchor_update.push_back(0);
  }

  // Initialize trilat (Must initialize before attaching new range)
  CreateTrilat(anchor_coordinates);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);


  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY, false); 

#ifdef USE_BEACON_PROTOCOL
  etl::array<double, 12> anchors_to_echo = {};
  // Fill the anchor positions from id 0 to 3:
  for (uint32_t i = 0; i < anchor_coordinates.rows() && i < anchors_to_echo.size()/3 ; i++) {
    anchors_to_echo[i*3] = anchor_coordinates(i, 0);
    anchors_to_echo[i*3 + 1] = anchor_coordinates(i, 1);
    anchors_to_echo[i*3 + 2] = anchor_coordinates(i, 2);
  }
  App::AnchorsToEcho(anchors_to_echo);
#endif
}

void UWBTag::Update()
{
  DW1000Ranging.loop();
}

bool UWBTag::Start()
{
  started = true;
  return true;  // No error checking for now
}

// collect distance data from anchors, presently configured for 4 anchors
// solve for position if all four beacons are current

static void newRange()
{
  auto& trilat = GetTrilat();
  
  int i;
  //index of this anchor, expecting values 1 to 4

  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07;     //expect devices 1 to 7
  float range = 0;
  if (index > 0 && index < 5) {
    last_anchor_update[index - 1] = millis();                                 //(-1) => array index
    range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance(index-1) = range;  // We store the unfiltered range for trilateration(It will be filtered out later)
    if (range < 0.0 || range > max_allawed_range)    {
      last_anchor_update[index - 1] = 0;   //sanity check, ignore this measurement
    } 
    else {
      // Send range sample to ardupilot
      uint8_t id = index-1;
#ifdef USE_BEACON_PROTOCOL
      App::SendRangeSample(id, range);
#endif
      printf("Range %d: %f m\n", index, range);
    } 
  }
  //check for four measurements within the last interval
  int detected = 0;  //count anchors recently seen

  for (i = 0; i < anchorCount; i++) {

    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }
  if ( (detected == anchorCount)) { //four recent measurements

    trilat.Update(last_anchor_distance);
    current_tag_position = trilat.Read();

    // Calculate RMS
    double rms__m = trilat.ReadRMS();
    uint16_t rms__cm = (uint16_t)(rms__m * 100.0);

    if (rms__cm > max_rms__cm) {
      // Discard the measurement
      printf("Discarding measurement due to high RMS: %u\n", rms__cm);
      return;
    }

    // current_tag_position(0) = kFilterX.updateEstimate(current_tag_position(0));
    // current_tag_position(1) = kFilterY.updateEstimate(current_tag_position(1));
    
    // Here we should send data through uart to ardupilot
    uint32_t now = millis();
    if (now - last_pos_timestamp > 500  || !started ) {
      App::SendSample(current_tag_position(0), current_tag_position(1), 0.0f);  // TWR mode: no covariance
      last_pos_timestamp = now;
    }

    // Calculate refresh rates of each distance plus final 3D position
    uint32_t refresh_rate_hz = Utils::GetRefreshRate(last_sample_timestamp);

    Front::wifiLittleFSFront.UpdateLastTWRSample(current_tag_position(0), current_tag_position(1), current_tag_position(2), refresh_rate_hz);
    
    Serial.print("P= ");  //result
    Serial.print(current_tag_position(0));
    Serial.write(',');
    Serial.print(current_tag_position(1));
    Serial.write(',');
    Serial.print(current_tag_position(2));
    Serial.write(", RMS: ");
    Serial.print(rms__cm);
    Serial.println(" cm");
    
  }
}  //end newRange

static void newDevice(DW1000Device *device)
{
  int index = device->getShortAddress() & 0x07;     //expect devices 1 to 7

  if (index > 0 && index < anchor_coordinates.rows()+1) {
    // App::AddAnchorEcho(index);
  } else {
    printf("Careful!!! Index out of bounds: %d\n", index);
  }

  Serial.print("Device added: ");
  Serial.println(index, HEX);
}

static void inactiveDevice(DW1000Device *device)
{
  int index = device->getShortAddress() & 0x07;     //expect devices 1 to 7

  // App::SendRemoveAnchor(index);

  Serial.print("delete inactive device: ");
  Serial.println(index, HEX);
}

static void CreateTrilat(const Eigen::MatrixX3d& anchors)
{
    // If anchors are in the same Z plane 2D trilat is used, otherwise 3D trilat is used.
    for (int i = 0; i < anchors.rows(); i++) {
        if (anchors(i, 2) != anchors(0, 2)) {
            trilat_storage = Trilat3D(anchors);
            return;
        }
    }
    trilat_storage = Trilat2D(anchors);
}

static Trilat::ITrilat& GetTrilat()
{
  return GetInterface<Trilat::ITrilat>(trilat_storage);
}

#endif // USE_UWB_MODE_TWR_TAG