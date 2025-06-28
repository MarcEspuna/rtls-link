
#include <Eigen.h>

#include <iostream>

#include <SPI.h>
#include <DW1000Ranging.h>
#include <DW1000.h>
#include <DW1000Time.h>


#include <etl/vector.h>

#include "trilat_generic.hpp"

#include "uwb_tag_posxyz.hpp"

#include "app.hpp"

#include "wifi/wifi_frontend.hpp"

struct coordinates_t {
    double x;
    double y;
    double z;
};

/////////////////////// POZYX  ///////////////////////
static void print_coordinates(coordinates_t coor);
static void print_anchor_coordinates();
static void get_ranges();
static void get_position();
static void send_beacon_config();
static void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm);
static void send_vehicle_position(coordinates_t& position, uint16_t pos_error);
static void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]);

static uint8_t stage = 0;   // 0 = initialisation, 1 = normal flight
static uint32_t beacon_loop_time = 0;

static bool new_range = false; 
static uint32_t new_range_idx = 0;
static constexpr float stage_1_hz = 0.5f; // 2 seconds
static uint32_t last_pos_update = 0;          // To track that the last position was updated in the last 2 seconds
//////////////////////////////////////////////////////

static constexpr uint32_t ANCHOR_DISTANCE_EXPIRED = 300;   //measurements older than this are ignore (milliseconds)
static constexpr uint32_t POSITION_EXPIRED = 300;   //position older than this are ignore (milliseconds)

static Trilat::Trilat<Trilat::Mode::k2D>& GetTrilat();

static void newRange();
static void newDevice(DW1000Device *device);
static void inactiveDevice(DW1000Device *device);

// leftmost two bytes below will become the "short address"
static char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";
static Eigen::Vector3d current_tag_position; //tag current position (meters with respect to origin anchor)

static Eigen::MatrixX3d anchor_coordinates; //matrix for least squares solution
static Eigen::VectorXd last_anchor_distance(UWBParams::maxAnchorCount); //distances from tag to anchors
static etl::vector<uint32_t, UWBParams::maxAnchorCount> last_anchor_update = {}; //millis() value last time anchor was seen
static uint8_t anchorCount = 0;

// @todo: make a calculation of each anchor to anchor distance for each anchor and set that maximum range. 
// We will then have a calculated maximum range for each anchor.
static float max_allawed_range = 0.0f;
static float max_allowed_range_added_ratio = 1.3f;

static uint32_t last_sample_timestamp = 0;

static constexpr uint16_t max_rms__cm = 150;       // Maximum RMS value in cm. If value is higher, discard the measurement



/////////////////////// KALMAN ///////////////////////
// Kalman parameters for now (Move them to parameter config)
static constexpr float e_mea_x = 0.5; // Measurement noise for x: Tshe measurements from your sensor could deviate from the true value by a standard deviation of 1.0 units due to noise.
static constexpr float e_est_x = 1.0; // Estimation error for x: Initial estimate of the state could be off by a standard deviation of 1.0 units from the true state.
static constexpr float q_x = 0.1;     // Process noise for x: 

static constexpr float e_mea_y = 1.0; // Measurement noise for y
static constexpr float e_est_y = 1.0; // Estimation error for y
static constexpr float q_y = 0.1;     // Process noise for y

static SimpleKalmanFilter kFilterX(e_mea_x, e_est_x, q_x);
static SimpleKalmanFilter kFilterY(e_mea_y, e_est_y, q_y);

// kalman filters for each range
// static SimpleKalmanFilter
// static   
/////////////////////////////////////////////////////




UWBTagPosXYZ::UWBTagPosXYZ(UWBFront& front, const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors)
    : UWBBackend(front, uwb_config)
{
  /**
   * @brief Reserve memory for trilat based on anchor data
   * 
   * For now we ignore anchor ids and just use LSB of short address as anchor id. We expect 1 through 4 and up to 7.
   */

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

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Initialize trilat
  GetTrilat();

  // start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false); 
}

void UWBTagPosXYZ::Update()
{
  DW1000Ranging.loop();

  // send beacon distances
  get_ranges();             // Send the new range if we have one available

  // during stage 0 (init) send position and beacon config as quickly as possible
  // during stage 1 send about every 2 seconds
  // TODO: Limit stage zero also for now
  // Here will only be stage 1. Stage zero will go to new range 
  uint32_t time_diff = millis() - beacon_loop_time;
  uint32_t time_diff_pos = millis() - last_pos_update;
  if (stage == 1 && time_diff > (1000.0/stage_1_hz) && time_diff_pos < POSITION_EXPIRED) {
      send_beacon_config();
      
      // send to ardupilot

      // TODO: Check if we actually have a position
      coordinates_t position;
      position.x = current_tag_position(0)*1000;
      position.y = current_tag_position(1)*1000;
      position.z = current_tag_position(2)*1000;
      send_vehicle_position(position, 0); // 40cm error for now
      
      beacon_loop_time = millis();
  }
}

// For pozxy it moves it to stage1
bool UWBTagPosXYZ::Start()
{
  stage = 1;
  return true;  // No error checking for now
}


// Send the new range on every new ranging event. Store corresponding results and timestamps.
static void newRange()
{
  auto trilat = GetTrilat();
  
  int i;
  //index of this anchor, expecting values 1 to 4

  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07;     //expect devices 1 to 7
  float range = 0;
  if (index > 0 && index < 5) {
    last_anchor_update[index - 1] = millis();                                 //(-1) => array index
    range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance(index-1) = range;
    if (range < 0.0 || range > max_allawed_range)    {
      last_anchor_update[index - 1] = 0;   //sanity check, ignore this measurement
    } 
    else {
      // Add data for sending to ardupilot
      new_range_idx = index;
      new_range = true;
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

    current_tag_position(0) = kFilterX.updateEstimate(current_tag_position(0));
    current_tag_position(1) = kFilterY.updateEstimate(current_tag_position(1));
    last_pos_update = millis();

    if (stage == 0) {
      send_beacon_config();
      
      // send to ardupilot
      coordinates_t position;
      // position.x = current_tag_position(0)*1000;
      // position.y = current_tag_position(1)*1000;
      // position.z = current_tag_position(2)*1000;
      position.x = 0;
      position.y = 0;
      position.z = 0;
      send_vehicle_position(position, 0); // Hardcoded 40cm error for now
    }

    // Here we should send data through uart to ardupilot (Handled on the main loop now for pozxy)
    // App::SendSample(current_tag_position(0), current_tag_position(1), 0.0f, 20); // Hardcode error for now (Added a fixed concervative)

    // Send data through debug socket as well. Web socket will do polling here to get refresh rate and 3D position for printing on screen
    
    // Calculate refresh rates of each distance plus final 3D position
    uint32_t refresh_rate_hz = Utils::GetRefreshRate(last_sample_timestamp);
    Front::wifiFront.UpdateLastTWRSample(current_tag_position(0), current_tag_position(1), current_tag_position(2), refresh_rate_hz);
    
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

  if (!(index > 0 && index < anchor_coordinates.rows()+1)) {
    printf("Careful!!! Index out of bounds: %d\n", index);
  }

  Serial.print("Device added: ");
  Serial.println(index, HEX);
}

static void inactiveDevice(DW1000Device *device)
{
  int index = device->getShortAddress() & 0x07;     //expect devices 1 to 7
  Serial.print("delete inactive device: ");
  Serial.println(index, HEX);
}

static Trilat::Trilat<Trilat::Mode::k2D>& GetTrilat()
{
  static Trilat::Trilat<Trilat::Mode::k2D> trilat(anchor_coordinates);
  return trilat;
}

// POZYX CODE

// get ranges for each anchor
static void get_ranges()
{
    // get range for each anchor
    if (new_range && new_range_idx > 0 && new_range_idx < anchorCount+1){
      // send_beacon_distance(new_range_idx-1, last_anchor_distance(new_range_idx-1)*1000); //convert to mm
      send_beacon_distance(new_range_idx-1, 0);
      new_range = false;
      new_range_idx = 6; //invalid index
    }
}

// send all beacon config to ardupilot
static void send_beacon_config()
{
  beacon_config_msg msg;
  msg.info.beacon_count = anchorCount;
  for (uint8_t i=0; i<anchorCount; i++) {
      msg.info.beacon_id = i;
      msg.info.x = anchor_coordinates(i, 0)*1000;
      msg.info.y = anchor_coordinates(i, 1)*1000;
      msg.info.z = anchor_coordinates(i, 2)*1000;
      send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
  }
  printf("Sent anchor info\n");
}

// send a beacon's distance to ardupilot
static void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}

// send vehicle's position to ardupilot
static void send_vehicle_position(coordinates_t& position, uint16_t pos_error)
{
    vehicle_position_msg msg;

    // sanity check position
    if (position.x == 0 || position.y == 0) {
        return;
    }

    msg.info.x = position.x;
    msg.info.y = position.y;
    //msg.info.z = position.z;
    msg.info.z = 0;
    msg.info.position_error = pos_error;
    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}

static void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }

    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    HardwareSerial& fcboardSerial = App::GetArdupilotSerial();

    // send message
    int16_t num_sent = 0;
    num_sent += fcboardSerial.write(MSG_HEADER);
    num_sent += fcboardSerial.write(msg_id);
    num_sent += fcboardSerial.write(msg_len);
    num_sent += fcboardSerial.write(data_buf, data_len);
    num_sent += fcboardSerial.write(&checksum, 1);
    // fcboardSerial.flush();
}
