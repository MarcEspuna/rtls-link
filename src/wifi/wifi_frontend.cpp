#include "wifi_params.hpp"
#include "wifi_frontend.hpp"

#include "wifi_websocket.hpp"
#include "wifi_uart_bridge.hpp"
#include "wifi_tcp_server.hpp"

#include "command_handler/command_handler.hpp"

#include <utils/utils.hpp>

#include <etl/vector.h>

// I will define a static FreeRTOS task holder for station connection checks
static StaticTaskHolder<etl::delegate<void()>, 2048> wifi_connection_task = {
    "WifiConnTask", // Task name
    1,              // Frequency: 1Hz
    1,              // Priority
    etl::delegate<void()>::create<WifiFront, Front::wifiFront, &WifiFront::StationConnectionThread>(),
    {},
    {}
};

/**
 * Here we should actually load settings from EEPROM and try to connect to AP or initialize AP
 * and handle connections.
 * 
 * Parameters that we will read from EEPROM will be: 
 * - SSID
 * - Password
 * - Mode (AP or Station)
 * 
 * Compile-time config parameters: 
 * - Clients update rate 
 * - Clients service task stack size
 * - Clients service task priority
 * 
 */


namespace Front {
    WifiFront wifiFront;
}

WifiFront::WifiFront()
    : Frontend(__FUNCTION__), m_Backends(), m_TcpLoggingServer(nullptr), m_stationConnected(false), m_currentMode(WifiMode::UNDEFINED)
{
  
}

void WifiFront::Init()
{
  /* *
  *  1. Wifi module setup
  *  1.1. The wifi can be specifed to operate as an access point or a station. Enable the configured mode.
  *  1.2  If station, connect to the specified network. If access point, start the access point.
  *  1.3  Start the web page. The access point should have a web page that allows the user to see and configure all the connected devices.
  * */
  Frontend::InitEeprom();

  // Read EEPROM parameters and setup the wifi module
  Frontend::ReadParam(&WifiParams::mode, &m_currentMode);

  // ********** Initialize wifi module **********
  bool healtyWifi = false;
  switch (m_currentMode)
  {
  case WifiMode::AP:
    healtyWifi = SetupAP();
    break;
  case WifiMode::STATION:
    // @todo
    // IN station mode a xSemaphoreTake assert is triggered when trying to connect to the network. Very possible issue is that
    // the wifi connection has not been established yet and the different backend network servers are initializing.
    // This could perfectly not be the issue since it was only observed once a UWB device was added and the TWR was working. Further investigation needed.
    SetupStation();
    healtyWifi = true;
    break;
  default:
    printf("Wifi mode not defined\n");
    return;
  }

  // **** Web server backend ****
  uint8_t enable = 0;
  Frontend::ReadParam(&WifiParams::enableWebServer, &enable);
  if (enable && healtyWifi) {
    m_Backends.push_back(new WifiWebSocket("Hello", "/ws", 80));
    printf("--- Web server enabled ---\n");
  }

  // **** Uart bridge backend ****
  Frontend::ReadParam(&WifiParams::enableUartBridge, &enable);
  if (enable && healtyWifi) {
    etl::array<char, MAX_IP_LENGTH> gcsIp = {};
    Frontend::ReadParam(&WifiParams::gcsIp, gcsIp);

    uint16_t udpPort = 0;
    Frontend::ReadParam(&WifiParams::udpPort, &udpPort);

    IPAddress ip;
    if(ip.fromString(gcsIp.data())) {
      m_Backends.push_back(new WifiUartBridge(Serial2, ip, udpPort));
      printf("--- Uart bridge enabled ---\n");
    } else {
      printf("Invalid GSC IP address\n");
    }
  }

  // **** Debug socket backend ****
  Frontend::ReadParam(&WifiParams::enableDebugSocket, &enable);
  if (enable && healtyWifi) {
    uint16_t dbgPort = 0;
    Frontend::ReadParam(&WifiParams::dbgPort, &dbgPort);

    m_TcpLoggingServer = new WifiTcpServer(dbgPort);
    m_Backends.push_back(m_TcpLoggingServer);
    printf("--- Debug socket enabled ---\n");
  }
  printf("------ Wifi Frontend Initialized ------\n");
  // Schedule the station connection monitoring task
  Scheduler::scheduler.CreateStaticTask(wifi_connection_task);
}


void WifiFront::Update()
{
    // Connection checks are now handled in a separate task

    // Update all the backends
    for (WifiBackend* backend : m_Backends) {
        backend->Update();
    }
}

etl::span<const ParamDef> WifiFront::GetParamLayout() const
{
  // @TODO: Maybe use a constexpr map in the future
    static const etl::array<ParamDef, 11> layout = {{
        PARAM_DEF(WifiParams, mode),
        PARAM_DEF(WifiParams, ssidAP),
        PARAM_DEF(WifiParams, pswdAP),
        PARAM_DEF(WifiParams, ssidST),
        PARAM_DEF(WifiParams, pswdST),
        PARAM_DEF(WifiParams, gcsIp),
        PARAM_DEF(WifiParams, dbgPort),
        PARAM_DEF(WifiParams, udpPort),
        PARAM_DEF(WifiParams, enableWebServer),
        PARAM_DEF(WifiParams, enableUartBridge),
        PARAM_DEF(WifiParams, enableDebugSocket)
    }};
    return layout;
}

const etl::string_view WifiFront::GetParamGroup() const
{
  static constexpr etl::string_view group = "wifi";
  return group;
}

bool WifiFront::SetupAP()
{
    bool success = false;
    // Read AP EEPROM parameters
    etl::array<char,MAX_SSID_LENGTH> ssidAP = {};
    etl::array<char,MAX_PSWD_LENGTH> pswdAP = {};
    
    Frontend::ReadParam(&WifiParams::ssidAP, ssidAP);
    Frontend::ReadParam(&WifiParams::pswdAP, pswdAP);

    printf("ssidAP: %s\n", &ssidAP[0]);
    printf("pswdAP: %s\n", &pswdAP[0]);

    // Connect to Wi-Fi network with SSID and password
    printf("Setting AP (Access Point)…\n");

    // Start the AP (Access Point)
    success = WiFi.softAP(ssidAP.data(), pswdAP.data());
    IPAddress IP = WiFi.softAPIP();
    printf("AP IP address: %s\n", IP.toString().c_str());

    return success;
}

void WifiFront::SetupStation()
{
    // Read required parameters
    etl::array<char,MAX_SSID_LENGTH> ssidST = {};
    etl::array<char,MAX_PSWD_LENGTH> pswdST = {};
    
    Frontend::ReadParam(&WifiParams::ssidST, ssidST);
    Frontend::ReadParam(&WifiParams::pswdST, pswdST);

    // --- Static IP Configuration ---
    // IMPORTANT: Replace with your desired static IP, gateway, and subnet
    IPAddress local_IP(192, 168, 4, 4); // Static IP for this device
    IPAddress gateway(192, 168, 4, 1);    // Router/Gateway IP
    IPAddress subnet(255, 255, 255, 0); // Subnet mask
    // IPAddress primaryDNS(8, 8, 8, 8);   // Optional: Removed
    // IPAddress secondaryDNS(8, 8, 4, 4); // Optional: Removed

    // Configure static IP before connecting (without DNS)
    if (!WiFi.config(local_IP, gateway, subnet)) {
      printf("STA Failed to configure static IP\n");
      // Handle error appropriately - maybe return false or don't proceed?
    }
    // -------------------------------

    // Connect to Wi-Fi network with SSID and password
    printf("Connecting to %s with static IP %s...\n", ssidST.data(), local_IP.toString().c_str());
    WiFi.begin(ssidST.data(), pswdST.data());
}

// TODO: Double check that client TCP uses internal queue and does not block the task
void WifiFront::UpdateLastTWRSample(float x, float y, float z, uint32_t hz)
{
  // Send latest tag position through dbg socket
  // Parse twr sample to string
  char buffer[100] = {};
  float lastTwrSample[3] = {x, y, z};

  // Do not send nan values
  for (int i = 0; i < 3; i++) {
    if (std::isnan(lastTwrSample[i])) {
      lastTwrSample[i] = 0.0f;
    }
  }

  snprintf(buffer, 100, "%.2f %.2f %.2f %u hz",  lastTwrSample[0],  lastTwrSample[1],  lastTwrSample[2], hz);
  if (m_TcpLoggingServer) {
    m_TcpLoggingServer->AddForSending(buffer);
  }

}

// I will add the implementation of the station connection thread
void WifiFront::StationConnectionThread()
{
    // Check connection status only if in STATION mode
    if (m_currentMode == WifiMode::STATION) {
        bool currentlyConnected = (WiFi.status() == WL_CONNECTED);
        // Check if status changed from disconnected to connected
        if (currentlyConnected && !m_stationConnected) {
            printf("WiFi Station Connected! IP Address: %s\n", WiFi.localIP().toString().c_str());
        } else if (!currentlyConnected && m_stationConnected) {
            printf("WiFi Station Disconnected.\n");
        }
        // Update the tracked state
        m_stationConnected = currentlyConnected;
    }
}