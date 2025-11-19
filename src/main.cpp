#include <Arduino.h>

#include "bsp/board.hpp"

#include "app.hpp"
#include "scheduler.hpp"

#include "uwb/uwb_frontend_littlefs.hpp"
#include "wifi/wifi_frontend_littlefs.hpp"
#include "bcn_konex/beacon_protocool.hpp"

#include "console/console.hpp"
#include "command_handler/command_handler.hpp"

#include "esp_task_wdt.h" // For Task Watchdog Timer

App app;

/**
 * @note Be careful with thread priority and update rate! If we input a too high update rate, the system will be overloaded and trigger a watchdog reset.
 * 
 */

/**
 * Definition of static tasks
 */
static StaticTaskHolder<etl::delegate<void()>, 8192> application_task = {
  "MainAppTask",
  10,                          
  1,                    // Priority
  etl::delegate<void()>::create<App, app, &App::Update>(),
  {},
  {}
};

// Wifi task holder
static StaticTaskHolder<etl::delegate<void()>, 8192> wifi_task = {
  "WifiTask",
  50,                // 50Hz         
  1,                 // Priority
  etl::delegate<void()>::create<WifiLittleFSFrontend, Front::wifiLittleFSFront, &WifiLittleFSFrontend::Update>(),
  {},
  {}
};

static StaticTaskHolder<etl::delegate<void()>, 8192> console_task = {
  "ConsoleTask",
  50,                // 50Hz for console refresh rate        
  1,                 // Priority
  etl::delegate<void()>::create<Console, Console::s_Console, &Console::Update>(),
  {},
  {}
};

#if STATUS_TASK == ENABLE
static StaticTaskHolder<etl::delegate<void()>, 8192> status_led_task = {
  "StatusLedTask",
  5,                // 10Hz for status led refresh rate        
  1,                 // Priority
  etl::delegate<void()>::create<App, app, &App::StatusLedTask>(),
  {},
  {}
};
#endif


void setup() {
    /**
     * 1. Wifi module setup
     *  1.1. The wifi can be specifed to operate as an access point or a station. Enable the configured mode.
     *  1.2  If station, connect to the specified network. If access point, start the access point.
     *  1.3  Start the web page. The access point should have a web page that allows the user to see and configure all the connected devices.
     * 
     * NOTE: 
     *  -  You can connect through USB to any device and configure it locally. Specifically useful for configuring network settings.
     *  -  There needs to be an EEPROM for configuring global settings. 
     *  -  The wifi module should allow for calibration of ANCHOR devices. It should also allow to specify the device as anchor or tag.
     * 
     * 2. UWB-TWR module setup
     * 
     * 3. Mavlink forwarding of messages.
     *    3.1 First usecase(Only on tag): A single first output through a IP address. 1 address 1 arducopter mavlink connection. 
     *    3.2 Second usecase(Anchor): Merge mavlink messages to a single mavlink connection so mission planner can see multiple copters.
     */

    /**
     * Initializes the application
     */
    app.Init();

    /**
     * Initializes and runs the scheduler
     */
    Scheduler::scheduler.Init();

    /**
     * Initialize all the frontends
     */
    Front::InitFrontends();

    /**
     * @brief Initialize command handler 
     * 
     */
    CommandHandler::Init();

    /**
     * Initialize console
     */
#if defined(MAKERFABS_ESP32_BOARD)
    Serial.flush(); // THis is done because if USB CDC is used it actually blocks until read is finished
#endif
    Serial.begin(115200); // THis is due to S3
    Console::s_Console.Init();

    esp_task_wdt_init(0xFFFF, false);  // Disable task watchdog
    // Disable the wdt for now

    /**
     * Instantiate tasks
     */
    Scheduler::scheduler.CreateStaticTask(application_task);
    Scheduler::scheduler.CreateStaticTask(wifi_task);
    Scheduler::scheduler.CreateStaticTask(console_task);

    #if STATUS_TASK == ENABLE
    Scheduler::scheduler.CreateStaticTask(status_led_task);
    #endif
}


// For now the main thread will run the uwb ranging frontend.
void loop() {
    // ISUES: Looks like this task triggers a wdt reset if we execute it from a periodic thread.
    Front::uwbLittleFSFront.Update();
}
