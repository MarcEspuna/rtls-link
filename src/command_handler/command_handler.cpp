#include "config/features.hpp"

#include <Arduino.h>
#include <SimpleCLI.h>

#include "command_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "front.hpp"
#include "scheduler.hpp"
#include "version.hpp"

#include "uwb/uwb_frontend_littlefs.hpp"
#include "app/app_frontend_littlefs.hpp"
#ifdef USE_CONSOLE_CONFIG_MGMT
#include "config_manager/config_manager.hpp"
#endif

static constexpr int COMMAND_QUEUE_SIZE = 4;
static SimpleCLI simpleCLI(COMMAND_QUEUE_SIZE, COMMAND_QUEUE_SIZE);
static SemaphoreHandle_t commandQueueMutex;
static String commandResult;

static void errorCallback(cmd_error* c);

#ifdef USE_CONSOLE_PARAM_RW
static void readCallback(cmd* c);
static void readAllCallback(cmd* c);
static void writeCallback(cmd* c);
#endif

#ifdef USE_CONSOLE_UWB_CONTROL
static void startCallback(cmd* c);
static void calibrateCallback(cmd* c);
#endif

#ifdef USE_CONSOLE_CONFIG_MGMT
static void loadConfigCallback(cmd* c);
static void saveConfigCallback(cmd* c);
static void backupConfigCallback(cmd* c);

// Multi-config management callbacks
static void listConfigsCallback(cmd* c);
static void saveConfigAsCallback(cmd* c);
static void loadConfigNamedCallback(cmd* c);
static void readConfigNamedCallback(cmd* c);
static void deleteConfigCallback(cmd* c);
#endif

#ifdef USE_CONSOLE_LED_CONTROL
// LED 2 control callbacks
static void toggleLed2Callback(cmd* c);
static void getLed2StateCallback(cmd* c);
#endif

// Helper functions for parameter read/write (used by PARAM_RW and CONFIG_MGMT)
#if defined(USE_CONSOLE_PARAM_RW) || defined(USE_CONSOLE_CONFIG_MGMT)
static bool IsUwbShortAddrName(const char* name) {
    if (strcmp(name, "devShortAddr") == 0) {
        return true;
    }
    if (strncmp(name, "devId", 5) == 0) {
        char idx = name[5];
        return idx >= '1' && idx <= '6' && name[6] == '\0';
    }
    return false;
}

static const UWBShortAddr* GetUwbShortAddrByName(const char* name) {
    const UWBParams& params = Front::uwbLittleFSFront.GetParams();
    if (strcmp(name, "devShortAddr") == 0) return &params.devShortAddr;
    if (strcmp(name, "devId1") == 0) return &params.devId1;
    if (strcmp(name, "devId2") == 0) return &params.devId2;
    if (strcmp(name, "devId3") == 0) return &params.devId3;
    if (strcmp(name, "devId4") == 0) return &params.devId4;
    if (strcmp(name, "devId5") == 0) return &params.devId5;
    if (strcmp(name, "devId6") == 0) return &params.devId6;
    return nullptr;
}

static String UwbShortAddrToString(const UWBShortAddr& addr) {
    char buf[3] = {};
    size_t len = 0;
    if (addr[0] != '\0') {
        buf[len++] = addr[0];
    }
    if (addr[1] != '\0') {
        buf[len++] = addr[1];
    }
    if (len == 0) {
        return String("0");
    }
    buf[len] = '\0';
    return String(buf);
}
#endif // USE_CONSOLE_PARAM_RW || USE_CONSOLE_CONFIG_MGMT

#ifdef USE_CONSOLE_PARAM_RW
static bool IsDigitChar(char c) {
    return c >= '0' && c <= '9';
}

static bool ParseShortAddrDigits(String input, char out[2], uint32_t& outLen) {
    input.trim();
    if (input.length() == 0 || input.length() > 2) {
        return false;
    }
    for (size_t i = 0; i < input.length(); i++) {
        if (!IsDigitChar(input[i])) {
            return false;
        }
    }
    out[0] = input[0];
    if (input.length() == 2) {
        out[1] = input[1];
        outLen = 2;
    } else {
        out[1] = '\0';
        outLen = 1;
    }
    return true;
}

static void TrimQuotedString(String& value) {
    value.trim();
    if (value.length() >= 2 && value[0] == '"' && value[value.length() - 1] == '"') {
        value = value.substring(1, value.length() - 1);
    }
}
#endif // USE_CONSOLE_PARAM_RW

#ifdef USE_CONSOLE_CONFIG_MGMT
static String escapeJsonString(const String& str) {
    String escaped;
    for (int i = 0; i < str.length(); i++) {
        char c = str[i];
        switch (c) {
            case '\"': escaped += "\\\""; break;
            case '\\': escaped += "\\\\"; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += c; break;
        }
    }
    return escaped;
}
#endif // USE_CONSOLE_CONFIG_MGMT

void CommandHandler::Init()
{
    commandQueueMutex = xSemaphoreCreateMutex();

    if (commandQueueMutex == NULL)
    {
        printf("Failed to create command queue mutex\n");
    }

#ifdef USE_CONSOLE_PARAM_RW
    Command readAll = simpleCLI.addCommand("readall", readAllCallback);
    readAll.addPositionalArgument("group", "all");

    // Read parameter: read --group <group parameter name> --name <parameter name>
    Command readCmd = simpleCLI.addCommand("read", readCallback);
    readCmd.addArgument("group");
    readCmd.addArgument("name");

    // Write command: write --group <group parameter name> --name <parameter name> --data <string to write>
    Command writeCmd = simpleCLI.addCommand("write", writeCallback);
    writeCmd.addArgument("group");
    writeCmd.addArgument("name");
    writeCmd.addArgument("data");
#endif // USE_CONSOLE_PARAM_RW

    // Reboot command: reboot (always available)
    Command rebootCmd = simpleCLI.addCommand("reboot", [](cmd* c) {
        ESP.restart();
    });

    // Firmware info command: firmware-info (always available)
    // Returns JSON with device info, version, board type, and build date/time
    Command firmwareInfoCmd = simpleCLI.addCommand("firmware-info", [](cmd* c) {
        commandResult = "{\"device\":\"";
        commandResult += DEVICE_TYPE;
        commandResult += "\",\"version\":\"";
        commandResult += FIRMWARE_VERSION;
        commandResult += "\",\"board\":\"";
        commandResult += BOARD_TYPE;
        commandResult += "\",\"buildDate\":\"";
        commandResult += BUILD_DATE;
        commandResult += "\",\"buildTime\":\"";
        commandResult += BUILD_TIME;
        commandResult += "\"}";
    });

#ifdef USE_CONSOLE_UWB_CONTROL
    Command startCmd = simpleCLI.addCommand("start", startCallback);
    Command calibrateCmd = simpleCLI.addCommand("calibrate", calibrateCallback);
#endif // USE_CONSOLE_UWB_CONTROL

#ifdef USE_CONSOLE_CONFIG_MGMT
    // LittleFS parameter management commands
    Command loadConfigCmd = simpleCLI.addCommand("load-config", loadConfigCallback);
    Command saveConfigCmd = simpleCLI.addCommand("save-config", saveConfigCallback);
    Command backupConfigCmd = simpleCLI.addCommand("backup-config", backupConfigCallback);

    // Multi-config management commands
    Command listConfigsCmd = simpleCLI.addCommand("list-configs", listConfigsCallback);

    Command saveConfigAsCmd = simpleCLI.addCommand("save-config-as", saveConfigAsCallback);
    saveConfigAsCmd.addArgument("name");

    Command loadConfigNamedCmd = simpleCLI.addCommand("load-config-named", loadConfigNamedCallback);
    loadConfigNamedCmd.addArgument("name");

    Command readConfigNamedCmd = simpleCLI.addCommand("read-config-named", readConfigNamedCallback);
    readConfigNamedCmd.addArgument("name");

    Command deleteConfigCmd = simpleCLI.addCommand("delete-config", deleteConfigCallback);
    deleteConfigCmd.addArgument("name");
#endif // USE_CONSOLE_CONFIG_MGMT

#ifdef USE_CONSOLE_LED_CONTROL
    // LED 2 control commands
    Command toggleLed2Cmd = simpleCLI.addCommand("toggle-led2", toggleLed2Callback);
    Command getLed2StateCmd = simpleCLI.addCommand("get-led2-state", getLed2StateCallback);
#endif // USE_CONSOLE_LED_CONTROL

    simpleCLI.setOnError(errorCallback);
}

String CommandHandler::ExecuteCommand(const char* command)
{
    if (xSemaphoreTake(commandQueueMutex, portMAX_DELAY) == pdTRUE) {
        // Parse command and execute
        simpleCLI.parse(command);
        String tmp = std::move(commandResult);
        xSemaphoreGive(commandQueueMutex);
        return tmp;
    }
    return "Failed to accuire command mutex";
}

// ********** Command Callbacks **********

#ifdef USE_CONSOLE_PARAM_RW
static void readCallback(cmd* c)
{
    Command cmd(c);

    Argument groupArg = cmd.getArgument("group");
    Argument nameArg = cmd.getArgument("name");

    String paramGroup = groupArg.getValue();
    String valueGroup = nameArg.getValue();

    if (paramGroup == "uwb" && IsUwbShortAddrName(valueGroup.c_str())) {
        const UWBShortAddr* addr = GetUwbShortAddrByName(valueGroup.c_str());
        if (!addr) {
            commandResult = "Name not found";
            return;
        }
        commandResult = "Param: " + UwbShortAddrToString(*addr);
        return;
    }

    // Read parameter
    char inData[128];
    uint32_t inDataLen = 0;
    ParamType inDataType = ParamType::UNDEFINED;
    ErrorParam ret = Front::ReadGlobalParam(paramGroup.c_str(), valueGroup.c_str(), inData, inDataLen, inDataType);
    inData[inDataLen] = '\0';

    switch (ret)
    {
    case ErrorParam::OK:
        // Respond with string
        // For now with the arguments that we received
        commandResult = "Param: " + String(inData, inDataLen);
        return;
        break;
    case ErrorParam::GROUP_NOT_FOUND:
        commandResult = "Group not found";
        break;
    case ErrorParam::NAME_NOT_FOUND:
        commandResult = "Name not found";
        break;
    case ErrorParam::FAILED_TO_READ:
        commandResult = "Failed to read";
        break;
    case ErrorParam::PARAM_TOO_LONG:
        commandResult = "Invalid param";
        break;
    }
}

static void writeCallback(cmd* c)
{
    Command cmd(c);

    Argument groupArg = cmd.getArgument("group");
    Argument nameArg = cmd.getArgument("name");
    Argument dataArg = cmd.getArgument("data");

    String paramGroup = groupArg.getValue();
    String valueGroup = nameArg.getValue();
    String data = dataArg.getValue();
    TrimQuotedString(data);

    if (paramGroup == "uwb" && IsUwbShortAddrName(valueGroup.c_str())) {
        char addrBytes[2] = {};
        uint32_t addrLen = 0;
        if (!ParseShortAddrDigits(data, addrBytes, addrLen)) {
            commandResult = "Invalid short address (expected 1-2 digits)";
            return;
        }
        ErrorParam ret = Front::WriteGlobalParam(paramGroup.c_str(), valueGroup.c_str(), addrBytes, addrLen);
        switch (ret)
        {
        case ErrorParam::OK:
            commandResult = "Param written";
            return;
        case ErrorParam::GROUP_NOT_FOUND:
            commandResult = "Group not found";
            break;
        case ErrorParam::NAME_NOT_FOUND:
            commandResult = "Name not found";
            break;
        case ErrorParam::FAILED_TO_WRITE:
            commandResult = "Failed to write";
            break;
        case ErrorParam::PARAM_TOO_LONG:
            commandResult = "Param too long";
            break;
        case ErrorParam::INVALID_DATA:
            commandResult = "Invalid data";
            break;
        default:
            commandResult = "Failed to write";
            break;
        }
        return;
    }

    // Write parameter
    ErrorParam ret = Front::WriteGlobalParam(paramGroup.c_str(), valueGroup.c_str(), data.c_str(), data.length());

    switch (ret)
    {
    case ErrorParam::OK:
        // Respond with string
        commandResult = "Param written";
        return;
        break;
    case ErrorParam::GROUP_NOT_FOUND:
        commandResult = "Group not found";
        break;
    case ErrorParam::NAME_NOT_FOUND:
        commandResult = "Name not found";
        break;
    case ErrorParam::FAILED_TO_WRITE:
        commandResult = "Failed to write";
        break;
    case ErrorParam::PARAM_TOO_LONG:
        commandResult = "Param too long";
        break;
    case ErrorParam::INVALID_DATA:
        commandResult = "Invalid data";
        break;
    }
}


static void readAllCallback(cmd* c)
{
    commandResult.clear();
    commandResult.reserve(512);
    
    Command cmd(c);
    Argument groupArg = cmd.getArgument("group");
    String group = groupArg.getValue();

    // Read layout of all frontends and read corresponding parameters

    // For now this is extreamly inefficient since the way we have to retreve those parameters, should use flatmap or something similar
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Front::Get();

    // Iterate over frontends, retrieve layout and read all parameters
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        const etl::span<const ParamDef>& layout = frontend->GetParamLayout();
        const etl::string_view frt_group = frontend->GetParamGroup();
        if (frt_group == group.c_str() || group == "all")
        {
            for (const ParamDef& param : layout)
            {
                char inData[128];
                uint32_t inDataLen = 0;
                ParamType inDataType = ParamType::UNDEFINED;
                ErrorParam ret = Front::ReadGlobalParam(frontend->GetParamGroup().data(), param.name, inData, inDataLen, inDataType);

                switch (ret)
                {
                case ErrorParam::OK:{
                    // Respond with string
                    String allparameters = String(frontend->GetParamGroup().data());
                    allparameters += ".";
                    allparameters += param.name;
                    allparameters += ": ";
                    allparameters += String(inData, inDataLen);
                    allparameters += "\n";
                    commandResult += allparameters;
                    break;
                }
                case ErrorParam::GROUP_NOT_FOUND:
                    commandResult += "Group not found\n";
                    break;
                case ErrorParam::NAME_NOT_FOUND:
                    commandResult += "Name not found\n";
                    break;
                case ErrorParam::FAILED_TO_READ:
                    commandResult += "Failed to read\n";
                    break;
                case ErrorParam::PARAM_TOO_LONG:
                    commandResult += "Invalid param\n";
                    break;
                }
            }
        }
    }
}
#endif // USE_CONSOLE_PARAM_RW

#ifdef USE_CONSOLE_UWB_CONTROL
static void startCallback(cmd* c)
{
    if (Front::uwbLittleFSFront.StartTag())
    {
        commandResult = "UWB Tag started";
    }
    else
    {
        commandResult = "Failed to start UWB Tag";
    }
}

static void calibrateCallback(cmd* c)
{
    Front::uwbLittleFSFront.PerformAnchorCalibration();
}
#endif // USE_CONSOLE_UWB_CONTROL

static void errorCallback(cmd_error* c)
{
    CommandError cmdError(c);

    commandResult = "Error: " + cmdError.toString() + "\n";

    // Respond with error message
    if (cmdError.hasCommand())
    {
        commandResult += "Did you mean: " + cmdError.getCommand().toString() + "?";
    }
}

#ifdef USE_CONSOLE_CONFIG_MGMT
static void loadConfigCallback(cmd* c)
{
    ErrorParam result = Front::LoadAllParams();
    
    switch (result) {
        case ErrorParam::OK:
            commandResult = "Configuration loaded successfully from /params.txt";
            break;
        case ErrorParam::FILE_NOT_FOUND:
            commandResult = "Warning: /params.txt not found, using default parameters";
            break;
        case ErrorParam::FILE_SYSTEM_ERROR:
            commandResult = "Error: Failed to initialize LittleFS";
            break;
        case ErrorParam::INVALID_DATA:
            commandResult = "Error: Invalid parameter format in /params.txt";
            break;
        default:
            commandResult = "Error: Failed to load configuration";
            break;
    }
}

static void saveConfigCallback(cmd* c)
{
    ErrorParam result = Front::SaveAllParams();
    
    switch (result) {
        case ErrorParam::OK:
            commandResult = "Configuration saved successfully to /params.txt";
            break;
        case ErrorParam::FILE_SYSTEM_ERROR:
            commandResult = "Error: Failed to write to LittleFS";
            break;
        default:
            commandResult = "Error: Failed to save configuration";
            break;
    }
}

static void backupConfigCallback(cmd* c)
{
    commandResult = "Current configuration:\n";
    commandResult += "{\n";
    
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Front::Get();
    bool first_group = true;
    
    for (size_t i = 0; i < frontends.size(); i++) {
        IFrontend* frontend = frontends[i];
        
        if (!first_group) {
            commandResult += ",\n";
        }
        first_group = false;
        
        commandResult += "  \"" + String(frontend->GetParamGroup().data()) + "\": {\n";
        
        etl::span<const ParamDef> params = frontend->GetParamLayout();
        bool first_param = true;
        
        for (size_t j = 0; j < params.size(); j++) {
            const ParamDef& param = params[j];
            
            if (!first_param) {
                commandResult += ",\n";
            }
            first_param = false;
            
            char value[256];
            uint32_t len = sizeof(value);
            ParamType type;
            
            if (frontend->GetParam(param.name, value, len, type) == ErrorParam::OK) {
                commandResult += "    \"" + String(param.name) + "\": ";

                if (frontend->GetParamGroup() == "uwb" && IsUwbShortAddrName(param.name)) {
                    const UWBShortAddr* addr = GetUwbShortAddrByName(param.name);
                    if (addr) {
                        commandResult += "\"" + UwbShortAddrToString(*addr) + "\"";
                    } else {
                        commandResult += "\"\"";
                    }
                } else if (type == ParamType::STRING) {
                    commandResult += "\"" + escapeJsonString(String(value)) + "\"";
                } else {
                    commandResult += String(value);
                }
            }
        }
        
        commandResult += "\n  }";
    }
    
    commandResult += "\n}";
}

// ********** Multi-Config Management Callbacks **********

static void listConfigsCallback(cmd* c)
{
    commandResult = ConfigManager::ListConfigsJson();
}

static void saveConfigAsCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::SaveConfigAs(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration saved as '" + name + "'\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name. Use only letters, numbers, underscores, and hyphens.\"}";
            break;
        case ConfigError::NAME_TOO_LONG:
            commandResult = "{\"success\":false,\"error\":\"Config name too long (max 32 characters)\"}";
            break;
        case ConfigError::MAX_CONFIGS_REACHED:
            commandResult = "{\"success\":false,\"error\":\"Maximum number of configurations reached (10)\"}";
            break;
        case ConfigError::FILE_SYSTEM_ERROR:
            commandResult = "{\"success\":false,\"error\":\"File system error\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}

static void loadConfigNamedCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::LoadConfigNamed(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration '" + name + "' loaded\"}";
            break;
        case ConfigError::CONFIG_NOT_FOUND:
            commandResult = "{\"success\":false,\"error\":\"Configuration not found\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name\"}";
            break;
        case ConfigError::FILE_SYSTEM_ERROR:
            commandResult = "{\"success\":false,\"error\":\"File system error\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}

static void readConfigNamedCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    commandResult = ConfigManager::ReadConfigNamedJson(name.c_str());
}

static void deleteConfigCallback(cmd* c)
{
    Command cmd(c);
    Argument nameArg = cmd.getArgument("name");
    String name = nameArg.getValue();

    ConfigError result = ConfigManager::DeleteConfig(name.c_str());

    switch (result) {
        case ConfigError::OK:
            commandResult = "{\"success\":true,\"message\":\"Configuration '" + name + "' deleted\"}";
            break;
        case ConfigError::CONFIG_NOT_FOUND:
            commandResult = "{\"success\":false,\"error\":\"Configuration not found\"}";
            break;
        case ConfigError::INVALID_NAME:
            commandResult = "{\"success\":false,\"error\":\"Invalid config name\"}";
            break;
        default:
            commandResult = "{\"success\":false,\"error\":\"Unknown error\"}";
            break;
    }
}
#endif // USE_CONSOLE_CONFIG_MGMT

// ********** LED 2 Control Callbacks **********

#ifdef USE_CONSOLE_LED_CONTROL
static void toggleLed2Callback(cmd* c)
{
    if (!Front::appLittleFSFront.IsLed2Configured()) {
        commandResult = "{\"success\":false,\"error\":\"LED 2 pin not configured\"}";
        return;
    }

    Front::appLittleFSFront.ToggleLed2();
    bool newState = Front::appLittleFSFront.GetLed2State();
    commandResult = "{\"success\":true,\"led2State\":" + String(newState ? "true" : "false") + "}";
}

static void getLed2StateCallback(cmd* c)
{
    if (!Front::appLittleFSFront.IsLed2Configured()) {
        commandResult = "{\"configured\":false}";
        return;
    }

    bool state = Front::appLittleFSFront.GetLed2State();
    commandResult = "{\"configured\":true,\"state\":" + String(state ? "true" : "false") + "}";
}
#endif // USE_CONSOLE_LED_CONTROL
