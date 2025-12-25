#include <Arduino.h>
#include <SimpleCLI.h>

#include "command_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "front.hpp"
#include "scheduler.hpp"

#include "uwb/uwb_frontend_littlefs.hpp"
#include "app/app_frontend_littlefs.hpp"
#include "config_manager/config_manager.hpp"

static constexpr int COMMAND_QUEUE_SIZE = 4;
static SimpleCLI simpleCLI(COMMAND_QUEUE_SIZE, COMMAND_QUEUE_SIZE);
static SemaphoreHandle_t commandQueueMutex;
static String commandResult;
    
static void readCallback(cmd* c);
static void readAllCallback(cmd* c);
static void writeCallback(cmd* c);
static void errorCallback(cmd_error* c);
static void startCallback(cmd* c);
static void calibrateCallback(cmd* c);
static void loadConfigCallback(cmd* c);
static void saveConfigCallback(cmd* c);
static void backupConfigCallback(cmd* c);

// Multi-config management callbacks
static void listConfigsCallback(cmd* c);
static void saveConfigAsCallback(cmd* c);
static void loadConfigNamedCallback(cmd* c);
static void deleteConfigCallback(cmd* c);

// LED 2 control callbacks
static void toggleLed2Callback(cmd* c);
static void getLed2StateCallback(cmd* c);

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

void CommandHandler::Init()
{
    commandQueueMutex = xSemaphoreCreateMutex();

    if (commandQueueMutex == NULL)
    {
        printf("Failed to create command queue mutex\n");
    }
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

    // Reboot command: reboot
    Command rebootCmd = simpleCLI.addCommand("reboot", [](cmd* c) {
        ESP.restart();
    });

    Command startCmd = simpleCLI.addCommand("start", startCallback);

    Command calibrateCmd = simpleCLI.addCommand("calibrate", calibrateCallback);
    
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

    Command deleteConfigCmd = simpleCLI.addCommand("delete-config", deleteConfigCallback);
    deleteConfigCmd.addArgument("name");

    // LED 2 control commands
    Command toggleLed2Cmd = simpleCLI.addCommand("toggle-led2", toggleLed2Callback);
    Command getLed2StateCmd = simpleCLI.addCommand("get-led2-state", getLed2StateCallback);

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
static void readCallback(cmd* c)
{
    Command cmd(c);

    Argument groupArg = cmd.getArgument("group");
    Argument nameArg = cmd.getArgument("name");

    String paramGroup = groupArg.getValue();
    String valueGroup = nameArg.getValue();
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
                
                if (type == ParamType::STRING) {
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

// ********** LED 2 Control Callbacks **********

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