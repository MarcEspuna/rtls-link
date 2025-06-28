#include <Arduino.h>
#include <SimpleCLI.h>

#include "command_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "front.hpp"
#include "scheduler.hpp"

#include "uwb/uwb_frontend.hpp"

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
    if (Front::uwbFront.StartTag())
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
    Front::uwbFront.PerformAnchorCalibration();
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