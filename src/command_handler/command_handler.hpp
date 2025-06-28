#ifndef COMMAND_HANDLER_HPP_
#define COMMAND_HANDLER_HPP_


#include <Arduino.h>


class CommandHandler {
public:

static void Init();

/**
 * @brief Execute a command and return the result
 * 
 * @param command 
 * @return String 
 * @note The input command must only contain a single command. 
 */
static String ExecuteCommand(const char* command);

};




#endif // COMMAND_HANDLER_HPP_