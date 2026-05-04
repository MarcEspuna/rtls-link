#ifndef COMMAND_HANDLER_HPP_
#define COMMAND_HANDLER_HPP_


#include <Arduino.h>

#include "protocol/rtls_binary_protocol.hpp"

using CommandBinaryFrame = rtls::protocol::BinaryFrameBuilder<2048>;

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

/**
 * @brief Execute a command that has a binary protocol response.
 *
 * @return true when the command was recognized and outFrame contains a complete frame.
 */
static bool TryExecuteBinaryCommand(const char* command, CommandBinaryFrame& outFrame);

};




#endif // COMMAND_HANDLER_HPP_
