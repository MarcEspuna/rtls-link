#include "console.hpp"

#include "freertos/FreeRTOS.h"

#include "command_handler/command_handler.hpp"

Console Console::s_Console(&Serial);

Console::Console(Stream *serial) : m_Serial(serial)
{
}

void Console::Init()
{
    if (m_Serial == nullptr)
    {
        return;
    }
    
    // Wait for 100ms to make sure serial has transmitted all the data previously written. We where using m_Serial.flush before but for 
    // USBCDC that caused an infinite wait when we where not connected to the PC with serial Monitor. Seems like flush for USBCDC need 
    // the host to be connected.
    delay(100);
    // m_Serial->begin(115200);
    m_Serial->print("------- Console Initialized -------\n> ");
}

void Console::Update()
{
    if (m_Serial == nullptr)
    {
        return;
    }

    while (m_Serial->available() > 0) {
        char received = m_Serial->read(); // Read the incoming byte

        if (received == '\n') {
            // If newline is received, execute the command 
            m_Serial->println();   
            ExecuteCommand();
            m_Input.clear();          // Clear the command string for the next command
            m_Serial->print("> "); // Print the prompt
        } else if (received == '\b' || received == 127) {
            // If backspace or delete is received
            if (m_Input.length() > 0) {
                m_Serial->print("\b \b"); // Move cursor back, print space to delete, then move back again
                m_Input.remove(m_Input.length() - 1); // Remove last character from input
            }
        } else {
            // If not newline or backspace/delete, add the character to the command string
            m_Input += received;
            m_Serial->print(received); // Echo the received character back to the console
        }
    }
}


void Console::ExecuteCommand()
{
    m_Input.trim();
    String cmdResult = CommandHandler::ExecuteCommand(m_Input.c_str());
    m_Serial->println(cmdResult);
}
