#ifndef CONSOLE_HPP_
#define CONSOLE_HPP_

#include <Arduino.h>

class Console 
{
public:
    Console(Stream* serial);

    void Init();

    void Update();

    static Console s_Console;
private:
    void ExecuteCommand();

private:
    String m_Input{""};
    Stream* m_Serial;

};




#endif  // CONSOLE_HPP_