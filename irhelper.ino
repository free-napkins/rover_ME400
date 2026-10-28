#include "irhelper.h"
#include <IRremote.hpp>
//
//  Define the class constructor.
//
irhelper::irhelper(int recvpin)
{}
//
//  Routine to initialize the IR Sensor.
//
void irhelper::IRInitialize(){
    // Begin the receiving process. This will enable the timer interrupt
    // which consumes a small amount of CPU every 50 µs.
    // Enable blinking the LED when during reception.
    // Because you can't see infrared light, blinking the LED
    // can be useful while troubleshooting, or just to give
    // visual feedback.
    IrReceiver.begin(RECEIVER_PIN);
}

 IR_KEYS irhelper::GetKeyPressed()
 {
    uint16_t received{KEY_NONE};
    String keyname;

    if (IrReceiver.decode())
    {
        if (IrReceiver.decodedIRData.protocol == NEC)
        {
            received = IrReceiver.decodedIRData.command;
        }
        IrReceiver.resume();
        return (IR_KEYS)received;
    }
    return KEY_NONE;
 } 

char irhelper::TranslateKey(unsigned long int result)
{
#if remotetype == 20     
    if (result == KEY_0)
        return '0';
    else if (result == KEY_1)
        return '1';
    else if (result == KEY_2)
        return '2';
    else if (result == KEY_3)
        return '3';
    else if (result == KEY_4)
        return '4';
    else if (result == KEY_5)
        return '5';
    else if (result == KEY_6)
        return '6';
    else if (result == KEY_7)
        return '7';
    else if (result == KEY_8)
        return '8';
    else if (result == KEY_9)
        return '9';
    else if (result == KEY_PLUS)
        return '+';
    else if (result == KEY_MINUS)
        return '-';  
    else if (result == KEY_POWER)
        return 'P';
    else if (result == KEY_MENU)
        return 'M';
    else if (result == KEY_TEST)
        return 'T';
    else if (result == KEY_RETURN)
        return 'R';
    else if (result == KEY_BACK)
        return 'B'; 
    else if (result == KEY_PLAY)
        return 'P';        
    else if (result == KEY_FORWARD)
        return 'F'; 
    else if (result == KEY_C)
        return 'C';     
    else
    {
        return 'N';
    }
#elif (remotetype == 21)
    if (result == KEY_0)
        return '0';
    else if (result == KEY_1)
        return '1';
    else if (result == KEY_2)
        return '2';
    else if (result == KEY_3)
        return '3';
    else if (result == KEY_4)
        return '4';
    else if (result == KEY_5)
        return '5';
    else if (result == KEY_6)
        return '6';
    else if (result == KEY_7)
        return '7';
    else if (result == KEY_8)
        return '8';
    else if (result == KEY_9)
        return '9';
    else if (result == KEY_PLUS)
        return '+';
    else if (result == KEY_MINUS)
        return '-';  
    else if (result == KEY_POWER)
        return 'P';
    else if (result == KEY_MENU)
        return 'M';
    else if (result == KEY_MUTE) //
        return 'S';
    else if (result == KEY_RETURN)
        return 'R';
    else if (result == KEY_BACK)
        return 'B'; 
    else if (result == KEY_PLAY)
        return 'P';        
    else if (result == KEY_FORWARD)
        return 'F'; 
    else if (result == KEY_EQUALS) //
        return 'E';     
    else if (result == KEY_USD) //
        return 'U'; 
    else
    {
        return 'N';
    }
#endif 
}