#pragma once

// Remote types supported (20 or 21 key)
#define remotetype 21

#define RECEIVER_PIN 47

typedef enum {
#if remotetype == 20    
    KEY_POWER=0x45,
    KEY_MENU=0x47,
    KEY_TEST=0x44,
    KEY_PLUS= 0x40,
    KEY_RETURN=0x43,
    KEY_BACK=0x7,
    KEY_PLAY=0x15,
    KEY_FORWARD=0x9,
    KEY_0=0x16,
    KEY_MINUS=0x19,
    KEY_C=0xD,
    KEY_1=0xC,
    KEY_2=0x18,
    KEY_3=0x5E,
    KEY_4=0x8,
    KEY_5=0x1C,
    KEY_6=0x5A,
    KEY_7=0x42,
    KEY_8=0x52,
    KEY_9=0x4A,
    KEY_NONE=0
#elif (remotetype == 21)
    KEY_POWER=0x45,
    KEY_MENU=0x46,
    KEY_MUTE=0x47,
    KEY_PLAY=0x44,
    KEY_BACK=0x40,
    KEY_FORWARD=0x43,
    KEY_EQUALS=0x7,
    KEY_MINUS=0x15,
    KEY_PLUS= 0x9,
    KEY_0=0x16,
    KEY_RETURN=0x19,
    KEY_USD=0xD,
    KEY_1=0xC,
    KEY_2=0x18,
    KEY_3=0x5E,
    KEY_4=0x8,
    KEY_5=0x1C,
    KEY_6=0x5A,
    KEY_7=0x42,
    KEY_8=0x52,
    KEY_9=0x4A,
    KEY_NONE=0
#endif    
} IR_KEYS;

class irhelper{
public:
    // Setup constructor to pass values to the IRrecv constructor
    irhelper(int recvpin=RECEIVER_PIN);
    void IRInitialize(); //Initialize the IR
    IR_KEYS GetKeyPressed(); 
    char TranslateKey(unsigned long int result);
private:
    // Indicator of last key pressed.
    unsigned long m_last_key_processed = 0;
};

