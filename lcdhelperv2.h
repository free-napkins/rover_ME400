#pragma once
//  Include UTFT library.
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

#define ILI9163_4L		38  // ILI9163	(Serial 5Pin)

#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

#define PORTRAIT 0
#define LANDSCAPE 1
#define PORTRAIT180 2
#define LANDSCAPE180 3

// Define colors since TFT_ colors are in RGB and we need then in BGR
// To get codes use a RBG565 converter with the R and B flipped in converter
#define RCB_BLACK       0x0000      
#define RCB_PURPLE      0x780F      
#define RCB_BLUE        0xF800      
#define RCB_GREEN       0x07E0      
#define RCB_RED         0x001F      
#define RCB_YELLOW      0x07FF     
#define RCB_WHITE       0xFFFF      
//  Declare screens
enum screen
{
    SC_MAIN = 0,
    SC_SUB1 = 1,
    SC_SUB2 = 2,
    SC_SUB3 = 3,
    SC_SUB4 = 4,
    SC_SUB5 = 5,
    SC_SUB6 = 6,
    SC_SUB7 = 7,
    SC_SUB8 = 8,
    SC_SUB9 = 9,
    SC_SUB10 = 10,
    SC_SUB11 = 11,
    SC_SUB12 = 12,
    SC_SUB13 = 13,
    SC_SUB14 = 14,
    SC_SUB15 = 15,
    SC_SUB16 = 16,
    SC_SUB17 = 17,
    SC_SUB18 = 18,
    SC_SUB19 = 19,
    SC_SUB20 = 20
};

class lcdhelper : public TFT_eSPI{
public:
   // Setup constructor to pass values to the UTFT constructor
   lcdhelper(byte model =ILI9163_4L, int RS = 3, int WR=2, int CS = 9, int RST=10, int SER = 7);
   void LCDInitialize(byte orientation);
   void LCDInitialize(byte orientation, bool initDisplay);
   void print(const String &st, int32_t x, int32_t y);
   void print(const String &st, int32_t x, int32_t y, int padding);
   void setColor(uint16_t color);
   void setBackColor(uint16_t color);
   void printNumF(double num, byte dec, int x, int y, char divider='.', int length=0, char filler=' ');
   void printNumI(long num, int x, int y, int length=0, char filler=' ');
   void _convert_float(char *buf, double num, int width, byte prec);
   short int SCREEN_STATE=0;
   //
   //  Variable to store the current state of a screen.
   //  For instance, a menu screen might display 
   //  different information depending on the state
   //  of the input
   //
   char OPTION_STATE=' ';
private:
   uint32_t currentBackgroundColor;
   uint32_t currentForegroundColor;
   // Declare variable to hold screen state
};

