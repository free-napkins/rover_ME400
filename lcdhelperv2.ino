#include "lcdhelperv2.h"

lcdhelper::lcdhelper(byte model, int RS, int WR, int CS, int RST, int SER):TFT_eSPI()
{}

//
//  Iniallize the LCD
//
void lcdhelper::LCDInitialize(byte orientation)
{
    TFT_eSPI::setRotation(orientation);
    currentBackgroundColor = RCB_PURPLE;
    currentForegroundColor = RCB_WHITE;
    // Can pass HEX code for any color if not satisfied with one of 24 default colors.
    TFT_eSPI::fillScreen(currentBackgroundColor);  
    //
    // Get screem coordinates.
    //
    int32_t x = TFT_eSPI::getViewportWidth();
    int32_t y = TFT_eSPI::getViewportHeight();
    //
    //  Draw rectangular border to screen.
    //
    TFT_eSPI::drawRoundRect(2,2,x-4,y-4,3,currentForegroundColor);
    //
    //  Setup fonts
    //
    TFT_eSPI::setTextColor(currentForegroundColor,currentBackgroundColor);
    TFT_eSPI::setTextFont(2);
}

void lcdhelper::setBackColor(uint16_t color){
	//
    //  Setup fonts
    //
    currentBackgroundColor = color;
    TFT_eSPI::setTextColor(currentForegroundColor,currentBackgroundColor);
}

void lcdhelper::setColor(uint16_t color){
    //
    //  Setup fonts
    //
    currentForegroundColor = color;
    TFT_eSPI::setTextColor(currentForegroundColor,currentBackgroundColor);
}

void lcdhelper::printNumI(long num, int x, int y, int length, char filler)
{
	char buf[25];
	char st[27];
	boolean neg=false;
	int c=0, f=0;
  
	if (num==0)
	{
		if (length!=0)
		{
			for (c=0; c<(length-1); c++)
				st[c]=filler;
			st[c]=48;
			st[c+1]=0;
		}
		else
		{
			st[0]=48;
			st[1]=0;
		}
	}
	else
	{
		if (num<0)
		{
			neg=true;
			num=-num;
		}
	  
		while (num>0)
		{
			buf[c]=48+(num % 10);
			c++;
			num=(num-(num % 10))/10;
		}
		buf[c]=0;
	  
		if (neg)
		{
			st[0]=45;
		}
	  
		if (length>(c+neg))
		{
			for (int i=0; i<(length-c-neg); i++)
			{
				st[i+neg]=filler;
				f++;
			}
		}

		for (int i=0; i<c; i++)
		{
			st[i+neg+f]=buf[c-i-1];
		}
		st[c+neg+f]=0;

	}

	print(st,x,y);
}

void lcdhelper::_convert_float(char *buf, double num, int width, byte prec)
{
	char format[10];
	
	sprintf(format, "%%%i.%if", width, prec);
	sprintf(buf, format, num);
}

void lcdhelper::printNumF(double num, byte dec, int x, int y, char divider, int length, char filler)
{
	char st[27];
	boolean neg=false;

	if (dec<1)
		dec=1;
	else if (dec>5)
		dec=5;

	if (num<0)
		neg = true;

	_convert_float(st, num, length, dec);

	if (divider != '.')
	{
		for (int i=0; i<sizeof(st); i++)
			if (st[i]=='.')
				st[i]=divider;
	}

	if (filler != ' ')
	{
		if (neg)
		{
			st[0]='-';
			for (int i=1; i<sizeof(st); i++)
				if ((st[i]==' ') || (st[i]=='-'))
					st[i]=filler;
		}
		else
		{
			for (int i=0; i<sizeof(st); i++)
				if (st[i]==' ')
					st[i]=filler;
		}
	}

	print(st,x,y);
}
//
//  Iniallize the LCD
//
void lcdhelper::LCDInitialize(byte orientation, bool initDisplay)
{
    if (initDisplay == true){
        TFT_eSPI::init();
    }
    LCDInitialize(orientation);
}

void lcdhelper::print(const String &st, int32_t x, int32_t y){
    if (x == CENTER)
    {
        TFT_eSPI::setTextDatum(MC_DATUM); // Middle center
        TFT_eSPI::drawString(st, TFT_eSPI::width() / 2, y);
    }
    else
    {
        TFT_eSPI::setTextDatum(TL_DATUM); // Top Left
        TFT_eSPI::drawString(st, x, y);
    }
}


void lcdhelper::print(const String &st, int32_t x, int32_t y, int padding){

    // tft.drawFloat(i/10.0, decimal_places, x, y, font);

    if (x == CENTER)
    {
        TFT_eSPI::setTextPadding(TFT_eSPI::textWidth(st, padding));
        TFT_eSPI::setTextDatum(MC_DATUM); // Middle center
        TFT_eSPI::drawString(st, TFT_eSPI::width() / 2, y);
        //Serial.println("yes");
    }
    else
    {
        TFT_eSPI::setTextPadding(TFT_eSPI::textWidth(st, padding));
        TFT_eSPI::setTextDatum(TL_DATUM); // Top Left
        TFT_eSPI::drawString(st, x, y);
    }
}