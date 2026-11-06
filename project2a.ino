#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>
//
// test change
// another test change
// pins
const int rencoder = 8;  // right optocoupler
const int lencoder = 10; // left optocoupler

// Motor driving pins
const int renablepwm = 14;
const int rphasedir = 13;
const int lenablepwm = 46;
const int lphasedir = 12;

volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;

//option 3, servo pin assignments
const int panPin = 36; 
const int tiltPin = 35;
const int trigPin = 38;
const int echoPin = 37;
//GLOBAL VARIABLES FOR OPTION 3
int targetAngle = 0;
int targetDist = 0;
bool option4Enabled = false;
const int maxDist = 120;   //farthest distance that counts as a target
const int minGroup = 2;    //minimum readings that would count as a target


// pwm channels
const int lpwmChannel = 1;
const int rpwmChannel = 2;
const int pwmFreq = 496;
const int pwmResolution = 8;
const int maxDutyCycle = (int)pow(2, pwmResolution) - 1;
long counttotarget = 0; // create global variable to be used in option 1

// wheel diamter in cm
float WheelDiameter = 6.5;
// slots per encoder disc
const int CountsPerRev = 20;

//bools for option completion
bool opt_1 = false;
bool opt_3 = false;

//switch intitialization
const int SW1 = 18;

//  Create an instance of the lcdhelper class named oLCD
//
lcdhelper oLCD(ILI9163_4L, 3, 2, 9, 10, 7);
//
//  Declare a sprite so we can display a rotated label on the TFT.
//
TFT_eSprite spr = TFT_eSprite(&oLCD); // Sprite object
//
//  Create an instance of irhelper class named oIR.
//
irhelper oIR;
// Get width of TFT display in pixels
int xScreen = 1;
// Get height of TFT display in pixels
int yScreen = 1;
// Define a spacer to represent margins and spacing between elements.
int mScreen = 2;
//
//  Define protypes for functions
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);
// *************************************************************************************
// BEGIN: USER FUNCTION
// *************************************************************************************
void clearText(int32_t x, int32_t y, int32_t width, uint16_t color)
{
    int32_t xStart;

    if (x == CENTER)
        xStart = (oLCD.width() - width) / 2;
    else
        xStart = x;

    oLCD.fillRect(xStart, y - 6, width, 15, color);
}
// functions used in option 1
void LeftEncoderISR();
void RightEncoderISR();
void updateDriveDisplay(float rightDuty, unsigned long leftCnt, unsigned long rightCnt, long err); 
void processKeyInput(char keypressed, String &aval);
long calculateTargetCounts(float wheelDiameter, int countsPerRev);
void straightDrive100cm(int baseDuty, long targetCounts);
void driveStraight(int baseDuty, float targetDistanceCm);

//  Option1 code
//
void Option1(char optionstate, char keypressed, bool initdisplay)
{
    char text[20];
    static String aval = "";

    if (optionstate == 'A' && keypressed == ' ')
    {
        // Initialize the display
        oLCD.LCDInitialize(PORTRAIT, initdisplay);
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
        sprintf(text, "DUTY CYCLE");
        oLCD.print(text, CENTER, 15);
        oLCD.print("PRESS", CENTER, 50);
        oLCD.print("<RETURN>", CENTER, 65);
        oLCD.print("WHEN", CENTER, 80);
        oLCD.print("READY", CENTER, 95);

        aval = "";
        clearText(CENTER, 32, 20, RCB_BLACK);
    }
    else if (optionstate == 'A' && keypressed != ' ')
    {
        // process all remote inputs
        processKeyInput(keypressed, aval);

        // Show current value of aval
        clearText(CENTER, 32, 20, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        if (aval.length() > 0)
        {
            oLCD.printNumI(aval.toInt(), CENTER, 32, 3, ' ');
        }
        else
        {
            oLCD.printNumI(0, CENTER, 32, 3, ' ');
        }
    }

    else if (optionstate == 'B')
    {
        if (aval.length() == 2)
        {
            int baseDuty = aval.toInt();
            counttotarget = calculateTargetCounts(WheelDiameter, CountsPerRev);
            straightDrive100cm(baseDuty, counttotarget);
        }
        bool opt_1 = true;
    }
}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    // DO NOT SHARE CODE WITH STUDENTS.  THIS IS A COMPETITION SO LETS SEE WHAT THEY COME UP.
}
//
//  Option3 code
//
float radarRadius(){
    return ((xScreen - mScreen)/2.0) - 2;
}

//--OPTION 3-- function to draw circles and gridlines 
void drawGrid(){
    float xC = xScreen/2.0;
    float yC = yScreen;
    float rMax = radarRadius();
    oLCD.drawCircle(xC, yC, rMax, RCB_WHITE);
    oLCD.drawCircle(xC, yC, rMax/2.0, RCB_WHITE);
    //draw gridlines from the center at 30 degree increments
    int gridlineAngles[5] = {30, 60, 90, 120, 150};
    for (int i = 0; i<5; i++){
        float rad = gridlineAngles[i]*PI/ 180.0;
        int x1 = xC + rMax*cos(rad); //use trig functions to get gridline coordinates
        int y1 = yC - rMax*sin(rad);
        oLCD.drawLine(xC, yC, x1, y1, RCB_WHITE);
    }
}

//--OPTION 3-- function to clear screen and draw radar text
void drawRadar(bool initdisplay){
    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    oLCD.print("ANGLE", 15, 5);
    oLCD.print("DEGREES", 100, 5);
    oLCD.print("DISTANCE", 15, 20);
    oLCD.print("CM", 100, 20);
    //empty black boxes for the readings
    clearText(60, 5, 35, RCB_BLACK);
    clearText(80, 20, 18, RCB_BLACK);
    drawGrid();
}

//--OPTION 3-- function to update the angle and distance readings
void updateReadings(int angle, int distance){
    clearText(60, 5, 35, RCB_BLACK);
    clearText(80, 20, 18, RCB_BLACK); 
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    oLCD.printNumI(angle, 65, 5, 3, ' ');
    oLCD.printNumI(distance, 81, 20);
}

//--OPTION 3-- function to measure distance from 0-120cm
int readDistance(){
    int d = calculateDistance(trigPin, echoPin);
    if (d <= 0 || d > maxDist){
        d = maxDist;
    }
    return d;
}

//--OPTION 3-- function to draw colored rays, blue for open space, red for obstacles
void drawRay(int angle, int dist){
    float xC = xScreen/2.0;
    float yC = yScreen;
    float rMax = radarRadius() - 1;  
    float rad = angle*PI / 180.0;
    float rMeas = (dist/(float)maxDist)*rMax; //make maxDist a float so it's not integer division
    int xBlue = xC + rMeas*cos(rad); // same logic as drawing the gridlines
    int yBlue = yC - rMeas*sin(rad);
    int xRed = xC + rMax*cos(rad);
    int yRed = yC - rMax*sin(rad);
    oLCD.drawLine(xC, yC, xBlue, yBlue, RCB_BLUE);
    if (dist < maxDist){
        oLCD.drawLine(xBlue, yBlue, xRed, yRed, RCB_RED);
    }
}

//--OPTION 3-- function to group readings into targets and keep the closest one
void findTarget(int dist, int angle, int &minDist, int &sumAngle, int &sumDist, int &count, int &theta, int &d){
    if (dist < maxDist){
        //part of a target, add to current group
        sumAngle += angle;
        sumDist += dist;
        count++;
    } else if (count > 0){
        //ignore groups too small to be a real target
        if (count >= minGroup){
            int avgDist = sumDist/count;
            if (avgDist < minDist){
                minDist = avgDist;
                theta = sumAngle/count;
                d = avgDist;
            }
        }
        sumAngle = 0;
        sumDist = 0;
        count = 0;
    }
}

//--OPTION 3-- function to run one sweep and return the closest target
void sweep(int startAngle, int endAngle, int step, int &theta, int &d){
    int minDist = 999;
    int sumAngle = 0;
    int sumDist = 0;
    int count = 0;
    theta = 0;
    d = 999;
    //get to the start angle first
    servowrite(panPin, startAngle, 10);
    //number of steps between the start and end angles
    int numSteps = (endAngle - startAngle) / step;
    for (int n = 0; n <= numSteps; n++){
        int i = startAngle + n*step;    //current angle
        servowrite(panPin, i, 3);
        int dist = readDistance();
        findTarget(dist, i, minDist, sumAngle, sumDist, count, theta, d);
        updateReadings(i, dist);
        drawRay(i, dist);
        drawGrid();    
    }
    findTarget(maxDist, endAngle, minDist, sumAngle, sumDist, count, theta, d);
}

//--OPTION 3-- function to display target result
void displayTarget(bool found, int angle, int dist){
    float xC = xScreen/2.0;
    float yC = yScreen;
    int boxWidth = 100;
    int boxHeight = 55;
    int x1 = xC - (boxWidth/2);
    int y1 = (yC/2) - 15;
    DrawBox(x1, y1, boxWidth, boxHeight, RCB_BLACK);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    if (found){
        oLCD.print("TARGET", CENTER, y1 + 12);
        oLCD.printNumI(angle, x1 + 6, y1 + 27, 3, ' ');
        oLCD.print(" DEGREES", x1 + 30, y1 + 27);
        oLCD.printNumI(dist, x1 + 26, y1 + 42, 3, ' ');
        oLCD.print(" CM", x1 + 50, y1 + 42);
    } else{
        oLCD.print("TARGET NOT", CENTER, y1 + 20);
        oLCD.print("FOUND", CENTER, y1 + 35);
    }
}

//--OPTION 3 MAIN--
void Option3(char optionstate, char keypressed, bool initdisplay){
    pinMode(panPin, OUTPUT);
    pinMode(tiltPin, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
    servowrite(panPin, 90, 10);
    servowrite(tiltPin, 90, 10);
    if (optionstate != 'A'){
        return; //makes so it does not keep looping 
    }
    int thetaf, df;    //target angle and distance for forward sweep
    int thetar, dr;    //target angle and distance for reverse sweep
    drawRadar(initdisplay);
    sweep(15, 165, 2, thetaf, df);
    drawRadar(false);    //clear before the reverse sweep
    sweep(165, 15, -1, thetar, dr);
    //take values from both sweeps
    if (df < maxDist && dr < maxDist){
        targetAngle = (thetaf + thetar)/2;
        targetDist = (df + dr)/2;
        option4Enabled = true;
        displayTarget(true, targetAngle, targetDist);
    } else {
        option4Enabled = false;
        displayTarget(false, 0, 0);
    }
    servowrite(panPin, 90, 10);    //return pan servo to center
}
//
//  Option4 code
//
void Option4(char optionstate, char keypressed, bool initdisplay)
{
    if (opt_1 == true && opt_3 == true)
    {

        oLCD.LCDInitialize(PORTRAIT, initdisplay);
        oLCD.print("TARGET DIR", 10, 10);
        oLCD.print(" DEGREES", 10, 24);
        oLCD.print("TARGET DIST", 10, 38);
        oLCD.print(" CM", 10, 52);
        oLCD.print("4. NAVIGATE", 10, 66);

        // Reset Encoder Counts
        leftEncoderCount = 0;
        rightEncoderCount = 0;

        // Calculate Ntarget for 14 cm
        float wheelCircumference = 3.14159 * WheelDiameter;          // get the circumference of the wheel by pi*diameter
        counttotarget = (14.0 / wheelCircumference) * CountsPerRev; // find the numbre of encoder counts to the target of 14 cm

        //Initial motor commands (Forward direction)
        digitalWrite(lphasedir, HIGH);
        digitalWrite(rphasedir, HIGH);

        float leftPWM = 0.3 * maxDutyCycle / 100.0;
        float rightDuty = 0.3;
        float rightPWM = leftPWM;                  // Starts equal to base duty cycle
        float gainfactor = 0.5;                    // Gain factor (change this if not working properly)
        float adjustedRightDuty = 0.3; // define the adjusted duty cycle

        ledcWrite(lpwmChannel, leftPWM);
        ledcWrite(rpwmChannel, rightPWM);

        unsigned long lastDisplayUpdate = millis();

        //Loop until both wheels hit target distance or POWER ('P') pressed
        while ((long)leftEncoderCount < counttotarget || (long)rightEncoderCount < counttotarget)
        {

            // Check for emergency stop key ('P')
            IR_KEYS currentKey = oIR.GetKeyPressed();
            if (oIR.TranslateKey(currentKey) == 'P')
            {
                break; // Stop loop immediately
            }

            long error = (long)leftEncoderCount - (long)rightEncoderCount; // Calculate Error
            adjustedRightDuty = 0.3 + (error * gainfactor);

            // Clamp right motor duty cycle between 20% and 80%
            if (adjustedRightDuty < 20.0)
                adjustedRightDuty = 20.0;
            if (adjustedRightDuty > 80.0)
                adjustedRightDuty = 80.0;

            rightPWM = adjustedRightDuty * maxDutyCycle / 100.0;
            ledcWrite(rpwmChannel, rightPWM);

            //Update TFT display every 1.0 second
            if (millis() - lastDisplayUpdate >= 1000)
            {
                lastDisplayUpdate = millis();
                // Update TFT with RIGHT DUTY CYCLE, LEFT COUNT, RIGHT COUNT, ERROR
                updateDriveDisplay(adjustedRightDuty, leftEncoderCount, rightEncoderCount, error);
            }
        }

        //Stop both motors immediately after reaching target or power key
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);

        pinMode(SW1, INPUT_PULLUP);

        if (digitalRead(SW1) == LOW)
        {
            
        }
    }
}
// *************************************************************************************
// END: USER FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// BEGIN: CORE FUNCTIONALITY
// *************************************************************************************
//
//  Setup(): Perform initialization
//
void setup()
{
    Serial.begin(115200);

    // configure encoder interrupt pins
    pinMode(lencoder, INPUT_PULLDOWN);
    pinMode(rencoder, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(lencoder), LeftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rencoder), RightEncoderISR, RISING);

    // Configure Motor Phase Direction Pins
    pinMode(lphasedir, OUTPUT);
    pinMode(rphasedir, OUTPUT);

    // configuring pwm channels for the left and right motoer
    ledcSetup(lpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(lenablepwm, lpwmChannel);
    ledcWrite(lpwmChannel, 0);
    ledcSetup(rpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(renablepwm, rpwmChannel);
    ledcWrite(rpwmChannel, 0);

    // Initialize IR Communications
    oIR.IRInitialize();
    // Show main menu when Arduino starts
    // for the 1st time or on reset.
    ShowDisplay(SC_MAIN, ' ', ' ', true);
    // Get width of TFT display in pixels
    xScreen = oLCD.getViewportWidth();
    // Get height of TFT display in pixels
    yScreen = oLCD.getViewportHeight();
}

//
//  Loops(): Perform looping operations
//
void loop()
{
    unsigned long last_key_processed;
    // Determine which key was pressed.
    last_key_processed = oIR.GetKeyPressed();
    // Return to the main menu any time the
    // menu key is pressed.
    if (last_key_processed == KEY_MENU)
    {
        ShowDisplay(SC_MAIN, ' ', ' ');
    }
    else
    {
        if (oLCD.SCREEN_STATE == SC_MAIN)
        {
            if (last_key_processed == KEY_1)
            {
                ShowDisplay(SC_SUB1, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_2)
            {
                ShowDisplay(SC_SUB2, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_3)
            {
                ShowDisplay(SC_SUB3, 'A', ' ', false);
            }
            else if (last_key_processed == KEY_4)
            {
                ShowDisplay(SC_SUB4, 'A', ' ', false);
            }
        }
        if (oLCD.SCREEN_STATE == SC_SUB1)
        {
            // If the user presses one of the numeric keys, then display the key in the correct place on the screen.
            if (last_key_processed == KEY_0 || last_key_processed == KEY_1 || last_key_processed == KEY_2 || last_key_processed == KEY_3 || last_key_processed == KEY_4 ||
                last_key_processed == KEY_5 || last_key_processed == KEY_6 || last_key_processed == KEY_7 || last_key_processed == KEY_8 || last_key_processed == KEY_9 ||
                last_key_processed == KEY_BACK)
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE, 'A', oIR.TranslateKey(last_key_processed), false);
            }
            else if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE, 'B', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB2)
        {
            Serial.print("Sub2 trace = ");
            Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB2, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB3)
        {
            Serial.print("Sub3 trace = ");
            Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB3, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB4)
        {
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else
            {
                ShowDisplay(SC_SUB4, ' ', oIR.TranslateKey(last_key_processed), false);
            }
        }
    }
    delay(1000);
}
//
//  Method to display menus on TFT display
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay)
{
    // Store screen state.
    oLCD.SCREEN_STATE = val;
    oLCD.OPTION_STATE = optionstate;
    int selectedpid = 0;

    if (val == SC_MAIN)
    {
        //
        //  Initialize the display
        //
        oLCD.LCDInitialize(LANDSCAPE, initdisplay);
        oLCD.print("SELECT OPTION", 10, 10);
        oLCD.print("1. CALIBRATE DRIVE", 10, 24);
        oLCD.print("2. SEEK TARGET", 10, 38);
        oLCD.print("3. LOCATE TARGET", 10, 52);
        oLCD.print("4. NAVIGATE", 10, 66);
    }
    if (val == SC_SUB1)
    {
        Serial.println("option1");
        Option1(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB2)
    {
        Serial.println("option2");
        Option2(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB3)
    {
        Serial.println("option3");
        Option3(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB4)
    {
        Serial.println("option4");
        Option4(optionstate, keypressed, initdisplay);
    }
}
// *************************************************************************************
// BEGIN: UTILITY FUNCTIONS
// *************************************************************************************
//
//  Function to move the servo to the specified angle in degrees
//
void servowrite(int pin, int angle, int numpulses)
{
    //  Move servo attached to pin to the specified angle.
    //  The numpulses shoul be set to approximately 3 for
    //  an angular movement of 2 or less degrees.  For a large
    //  angular change numpulses should be set to 10.
    //  Set the pulse height in microseconds associated
    //  with the servo angles of 0 and 180 degrees
    //
    int microzero = 500;
    int micro180 = 2400;
    int lenMicroSecondsOfPeriod = 20 * 1000;
    // pin for servo, angle to move, microseconds for zero degrees, microseconds for 180 degrees
    int lenMicroSecondsOfPulse = map(angle, 0, 180, microzero, micro180); // map angle to microseconds
    // Loop long enough for servo to see enough of waveform to fully
    // move to the new position.
    for (int i = 0; i < numpulses; i++)
    {
        // Servos work by sending a 20 ms pulse.
        // 1.0 ms at the start of the pulse will turn the servo to the 0 degree position
        // 1.5 ms at the start of the pulse will turn the servo to the 90 degree position
        // 2.0 ms at the start of the pulse will turn the servo to the 180 degree position
        // Turn voltage high to start the period and pulse
        digitalWrite(pin, HIGH);

        // Delay for the length of the pulse
        delayMicroseconds(lenMicroSecondsOfPulse);

        // Turn the voltage low for the remainder of the pulse
        digitalWrite(pin, LOW);

        // Delay this loop for the remainder of the period so we don't
        // send the next signal too soon or too late
        delayMicroseconds(lenMicroSecondsOfPeriod - lenMicroSecondsOfPulse);
    }
}
//
// Function for calculating the distance measured by the Ultrasonic sensor
//
int calculateDistance(int tPin, int ePin)
{
    // Variables for the duration and the distance
    long duration;
    int distance;
    digitalWrite(tPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(tPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(tPin, LOW);
    duration = pulseIn(ePin, HIGH, 15000); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2;
    return distance;
}
//
//  Code to display y axis label (rotated 90 degrees)
//
void DisplayRotatedText(int x, int y)
{
    oLCD.setPivot(x / 2, y / 2); // Set pivot to middle of TFT screen
    // Create the Sprite
    spr.setColorDepth(16);     // Create an 8bpp Sprite of 60x30 pixels
    spr.createSprite(124, 30); // 8bpp requires 64 * 30 = 1920 bytes
    //
    // Set pivot relative to top left corner of Sprite (increasing y will push the text
    // closer to the left hand side of the screen.  Increasing x will move the label up and down)
    spr.setPivot(57, 85);
    spr.fillSprite(RCB_PURPLE); // Fill the Sprite with black

    spr.setTextColor(RCB_WHITE);              // Green text
    spr.setTextDatum(MC_DATUM);               // Middle centre datum
    spr.drawString("speed (rpm)", 60, 15, 2); // Plot text, font 4, in Sprite at 12, 15

    // rotate text on screen by 270 degrees
    spr.pushRotated(270);

    spr.deleteSprite();
}
// *************************************************************************************
// END: UTILITY FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// START: OPTION1 SUPPORT
// *************************************************************************************
void LeftEncoderISR()
{
    leftEncoderCount++;
}
void RightEncoderISR()
{
    rightEncoderCount++;
}
//
// Helper function to render motor status formatted for Step 5g / 5h / 5i
//
void updateDriveDisplay(float rightDuty, unsigned long leftCnt, unsigned long rightCnt, long err)
{
    oLCD.LCDInitialize(PORTRAIT, false);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    oLCD.print("RIGHT DUTY CYCLE", CENTER, 10);
    oLCD.printNumF(rightDuty, 1, CENTER, 25);

    oLCD.print("LEFT COUNT", CENTER, 45);
    oLCD.printNumI(leftCnt, CENTER, 60, 5, '0');

    oLCD.print("RIGHT COUNT", CENTER, 80);
    oLCD.printNumI(rightCnt, CENTER, 95, 5, '0');

    oLCD.print("ERROR", CENTER, 115);
    if (err >= 0)
    {
        char errStr[10];
        sprintf(errStr, "+%03ld", err);
        oLCD.print(errStr, CENTER, 130);
    }
    else
    {
        oLCD.printNumI(err, CENTER, 130, 4, ' ');
    }
}

void processKeyInput(char keypressed, String &aval)
{
    if (keypressed == 'B') // Handle Backspace
    {
        if (aval.length() > 0)
        {
            aval.remove(aval.length() - 1);
        }
    }
    else if (keypressed >= '0' && keypressed <= '9') // Validate numeric entry
    {
        if (aval.length() == 0)
        {
            // Reject 0, 1, 2, 3 (Step 4a)
            if (keypressed >= '4' && keypressed <= '7')
            {
                aval += keypressed;
            }
        }
        else if (aval.length() == 1)
        {
            char firstDigit = aval.charAt(0);
            if (firstDigit == '7')
            {
                // Max 70 (Step 4b)
                if (keypressed == '0')
                {
                    aval += keypressed;
                }
            }
            else
            {
                // 40-69 allowed
                aval += keypressed;
            }
        }
    }
}

long calculateTargetCounts(float wheelDiameter, int countsPerRev)
{
    float wheelCircumference = 3.14159 * wheelDiameter;
    return (long)((100.0 / wheelCircumference) * countsPerRev);
}

void straightDrive100cm(int baseDuty, long targetCounts)
{
    bool rerunRequested = true; // Create variable in case of rerunning in option 1

    while (rerunRequested)
    {
        rerunRequested = false;
        // Reset Encoder Counts
        leftEncoderCount = 0;
        rightEncoderCount = 0;

        // Calculate Ntarget for 100 cm
        float wheelCircumference = 3.14159 * WheelDiameter;          // get the circumference of the wheel by pi*diameter
        counttotarget = (100.0 / wheelCircumference) * CountsPerRev; // find the numbre of encoder counts to the target of 100 cm

        // Step 5d: Initial motor commands (Forward direction)
        digitalWrite(lphasedir, HIGH);
        digitalWrite(rphasedir, HIGH);

        float leftPWM = (float)baseDuty * maxDutyCycle / 100.0;
        float rightDuty = baseDuty;
        float rightPWM = leftPWM;                  // Starts equal to base duty cycle
        float gainfactor = 0.08;                   // Gain factor (change this if not working properly)
        float adjustedRightDuty = (float)baseDuty; // define the adjusted duty cycle

        ledcWrite(lpwmChannel, leftPWM);
        ledcWrite(rpwmChannel, rightPWM);

        unsigned long lastDisplayUpdate = millis();

        // Step 5e-h: Loop until both wheels hit target distance or POWER ('P') pressed
        while ((long)leftEncoderCount < counttotarget || (long)rightEncoderCount < counttotarget)
        {

            // Check for emergency stop key ('P')
            IR_KEYS currentKey = oIR.GetKeyPressed();
            if (oIR.TranslateKey(currentKey) == 'P')
            {
                break; // Stop loop immediately
            }

            long error = (long)leftEncoderCount - (long)rightEncoderCount; // Calculate Error
            adjustedRightDuty = baseDuty + (error * gainfactor);

            // Clamp right motor duty cycle between 20% and 80%
            if (adjustedRightDuty < 20.0)
                adjustedRightDuty = 20.0;
            if (adjustedRightDuty > 80.0)
                adjustedRightDuty = 80.0;

            rightPWM = adjustedRightDuty * maxDutyCycle / 100.0;
            ledcWrite(rpwmChannel, rightPWM);

            // Step 5g: Update TFT display every 1.0 second
            if (millis() - lastDisplayUpdate >= 1000)
            {
                lastDisplayUpdate = millis();
                // Update TFT with RIGHT DUTY CYCLE, LEFT COUNT, RIGHT COUNT, ERROR
                updateDriveDisplay(adjustedRightDuty, leftEncoderCount, rightEncoderCount, error);
            }
        }
        // Emergency Stop Motor PWM (Step 5h/i)
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);

        // Final TFT Screen Refresh
        long finalError = (long)leftEncoderCount - (long)rightEncoderCount;
        updateDriveDisplay(adjustedRightDuty, leftEncoderCount, rightEncoderCount, finalError);

        // Post-Run Decision Loop (Step 6)
        bool waitingForUserChoice = true;
        while (waitingForUserChoice)
        {
            IR_KEYS keyAfterStop = oIR.GetKeyPressed();
            char translatedKey = oIR.TranslateKey(keyAfterStop);

            if (keyAfterStop == KEY_MENU || translatedKey == 'M')
            {
                waitingForUserChoice = false;
                ShowDisplay(SC_MAIN, ' ', ' '); // Return to main menu
            }
            else if (keyAfterStop == KEY_RETURN || translatedKey == 'R')
            {
                waitingForUserChoice = false;
                rerunRequested = true; // Repeat drive test
            }
            delay(50);
        }
    }
}

void driveStraight(int baseDuty, float targetDistanceCm)
{
    // Reset Encoder Counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Calculate target encoder counts for the specified distance
    float wheelCircumference = 3.14159 * WheelDiameter;
    long targetCounts = (long)((targetDistanceCm / wheelCircumference) * CountsPerRev);

    // Set motor direction to forward
    digitalWrite(lphasedir, HIGH);
    digitalWrite(rphasedir, HIGH);

    // Initialize PWM duty cycles
    float leftPWM = (float)baseDuty * maxDutyCycle / 100.0;
    float adjustedRightDuty = (float)baseDuty;
    float rightPWM = leftPWM;
    float gainfactor = 0.08;

    ledcWrite(lpwmChannel, leftPWM);
    ledcWrite(rpwmChannel, rightPWM);

    // Drive loop: Keep running until target counts are reached or emergency stop ('P') is pressed
    while ((long)leftEncoderCount < targetCounts || (long)rightEncoderCount < targetCounts)
    {
        // Check for emergency stop key ('P')
        IR_KEYS currentKey = oIR.GetKeyPressed();
        if (oIR.TranslateKey(currentKey) == 'P')
        {
            break; // Stop immediately
        }

        // Proportional straight-line correction error calculation
        long error = (long)leftEncoderCount - (long)rightEncoderCount;
        adjustedRightDuty = baseDuty + (error * gainfactor);

        // Clamp right motor duty cycle between 20% and 80%
        if (adjustedRightDuty < 20.0)
            adjustedRightDuty = 20.0;
        if (adjustedRightDuty > 80.0)
            adjustedRightDuty = 80.0;

        rightPWM = adjustedRightDuty * maxDutyCycle / 100.0;
        ledcWrite(rpwmChannel, rightPWM);

        // Small delay to prevent CPU thrashing (no screen refreshing overhead)
        delay(10);
    }

    // Stop both motors immediately upon reaching target or stopping
    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);
}
// *************************************************************************************
// END: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION2 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION2 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION3 SUPPORT
// *************************************************************************************
//
//  Option 3: Function to draw a box.
//
void DrawBox(int x1, int y1, int x2, int y2, uint32_t color)
{
    oLCD.fillRoundRect(x1, y1, x2, y2, 3, color);
}
// *************************************************************************************
// END: OPTION3 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION4 SUPPORT
// *************************************************************************************

// *************************************************************************************
// END: OPTION4 SUPPORT
// *************************************************************************************