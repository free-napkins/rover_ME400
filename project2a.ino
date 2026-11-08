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

// option 3, servo pin assignments
const int trigPin = 38;
const int echoPin = 37;
// GLOBAL VARIABLES FOR OPTION 3
int targetAngle = 0;
int targetDist = 0;
bool option4Enabled = false;
const int maxDist = 120; // farthest distance that counts as a target
const int minGroup = 2;  // minimum readings that would count as a target

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

const int tiltServoPin = 35; // signal wire of the TILT servo (output)
const int panServoPin = 36;  // signal wire of the PAN servo (output); swings the sensor left/right

// ----- Option 2: tuning values (the defaults should be a fine starting point) -----
// Turning
const int TURN_DUTY = 30;            // in-place turns must use a 30% duty cycle (handout step 7)
const int CENTER_TOLERANCE_DEG = 10; // target counts as "centered" within 90 +/- this (handout step 6)
// Scanning
const int SCAN_STEP_DEG = 5;          // how far the pan servo moves between readings (deg); smaller = finer but slower
const int NARROW_HALF_WIDTH_DEG = 30; // after the first full sweep, only scan 90 +/- this (faster re-measuring)
const int MAX_SCAN_POINTS = 181;      // size of the array that holds one sweep (enough for 1-degree steps over 0-180)
const int MIN_VALID_CM = 1;           // readings below this are ignored (0 means "no echo")
const int TARGET_EDGE_TOL_CM = 5;     // angles whose reading is within this of the closest one are treated as the same target
const int PING_GAP_MS = 25;           // pause between ultrasonic pings so an old echo is not mistaken for a new one
const int SERVO_PULSES_STEP = 3;      // servowrite() pulses for a small pan move (about 20 ms each)
const int SERVO_PULSES_LONG = 10;     // servowrite() pulses for a large pan move
// Moving
const float STEP_FRACTION = 0.5;            // each forward move is this fraction of the remaining distance (handout step 8)
const float MIN_STEP_CM = 1.0;              // smallest forward move (one encoder count is about 1 cm)
const float MAX_STEP_CM = 30.0;             // largest forward move
const int LOST_CLOSE_CM = 8;                // if the target disappears after being this close, it is too close to read: treat as arrived
const int SETTLE_MS = 200;                  // pause after each move so the robot stops rocking before the next measurement
const unsigned long TURN_TIMEOUT_MS = 4000; // give up on a turn that never finishes (e.g. a wheel is stalled)
const unsigned long MOVE_TIMEOUT_MS = 5000; // give up on a forward move that never finishes
const float OPT2_GAIN = 0.5;                // straight-line correction gain, same idea as "gainfactor" in Option 1

bool option2Abort = false;
const int SEEK_MIN_ANGLE = 20;          // from handout
const int SEEK_MAX_ANGLE = 160;         // from handout
const int MAX_TARGET_DIST_CM = 200;     // from handout
const int STOP_DISTANCE_CM = 2;         // from handout
const float TrackWidthCm = 10.0;        // MEASURE wheel center to wheel center
const int FORWARD_DUTY = 50;            // duty that drove straight in Option 1
const bool PAN_ABOVE_90_IS_LEFT = true; // flip if the robot turns away from the target

// bools for option completion
bool opt_1 = false;
bool opt_3 = false;

// switch intitialization
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

// Functions used in option 2

void processKeyInput(char keypressed, String &aval);
long calculateTargetCounts(float wheelDiameter, int countsPerRev);
void straightDrive100cm(int baseDuty, long targetCounts);
void driveStraight(int baseDuty, float targetDistanceCm);
void servowrite(int pin, int angle, int numpulses);                                          // provided: moves a servo to an angle
int calculateDistance(int tPin, int ePin);                                                   // provided: one ultrasonic reading in cm (0 = no echo)
void Opt2_SeekTarget();                                                                      // runs the whole Option 2 sequence
bool Opt2_CheckAbort();                                                                      // true if RETURN was pressed (and stops the motors)
void Opt2_StopMotors();                                                                      // both motors off
int Opt2_DutyToPWM(float dutyPercent);                                                       // percent duty cycle -> 0-255 PWM value
long Opt2_CmToCounts(float cm);                                                              // distance in cm -> encoder counts
void Opt2_ShowPreparing();                                                                   // TFT: "PREPARING TO ACQUIRE TARGET"
void Opt2_ShowTarget(int distCm, int angleDeg);                                              // TFT: target distance and angle
int Opt2_MeasureCm();                                                                        // one filtered distance reading
bool Opt2_IsValidReading(int distCm);                                                        // is a reading a real echo inside the operating region?
bool Opt2_ScanForTarget(int minAngle, int maxAngle, int &targetDistCm, int &targetAngleDeg); // sweep and locate the target
void Opt2_RotateInPlace(float degrees, bool turnLeft);                                       // spin the robot in place
void Opt2_TurnTowardTarget(int targetAngleDeg);                                              // decide which way to spin, then spin
void Opt2_DriveForward(float distanceCm);                                                    // drive straight for a distance
float Opt2_ForwardIncrementCm(int distCm);                                                   // how far to drive this time
void Opt2_WaitForReturn();                                                                   // wait for the RETURN key

//
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
    }
}
//

//
//  Option3 code
//
float radarRadius()
{
    return ((xScreen - mScreen) / 2.0) - 2;
}

//--OPTION 3-- function to draw circles and gridlines
void drawGrid()
{
    float xC = xScreen / 2.0;
    float yC = yScreen;
    float rMax = radarRadius();
    oLCD.drawCircle(xC, yC, rMax, RCB_WHITE);
    oLCD.drawCircle(xC, yC, rMax / 2.0, RCB_WHITE);
    // draw gridlines from the center at 30 degree increments
    int gridlineAngles[5] = {30, 60, 90, 120, 150};
    for (int i = 0; i < 5; i++)
    {
        float rad = gridlineAngles[i] * PI / 180.0;
        int x1 = xC + rMax * cos(rad); // use trig functions to get gridline coordinates
        int y1 = yC - rMax * sin(rad);
        oLCD.drawLine(xC, yC, x1, y1, RCB_WHITE);
    }
}

//--OPTION 3-- function to clear screen and draw radar text
void drawRadar(bool initdisplay)
{
    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    oLCD.print("ANGLE", 15, 5);
    oLCD.print("DEGREES", 100, 5);
    oLCD.print("DISTANCE", 15, 20);
    oLCD.print("CM", 100, 20);
    // empty black boxes for the readings
    clearText(60, 5, 35, RCB_BLACK);
    clearText(80, 20, 18, RCB_BLACK);
    drawGrid();
}

//--OPTION 3-- function to update the angle and distance readings
void updateReadings(int angle, int distance)
{
    clearText(60, 5, 35, RCB_BLACK);
    clearText(80, 20, 18, RCB_BLACK);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    oLCD.printNumI(angle, 65, 5, 3, ' ');
    oLCD.printNumI(distance, 81, 20);
}

//--OPTION 3-- function to measure distance from 0-120cm
int readDistance()
{
    int d = calculateDistance(trigPin, echoPin);
    if (d <= 0 || d > maxDist)
    {
        d = maxDist;
    }
    return d;
}

//--OPTION 3-- function to draw colored rays, blue for open space, red for obstacles
void drawRay(int angle, int dist)
{
    float xC = xScreen / 2.0;
    float yC = yScreen;
    float rMax = radarRadius() - 1;
    float rad = angle * PI / 180.0;
    float rMeas = (dist / (float)maxDist) * rMax; // make maxDist a float so it's not integer division
    int xBlue = xC + rMeas * cos(rad);            // same logic as drawing the gridlines
    int yBlue = yC - rMeas * sin(rad);
    int xRed = xC + rMax * cos(rad);
    int yRed = yC - rMax * sin(rad);
    oLCD.drawLine(xC, yC, xBlue, yBlue, RCB_BLUE);
    if (dist < maxDist)
    {
        oLCD.drawLine(xBlue, yBlue, xRed, yRed, RCB_RED);
    }
}

//--OPTION 3-- function to group readings into targets and keep the closest one
void findTarget(int dist, int angle, int &minDist, int &sumAngle, int &sumDist, int &count, int &theta, int &d)
{
    if (dist < maxDist)
    {
        // part of a target, add to current group
        sumAngle += angle;
        sumDist += dist;
        count++;
    }
    else if (count > 0)
    {
        // ignore groups too small to be a real target
        if (count >= minGroup)
        {
            int avgDist = sumDist / count;
            if (avgDist < minDist)
            {
                minDist = avgDist;
                theta = sumAngle / count;
                d = avgDist;
            }
        }
        sumAngle = 0;
        sumDist = 0;
        count = 0;
    }
}

//--OPTION 3-- function to run one sweep and return the closest target
void sweep(int startAngle, int endAngle, int step, int &theta, int &d)
{
    int minDist = 999;
    int sumAngle = 0;
    int sumDist = 0;
    int count = 0;
    theta = 0;
    d = 999;
    // get to the start angle first
    servowrite(panServoPin, startAngle, 10);
    // number of steps between the start and end angles
    int numSteps = (endAngle - startAngle) / step;
    for (int n = 0; n <= numSteps; n++)
    {
        int i = startAngle + n * step; // current angle
        servowrite(panServoPin, i, 3);
        int dist = readDistance();
        findTarget(dist, i, minDist, sumAngle, sumDist, count, theta, d);
        updateReadings(i, dist);
        drawRay(i, dist);
        drawGrid();
    }
    servowrite(panServoPin, endAngle, 10);
    findTarget(maxDist, endAngle, minDist, sumAngle, sumDist, count, theta, d);
}

//--OPTION 3-- function to display target result
void displayTarget(bool found, int angle, int dist)
{
    float xC = xScreen / 2.0;
    float yC = yScreen;
    int boxWidth = 100;
    int boxHeight = 55;
    int x1 = xC - (boxWidth / 2);
    int y1 = (yC / 2) - 15;
    DrawBox(x1, y1, boxWidth, boxHeight, RCB_BLACK);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);
    if (found)
    {
        oLCD.print("TARGET", CENTER, y1 + 12);
        oLCD.printNumI(angle, x1 + 6, y1 + 27, 3, ' ');
        oLCD.print(" DEGREES", x1 + 30, y1 + 27);
        oLCD.printNumI(dist, x1 + 26, y1 + 42, 3, ' ');
        oLCD.print(" CM", x1 + 50, y1 + 42);
    }
    else
    {
        oLCD.print("TARGET NOT", CENTER, y1 + 20);
        oLCD.print("FOUND", CENTER, y1 + 35);
    }
}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    // ShowDisplay() calls this with optionstate 'A' and keypressed ' ' when option 2 is first chosen.
    // Opt2_SeekTarget() runs the whole seek and returns to the main menu when it is finished,
    // so any other call (other optionstate/key combinations) is ignored.
    if (optionstate == 'A' && keypressed == ' ')
    {
        Opt2_SeekTarget();
    }
}

//--OPTION 3 MAIN--
void Option3(char optionstate, char keypressed, bool initdisplay)
{
    // Only run when the option is first selected; later key presses on this screen do nothing.
    // (Pins are already configured in setup().)
    if (optionstate != 'A')
    {
        return; // makes so it does not keep looping
    }
    digitalWrite(trigPin, LOW);
    servowrite(tiltServoPin, 15, 10); // 15 degrees = platform level, same as setup()
    servowrite(panServoPin, 90, 10);
    int thetaf, df; // target angle and distance for forward sweep
    int thetar, dr; // target angle and distance for reverse sweep
    drawRadar(initdisplay);
    sweep(15, 165, 2, thetaf, df);
    drawRadar(false); // clear before the reverse sweep
    sweep(165, 15, -1, thetar, dr);
    // take values from both sweeps
    if (df < maxDist && dr < maxDist)
    {
        targetAngle = (thetaf + thetar) / 2;
        targetDist = (df + dr) / 2;
        option4Enabled = true;
        displayTarget(true, targetAngle, targetDist);
    }
    else
    {
        option4Enabled = false;
        displayTarget(false, 0, 0);
    }
    servowrite(panServoPin, 90, 10); // return pan servo to center
    opt_3 = true;
}
//
//  Option4 code
//

void DisplaytargetCords(int angle, int distance)
{
    oLCD.LCDInitialize(LANDSCAPE, false);
    oLCD.print("TARGET DIR", 10, 10);
    oLCD.print(String(angle) + " DEGREES", 10, 24);
    oLCD.print("TARGET DIST", 10, 38);
    oLCD.print(String(distance) + " CM", 10, 52);
    oLCD.print("4. NAVIGATE", 10, 66);
}

void Option4(char optionstate, char keypressed, bool initdisplay)
{
    pinMode(SW1, INPUT_PULLUP);
    // Only run when the option is first selected; otherwise every key press on this screen would drive again.
    if (optionstate != 'A')
    {
        return;
    }
    if (opt_1 == true && opt_3 == true)
    {

        DisplaytargetCords(targetAngle, targetDist);

        Opt2_RotateInPlace(90, true);
        delay(100);
        Opt2_DriveForward(2);
        delay(100);
        Opt2_RotateInPlace(90, false);
        Opt2_DriveForward(10);

        while (digitalRead(SW1) == HIGH)
        {
            if (oIR.GetKeyPressed() == KEY_RETURN)
                return;
            delay(10);
        }
        delay(30); // debounce: let the switch contacts settle

        double turnAmount = targetAngle - 90;

        bool turnleft = (turnAmount > 0);
        Opt2_RotateInPlace(abs(turnAmount), turnleft);
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

    // declare the sensor and servo pins (handout steps 13 and 14)
    pinMode(tiltServoPin, OUTPUT);
    pinMode(panServoPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);

    // configuring pwm channels for the left and right motoer
    ledcSetup(lpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(lenablepwm, lpwmChannel);
    ledcWrite(lpwmChannel, 0);
    ledcSetup(rpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(renablepwm, rpwmChannel);
    ledcWrite(rpwmChannel, 0);

    servowrite(tiltServoPin, 15, 10); // tilt servo to 15 degrees (platform level with the ground)
    servowrite(panServoPin, 90, 10);  // pan servo to 90 degrees (pointing straight ahead)

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
                // Option 2 cannot be selected until Option 1 has been completed.
                // Until then, pressing 2 simply does nothing.
                if (opt_1)
                {
                    ShowDisplay(SC_SUB2, 'A', ' ', false);
                }
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
        // Show option 2 in red (disabled) until Option 1 has been completed (project handout step 8).
        // RCB_RED is the project's red; the background stays purple like the rest of the menu.
        if (!opt_1)
            oLCD.setTextColor(RCB_RED, RCB_PURPLE);
        oLCD.print("2. SEEK TARGET", 10, 38);
        // Switch back to white so the lines printed after this one are normal
        oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
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

        // Option 1 counts as complete only if both wheels reached the full 100 cm (not stopped early with POWER).
        // Set here, before the menu is redrawn below, so option 2 shows as unlocked right away.
        if ((long)leftEncoderCount >= counttotarget && (long)rightEncoderCount >= counttotarget)
        {
            opt_1 = true;
        }

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
//  HOW OPTION 2 WORKS (overview)
//
//  Opt2_SeekTarget() repeats this cycle until the ultrasonic sensor reads STOP_DISTANCE_CM or less:
//    1. SCAN    - Opt2_ScanForTarget() sweeps the pan servo, takes a distance reading at every
//                 angle, and works out the target's distance and direction.
//    2. DISPLAY - Opt2_ShowTarget() puts that distance and angle on the TFT.
//    3. ACT     - if the target is not within +/-10 degrees of straight ahead, spin in place
//                 toward it (Opt2_RotateInPlace); otherwise drive a short distance forward
//                 (Opt2_DriveForward).  The moves get shorter as the robot gets closer.
//                 Then the robot stops and the cycle starts again with a fresh measurement.
//
//  The wheel encoders measure how far each move goes, but only the ultrasonic sensor decides
//  when the robot has arrived.  Pressing RETURN at any time stops everything (Opt2_CheckAbort).
// ---------------------------------------------------------------------------------------

//
//  Opt2_SeekTarget(): the top-level routine called by Option2().
//  Runs the whole measure -> turn/move -> measure cycle (handout steps 3 to 12).
//
void Opt2_SeekTarget()
{
    option2Abort = false;    // clear any abort left over from a previous run
    bool arrived = false;    // becomes true when the sensor says we are close enough
    bool useFullScan = true; // the first sweep covers the whole operating region; later ones are narrower
    int lastDistCm = -1;     // distance from the previous good scan (-1 = none yet)
    int distCm = 0;          // distance to the target from the latest scan (cm)
    int angleDeg = 90;       // direction of the target from the latest scan (pan angle, 90 = straight ahead)

    // Step 3: tell the user the robot is preparing to seek
    Opt2_ShowPreparing();

    // Keep going until we arrive, or RETURN is pressed
    while (!arrived && !option2Abort)
    {
        // Step 4: sweep the pan servo + ultrasonic sensor.
        // First time: the full operating region.  Afterwards: just 90 +/- 30 degrees,
        // because after turning the target should be roughly straight ahead (saves time).
        int scanMin = useFullScan ? SEEK_MIN_ANGLE : (90 - NARROW_HALF_WIDTH_DEG);
        int scanMax = useFullScan ? SEEK_MAX_ANGLE : (90 + NARROW_HALF_WIDTH_DEG);
        bool found = Opt2_ScanForTarget(scanMin, scanMax, distCm, angleDeg);
        if (option2Abort)
            break; // RETURN was pressed during the sweep

        if (!found)
        {
            Serial.println("Option2: no target found");
            // A target closer than about 2 cm cannot return an echo, so if we were already
            // close on the last scan, "nothing found" means we have arrived.
            if (lastDistCm > 0 && lastDistCm <= LOST_CLOSE_CM)
                arrived = true;
            useFullScan = true; // otherwise widen the search and try again
            continue;           // skip the rest of this pass and go back to the top of the loop
        }
        useFullScan = false; // we know roughly where it is now, so narrow sweeps will do
        lastDistCm = distCm;

        // Steps 5 and 9: show the (updated) target distance and angle on the TFT
        Opt2_ShowTarget(distCm, angleDeg);
        // How far is the target from straight ahead (90)?  Drop the minus sign so left and right count the same.
        int offCenterDeg = angleDeg - 90;
        if (offCenterDeg < 0)
        {
            offCenterDeg = -offCenterDeg;
        }
        if (distCm <= STOP_DISTANCE_CM)
        {
            // Step 10: the ultrasonic reading (not the encoders) decides when we are done
            arrived = true;
        }
        else if (offCenterDeg > CENTER_TOLERANCE_DEG)
        {
            // Steps 6 and 7: target is more than 10 degrees off center, so turn in place toward it
            Opt2_TurnTowardTarget(angleDeg);
        }
        else
        {
            // Step 8: target is centered, so drive a limited distance that shrinks as we get closer
            Opt2_DriveForward(Opt2_ForwardIncrementCm(distCm));
        }

        if (!arrived && !option2Abort)
            delay(SETTLE_MS); // let the robot stop rocking before the next measurement (step 9)
    }

    Opt2_StopMotors(); // make sure both motors are off no matter how we got here

    if (!option2Abort)
    {
        // Step 12: we arrived, so leave the final readout on the TFT until the instructor presses RETURN
        Opt2_WaitForReturn();
    }
    // Steps 11 and 12: back to the main menu (after either an abort or the instructor's RETURN)
    ShowDisplay(SC_MAIN, ' ', ' ');
}
//
//  Opt2_CheckAbort(): poll the IR remote.  If RETURN was pressed, stop both motors and set the
//  abort flag (handout step 11).  Returns true if Option 2 should stop.
//  Called constantly inside the scan and the movement loops.
//
bool Opt2_CheckAbort()
{
    if (oIR.GetKeyPressed() == KEY_RETURN)
    {
        Opt2_StopMotors();
        option2Abort = true;
    }
    return option2Abort;
}
//
//  Opt2_WaitForReturn(): do nothing until RETURN is pressed.
//  Used after arriving, so the instructor can measure and read the TFT first.
//
void Opt2_WaitForReturn()
{
    while (oIR.GetKeyPressed() != KEY_RETURN)
    {
        delay(50); // small pause so we are not polling the remote flat out
    }
}
//
//  Opt2_StopMotors(): a duty cycle of 0 turns the motor ENABLE pins off, which brakes both motors.
//
void Opt2_StopMotors()
{
    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);
}
//
//  Opt2_DutyToPWM(): convert a duty cycle in percent (0-100) to the 8-bit value (0-255)
//  that ledcWrite() expects.  Same math Option 1 uses.
//
int Opt2_DutyToPWM(float dutyPercent)
{
    return (int)(dutyPercent * maxDutyCycle / 100.0);
}
//
//  Opt2_CmToCounts(): convert a travel distance in cm to encoder counts.
//  One wheel turn = circumference cm = CountsPerRev counts, same math as Option 1.
//  The +0.5 rounds to the nearest whole count instead of always rounding down.
//
long Opt2_CmToCounts(float cm)
{
    float wheelCircumference = 3.14159 * WheelDiameter;
    return (long)((cm / wheelCircumference) * CountsPerRev + 0.5);
}
//
//  Opt2_ShowPreparing(): TFT screen for step 3.  Four centered lines about the middle of the screen.
//
void Opt2_ShowPreparing()
{
    oLCD.LCDInitialize(PORTRAIT, false);        // clear the screen to purple (false = do not re-initialize the display hardware)
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);   // white text on purple
    int yMid = oLCD.height() / 2;               // vertical middle of the screen, so the text block is centered
    oLCD.print("PREPARING", CENTER, yMid - 24); // CENTER centers each line horizontally
    oLCD.print("TO", CENTER, yMid - 8);
    oLCD.print("ACQUIRE", CENTER, yMid + 8);
    oLCD.print("TARGET", CENTER, yMid + 24);
}
//
//  Opt2_ShowTarget(): TFT screen for steps 5 and 9.  Shows the target distance and angle.
//  The whole screen is redrawn each time so leftover digits from a longer number never remain.
//
void Opt2_ShowTarget(int distCm, int angleDeg)
{
    oLCD.LCDInitialize(PORTRAIT, false);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    int yMid = oLCD.height() / 2;
    oLCD.print("TARGET DIST", CENTER, yMid - 24);
    oLCD.print(String(distCm) + " CM", CENTER, yMid - 8); // e.g. "98 CM"
    oLCD.print("TARGET ANGLE", CENTER, yMid + 8);
    oLCD.print(String(angleDeg) + " DEGREES", CENTER, yMid + 24); // e.g. "47 DEGREES"
}
//
//  Opt2_IsValidReading(): a reading is usable if it is a real echo (not 0 = "no echo")
//  and it is inside the operating region.
//
bool Opt2_IsValidReading(int distCm)
{
    return (distCm >= MIN_VALID_CM && distCm <= MAX_TARGET_DIST_CM);
}
//
//  Opt2_MeasureCm(): one filtered distance reading.
//  The sensor is noisy and sometimes misses, so take three pings and return the MEDIAN
//  (the middle value).  One bad ping then cannot spoil the result.
//  calculateDistance() is the provided function; it returns 0 when no echo comes back.
//
int Opt2_MeasureCm()
{
    int a = calculateDistance(trigPin, echoPin);
    delay(PING_GAP_MS); // wait so the echo of this ping has died away before the next one
    int b = calculateDistance(trigPin, echoPin);
    delay(PING_GAP_MS);
    int c = calculateDistance(trigPin, echoPin);
    delay(PING_GAP_MS);

    // Sort a, b, c into ascending order by swapping neighbors; afterwards b is the middle value.
    int t; // temporary holder used for swapping
    if (a > b)
    {
        t = a;
        a = b;
        b = t;
    }
    if (b > c)
    {
        t = b;
        b = c;
        c = t;
    }
    if (a > b)
    {
        t = a;
        a = b;
        b = t;
    }
    return b;
}
//
//  Opt2_ScanForTarget(): sweep the pan servo from minAngle to maxAngle taking a distance at each
//  step, then decide where the target is.
//    - targetDistCm / targetAngleDeg are passed by reference (&), so the function fills them in.
//    - The angle is the PAN angle: 90 = straight ahead, as the handout requires.
//    - Returns true if a target was found, false if nothing valid was seen (or RETURN was pressed).
//
bool Opt2_ScanForTarget(int minAngle, int maxAngle, int &targetDistCm, int &targetAngleDeg)
{
    // How many readings will this sweep take?
    int numPoints = (maxAngle - minAngle) / SCAN_STEP_DEG + 1;
    if (numPoints > MAX_SCAN_POINTS)
        numPoints = MAX_SCAN_POINTS; // never run past the end of the array
    int scanDist[MAX_SCAN_POINTS];   // scanDist[i] = distance seen at angle (minAngle + i * SCAN_STEP_DEG)

    // ---- Part 1: sweep and record a distance at every angle ----
    for (int i = 0; i < numPoints; i++)
    {
        int angle = minAngle + i * SCAN_STEP_DEG;
        // The first move of a sweep may be a long swing, so give it more servo pulses
        servowrite(panServoPin, angle, (i == 0) ? SERVO_PULSES_LONG : SERVO_PULSES_STEP);
        scanDist[i] = Opt2_MeasureCm();
        if (Opt2_CheckAbort())
            return false; // RETURN pressed: give up on this sweep
    }
    // Point the sensor straight ahead again for the drive that follows
    servowrite(panServoPin, 90, SERVO_PULSES_LONG);

    // ---- Part 2: the target is the closest valid reading in the sweep ----
    int bestIdx = -1;     // index of the closest reading (-1 = none found yet)
    int bestDist = 10000; // closest distance so far (start absurdly large)
    for (int i = 0; i < numPoints; i++)
    {
        if (Opt2_IsValidReading(scanDist[i]) && scanDist[i] < bestDist)
        {
            bestDist = scanDist[i];
            bestIdx = i;
        }
    }
    if (bestIdx < 0)
        return false; // every reading was "no echo" or outside the operating region

    // ---- Part 3: find the target's direction ----
    // The sensor beam is about 15 degrees wide, so a target is seen at several neighboring angles.
    // Walk outward from the closest reading while neighbors are still valid and about the same
    // distance; the middle of that run of angles is the direction of the target.
    int first = bestIdx; // first angle (index) that sees the target
    int last = bestIdx;  // last angle (index) that sees the target
    while (first > 0 && Opt2_IsValidReading(scanDist[first - 1]) && (scanDist[first - 1] - bestDist) <= TARGET_EDGE_TOL_CM)
        first--;
    while (last < numPoints - 1 && Opt2_IsValidReading(scanDist[last + 1]) && (scanDist[last + 1] - bestDist) <= TARGET_EDGE_TOL_CM)
        last++;

    targetDistCm = bestDist;
    targetAngleDeg = minAngle + ((first + last) * SCAN_STEP_DEG) / 2; // middle of the run, converted back to degrees

    // Print to the Serial Monitor to help with testing
    Serial.print("Option2: target dist = ");
    Serial.print(targetDistCm);
    Serial.print(" cm, angle = ");
    Serial.println(targetAngleDeg);
    return true;
}
//
//  Opt2_RotateInPlace(): spin the robot in place by the requested number of degrees (handout step 7).
//  One wheel goes forward and the other in reverse, both at TURN_DUTY (30%).
//  The encoders measure each wheel, and each wheel is stopped on its own once it has rolled far enough.
//
void Opt2_RotateInPlace(float degrees, bool turnLeft)
{
    // Geometry: spinning in place, each wheel travels along a circle of diameter TrackWidthCm.
    // A full 360 degree spin would be pi * TrackWidthCm of wheel travel, so for a partial spin
    // take that fraction.
    float arcCm = 3.14159 * TrackWidthCm * degrees / 360.0;
    long targetCounts = Opt2_CmToCounts(arcCm); // how many encoder counts each wheel must reach
    if (targetCounts < 1)
        targetCounts = 1; // always move at least one count

    // Start counting from zero for this move
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Set the directions BEFORE enabling the motors.  HIGH = forward on each phase pin (same as Option 1).
    // Turning left: left wheel backward, right wheel forward.  Turning right: the opposite.
    digitalWrite(lphasedir, turnLeft ? LOW : HIGH);
    digitalWrite(rphasedir, turnLeft ? HIGH : LOW);

    // Turn both motors on at 30%
    int pwm = Opt2_DutyToPWM(TURN_DUTY);
    ledcWrite(lpwmChannel, pwm);
    ledcWrite(rpwmChannel, pwm);

    bool leftDone = false;            // has the left wheel reached its count?
    bool rightDone = false;           // has the right wheel reached its count?
    unsigned long startMs = millis(); // used for the timeout so a stalled wheel cannot hang the program
    while ((!leftDone || !rightDone) && (millis() - startMs) < TURN_TIMEOUT_MS)
    {
        if (Opt2_CheckAbort())
        {
            break;
        }
        // RETURN pressed: motors are already stopped
        // Stop each wheel as soon as its own encoder reaches the target count
        if (!leftDone && (long)leftEncoderCount >= targetCounts)
        {
            ledcWrite(lpwmChannel, 0);
            leftDone = true;
        }
        if (!rightDone && (long)rightEncoderCount >= targetCounts)
        {
            ledcWrite(rpwmChannel, 0);
            rightDone = true;
        }
    }
    Opt2_StopMotors();

    // Put both phase pins back to forward so the next forward drive starts correctly
    digitalWrite(lphasedir, HIGH);
    digitalWrite(rphasedir, HIGH);
}
//
//  Opt2_TurnTowardTarget(): work out which way to spin, and by how much, for a target seen at the
//  given pan angle, then spin.  A target straight ahead (90) needs no turn.
//
void Opt2_TurnTowardTarget(int targetAngleDeg)
{
    int offsetDeg = targetAngleDeg - 90; // how far from straight ahead; the sign says which side
    // If pan angles above 90 look left (PAN_ABOVE_90_IS_LEFT), a positive offset means "turn left".
    bool targetIsLeft = ((offsetDeg > 0) == PAN_ABOVE_90_IS_LEFT);
    // The size of the turn is the offset without its minus sign (the sign was already used above to pick the side)
    int turnDeg = offsetDeg;
    if (turnDeg < 0)
        turnDeg = -turnDeg;
    Opt2_RotateInPlace(turnDeg, targetIsLeft);
}
//
//  Opt2_ForwardIncrementCm(): how far to drive this time (handout step 8).
//  A fraction (STEP_FRACTION) of the distance still to go, limited to MIN/MAX_STEP_CM.
//  The moves shrink as the robot closes in, so it does not overshoot the stopping distance.
//
float Opt2_ForwardIncrementCm(int distCm)
{
    float step = STEP_FRACTION * (distCm - STOP_DISTANCE_CM);
    // Keep the step between the minimum and maximum
    if (step < MIN_STEP_CM)
        step = MIN_STEP_CM;
    if (step > MAX_STEP_CM)
        step = MAX_STEP_CM;
    return step;
}
//
//  Opt2_DriveForward(): drive straight for distanceCm, using the same idea as Option 1:
//  the left motor runs at FORWARD_DUTY and the right motor's duty cycle is adjusted from the
//  difference between the two encoder counts so the robot does not curve.
//  Each wheel is stopped on its own once its encoder reaches the target count.
//
void Opt2_DriveForward(float distanceCm)
{
    long targetCounts = Opt2_CmToCounts(distanceCm); // counts each wheel must reach
    if (targetCounts < 1)
        targetCounts = 1;

    // Start counting from zero for this move
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Both phase pins HIGH = both wheels forward
    digitalWrite(lphasedir, HIGH);
    digitalWrite(rphasedir, HIGH);

    // Start both motors at the same duty cycle
    float baseDuty = (float)FORWARD_DUTY;
    ledcWrite(lpwmChannel, Opt2_DutyToPWM(baseDuty));
    ledcWrite(rpwmChannel, Opt2_DutyToPWM(baseDuty));

    bool leftDone = false;
    bool rightDone = false;
    unsigned long startMs = millis(); // timeout protection, as in Opt2_RotateInPlace
    while ((!leftDone || !rightDone) && (millis() - startMs) < MOVE_TIMEOUT_MS)
    {
        if (Opt2_CheckAbort())
            break; // RETURN pressed: motors are already stopped

        // Stop each wheel as soon as it has covered the distance
        if (!leftDone && (long)leftEncoderCount >= targetCounts)
        {
            ledcWrite(lpwmChannel, 0);
            leftDone = true;
        }
        if (!rightDone && (long)rightEncoderCount >= targetCounts)
        {
            ledcWrite(rpwmChannel, 0);
            rightDone = true;
        }

        // Option 1 style correction, only while the right wheel is still running:
        // error > 0 means the left wheel is ahead, so speed the right wheel up (and vice versa).
        if (!rightDone)
        {
            long error = (long)leftEncoderCount - (long)rightEncoderCount;
            float rightDuty = baseDuty + (error * OPT2_GAIN);
            // Keep the right duty cycle between 20% and 80%, like Option 1
            if (rightDuty < 20.0)
                rightDuty = 20.0;
            if (rightDuty > 80.0)
                rightDuty = 80.0;
            ledcWrite(rpwmChannel, Opt2_DutyToPWM(rightDuty));
        }
    }
    Opt2_StopMotors();
}
// Here’s what’s left, in the order I’d do it.

// 1. Fill in the values in block 1 (the CONFIRM/MEASURE/TUNE lines)

// SEEK_MIN_ANGLE, SEEK_MAX_ANGLE, MAX_TARGET_DIST_CM: the code only sweeps and accepts readings inside this region, so I need your instructor’s actual numbers (check Project1Option2.docx). My 20–160° and 200 cm are placeholders. Too wide and the robot may lock onto a wall or table. Too narrow and it can miss the target.
// STOP_DISTANCE_CM: I read “within 2 cm” as the sensor reading 2 cm or less. The HC-SR04’s minimum range is about 2 cm, so readings that close are shaky. If the target disappears after the robot was already close (LOST_CLOSE_CM), the code treats that as arrived. Confirm with your instructor that this is what they mean.
// TrackWidthCm: measure from the center of one wheel to the center of the other. The code uses it to turn “degrees” into how far each wheel must roll. A wrong value isn’t fatal, because the robot re-measures after every turn and corrects. With one encoder count at about 1 cm of wheel travel, each count is a turn of roughly 8°, which is why the ±10° centering tolerance works.
// FORWARD_DUTY: set it to the duty cycle that drove your robot straight in Option 1.

// 2. Check the hardware
// The pan/tilt platform should sit level at the 90° pan position after power-up (handout step 19a). The sensor should be mounted on the top plate facing forward. If the pan servo isn’t centered at 90°, the angle shown on the TFT won’t match the robot’s actual centerline.

// 3. Unlock Option 2 on the robot
// Run Option 1 for a full 100 cm run. The lock resets every time you unplug the battery, so you’ll repeat this after each power-up. For quick tests you can temporarily set opt_1 = true in block 1, but put it back to false before submitting.

// 4. Test the directions

// Put the target to the robot’s left. If it turns away from it, flip PAN_ABOVE_90_IS_LEFT.
// Make sure the two wheels spin opposite ways during a turn and the same way in a straight move. I assumed HIGH means forward on both phase pins, as in Option 1.

// 5. Test the full sequence against the handout

// The TFT shows “PREPARING TO ACQUIRE TARGET”, then the distance and angle after each measurement.
// The robot ends within 2 cm by the sensor reading.
// RETURN mid-move stops the motors and brings back the menu.
// RETURN after arrival brings back the menu.

// 6. Tune if needed

// If it’s too slow, increase SCAN_STEP_DEG or shorten PING_GAP_MS.
// If it overshoots, lower STEP_FRACTION.
// If turns end early, the 30% turn may be too slow to finish inside TURN_TIMEOUT_MS. Watch for that, since the handout requires 30%.
//  *************************************************************************************
//  END: OPTION2 SUPPORT
//  *************************************************************************************
//  *************************************************************************************
//  START: OPTION3 SUPPORT
//  *************************************************************************************
//
//   Option 3: Function to draw a box.
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