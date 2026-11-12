// Includes
#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>

// pins
const int rencoder = 8;      // right optocoupler
const int lencoder = 10;     // left optocoupler
const int tiltServoPin = 35; // signal wire of the TILT servo (output)
const int panServoPin = 36;  // signal wire of the PAN servo (output); swings the sensor left/right
// Motor driving pins
const int renablepwm = 14;
const int rphasedir = 13;
const int lenablepwm = 46;
const int lphasedir = 12;
// servo pin assignments
const int trigPin = 38;
const int echoPin = 37;
// pwm channels and constants
const int lpwmChannel = 1;
const int rpwmChannel = 2;
const int pwmFreq = 496;
const int pwmResolution = 8;
const int maxDutyCycle = (int)pow(2, pwmResolution) - 1;
// switch intitialization
const int SW1 = 18;
const int SW2 = 17;
// screen globals
int xScreen = 1; // Get width of TFT display in pixels
int yScreen = 1; // Get height of TFT display in pixels
int mScreen = 2; // Define a spacer to represent margins and spacing between elements.

//  Create an instance of the lcdhelper class named oLCD
lcdhelper oLCD(ILI9163_4L, 3, 2, 9, 10, 7);

//  Declare a sprite so we can display a rotated label on the TFT.
TFT_eSprite spr = TFT_eSprite(&oLCD); // Sprite object

//  Create an instance of irhelper class named oIR.
irhelper oIR;

// Globals for option 1
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
const unsigned long dtime = 2000;
volatile unsigned long lastLeftTime = 0;
volatile unsigned long lastRightTime = 0;
int baseDuty = 0;
bool opt_1 = false;
long counttotarget = 0;

// Globals for option 2
const int ScanStep = 3;                 // how far the pan servo moves between readings
const int NarrowScan2 = 30;             // after the first full sweep, only scan 90 +/- this amount of degrees
const int SameTargetDeg = 5;            // angles whose reading is within this of the closest one are treated as the same target
const float StepFraction = 0.5;         // forward move is this % of distance left
const float MinMove = 1.0;              // smallest forward move
const float MaxMove = 30.0;             // largest forward move
const int TooClose = 8;                 // if the target disappears after being this close, arrived
const unsigned long TurnTimeout = 6000; // give up on a turn that never finishes
bool option2Abort = false;
const int SeekMinAngle = 20;
const int SeekMaxAngle = 160;
const float TrackWidthCm = 11.43; // measured wheel center to wheel center

// Globals for option 3
int targetAngle = 0;
int targetDist = 0;
bool option4Enabled = false;
const int maxDist = 120;     // farthest distance that counts as a target
const int minGroup = 2;      // minimum readings that would count as a target
float WheelDiameter = 6.5;   // wheel diamter in cm
const int CountsPerRev = 40; // slots per encoder disc
bool opt_3 = false;

// ShowDisplay needs its default argument declared up front
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);

// *************************************************************************************
// BEGIN: USER FUNCTION
// *************************************************************************************
// **********************************************
//                 Option 1 code
// **********************************************
// Clear screen function
void clearText(int32_t x, int32_t y, int32_t width, uint16_t color)
{
    int32_t xStart;

    if (x == CENTER)
        xStart = (oLCD.width() - width) / 2;
    else
        xStart = x;

    oLCD.fillRect(xStart, y - 6, width, 15, color);
}
//  Method to display menus on TFT display
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
// Function for calculating the distance measured by the Ultrasonic sensor
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
// Code to display y axis label (rotated 90 degrees)
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
// Count left encoder signal
void LeftEncoderISR()
{
    leftEncoderCount++;
}
// Count right encoder signal
void RightEncoderISR()
{
    rightEncoderCount++;
}
// Helper function to render motor status formatted for Step 5g / 5h / 5i
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
// Takes in and interperets key input from IR remote
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
// Code to make robot drive straight a given distance
void driveStraight(int baseDuty, float targetDistanceCm, bool isOption1)
{
    float adjustedRightDuty = baseDuty;
    bool rerunRequested = true; // Create variable in case of rerunning in option 1
    bool stoppedEarly = false;

    while (rerunRequested)
    {
        rerunRequested = false;
        // Reset Encoder Counts
        leftEncoderCount = 0;
        rightEncoderCount = 0;

        // Calculate target encoder counts for the specified distance
        float wheelCircumference = 3.14159 * WheelDiameter;
        long targetCounts = (long)(((targetDistanceCm) / wheelCircumference) * CountsPerRev);

        // Set motor direction to forward
        digitalWrite(lphasedir, HIGH);
        digitalWrite(rphasedir, HIGH);

        float gainFactor = 0.01;

        int leftPWM = (int)(baseDuty * maxDutyCycle / 100.0);
        int rightPWM = leftPWM;

        ledcWrite(lpwmChannel, leftPWM);
        ledcWrite(rpwmChannel, rightPWM);

        long leftSnap = 0;
        long rightSnap = 0;
        long avgCounts = 0;
        long error =0;

        unsigned long lastDisplayUpdate = millis();

        while (leftEncoderCount < targetCounts)
        {
            // Check for emergency stop key ('P')
            IR_KEYS currentKey = oIR.GetKeyPressed();
            if (oIR.TranslateKey(currentKey) == 'P')
            {
                stoppedEarly = true;
                break;
            }

            if (!stoppedEarly)
            {
                opt_1 = true;
            }
            leftSnap = leftEncoderCount;
            rightSnap = rightEncoderCount;
            error = leftSnap - rightSnap;
            avgCounts = (leftSnap + rightSnap) / 2;
            adjustedRightDuty = (float)baseDuty + (error * gainFactor);

            if (adjustedRightDuty < 20.0)
            {
                adjustedRightDuty = 20.0;
            }
            else if (adjustedRightDuty > 80.0)
            {
                adjustedRightDuty = 80.0;
            }

            rightPWM = (int)(adjustedRightDuty * maxDutyCycle / 100.0);
            ledcWrite(rpwmChannel, rightPWM);

            delay(10);

            if (isOption1 == true)
            {
                // Step 5g: Update TFT display every 1.0 second
                if (millis() - lastDisplayUpdate >= 1000)
                {
                    lastDisplayUpdate = millis();
                    // Update TFT with RIGHT DUTY CYCLE, LEFT COUNT, RIGHT COUNT, ERROR
                    updateDriveDisplay(adjustedRightDuty, leftSnap, rightSnap, error);
                }
            }
        }

        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);

        if (isOption1 == true)
        {
            // Final TFT Screen Refresh
            long finalError = (long)leftEncoderCount - (long)rightEncoderCount;
            updateDriveDisplay(adjustedRightDuty, leftEncoderCount, rightEncoderCount, finalError);

            // Option 1 counts as complete only if both wheels reached the full 100 cm (not stopped early with POWER).
            // Set here, before the menu is redrawn below, so option 2 shows as unlocked right away.
            if ((long)leftEncoderCount >= targetCounts && (long)rightEncoderCount >= targetCounts)
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
}
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
            baseDuty = aval.toInt();
            driveStraight(baseDuty, 100.0, true); 
        }
    }
}
// **********************************************
//                 Option 2 code
// **********************************************
// Check if return is pressed, and returns to main menu if so
bool Opt2_CheckAbort()
{
    if (oIR.GetKeyPressed() == KEY_RETURN)
    {
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);
        option2Abort = true;
    }
    return option2Abort;
}
// Display the Preparing to acquire target screen
void Opt2_ShowPreparing()
{
    oLCD.LCDInitialize(PORTRAIT, false);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    int yMid = oLCD.height() / 2;
    oLCD.print("PREPARING", CENTER, yMid - 24);
    oLCD.print("TO", CENTER, yMid - 8);
    oLCD.print("ACQUIRE", CENTER, yMid + 8);
    oLCD.print("TARGET", CENTER, yMid + 24);
}
// Display the target distance and angle
void Opt2_ShowTarget(int distCm, int angleDeg)
{
    oLCD.LCDInitialize(PORTRAIT, false);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
    int yMid = oLCD.height() / 2;
    oLCD.print("TARGET DIST", CENTER, yMid - 24);
    oLCD.print(String(distCm) + " CM", CENTER, yMid - 8);
    oLCD.print("TARGET ANGLE", CENTER, yMid + 8);
    oLCD.print(String(angleDeg) + " DEGREES", CENTER, yMid + 24);
}
// Make sure a reading is real and usable
bool Opt2_IsValidReading(int distCm)
{
    return (distCm >= 1 && distCm <= 200);
}
// Find median value of 3 pings then calculate distance
int Opt2_MeasureCm()
{
    int a = calculateDistance(trigPin, echoPin);
    delay(25); // wait so the echo of this ping has died away before the next one
    int b = calculateDistance(trigPin, echoPin);
    delay(25);
    int c = calculateDistance(trigPin, echoPin);
    delay(25);

    // Sort a, b, c into ascending order by swapping neighbors; afterwards b is the middle value.
    int t;
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
// Entire Scan function, sweeping sensor over range in steps, and finds target; returns false for no target
bool Opt2_ScanForTarget(int minAngle, int maxAngle, int &targetDistCm, int &targetAngleDeg)
{
    // How many readings will this sweep take
    int numPoints = (maxAngle - minAngle) / ScanStep + 1;
    if (numPoints > 181)
        numPoints = 181; // never run past the end of the array
    int scanDist[181];

    // sweep and record a distance at every angle
    for (int i = 0; i < numPoints; i++)
    {
        int angle = minAngle + i * ScanStep;
        servowrite(panServoPin, angle, (i == 0) ? 10 : 3);
        scanDist[i] = Opt2_MeasureCm();
        if (Opt2_CheckAbort())
            return false;
    }
    // Point the sensor straight ahead again for the drive that follows
    servowrite(panServoPin, 90, 10);

    // Find and verify target reading
    int bestIdx = -1;
    int bestDist = 10000;
    for (int i = 0; i < numPoints; i++)
    {
        if (Opt2_IsValidReading(scanDist[i]) && scanDist[i] < bestDist)
        {
            bestDist = scanDist[i];
            bestIdx = i;
        }
    }
    if (bestIdx < 0)
    {
        return false;
    }
    // Determine target direction by finding groupings of equal distance reading, then center is direction
    int first = bestIdx; // first angle (index) that sees the target
    int last = bestIdx;  // last angle (index) that sees the target
    while (first > 0 && Opt2_IsValidReading(scanDist[first - 1]) && (scanDist[first - 1] - bestDist) <= SameTargetDeg)
        first--;
    while (last < numPoints - 1 && Opt2_IsValidReading(scanDist[last + 1]) && (scanDist[last + 1] - bestDist) <= SameTargetDeg)
        last++;

    targetDistCm = bestDist;
    targetAngleDeg = minAngle + ((first + last) * ScanStep) / 2; // middle of the target, converted back to degrees
    return true;
}
// Spin robot in place a set amount of degrees
void Opt2_RotateInPlace(float degrees, bool turnLeft)
{
    float arcCm = 3.14159 * TrackWidthCm * degrees / 360.0;
    float wheelCircumference = 3.14159 * WheelDiameter;
    long targetCounts = (long)((arcCm / wheelCircumference) * CountsPerRev + 0.5); // how many encoder counts each wheel must reach
    if (targetCounts < 1)
    {
        targetCounts = 1; // always move at least one count
    }

    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // determine which way each wheel should spin
    digitalWrite(lphasedir, turnLeft ? HIGH : LOW);
    digitalWrite(rphasedir, turnLeft ? LOW : HIGH);

    // Turn both motors on at 30%
    int pwm = (int)(30 * maxDutyCycle / 100.0);
    ledcWrite(lpwmChannel, pwm);
    ledcWrite(rpwmChannel, pwm);

    bool leftDone = false;
    bool rightDone = false;
    unsigned long startMs = millis();
    while ((!leftDone || !rightDone) && (millis() - startMs) < TurnTimeout)
    {
        if (Opt2_CheckAbort())
        {
            break;
        }
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

    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);

    // Put both phase pins back to forward so the next forward drive starts correctly
    digitalWrite(lphasedir, HIGH);
    digitalWrite(rphasedir, HIGH);
}
// Turn to face target
void Opt2_TurnTowardTarget(int targetAngleDeg)
{
    int offsetDeg = targetAngleDeg - 90;
    bool targetIsLeft = ((offsetDeg > 0) == false); // Find which way to turn
    int turnDeg = offsetDeg;                        // Make sure angle is positive once direction is determined
    if (turnDeg < 0)
        turnDeg = -turnDeg;
    Opt2_RotateInPlace(turnDeg, targetIsLeft);
}
// Decide how far to drive, based on step fraction and min/max drive distances
float Opt2_ForwardIncrementCm(int distCm)
{
    float step = StepFraction * (distCm - 2);
    // Keep the step between the minimum and maximum
    if (step < MinMove)
        step = MinMove;
    if (step > MaxMove)
        step = MaxMove;
    return step;
}
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    if (optionstate == 'A' && keypressed == ' ')
    {
        option2Abort = false;
        bool arrived = false;
        bool useFullScan = true;
        int lastDistCm = -1;
        int distCm = 0;
        int angleDeg = 90;

        // Step 3: tell the user the robot is preparing to seek
        Opt2_ShowPreparing();

        while (!arrived && !option2Abort)
        {
            // Step 4: sweep the pan servo and ultrasonic sensor.
            int scanMin = useFullScan ? SeekMinAngle : (90 - NarrowScan2);
            int scanMax = useFullScan ? SeekMaxAngle : (90 + NarrowScan2);
            bool found = Opt2_ScanForTarget(scanMin, scanMax, distCm, angleDeg);
            if (option2Abort)
                break;

            if (!found) // If nothing is found we widen the scan and try again
            {
                Serial.println("Option2: no target found");
                if (lastDistCm > 0 && lastDistCm <= TooClose)
                    arrived = true;
                useFullScan = true;
                continue;
            }
            useFullScan = false; // Now we use a smaller scan for time save
            lastDistCm = distCm;

            // Steps 5 and 9: show the target distance and angle on the TFT
            Opt2_ShowTarget(distCm, angleDeg);

            int offCenterDeg = angleDeg - 90;
            if (offCenterDeg < 0)
            {
                offCenterDeg = -offCenterDeg;
            }
            if (distCm <= 2)
            {
                // Step 10: the ultrasonic reading (not the encoders) decides when we are done
                arrived = true;
            }
            else if (offCenterDeg > 10)
            {
                // Steps 6 and 7: target is more than 10 degrees off center, so turn in place toward it
                Opt2_TurnTowardTarget(angleDeg);
            }
            else
            {
                // Step 8: target is centered, so drive a limited distance that shrinks as we get closer
                driveStraight(baseDuty, Opt2_ForwardIncrementCm(distCm), false);
            }

            if (!arrived && !option2Abort)
                delay(200);
        }
        // Stop motors
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);

        if (!option2Abort)
        {
            // Step 12: we arrived, so leave the final readout on the TFT until return is pressed
            while (oIR.GetKeyPressed() != KEY_RETURN)
            {
                delay(50);
            }
        }
        // Steps 11 and 12: back to the main menu
        ShowDisplay(SC_MAIN, ' ', ' ');
    }
}
// **********************************************
//                 Option 3 code
// **********************************************
//--OPTION 3-- function to calculate radar radius
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
//--OPTION 3__ function to draw a box on the screen
void DrawBox(int x1, int y1, int x2, int y2, uint32_t color)
{
    oLCD.fillRoundRect(x1, y1, x2, y2, 3, color);
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
    servowrite(panServoPin, startAngle, 3);
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
    servowrite(panServoPin, endAngle, 3);
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
void Option3(char optionstate, char keypressed, bool initdisplay)
{
    // Only run when the option is first selected; later key presses on this screen do nothing.
    // (Pins are already configured in setup().)
    if (optionstate != 'A')
    {
        return; // makes so it does not keep looping
    }
    digitalWrite(trigPin, LOW);
    servowrite(tiltServoPin, 15, 3); // 15 degrees = platform level, same as setup()
    servowrite(panServoPin, 90, 3);
    int thetaf, df; // target angle and distance for forward sweep
    int thetar, dr; // target angle and distance for reverse sweep
    drawRadar(initdisplay);
    sweep(15, 165, 2, thetaf, df);
    drawRadar(false); // clear before the reverse sweep
    sweep(165, 15, -2, thetar, dr);
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
// **********************************************
//                 Option 4 code
// **********************************************
// Option 4 Step 4
void DisplaytargetCords(int angle, int distance, double facing, bool step8)
{
    if (step8 == false)
    {
        oLCD.LCDInitialize(LANDSCAPE, false);
        oLCD.print("TARGET DIR", 10, 10);
        oLCD.print(String(angle) + " DEGREES", 10, 24);
        oLCD.print("TARGET DIST", 10, 38);
        oLCD.print(String(distance) + " CM", 10, 52);
        oLCD.print("4. NAVIGATE", 10, 66);
    }
    else if (step8 == true)
    {
        oLCD.LCDInitialize(LANDSCAPE, false);
        oLCD.print("TARGET DIR", 10, 10);
        oLCD.print(String(angle) + " DEGREES", 10, 24);
        oLCD.print("TARGET DIST", 10, 38);
        oLCD.print(String(distance) + " CM", 10, 52);
        oLCD.print("4. NAVIGATE", 10, 66);
        oLCD.print("CURRENT DIR", 10, 80);
        oLCD.print(String(facing) + "DEGREES", 10, 94);
    }
}
// Option 4 Step 6
void movetoSensor(int rotation, int sideDist, int forwardDist)
{
    Opt2_RotateInPlace(rotation, true);
    delay(100);
    driveStraight(30, sideDist, false);
    delay(100);
    Opt2_RotateInPlace(rotation, false);
    driveStraight(30, forwardDist, false);
}
void Option4(char optionstate, char keypressed, bool initdisplay)
{
    // set up switch one
    pinMode(SW1, INPUT_PULLUP);
    pinMode(SW2, INPUT_PULLUP);

    if (optionstate != 'A')
    {
        return;
    }
    // step 1
    if (opt_1 == true && opt_3 == true)
    {
        // calculate turning and left or right turn
        double turnAmount = targetAngle - 90;
        bool turnleft = (turnAmount > 0);

        // step 4
        DisplaytargetCords(targetAngle, targetDist, 0, false);

        // step 6
        movetoSensor(90, 2, 10);

        // step 9
        DisplaytargetCords(targetAngle, targetDist, turnAmount, true);

        // step 7
        while (digitalRead(SW1) == HIGH)
        {
            if (oIR.GetKeyPressed() == KEY_RETURN)
                return;
            delay(10);
        }
        delay(30);

        Opt2_RotateInPlace(abs(turnAmount), turnleft);

        // step 10
        while (digitalRead(SW2) == HIGH)
        {
            if (oIR.GetKeyPressed() == KEY_RETURN)
                return;
            delay(10);
        }

        driveStraight(baseDuty, float(targetDist), false);
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
    attachInterrupt(digitalPinToInterrupt(lencoder), LeftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rencoder), RightEncoderISR, CHANGE);

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
// ************************************************
//                   END PROGRAM
// ************************************************