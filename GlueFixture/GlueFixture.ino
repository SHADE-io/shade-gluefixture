/*****************************************************************************
* Shade LLC
* V1.1 Automated Gluing Fixture
*
* Usage Notes:
* Glue dispenser should be set to 12 PSI
*
* Dependencies:
* AccelStepper https://github.com/adafruit/AccelStepper/archive/master.zip
* Adafruit_MotorShield https://github.com/ladyada/Adafruit_Motor_Shield_V2_Library/archive/master.zip
*
* Authors:
* Vincent Po
* Steven Eisinger
*****************************************************************************/

#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/******************************* Constants **********************************/

//#define LA_DEBUG // Uncomment to get Average and target counts for the linear actuator position

#define TOP 0
#define BOTTOM 1
#define TOP_STEPPER_SPEED 20
#define BOTTOM_STEPPER_SPEED 17
#define TOP_STEPPER_STEPS 193
#define BOTTOM_STEPPER_STEPS 197

// Math for voltage calucations
#define LA_VOLTAGE_RATIO 0.36738906 // 356.0 / (613.0 + 356.0);
#define LA_SUPPLY_VOLTAGE 12
#define LA_LOWER_VOLTAGE 2.488
#define LA_UPPER_VOTAGE 11.7
#define V_PER_COUNT .004882

// Linear Actuator Positions (in mm). 0 is fully extended, ~50 is fully retracted.
#define LA_TOP_POSITION 40.2 //41.5
#define LA_BOTTOM_POSITION 43.5
#define LA_DEFAULT_POSITION 10

// Linear Actuator Position running sum
#define NUM_SAMPLES 30

// Analog Pins
const int laPositionPin = A0;

// Relay control pin
const int glueRelayPin = 4;

// Interrupt pins for the top and bottom layers. The stop button interrupt is implemented as a Pin Change Interrupt
const int topGluePin = 2;
const int bottomGluePin = 3;
const int stopButtonPin = 8;
const int diffuserGluePin = A1; // Not currently in use

/*************************** Global Variables *******************************/

// Interrupt timing
uint32_t currStopButtonTime = 0;
uint32_t prevStopButtonTime = 0;

// Interrupt Flags
volatile byte topButtonPressed = false;
volatile byte bottomButtonPressed = false;
volatile byte stopButtonPressed = false;

// Motor Declarations
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepperMotor = AFMS.getStepper(200, 2);
Adafruit_DCMotor *laMotor = AFMS.getMotor(1);

/********************************** ISRs ************************************/
// ISR's written to be extremely short or else the arduino halts. Check flags in the main arduino loop.

void glueBottom() {
    bottomButtonPressed = true;
}

void glueTop() {
    topButtonPressed = true;
}

ISR (PCINT0_vect) { // handle pin change interrupt for D8 to D13 here
    currStopButtonTime = millis();
    if (currStopButtonTime - prevStopButtonTime > 250) {
        Serial.println("STOP!");
        digitalWrite(glueRelayPin, HIGH);
        stopButtonPressed = true;
        topButtonPressed = false;
        bottomButtonPressed = false;
	prevStopButtonTime = currStopButtonTime;
    }
}

/******************************* Helper Functions ***************************/

// Clears the buttons in case they were pressed during another function
void clearButtons(void) {
    bottomButtonPressed = false;
    topButtonPressed = false;
}

// Moves the stepper motor while still being able to use the stop button. Direction = FORWARD or BACKWARD. Default BACKWARD if no valid entry.
void moveStepper(int steps, int direction) {
    if(steps < 0)
        return;

    String directionString;
    switch (direction) {
	    case FORWARD:
	        directionString = "forward";
		break;
	    case BACKWARD:
	        directionString = "backward";
		break;
	    default:
	        Serial.println("Invalid motor direction given");
	        return;
    }
    
    Serial.print("Moving stepper motor ");
    Serial.print(directionString);
    Serial.print(" by ");
    Serial.print(steps);
    Serial.println(" steps");
    for (int i = 0; i < steps && !stopButtonPressed; i += 1)
        stepperMotor->step(1, direction, MICROSTEP);
}

// Converts mm to the counts on the arduino's ADC, based on 12V supply and voltage division into the arduino.
uint16_t mmToCounts(float mm) {
    if (mm < 0 || mm > 50.8)
        return -1;

    float maxMM = 50.8;
    float targetVoltage = (mm / maxMM) * (LA_UPPER_VOTAGE - LA_LOWER_VOLTAGE) + LA_LOWER_VOLTAGE;

    return (uint16_t) ((targetVoltage * LA_VOLTAGE_RATIO) / V_PER_COUNT);
}

// Takes in a float mm value and moves the linear actuator. Valid values from 0 to 50.8 mm
void moveLA(float mm) {
    Serial.print("Moving linear actuator to ");
    Serial.println(mm);

    if (mm < 0 || mm > 50.8)
        return;

    uint16_t avgSum = 0;

    // Find the starting position
    for(int i = 0; i < NUM_SAMPLES; i++)
        avgSum += analogRead(laPositionPin);

    uint16_t avgCounts = (avgSum / NUM_SAMPLES);

    // Find the target counts
    uint16_t targetCounts = mmToCounts(mm);

    if (avgCounts > targetCounts) {
        while (avgCounts > targetCounts && !stopButtonPressed) {
            laMotor->run(FORWARD);

            avgSum = 0;
            for(int i = 0; i < NUM_SAMPLES; i++)
                avgSum += analogRead(laPositionPin);

            avgCounts = (uint16_t) (avgSum / NUM_SAMPLES);
        }
    }
    else if (avgCounts < targetCounts) {
        while (avgCounts < targetCounts && !stopButtonPressed) {
            if(abs(avgCounts - targetCounts) < 30) {
                laMotor->run(BACKWARD);
                delay(5);
                laMotor->run(RELEASE);
            }
            else laMotor->run(BACKWARD);

            avgSum = 0;
            for(int i = 0; i < NUM_SAMPLES; i++)
                avgSum += analogRead(laPositionPin);

            avgCounts = (uint16_t) (avgSum / NUM_SAMPLES);
        }
            #ifdef LA_DEBUG
            Serial.println("Average Counts:");
            Serial.println(avgCounts);
            Serial.println("Target Counts:");
            Serial.println(targetCounts);
            Serial.println();
            #endif // LA_DEBUG
    }
    laMotor->run(RELEASE);
}

// Dispenses glue for top or bottom case of the Shade device
void executeGlue(int position) {
    String positionString;
    int stepperSpeed = 0;
    int stepperSteps = 0;
    int stepperDirection = RELEASE;
    float laPosition = LA_DEFAULT_POSITION;
    switch (position) {
        case TOP:
            positionString = "top";
            stepperSpeed = TOP_STEPPER_SPEED;
            stepperSteps = TOP_STEPPER_STEPS;
            stepperDirection = BACKWARD;
            laPosition = LA_TOP_POSITION;
            break;
        case BOTTOM:
            positionString = "bottom";
            stepperSpeed = BOTTOM_STEPPER_SPEED;
            stepperSteps = BOTTOM_STEPPER_STEPS;
            stepperDirection = FORWARD;
            laPosition = LA_BOTTOM_POSITION;
            break;
        default:
            Serial.println("Invalid position given");
            return;
    }
    
    Serial.print("Starting to glue ");
    Serial.println(positionString);
    delay(200);
    
    stepperMotor->setSpeed(stepperSpeed);

    detachButtonISR();

    // Move the LA to the bottom position
    moveLA(laPosition);
    delay(1000);
    if (stopButtonPressed) {
        attachButtonISR();
        return;
    }

    // Start dispensing then immediately start turning one revolution
    digitalWrite(glueRelayPin, LOW);
    moveStepper(stepperSteps, stepperDirection);
    stepperMotor->release();
    digitalWrite(glueRelayPin, HIGH);
    delay(1000);

    moveLA(LA_DEFAULT_POSITION);

    attachButtonISR();

    bottomButtonPressed = false;

    Serial.print("Finished gluing ");
    Serial.println(positionString);
}

// Removes ISRs, called in the middle of functions.
void detachButtonISR() {
    // Detatch interrupts
    detachInterrupt(digitalPinToInterrupt(topGluePin));
    detachInterrupt(digitalPinToInterrupt(bottomGluePin));
}

// Reattaches ISRs, called after end of functions.
void attachButtonISR() {
    attachInterrupt(digitalPinToInterrupt(bottomGluePin), glueBottom, FALLING);
    attachInterrupt(digitalPinToInterrupt(topGluePin), glueTop, FALLING);
}

/********************************** Arduino *********************************/

void setup() {
    // set up Serial library at 9600 bps
    Serial.begin(9600);
    Serial.println("Setup begin");

    // Button pins
    pinMode(topGluePin, INPUT_PULLUP);
    pinMode(bottomGluePin, INPUT_PULLUP);
    pinMode(stopButtonPin, INPUT_PULLUP);
    attachButtonISR();

    // Set up the stop button, https://playground.arduino.cc/Main/PinChangeInterrupt
    *digitalPinToPCMSK(stopButtonPin) |= bit (digitalPinToPCMSKbit(stopButtonPin));
    PCIFR |= bit (digitalPinToPCICRbit(stopButtonPin));
    PCICR |= bit (digitalPinToPCICRbit(stopButtonPin));

    // Relay pins
    pinMode(glueRelayPin, OUTPUT);
    digitalWrite(glueRelayPin, HIGH);

    // Setup the motor shield, create with the default frequency 1.6KHz
    AFMS.begin();
    stepperMotor->setSpeed(30); // 10 rpm

    // Move the LA totally extended. Speed of 27 is the minimum!
    laMotor->setSpeed(80);

    //moveLA(50);
    moveLA(LA_DEFAULT_POSITION);

    Serial.println("Setup ended");
    clearButtons();
}

void loop() {
    if (bottomButtonPressed)
        executeGlue(BOTTOM);
    else if (topButtonPressed)
        executeGlue(TOP);
    else if (stopButtonPressed) {
        stopButtonPressed = false;
        moveLA(LA_DEFAULT_POSITION);
    }
    clearButtons();
}
