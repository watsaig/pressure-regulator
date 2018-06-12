/*
* Pressure regulator
* Author: Craig F. Watson (craig.watson@case.edu)
* Created on 2018-05-29
* Code is licensed under MIT license.
*
*
* The microcontroller used is an Atmega32u4, running at 8MHz / 3.3V.
* This is the same as the Sparkfun Pro Micro 8MHz/3.3V, so presets for that board
* can be used.
* Do not use the presets for a 5V version of this board, such as the 5V pro micro
* or the Arduino Leonardo.
*/

#include <Arduino.h>
#include <PID_v1.h> // PID library by Brett Beauregard
#include <Wire.h>

#include "fastPWM.h"

// comment the following line for production code
//#define debugging

// Depending on the type of sensor (SPI or analog), comment/uncomment one of the two following lines
//#define spiSensor
#define analogSensor

#ifdef spiSensor
    #include <SPI.h>
    SPISettings spi_settings = SPISettings(800000, MSBFIRST, SPI_MODE0);
#endif

// Minimum and maximum pressure, in PSI (or whatever other unit)
#define minPressure 0.0
#define maxPressure 30.0

int i2cAddress = 43;

#define sensorAnalogPin A5 // PF0
#define analogSetpointPin A4 // PF1
#define analogPVPin 9 // PB5

// The regulator can be controlled either through analog voltages (by default),
// i2c, or USB.
enum ControlInterface {
    analogControl,
    i2cControl,
    usbControl
};

/// The currently-used control interface
ControlInterface controlInterface = analogControl;

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 50;
uint8_t valveMaxValue = 255;

// Function headers
//

/// Set valve openings. 0: fully closed; 255: fully open
void setValve1(uint8_t val);
void setValve2(uint8_t val);
/// Send all the useful values (setpoint, pv, etc.. over USB)
void sendSerialData();
/// Read incoming serial data and update the controller
void processSerialData();
/// Read the current pressure and update the currentPressure variable
void readPressure();
/// Update the PID constants and/or setpoint
void updateController(double kp_, double ki_, double kd_, double setpoint_);
/// Update the setpoint only
void updateController(double setpoint_);
/// Open and close valves based on PID controller output
void handleControllerOutput();
/// Read analog SP pin and update the pressure setpoint
void readAnalogSetpoint();
/// Output the current process value on the analog PV pin
void updateAnalogPV();
/// Initialize the i2c address; it can be modified by closing jumpers J8 and J9
void initI2cAddress();
/// Called when data is received via i2c
void i2cReceiveEvent(int nBytes);
/// Called when data is requested via i2c
void i2cRequestEvent();

// Timers
//

/// The time between reads of the sensor, in microseconds
unsigned long sensorReadPeriod_us = 2500;
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;

/// The time between transmission of data over serial, in milliseconds
unsigned long serialTransmissionPeriod_ms = 50;
/// The last time the data was sent
unsigned long serialLastSendTime_ms;

/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms = 5;
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

/// The time between reads of the analog pressure setpoint
unsigned long analogSetpointUpdatePeriod_ms = 100;
/// The last time the analog setpoint was updated
unsigned long analogSetpointLastUpdateTime_ms;

/// The time between updates of the process value over analog
unsigned long analogPVUpdatePeriod_ms = 100;
/// The last time the analog PV was updated
unsigned long analogPVLastUpdateTime_ms;


// PID-related variables
//

double setPoint, currentPressure;
double kp, ki, kd;
double pidMaxOutput = 1;
double pidMinOutput = -1;
double pidOutput;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);


int lastAnalogSetpoint;

// Debugging  aids
unsigned long timer1_us = 0;
unsigned long n = 0;

void setup()
{
    Serial.begin(115200);

#ifdef debugging
    // Wait for serial to attach, but only for debugging purposes
    while(!Serial);
#endif

    // Timers
    sensorLastReadTime_us = micros();
    serialLastSendTime_ms = millis();
    lastControlTime_ms = millis();
    analogSetpointLastUpdateTime_ms = millis();
    analogPVLastUpdateTime_ms = millis();

    // Setup pins
    pinMode(sensorAnalogPin, INPUT);
    pinMode(analogSetpointPin, INPUT);
    pwm613configure(PWM23k);
    pwm91011configure(PWM8k);

    // i2c communication
    initI2cAddress();
    Wire.begin(i2cAddress);
    Wire.onReceive(i2cReceiveEvent);
    Wire.onRequest(i2cRequestEvent);

    setValve1(0);
    setValve2(0);

    lastAnalogSetpoint = 0;
    setPoint = 0;

    // Setup PID controller
    // TODO: save these values to flash; load upon startup
    kp = 0.12;
    ki = 0.6;
    kd = 0;

    pid.SetOutputLimits(pidMinOutput, pidMaxOutput);
    pid.SetSampleTime(controlLoopPeriod_ms);
    pid.SetMode(AUTOMATIC);
    pid.SetTunings(kp, ki, kd);
}


void loop()
{
    if (micros() - sensorLastReadTime_us > sensorReadPeriod_us) {
        //timer1_us = micros();
        readPressure();
        sensorLastReadTime_us = micros();
        //Serial.print("ADC read time: ");
        //Serial.println(sensorLastReadTime_us - timer1_us);
    }

    if (millis() - serialLastSendTime_ms > serialTransmissionPeriod_ms) {
        sendSerialData();
        serialLastSendTime_ms = millis();
    }

    if (Serial.available()) {
        processSerialData();
        // if parameters have been updated over USB, we switch to using that as the control interface
        controlInterface = usbControl;
    }

    // This loop takes ~250-300ms
    if (millis() - lastControlTime_ms > controlLoopPeriod_ms) {
        pid.Compute();
        lastControlTime_ms = millis();
        handleControllerOutput();
    }

    if (millis() - analogPVLastUpdateTime_ms > analogPVUpdatePeriod_ms) {
        updateAnalogPV();
        analogPVLastUpdateTime_ms = millis();
    }

    if (controlInterface == analogControl && millis() - analogSetpointLastUpdateTime_ms > analogSetpointUpdatePeriod_ms) {
        readAnalogSetpoint();
        analogSetpointLastUpdateTime_ms = millis();
    }

}

void setValve1(uint8_t val)
{
    pwmSet13(val);
}

void setValve2(uint8_t val)
{
    pwmSet6(val);
}

void sendSerialData()
{
    if (Serial) {
        Serial.print(setPoint);
        Serial.print(",");
        Serial.print(currentPressure);
        Serial.print(",");
        Serial.print(kp);
        Serial.print(",");
        Serial.print(ki);
        Serial.print(",");
        Serial.print(kd);
        Serial.print(",");
        Serial.println(pidOutput);
    }
}

void processSerialData()
{
    // data transmission format: "KP, KI, KD, SETPOINT"
    float kp_ = Serial.parseFloat();
    float ki_ = Serial.parseFloat();
    float kd_ = Serial.parseFloat();
    float sp_ = Serial.parseFloat();

    updateController(kp_, ki_, kd_, sp_);
}

void updateController(double kp_, double ki_, double kd_, double setpoint_)
{
    if (setpoint_ != setPoint && setpoint_ >= minPressure && setpoint_ <= maxPressure) {
        setPoint = setpoint_;

        // If the setpoint is negative (and we are pulling a vacuum), then opening the
        // input will lower the pressure, and opening the vent will increase it. This is the opposite
        // of what happens with positive pressure, so the controller direction should be switched accordingly.
        if (setPoint < 0 && pid.GetDirection() == DIRECT)
            pid.SetControllerDirection(REVERSE);
        else if (setPoint > 0 && pid.GetDirection() == REVERSE)
            pid.SetControllerDirection(DIRECT);
    }

    if (kp_ != kp || ki_ != ki || kd_ != kd) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        pid.SetTunings(kp, ki, kd);
    }
}

void updateController(double setpoint_)
{
    updateController(kp, ki, kd, setpoint_);
}

void readPressure()
{
#ifdef analogSensor
    double val = analogRead(sensorAnalogPin);
    double max = 1023.; // max possible value of analogRead

    // transfer function for Honeywell HSC sensors is from 10% to 90% of possible values.
    currentPressure = minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max);
    // Bound output between minimum and maximum pressure
    currentPressure = max(minPressure, min(currentPressure, maxPressure));

#else
    // SPI sensor, TODO

#endif

}

void handleControllerOutput()
{
    // Positive output: inlet valve is opened; vent is closed
    if (pidOutput >= 0) {
        setValve2(valveMinValue);
        int val = valveMinValue + round(pidOutput * (valveMaxValue - valveMinValue));
        setValve1(val);
    }
    // Negative output: vent valve is opened; inlet is closed
    else {
        setValve1(valveMinValue);
        int val = valveMinValue + round(-pidOutput * (valveMaxValue - valveMinValue));
        setValve2(val);
    }
}

void readAnalogSetpoint()
{
    // to avoid changing the setpoint 100 times a second, it is only updated if it has changed substantially
    int val = analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val /= 5.;

    if (abs(val - lastAnalogSetpoint) >= 10) {
        lastAnalogSetpoint = val;
        double s = minPressure + double(val) * (maxPressure - minPressure) / 1023.;
        setPoint = s;
    }
}

void updateAnalogPV()
{
    int val = round((currentPressure - minPressure)/(maxPressure - minPressure) * 255.);
    val = max(0, min(val, 255));
    pwmSet9(val);
}

void initI2cAddress()
{
    // J8 and J9 are wired to PF4 and PF5 (A3 and A2), resp.
    pinMode(A3, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);

    if (digitalRead(A3) == LOW)
        i2cAddress += 1;
    if (digitalRead(A2) == LOW)
        i2cAddress += 2;

    Serial.print("i2c address initialized to ");
    Serial.println(i2cAddress);
}

void i2cRequestEvent()
{
    // send the current pressure, between 0 and 255, and whether the supply pressure is too low or not.
    int val = round((currentPressure - minPressure)/(maxPressure - minPressure) * 255.);
    uint8_t pv = max(0, min(val, 255));

    // Simple indication of whether the input pressure is too low. If PID output is positive, we assume
    // that the pressure is too low. Could be refined by taking into account rate of change of PV, for example.
    uint8_t supplyTooLow = 0;
    if (pidOutput > 0.2) // bit of an arbitrary threshold
        supplyTooLow = 1;
    uint8_t toSend[2] = {pv, supplyTooLow};
    Wire.write(toSend, sizeof toSend);
}

void i2cReceiveEvent(int nBytes)
{
    // In case more than one byte is received, discard all but the last one
    while (Wire.available() > 1)
        Wire.read();

    uint8_t val = Wire.read();
    setPoint = minPressure + double(val) * (maxPressure - minPressure) / 255.;

    controlInterface = i2cControl;
}
