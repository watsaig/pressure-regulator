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
#define spiSensor
//#define analogSensor

// Minimum and maximum pressure of the sensor, in PSI (or whatever other unit)
#define minPressure 0
#define maxPressure 30.0

// If the pressure goes above the maximum of the sensor, there is no way to control it.
// Keeping the max and min setpoints somewhat within the sensor's max and min
// helps prevent this issue.
// The setpoint and current presure transmitted via analog or i2c interfaces
// are both bounded by these values. E.g. if using a 30psi sensor, a value
// of 3.3v on analog or 255 on i2c corresponds to 29.5 psi, not 30.
#define minPressureSetpoint 0.98*minPressure
#define maxPressureSetpoint 0.98*maxPressure

// This regulator's base i2c address. Can be modified here or in hardware,
// by soldering J8 and/or J9 (see initI2cAddress())
int i2cAddress = 43;

// Valve minimum and maximum value (0-255). The minimum value is the value at
// which the valve starts to open.
uint8_t valveMinValue = 40;
uint8_t valveMaxValue = 255;

// Timers
//

/// The time between reads of the sensor, in microseconds
unsigned long sensorReadPeriod_us = 200;
/// The last time the sensor was read
unsigned long sensorLastReadTime_us;

/// The time between transmission of data over serial, in milliseconds
unsigned long serialTransmissionPeriod_ms = 50;
/// The last time the data was sent
unsigned long serialLastSendTime_ms;

/// The time between each computation of the control loop, in milliseconds
unsigned long controlLoopPeriod_ms = 1;
/// The last time the PID loop was run
unsigned long lastControlTime_ms;

/// The time between reads of the analog pressure setpoint
unsigned long analogSetpointUpdatePeriod_ms = 100;
/// The last time the analog setpoint was updated
unsigned long analogSetpointLastUpdateTime_ms;

/// The time between updates of the process value over analog
unsigned long analogPVUpdatePeriod_ms = 5;
/// The last time the analog PV was updated
unsigned long analogPVLastUpdateTime_ms;


// PID-related variables
//

double setPoint, currentPressure;
float kp, ki, kd;
float pidMaxOutput = 1;
float pidMinOutput = -1;
double pidOutput;

PID pid(&currentPressure, &pidOutput, &setPoint, 0, 0, 0, DIRECT);

int lastAnalogSetpoint;

// Pin assignments and SPI settings -- Do not modify
//

#define sensorAnalogPin A5 // PF0
#define analogSetpointPin A4 // PF1
#define analogPVPin 9 // PB5

#ifdef spiSensor
    #include <SPI.h>
    SPISettings spi_settings = SPISettings(800000, MSBFIRST, SPI_MODE0);
    #define slave_select_pin 7 // PE6
    #define max_counts 16384
#endif

// The regulator can be controlled either through analog voltages (by default),
// i2c, or USB.
enum ControlInterface {
    analogControl,
    i2cControl,
    usbControl
};
/// The currently-used control interface
ControlInterface controlInterface = analogControl;

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
void updateController(float kp_, float ki_, float kd_, float setpoint_);
/// Update the setpoint only
void updateController(float setpoint_);
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
    pwm613configure(PWM47k);
    pwm91011configure(PWM8k);

#ifdef spiSensor
    pinMode(slave_select_pin, OUTPUT);
    digitalWrite(slave_select_pin, HIGH);
    SPI.begin();
#endif

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
    kp = 0.4;
    ki = 0.3;
    kd = 0;

    pid.SetOutputLimits(pidMinOutput, pidMaxOutput);
    pid.SetSampleTime(controlLoopPeriod_ms);
    pid.SetMode(AUTOMATIC);
    pid.SetTunings(kp, ki, kd);
}


void loop()
{
    if (micros() - sensorLastReadTime_us > sensorReadPeriod_us) {
        readPressure();
        sensorLastReadTime_us = micros();
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

void updateController(float kp_, float ki_, float kd_, float setpoint_)
{
    if (setpoint_ != setPoint && setpoint_ >= minPressureSetpoint && setpoint_ <= maxPressureSetpoint) {
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

void updateController(float setpoint_)
{
    updateController(kp, ki, kd, setpoint_);
}

void readPressure()
{
#ifdef analogSensor
    float val = analogRead(sensorAnalogPin);
    float max = 1023.; // max possible value of analogRead

    // transfer function for Honeywell HSC sensors is from 10% to 90% of possible values.
    currentPressure = minPressure + (maxPressure - minPressure) * (val - 0.1*max)/(0.8*max);
    // Bound output between minimum and maximum pressure
    //currentPressure = max(minPressure, min(currentPressure, maxPressure));

#else // SPI sensor
    // The code below is mostly taken from https://github.com/AlexSatrapa/SSC
    // (Author: Alex Satrapa, grail@goldweb.com.au)
    float p_raw;

    uint8_t x, y, v, w, s;
    s = 0xAA; // Just a convenient value for monitoring MOSI

    SPI.beginTransaction(spi_settings);
    digitalWrite(slave_select_pin, LOW);
    x = SPI.transfer(s);
    y = SPI.transfer(s);
    v = SPI.transfer(s);
    w = SPI.transfer(s);
    digitalWrite(slave_select_pin, HIGH);
    SPI.endTransaction();

    s = x >> 6;

    if (s == 0) {
        p_raw = (((int) (x & 0x3f)) << 8) | y;
        float p = ((p_raw - 0.1 * float(max_counts)) * (maxPressure - minPressure) / (0.8 * float(max_counts))) + minPressure;
        currentPressure = p;
    }
    else {
        Serial.print("SPI error: ");
        Serial.println(s);
    }

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
    int val = analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val += analogRead(analogSetpointPin);
    val /= 5.;

    // to avoid changing the setpoint 100 times a second, it is only updated if it has changed substantially
    if (abs(val - lastAnalogSetpoint) >= 10) {
        lastAnalogSetpoint = val;
        float s = minPressureSetpoint + float(val) * (maxPressureSetpoint - minPressureSetpoint) / 1023.;
        setPoint = s;
    }
}

void updateAnalogPV()
{
    int val = round((currentPressure - minPressureSetpoint)/(maxPressureSetpoint - minPressureSetpoint) * 255.);
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
    int val = round((currentPressure - minPressureSetpoint)/(maxPressureSetpoint - minPressureSetpoint) * 255.);
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
    // The master can either send just a setpoint, or a setpoint
    // and k_p, k_i, and k_d constants. The setpoint is one byte,
    // while the PID constants are floats, i.e. 4 bytes each.

    // once we've received an instruction over i2c, we stop
    // listening to analog setpoint changes.
    controlInterface = i2cControl;

    if (nBytes == 1) {
        uint8_t sp_ = Wire.read();
        float sp_psi = minPressureSetpoint + float(sp_) * (maxPressureSetpoint - minPressureSetpoint) / 255.;
        updateController(sp_psi);
    }

    else if (nBytes == 13) {
        uint8_t sp_ = Wire.read();
        float sp_psi = minPressureSetpoint + float(sp_) * (maxPressureSetpoint - minPressureSetpoint) / 255.;

        float kp_, ki_, kd_;
        int a = Wire.readBytes((uint8_t*)&kp_, 4);
        int b = Wire.readBytes((uint8_t*)&ki_, 4);
        int c = Wire.readBytes((uint8_t*)&kd_, 4);
        if (a == 4 && b == 4 && c == 4)
            updateController(kp_, ki_, kd_, sp_psi);
        else
            updateController(sp_psi);
    }
}
