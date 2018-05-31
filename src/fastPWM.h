/**********************************************************

    Author: Vicente Jimenéz
    Source: http://r6500.blogspot.com/2014/12/fast-pwm-on-arduino-leonardo.html

   Fast PWM on pins 6, 13 (High Speed TIMER 4)

   Do not use analogWrite to pins 6 or 13 if using
   this functions as they use the same timer.

   Those functions will conflict with the
   MSTIMER2 library.
   Uses 7 PWM frequencies between 2930Hz and 187.5kHz

   Timer 4 uses a PLL as input so that its clock frequency
   can be up to 96MHz on standard Arduino Leonardo.
   We limit imput frequency to 48MHz to generate 187.5kHz PWM
   If needed, we can double that up to 375kHz
**********************************************************/

#include <Arduino.h>

// Frequency modes for TIMER4
#define PWM187k 1   // 187500 Hz
#define PWM94k  2   //  93750 Hz
#define PWM47k  3   //  46875 Hz
#define PWM23k  4   //  23437 Hz
#define PWM12k  5   //  11719 Hz
#define PWM6k   6   //   5859 Hz
#define PWM3k   7   //   2930 Hz

// Direct PWM change variables
#define PWM6        OCR4D
#define PWM13       OCR4A

// Terminal count
#define PWM6_13_MAX OCR4C

// Configure the PWM clock
// The argument is one of the 7 previously defined modes
void pwm613configure(int mode)
{
    // TCCR4A configuration
    TCCR4A=0;

    // TCCR4B configuration
    TCCR4B=mode;

    // TCCR4C configuration
    TCCR4C=0;

    // TCCR4D configuration
    TCCR4D=0;

    // TCCR4D configuration
    TCCR4D=0;

    // PLL Configuration
    // Use 96MHz / 2 = 48MHz
    PLLFRQ=(PLLFRQ&0xCF)|0x30;
    // PLLFRQ=(PLLFRQ&0xCF)|0x10; // Will double all frequencies

    // Terminal count for Timer 4 PWM
    OCR4C=255;
}

// Set PWM to D6 (Timer4 D)
// Argument is PWM between 0 and 255
void pwmSet6(int value)
{
    OCR4D=value;   // Set PWM value
    DDRD|=1<<7;    // Set Output Mode D7
    TCCR4C|=0x09;  // Activate channel D
}

// Set PWM to D13 (Timer4 A)
// Argument is PWM between 0 and 255
void pwmSet13(int value)
{
    OCR4A=value;   // Set PWM value
    DDRC|=1<<7;    // Set Output Mode C7
    TCCR4A=0x82;  // Activate channel A
}
