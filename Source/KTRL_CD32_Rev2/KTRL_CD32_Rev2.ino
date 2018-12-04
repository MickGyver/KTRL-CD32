/* KTRL CD32 Rev. 2
 * by MickGyver @ DaemonBite 2018
 *
 * Copyright (C) 2018 Mikael Norrgard
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 *
 * Rev. 2 changes:
 * - PB1 moved to PD6
 * - PB2 moved to PD7
 * - PD7 moved to PD5
 * - DPAD DOWN -> PB0
 * - PB2 -> DOWN OUT
 * - PD0 -> LS125 enable pin 10 (low to enable, used to enable float/low button2 behaviour)
 * 
 * Changelog
 * ---------
 * 1.0.4
 * -----
 * - Impossible direction bug (up/down at the same time) is fixed. Up has the higher priority.
 * ----- 
 * 1.0.3
 * - Forced two button mode is saved/loaded with presets.
 * -----
 * 1.0.2 (Rev 2)
 * -----
 * - DPAD down can be also be remapped.
 * - Forced two button mode version 1 (float/low)
 * - Improved button debouncing
 * - dataBit variable reset in one more place in the loop
 * -----
 * 1.0.1
 * -----
 * - Small change with red button handling in two button mode
 * - Data (pin9) set high at boot
 * - Data bit reset when latch pin is checked and found to be low (two button mode)
 * -----
 * 1.0.0
 * -----
 * - First stable version (with autofire speed change possible)
 * 
 */

// D 0-7
// B 8-13
// C 14-19 (A0-A5)


// ---------------------------------------------------------------
// Includes
// ---------------------------------------------------------------
#include <EEPROM.h>

// ---------------------------------------------------------------
// Defines
// ---------------------------------------------------------------
#define TIME_DEBOUNCE     50   // Debounce time in milliseconds (used when configuring the gamepad)
#define AUTOFIRE_SPEED    25   // Auto fire delay in milliseconds/2
#define AUTOFIRE_MAX      250  // msec/2
#define AUTOFIRE_MIN      10   // msec/2
#define AUTOFIRE_STEP     5    // msec/2
#define LONGPRESS_TIME    750  // The time in milliseconds for a long press to register (decreased from 1000)
#define BLINK_TIME_VERY_SLOW 500
#define BLINK_TIME_SLOW   250
#define BLINK_TIME_MEDIUM 150
#define BLINK_TIME_FAST   50
#define STATE_FROM        0
#define STATE_TO          1
#define BUTTON_COUNT      9    // Last buttons are DPAD UP and DOWN. These are handled a little bit differently (sent to their own outputs)
#define PRESET_EXISTS     254  
#define PRESET_BYTE_COUNT 50   // Number of bytes to reserve for each preset (now only 17 bytes are used, but for future changes...)
#define INPUT_REG_C       0
#define INPUT_REG_B       1
#define INPUT_REG_D       2
#define BUTTON_BLUE       0
#define BUTTON_RED        1
#define BUTTON_YELLOW     2
#define BUTTON_GREEN      3
#define BUTTON_RTRIGGER   4
#define BUTTON_LTRIGGER   5
#define BUTTON_START      6
#define BUTTON_DPAD_UP    7
#define BUTTON_DPAD_DOWN  8
#define BUTTON_UNDEFINED  255
#define PIN_LATCH         PIND
#define MASK_LATCH        B00001000
#define PORT_DATA_FIRE2   PORTD        // Port needs to be changed in code if changed to pin on other port
#define MASK_DATA_FIRE2   B00010000
#define DDR_UP            DDRB         // Port needs to be changed in code if changed to pin on other port
#define MASK_UP           B00100000
#define DDR_DOWN          DDRB         // Port needs to be changed in code if changed to pin on other port
#define MASK_DOWN         B00000100
#define PORT_LED          PORTB        // Port needs to be changed in code if changed to pin on other port
#define MASK_LED          B00001000 
#define DDR_FIRE1         DDRD         // Port needs to be changed in code if changed to pin on other port
#define MASK_FIRE1        B00100000
#define PORT_TWO_BUTTON   DDRD
#define MASK_TWO_BUTTON   B00000001
#define MASK_SELECT       B10000000
#define MASK_START        B01000000

// Fire1                  7
// Fire2 / Data           4
// Clock                  2
// Latch                  3     
// Up                     13
// Down                   10
// LED                    11 


// ---------------------------------------------------------------
// Declare variables
// ---------------------------------------------------------------
long millisCurrent = 0;              // The current time (milliseconds)
long millisAutoFire;                 // Timestamp used for autofire       
byte autoFireValue = 1;              // Auto fire value, active low (will toggle between 1 and 0 according to autofire speed)
byte start = 0;                      // Start button status, active low but inverted
byte startPrev = 0;                  // Previous start button status
byte selectPrev = 0;                 // Previous select button status
int  selectMapFrom = -1;             // For mapping buttons (saves index of first button pressed) 
byte selectState = STATE_FROM;       // State when mapping a button
byte inputRegister[] = {0, 0, 0};    // Holds port C, B and D values
bool eepromAccessed = false;         // Keep track if a preset has been saved or loaded (select needs to be released and pressed for every access)
int  i;                              // Used in for loops (in main procedure)
byte autoFireSpeed = AUTOFIRE_SPEED; // The default autofire speed

// Arrays for the buttons
byte inputRegisters[] = {INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_B, INPUT_REG_D, INPUT_REG_C, INPUT_REG_B};
byte inputBits[] = {2, 3, 1, 5, 0, 4, 6, 4, 0};                     // Blue, Red, Yellow, Green, Right trigger, Left Trigger, Start, Up, Down
long inputTimePressed[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};     // Timestamp for short button presses (select mode only)
long inputTimePressedHold[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1}; // Timestamp for long button presses (select mode only)
long inputTimeReleased[] = {-1, -1, -1 , -1, -1, -1, -1, -1, -1};   // Timestamp for button release (select mode only)
long inputTimeSpanPressed[] = {-1, -1, -1 , -1, -1, -1, -1, -1, -1};// The amount of time the button was pressed (readable after release)
byte inputValues[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};                   // Active low
byte inputValuesPrev[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};               // Active low
byte outputValues[] = {1, 1, 1, 1, 1, 1, 1, 1, 1};                  // Active low
byte outputMap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};                     // For mapping buttons
byte autoFire[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};                      // Autofire flags for all buttons (DPAD up and START can't have autofire, they are used for forcing CD32 or joystick mode, and resetting the gamepad)

// Volatile variables (safe to use in interrupts)
volatile byte data[] = {1,1,1,1,1,1,1,1,0};    // Blue, Red, Yellow, Green, Right trigger, Left Trigger, Start, 1, 0
volatile byte dataBit = 1;                     // The data bit index, increased at every clock pulse (and reset to zero after inputs have been read)
volatile bool shifting = false;                // True when shifting out buttons
volatile bool select = false;                  // Active low but inverted (select button status)
volatile bool alternateTwoButtonMode = false;  // Alternate two button mode (float/low for button2)
volatile long microsLatch = 0;

// Variables for status LED
int  ledBlinkDelay = 100;
byte ledBlinkCount = 0;
byte led = 0;
long ledBlinkTime = 0;


// ---------------------------------------------------------------
// Setup routine
// ---------------------------------------------------------------
void setup() 
{
  // Set pin modes for port D
  DDRD  &= ~B11101110; // Set pins as input (PD5=Button1 out and should toggle between input/output)
  PORTD |=  B11010010; // Enable internal pull-up resistors (also set PD4=Data as HIGH)
  DDRD  |=  B00010001; // Set pins as output

  // Set pin modes for port C
  DDRC  &= ~B00111111; // Set pins as input
  PORTC |=  B00111111; // Enable internal pull-up resistors

  // Set pin modes for port B
  DDRB  &= ~B00110111; // Set pins as input (PB2 and PB5 are DOWN/UP out and should toggle between input and output)
  PORTB |=  B00010011; // Enable internal pull-up resistors
  DDRB  |=  B00001000; // Set pins as output

  // Interrupt 0 for clock (PD2, pin 2, joystick port pin 6)
  EICRA &= ~(bit(ISC00) | bit (ISC01)); // Clear existing flags of interrupt 0 
  EICRA |= bit (ISC00) | bit (ISC01);   // Set wanted flags (rising level interrupt)
  EIMSK |= bit (INT0);                  // Enable INT0

  // Interrupt 1 for latch (PD3, pin 3, joystick port pin 5)
  EICRA &= ~(bit(ISC10) | bit (ISC11)); // Clear existing flags of interrupt 1
  EICRA |= bit (ISC11);                 // Set wanted flags (falling level interrupt), LS125 will invert the latch signal
  //EICRA |= bit (ISC10) | bit (ISC11); // Set interrupt on rising
  //EICRA |= bit (ISC10);               // Set interrupt on rising and falling
  EIMSK |= bit (INT1);                  // Enable INT1

  // Perform boot checks
  PerformBootChecks();

  // Blink led (as a confirmation that the processor is working)
  // Note that Rev2 blinks two times instead of three
  blinkLed(2, BLINK_TIME_MEDIUM);

  // Reset timestamps
  millisAutoFire = millis();
}


// ---------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------
void loop() { while(1) // This will create a faster loop (jump vs. relative jump in assembly code)
{
  // Get current time
  millisCurrent = millis();

  // Blink led?
  // ------------------------------------------------------------------------
  if(ledBlinkCount > 0 && millisCurrent > (ledBlinkTime+ledBlinkDelay)) {
    led = !led;
    if(led) PORT_LED |= MASK_LED; else PORT_LED &= ~(MASK_LED);
    ledBlinkTime = millisCurrent;
    ledBlinkCount--;
  }

  // Change autofire value?
  // ------------------------------------------------------------------------
  if(millisCurrent > (millisAutoFire+autoFireSpeed*2)) {
    autoFireValue = !autoFireValue;
    millisAutoFire = millisCurrent;
  }
  
  // Read input registers   
  // ------------------------------------------------------------------------
  inputRegister[INPUT_REG_C] = PINC; // Analog pins 0 to 5
  inputRegister[INPUT_REG_B] = PINB; // Digital pins 8 to 13
  inputRegister[INPUT_REG_D] = PIND; // Digital pins 0 to 7

  // Select and start buttons (note the ! to invert readings)
  // ------------------------------------------------------------------------
  start = !(inputRegister[INPUT_REG_D] & MASK_START);
  select = !(inputRegister[INPUT_REG_D] & MASK_SELECT);

  // Iterate inputs
  // ------------------------------------------------------------------------
  for(i=0; i<BUTTON_COUNT; i++) 
  {
    // Read input (correct bit in correct register)
    inputValues[i] = (inputRegister[inputRegisters[i]] >> inputBits[i]) & 0x1;

    // Get timestamps only if select is held
    if(select)
    {
      // if current value is LOW and previous is HIGH (button is pressed)
      if(!inputValues[i] && inputValuesPrev[i]) {
        inputTimePressed[i] = millisCurrent;
        inputTimePressedHold[i] = millisCurrent;
      }
      // if current value is HIGH and previous is LOW (button is released)
      else if(inputValues[i] && !inputValuesPrev[i]) {
        inputTimeReleased[i] = millisCurrent;
        inputTimeSpanPressed[i] = inputTimeReleased[i]-inputTimePressed[i];
        inputTimePressed[i] = -1;
        inputTimePressedHold[i] = -1;
      }
    }

    // Set previous value to current
    inputValuesPrev[i] = inputValues[i];
  }

  // Reset databit?
  if(!(PIN_LATCH & MASK_LATCH))
    dataBit = 1;

  // Handle select settings
  // ------------------------------------------------------------------------
  if(select) 
  {
    // Reset variables when select is pressed
    // -------------------------------------------
    if(!selectPrev) {
      for(i=0; i<BUTTON_COUNT; i++) {
        inputTimePressed[i] = -1;
        inputTimePressedHold[i] = -1;
        inputTimeReleased[i] = -1;
      }
      eepromAccessed = false;
      selectState = STATE_FROM;
    }

    // Start and Select are pressed, handle saving / loading presets etc.
    // -------------------------------------------
    if(start)
    {
      // Reset released times when start is pressed
      // -----------------------------
      if(!startPrev) {
        for(i=0; i<BUTTON_COUNT; i++)
          inputTimeReleased[i] = -1;
      }
      
      // Up pressed while holding select and start, reset controller
      // -----------------------------
      if(inputTimePressed[BUTTON_DPAD_UP] > inputTimeReleased[BUTTON_DPAD_UP]+TIME_DEBOUNCE)
      {
        // Reset all autofire and mapping settings to default
        for(i=0; i<BUTTON_COUNT; i++) {
          autoFire[i] = 0;
          outputMap[i] = i;
        }
        // Reset autofire speed
        autoFireSpeed = AUTOFIRE_SPEED;
        // Blink the led three times slowly
        blinkLed(4, BLINK_TIME_MEDIUM);
        // Reset to normal two button mode
        ToggleAlternateTwoButtonMode(false, true, false);
        // Reset pressed time for up button (so a release and press is needed for the reset code to run again)
        inputTimePressed[BUTTON_DPAD_UP] = -1;
      }

      // Down pressed while holding select and start, toggle alternate two button mode
      // -----------------------------
      if(inputTimePressed[BUTTON_DPAD_DOWN] > inputTimeReleased[BUTTON_DPAD_DOWN]+TIME_DEBOUNCE)
      {
        ToggleAlternateTwoButtonMode(!alternateTwoButtonMode, true, true);
        // Reset pressed time for down button (so a release and press is needed for the reset code to run again)
        inputTimePressed[BUTTON_DPAD_DOWN] = -1;
      }

      // Set autofire speed with left and right triggers
      if(inputTimePressed[BUTTON_RTRIGGER] > inputTimeReleased[BUTTON_RTRIGGER]+TIME_DEBOUNCE)
      {
        if(autoFireSpeed < AUTOFIRE_MAX) {
          autoFireSpeed += AUTOFIRE_STEP;
          blinkLed(4, autoFireSpeed*2);
        }
        inputTimePressed[BUTTON_RTRIGGER] = -1;
      }
      if(inputTimePressed[BUTTON_LTRIGGER] > inputTimeReleased[BUTTON_LTRIGGER]+TIME_DEBOUNCE)
      {
        if(autoFireSpeed > AUTOFIRE_MIN) {
          autoFireSpeed -= AUTOFIRE_STEP;
          blinkLed(4, autoFireSpeed*2);
        }
        inputTimePressed[BUTTON_LTRIGGER] = -1;
      }

      // Check for button press to save or load preset
      // -----------------------------
      if(!eepromAccessed)
      {
        for(i=0; i<4; i++) // Face buttons only
        {  
          if(inputTimePressedHold[i] > inputTimeReleased[i]+TIME_DEBOUNCE)
          {
            if(millisCurrent > inputTimePressedHold[i]+LONGPRESS_TIME) // Long press, save preset
            {
              WritePreset(i);
              blinkLed(3, BLINK_TIME_SLOW);
              inputTimePressedHold[i] = -1;
              eepromAccessed = true;
            }
          }
          else if(inputTimeReleased[i] > -1 && inputTimeSpanPressed[i] > TIME_DEBOUNCE) // Button released, load preset
          {
            if(ReadPreset(i))
              blinkLed(2, BLINK_TIME_MEDIUM);
            else
              blinkLed(1, BLINK_TIME_MEDIUM);
            inputTimeReleased[i] = -1;
            eepromAccessed = true;
          }
        }
      }

      
    }
    // Check button presses to handle autofire and mapping
    // -------------------------------------------
    else
    {
      for(i=0; i<BUTTON_COUNT; i++) 
      {      
        if(i != BUTTON_START) // Start button is not mappable and can't be used for autofire, so do nothing
        {
          // Short presses for mapping
          // -----------------------------
          if(inputTimePressed[i] > inputTimeReleased[i]+TIME_DEBOUNCE)
          {
            // Get button to map (copy function from this button)
            if(selectState == STATE_FROM)
            {
              selectMapFrom = i;
              selectState = STATE_TO;   
            }
            // Get button to map to and do the actual mapping (paste function to this button)
            else
            {
              // If button is mapped to itself and it is that already, then disable the button
              if(i == selectMapFrom && outputMap[i] == selectMapFrom) {
                if( (inputTimePressed[i]-inputTimeReleased[i]) > TIME_DEBOUNCE) { // Debounce
                  // Map to BUTTON_UNDEFINED=255 to disable (we never have that many buttons)
                  outputMap[i] = BUTTON_UNDEFINED;
                  // Button cleared, blink led 1 time very slowly
                  blinkLed(1, BLINK_TIME_VERY_SLOW);
                  // Reset select state
                  selectState = STATE_FROM;
                }
              }
              // otherwise map the button normally
              else {
                if( i != selectMapFrom || (inputTimePressed[i]-inputTimeReleased[i]) > TIME_DEBOUNCE) { // Debounce
                  // Map button
                  outputMap[i] = selectMapFrom;
                  // Button mapped, blink led 2 times slowly
                  blinkLed(2, BLINK_TIME_SLOW);
                  // Reset select state
                  selectState = STATE_FROM;
                }
              }
              
            }
            inputTimePressed[i] = -1;
            break;
          }
    
          // Long presses for autofire toggle or force mode change
          // -----------------------------
          if(inputTimePressedHold[i] > -1)
          {
            if(millisCurrent > inputTimePressedHold[i]+LONGPRESS_TIME)
            {
              // Toggle autofire
              autoFire[i] = !autoFire[i];
      
              // Blink LED differently depending on on/off
              if(autoFire[i])
                blinkLed(6, BLINK_TIME_FAST);
              else
                blinkLed(2, BLINK_TIME_MEDIUM);
                  
              selectState = STATE_FROM;
              inputTimePressedHold[i] = -1;         
              break;
            }
          }
        }
      }
    }   
  }

  // Reset output values (active low)
  // -------------------------------------------
  for(i=0; i<BUTTON_COUNT; i++)
    outputValues[i] = 1;
  
  // Iterate outputs
  // -------------------------------------------
  for(i=0; i<BUTTON_COUNT; i++) 
  {
    // If button mapped to valid output
    if(outputMap[i] < BUTTON_COUNT)
    {
      // Set output value
      if(autoFire[i]) {
        if(autoFireValue && !inputValues[i])
          outputValues[outputMap[i]] = 0; 
      }
      else if(!inputValues[i])
        outputValues[outputMap[i]] = 0;      
    }
  }

  // Set data for shifting
  // -------------------------------------------
  for(i=0; i<7; i++)
    data[i] = outputValues[i];

  // Set button 1 and 2 only when not shifting out buttons in cd32 mode and select is not pressed
  // -------------------------------------------
  if(!alternateTwoButtonMode) {
    if(!(PIN_LATCH & MASK_LATCH)) { 
      if(outputValues[BUTTON_BLUE]) PORT_DATA_FIRE2 |= MASK_DATA_FIRE2; else PORT_DATA_FIRE2 &= ~(MASK_DATA_FIRE2);
      if(outputValues[BUTTON_RED]) DDR_FIRE1 &= ~(MASK_FIRE1); else DDR_FIRE1 |= MASK_FIRE1;
      // Reset databit
      dataBit = 1;
    }
  }
  else {
    if(outputValues[BUTTON_BLUE]) PORT_TWO_BUTTON |= MASK_TWO_BUTTON; else PORT_TWO_BUTTON &= ~(MASK_TWO_BUTTON);
    if(outputValues[BUTTON_RED]) DDR_FIRE1 &= ~(MASK_FIRE1); else DDR_FIRE1 |= MASK_FIRE1;
  }

  // Set UP and DOWN pins
  // -------------------------------------------
  if(!outputValues[BUTTON_DPAD_UP]) {
    DDR_UP |= MASK_UP;
    DDR_DOWN &= ~(MASK_DOWN);
  }
  else if(!outputValues[BUTTON_DPAD_DOWN]) {
    DDR_DOWN |= MASK_DOWN;
    DDR_UP &= ~(MASK_UP);
  } 
  else {
    DDR_UP &= ~(MASK_UP);
    DDR_DOWN &= ~(MASK_DOWN);
  }

  // Set previous states for start and select buttons from current states
  // ------------------------------------------------------------------------
  startPrev = start;
  selectPrev = select;

}} // Two brackets due to the while loop


// ---------------------------------------------------------------
// Clock pulse ISR, shift the buttons (triggered by clock pin)
// ---------------------------------------------------------------
ISR (INT0_vect)
{
  // Set output data pin to next button value (and CD32 gamepad specific values at the end)
  // Red, Yellow, Green, Right trigger, Left Trigger, Start, 1, 0
  // Blue is first and set already before this 

  if(PIN_LATCH & MASK_LATCH) // Handle interrupt only if shifting out buttons (latch pin high)
  {
    // Set output data pin
    if(dataBit <= 8) {
      if(data[dataBit]) PORTD |= MASK_DATA_FIRE2; else PORTD &= ~(MASK_DATA_FIRE2);
    }   
    // Increase dataBit index
    dataBit++;
  }
  
}


// ---------------------------------------------------------------
// Latch ISR routine (triggered by latch pin)
// ---------------------------------------------------------------
ISR (INT1_vect)
{
  // Set pin 9 to blue button status
  if(data[BUTTON_BLUE]) PORTD |= MASK_DATA_FIRE2; else PORTD &= ~(MASK_DATA_FIRE2);
  
  // Reset dataBit index
  dataBit = 1;
}


// ----------------------------------------------------------
// Perform boot checks
// ----------------------------------------------------------
void PerformBootChecks()
{
  // Read input registers   
  // ------------------------------------------------------------------------
  inputRegister[INPUT_REG_C] = PINC; // Analog pins 0 to 5
  inputRegister[INPUT_REG_B] = PINB; // Digital pins 8 to 13
  inputRegister[INPUT_REG_D] = PIND; // Digital pins 0 to 7

  // Iterate inputs
  // ------------------------------------------------------------------------
  for(i=0; i<BUTTON_COUNT; i++) 
  {
    // Read input (correct bit in correct register)
    inputValues[i] = (inputRegister[inputRegisters[i]] >> inputBits[i]) & 0x1;
  }

  // Enable alternate two button mode if blue button is pressed
  if(!inputValues[BUTTON_BLUE]) {
    ToggleAlternateTwoButtonMode(true, true, true);
  }
}


// ----------------------------------------------------------
// Toggle alternate two button mode
// ----------------------------------------------------------
void ToggleAlternateTwoButtonMode(bool enable, bool forced, bool blink)
{
  alternateTwoButtonMode = enable;

  if(alternateTwoButtonMode) {
    // Set D0 high (this will set pin 9 to high impedance state)
    PORTD |=  B00000001; 
    // Set D4 low (this pin should remain low in alternate two button mode)
    PORTD &= ~B00010000;
    // Disable interrupts
    EnableInterrupts(false);
    // Blink LED two times
    if(blink)
      blinkLed(2, BLINK_TIME_MEDIUM);
  }
  else {
    // Set D0 low (this will set "activate" pin 9)
    PORTD &= ~B00000001;
    // Set D4 high (blue button will control this value)
    PORTD |=  B00010000;
    // Enable interrupts
    EnableInterrupts(true);
    // Blink LED one time
    if(blink)
      blinkLed(1, BLINK_TIME_SLOW);
  }
  
}


// ----------------------------------------------------------
// Write preset to EEPROM
// ----------------------------------------------------------
void WritePreset(byte presetNumber)
{
  byte btn;
  // Disable interrupts
  if(!alternateTwoButtonMode)
    EnableInterrupts(false);
  // Write PRESET_EXISTS=254 to indicate that a preset exists
  EEPROM.update(presetNumber*PRESET_BYTE_COUNT, PRESET_EXISTS); // byte 0
  // Write map values
  for(btn=0; btn<BUTTON_COUNT; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+btn, outputMap[btn]); // bytes 1-9
  // Write autofire values
  for(btn=0; btn<BUTTON_COUNT; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT+btn, autoFire[btn]);  // bytes 10-18
  // Write dummy zeroes for padding (future use)
  for(btn=0; btn<4; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT*2+btn, 0); // bytes 19-22
  // Write forced two button mode
  EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT*2+4, alternateTwoButtonMode);
  // Write autofire speed
  EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT*2+5, autoFireSpeed);
  // Enable interrupts
  if(!alternateTwoButtonMode)
    EnableInterrupts(true);
}


// ----------------------------------------------------------
// Read preset from EEPROM
// ----------------------------------------------------------
bool ReadPreset(byte presetNumber)
{
  byte btn, value;
  bool success = false;
  // Disable interrupts
  if(!alternateTwoButtonMode)
    EnableInterrupts(false);
  // Check if a preset is previously stored (first byte of preset should equal the constant PRESET_EXISTS)
  if(EEPROM.read(presetNumber*PRESET_BYTE_COUNT) == PRESET_EXISTS)
  {
    // Read map values
    for(btn=0; btn<BUTTON_COUNT; btn++) {
      value = EEPROM.read(presetNumber*PRESET_BYTE_COUNT+1+btn); // Bytes 1-9
      if(btn != BUTTON_START && ((value >=0 && value < BUTTON_COUNT) || value == BUTTON_UNDEFINED)) // Check if we have a valid value and ignore start button
        outputMap[btn] = value;
    }
    // Read autofire values
    for(btn=0; btn<BUTTON_COUNT; btn++) {
      if(btn != BUTTON_START) // Ignore start button
        autoFire[btn] = EEPROM.read(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT+btn); // Bytes 10-18
    }
    // Read alternate two button mode
    alternateTwoButtonMode = EEPROM.read(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT*2+4);
    // Read autofire speed
    autoFireSpeed = (EEPROM.read(presetNumber*PRESET_BYTE_COUNT+1+BUTTON_COUNT*2+5) / 5) * 5;
    // Set success
    success = true;
  }
  // Set forced two button mode? This will also enable interrupts if needed
  ToggleAlternateTwoButtonMode(alternateTwoButtonMode, true, false);
  
  // Return success variable (will be false if first byte did not equal the constant PRESET_EXISTS)
  return success;
}


// ----------------------------------------------------------
// Blink LED
// ----------------------------------------------------------
void blinkLed(byte blinkCount, int blinkMillis)
{
  led = 0;
  ledBlinkCount = blinkCount*2;
  ledBlinkDelay = blinkMillis;
  ledBlinkTime = 0;
}


// ----------------------------------------------------------
// Enable / Disable interrupts
// ----------------------------------------------------------
void EnableInterrupts(bool enable)
{
  if(enable) {
    EIMSK |= bit(INT0) | bit(INT1);    // Enable INT0 and INT1
    dataBit = 1;
    shifting = false;
  }
  else {
    EIMSK &= ~(bit(INT0) | bit(INT1)); // Disable INT0 and INT1       
  }
}



