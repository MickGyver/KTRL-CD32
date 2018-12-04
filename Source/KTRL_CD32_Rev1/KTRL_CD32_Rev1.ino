/* KTRL CD32 Rev. 1
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
 * NOTE!: This code should be used with older KTRL CD32 gamepads (PCB of those doesn't state Rev. 2x)
 *        For Rev. 2 gamepads, use the Rev. 2 code available in the same repository.
 * 
 * Changelog
 * ---------
 * 1.0.2
 * -----
 * - Button debouncing improvements
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
// Other defines
// ---------------------------------------------------------------
#define MODE_JOYSTICK     0
#define MODE_CD32         1
#define TIME_DEBOUNCE     50   // Debounce time in milliseconds
#define AUTOFIRE_SPEED    25   // Auto fire delay in milliseconds/2
#define AUTOFIRE_MAX      250  // msec/2
#define AUTOFIRE_MIN      10   // msec/2
#define AUTOFIRE_STEP     5    // msec/2
#define LONGPRESS_TIME    1000 // The time in milliseconds for a long press to register
#define BLINK_TIME_VERY_SLOW 500
#define BLINK_TIME_SLOW   250
#define BLINK_TIME_MEDIUM 150
#define BLINK_TIME_FAST   50
#define STATE_FROM        0
#define STATE_TO          1
#define BUTTON_COUNT      8    // Last button is DPAD UP and is handled a little bit differently (sent to it's own output)
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
#define BUTTON_PLAY       6
#define BUTTON_DPAD_UP    7
#define BUTTON_UNDEFINED  255
#define PIN_CLOCK_FIRE1   2
#define MASK_CLOCK_FIRE1  B00000100
#define PIN_LATCH         3
#define MASK_LATCH        B00001000
#define PIN_DATA_FIRE2    4         // Port needs to be changed in code if changed to pin on other port
#define MASK_DATA_FIRE2   B00010000
#define PIN_UP            13        // Port needs to be changed in code if changed to pin on other port
#define MASK_UP           B00100000
#define PIN_LED           11        // Port needs to be changed in code if changed to pin on other port
#define MASK_LED          B00001000 
#define PIN_FIRE_OUT      7         // Port needs to be changed in code if changed to pin on other port
#define MASK_FIRE_OUT     B10000000
#define MASK_SELECT       B00000100
#define MASK_PLAY         B00000010


// ---------------------------------------------------------------
// Declare variables
// ---------------------------------------------------------------
long millisCurrent = 0;              // The current time
long millisAutoFire;                 // Timestamp used for autofire       
byte autoFireValue = 1;              // Active low
byte play = 0;                       // Active low but inverted (play button status)
byte playPrev = 0;                   // Previous play button status
byte selectPrev = 0;                 // Previous select button status
int  selectMapFrom = -1;             // For mapping buttons (saves index of first button pressed) 
byte selectState = STATE_FROM;       // State when mapping a button
byte inputRegister[] = {0, 0};       // Holds port C and B values
bool eepromAccessed = false;         // Keep track if a preset has been saved or loaded (select needs to be released and pressed for every access)
int  i;                              // Used in for loops (in main procedure)
byte autoFireSpeed = AUTOFIRE_SPEED; // The default autofire speed

// Arrays for the buttons
byte inputRegisters[] = {INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_C, INPUT_REG_B, INPUT_REG_B, INPUT_REG_C};
byte inputBits[] = {2, 3, 1, 5, 0, 4, 1, 4};          // Blue, Red, Yellow, Green, Right trigger, Left Trigger, Play, Up
long inputTimePressed[] = {-1, -1, -1, -1, -1, -1, -1, -1};     // Timestamp for short button presses (select mode only)
long inputTimePressedHold[] = {-1, -1, -1, -1, -1, -1, -1, -1}; // Timestamp for long button presses (select mode only)
long inputTimeReleased[] = {-1, -1, -1 , -1, -1, -1, -1, -1};   // Timestamp for button release (select mode only)
long inputTimeSpanPressed[] = {-1, -1, -1 , -1, -1, -1, -1, -1};// The amount of time the button was pressed (readable after release)
byte inputValues[] = {1, 1, 1, 1, 1, 1, 1, 1};        // Active low
byte inputValuesPrev[] = {1, 1, 1, 1, 1, 1, 1, 1};    // Active low
byte outputValues[] = {1, 1, 1, 1, 1, 1, 1, 1};       // Active low
byte outputMap[] = {0, 1, 2, 3, 4, 5, 6, 7};          // For mapping buttons
byte autoFire[] = {0, 0, 0, 0, 0, 0, 0, 0};           // Autofire flags for all buttons (DPAD up and PLAY can't have autofire, they are used for forcing CD32 or joystick mode, and resetting the gamepad)

// Volatile variables (safe to use in interrupts)
volatile byte data[] = {1,1,1,1,1,1,1,1,0};           // Blue, Red, Yellow, Green, Right trigger, Left Trigger, Play, 1, 0
volatile byte dataBit = 1;                            // The data bit index, increased at every clock pulse (and reset to zero after inputs have been read)
volatile bool shifting = false;                       // True when shifting out buttons
volatile bool select = false;                         // Active low but inverted (select button status)

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
  // Set D9-D10, D12 and A0-A5 as inputs (with internal pull-ups enabled)
  for(i=9; i<=10; i++)
    pinMode(i, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  for(i=14; i<=19; i++)
    pinMode(i, INPUT_PULLUP);
  
  // Set unused pins as inputs (with internal pull-ups)
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  for(i=5; i<=6; i++)
    pinMode(i, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP); 

  // Set fire1 button output to input (output in joystick mode)
  DDRD  &= ~(MASK_FIRE_OUT);
  PORTD &= ~(MASK_FIRE_OUT); // Disable internal pull-up to set floating (set pin low)

  // Set clock pin as input (with external pull-up, important)
  pinMode(PIN_CLOCK_FIRE1, INPUT);

  // Set latch pin as input (with external pull-up, important)
  pinMode(PIN_LATCH, INPUT);

  // Set data pin as output and set it high
  pinMode(PIN_DATA_FIRE2, OUTPUT);
  digitalWrite(PIN_DATA_FIRE2, HIGH);

  // Set up pin as input with high impedance (floating, pull-up resistor disabled)
  DDRB  &= ~(MASK_UP);
  PORTB &= ~(MASK_UP); // Set pin low to definately disable pull-up resistor (probably not needed as the digital pins should have the pull-ups disabled by default)

  // Set LED pin as output
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Interrupt 0 for clock (pin 2)
  EICRA &= ~(bit(ISC00) | bit (ISC01)); // Clear existing flags of interrupt 0 
  EICRA |= bit (ISC00) | bit (ISC01);   // Set wanted flags (rising level interrupt)
  EIMSK |= bit (INT0);                  // Enable INT0

  // Interrupt 1 for latch (pin 3)
  EICRA &= ~(bit(ISC10) | bit (ISC11)); // Clear existing flags of interrupt 1
  EICRA |= bit (ISC11);                 // Set wanted flags (falling level interrupt), LS125 will be invert latch signal
//EICRA |= bit (ISC10) | bit (ISC11);   // Set interrupt on rising
//EICRA |= bit (ISC10);                 // Set interrupt on rising and falling
  EIMSK |= bit (INT1);                  // Enable INT1

  // Blink led (as a confirmation that the processor is working)
  blinkLed(3, BLINK_TIME_MEDIUM);

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
    if(led) PORTB |= MASK_LED; else PORTB &= ~(MASK_LED);
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

  // Select and play buttons (note the ! to invert readings)
  // ------------------------------------------------------------------------
  play = !(inputRegister[INPUT_REG_B] & MASK_PLAY);
  select = !(inputRegister[INPUT_REG_B] & MASK_SELECT);

  // Iterate inputs
  // ------------------------------------------------------------------------
  for(i=0; i<BUTTON_COUNT; i++) 
  {
    // Read input (correct bit in correct register)
    inputValues[i] = (inputRegister[inputRegisters[i]] >> inputBits[i]) & 0x1;  //bitRead(inputRegister[inputRegisters[i]], inputBits[i]);

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
  if(!(PIND & MASK_LATCH))
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

    // Play and Select are pressed, handle saving / loading presets etc.
    // -------------------------------------------
    if(play)
    {
      // Reset released times when play is pressed
      // -----------------------------
      if(!playPrev) {
        for(i=0; i<BUTTON_COUNT; i++)
          inputTimeReleased[i] = -1;
      }
      
      // Up pressed while holding select and play, reset controller
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
        // Reset pressed time for up button (so a release and press is needed for the reset code to run again)
        inputTimePressed[BUTTON_DPAD_UP] = -1;
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
        if(i != BUTTON_PLAY) // Play button is not mappable and can't be used for autofire, so do nothing
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
  if(!(PIND & MASK_LATCH)) { 
    if(outputValues[BUTTON_BLUE]) PORTD |= MASK_DATA_FIRE2; else PORTD &= ~(MASK_DATA_FIRE2);
    if(outputValues[BUTTON_RED]) DDRD &= ~(MASK_FIRE_OUT); else DDRD |= MASK_FIRE_OUT;
    // Reset databit
    dataBit = 1;
  }

  // Set UP pin
  // -------------------------------------------
  if(outputValues[BUTTON_DPAD_UP]) DDRB &= ~(MASK_UP); else DDRB |= MASK_UP;

  // Set previous states for play and select buttons from current states
  // ------------------------------------------------------------------------
  playPrev = play;
  selectPrev = select;

}} // Two brackets due to the while loop


// ---------------------------------------------------------------
// Clock pulse ISR, shift the buttons (triggered by clock pin)
// ---------------------------------------------------------------
ISR (INT0_vect)
{
  // Set output data pin to next button value (and CD32 gamepad specific values at the end)
  // Red, Yellow, Green, Right trigger, Left Trigger, Play, 1, 0
  // Blue is first and set already before this 
  
  if(PIND & MASK_LATCH) // Handle interrupt only if shifting out buttons (latch pin high)
  {
    // Set output data pin
    if(dataBit <= 8) 
    {
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
// Write preset to EEPROM
// ----------------------------------------------------------
void WritePreset(byte presetNumber)
{
  byte btn;
  // Disable interrupts
  EnableInterrupts(false);
  // Write PRESET_EXISTS=254 to indicate that a preset exists
  EEPROM.update(presetNumber*PRESET_BYTE_COUNT, PRESET_EXISTS); // byte 0
  // Write map values
  for(btn=0; btn<BUTTON_COUNT; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+1+btn, outputMap[btn]); // bytes 1-8
  // Write autofire values
  for(btn=0; btn<BUTTON_COUNT; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+9+btn, autoFire[btn]);  // bytes 9-16
  // Write dummy zeroes for padding (future use)
  for(btn=0; btn<5; btn++)
    EEPROM.update(presetNumber*PRESET_BYTE_COUNT+17+btn, 0); // bytes 17-21
  // Write autofire speed
  EEPROM.update(presetNumber*PRESET_BYTE_COUNT+22, autoFireSpeed);
  // Enable interrupts
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
  EnableInterrupts(false);
  // Check if a preset is previously stored (first byte of preset should equal the constant PRESET_EXISTS)
  if(EEPROM.read(presetNumber*PRESET_BYTE_COUNT) == PRESET_EXISTS)
  {
    // Read map values
    for(btn=0; btn<BUTTON_COUNT; btn++) {
      value = EEPROM.read(presetNumber*PRESET_BYTE_COUNT+1+btn); // Bytes 1-8
      if(btn != BUTTON_PLAY && ((value >=0 && value < BUTTON_COUNT) || value == BUTTON_UNDEFINED)) // Check if we have a valid value and ignore play button
        outputMap[btn] = value;
    }
    // Read autofire values
    for(btn=0; btn<BUTTON_COUNT; btn++) {
      if(btn != BUTTON_PLAY) // Ignore play button
        autoFire[btn] = EEPROM.read(presetNumber*PRESET_BYTE_COUNT+9+btn); // Bytes 9-16
    }
    // Read autofire speed
    autoFireSpeed = (EEPROM.read(presetNumber*PRESET_BYTE_COUNT+22) / 5) * 5;
    // Set success
    success = true;
  }
  // Enable interrupts
  EnableInterrupts(true);
  
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



