/*
   Teensy sketch for building a 3D gesture and tracking MIDI controller using the Flick gesture breakout board.
   Code developed by Liam Lacey.

   Flick can detect gestures, touches, and XYZ tracking.

   Connecting the Flick board to a Teensy:
   VCC -> 3.3V
   SDA -> Pin 18
   SCL - > Pin 19
   RESET -> Pin 3 (though could be any other digital pin)
   TS -> Pin 2 (though could be any other digital pin)
   GND -> GND
   LED1 -> Pin 0 (though could be any other digital pin)
   LED2 -> Pin 1 (though could be any other digital pin)
*/

#include <Wire.h>
#include <skywriter.h>

//=====================================================================
//The below set of defines are MIDI message attributes that you may want to change

//This value sets the MIDI channel that all messages are sent on
#define MIDI_CHANNEL 1

//These values set the MIDI CC numbers that each tracking dimension sends
#define MIDI_CC_X 72
#define MIDI_CC_Y 73
#define MIDI_CC_Z 74

//These values set the MIDI CC numbers that each of the on-off gestures send
const uint8_t gestureMidiCcNumbers[5] =
{
  101, //SW_GESTURE_GARBAGE: 1
  102, //SW_FLICK_WEST_EAST: 2
  103, //SW_FLICK_EAST_WEST: 3
  104, //SW_FLICK_SOUTH_NORTH: 4
  105 //SW_FLICK_NORTH_SOUTH: 5
};

//This value sets the MIDI CC number that the airwheel gesture sends
#define MIDI_CC_AIRWHEEL 7

//=====================================================================

//Uncomment these defines to see debugging in the Arduino Serial Monitor
//#define DEBUG_GESTURES 1
//#define DEBUG_XYZ 1
//#define DEBUG_TOUCH 1
//#define DEBUG_X_MIDI 1
//#define DEBUG_Y_MIDI 1
//#define DEBUG_Z_MIDI 1

//Connect the Flick LED pins to the below Teensy digital pins
#define PIN_LED_RED 0 //LED1  
#define PIN_LED_GREEN 1 //LED2 

//Connect the Flick TS and RESET pins to the below Teensy digital pins
#define PIN_FLICK_TRFR  2 //TRFR Pin of Flick
#define PIN_FLICK_RESET 3 //Reset Pin of Flick

//=====================================================================

//Variable that stores whether the device is in 'tracking' mode or 'gesture' mode
bool xyzModeEnabled = false;

//Variables that store the previous/current MIDI CC value for each tracking dimension
uint8_t prevXMidiVal = 0;
uint8_t prevYMidiVal = 0;
uint8_t prevZMidiVal = 0;

//Variable that stores the previous/current MIDI CC value for each of the on-off gestures
uint8_t prevGestureMidiVals[5] = {0};

//Variable that stores the previous/current MIDI CC value for the airwheel gesture
uint8_t prevAirwheelMidiVal = 0;

//Varible that stores the last time that the bottom panel of the Flick board was pressed
unsigned long prevPressTime = millis();

//=====================================================================
//=====================================================================
//=====================================================================
//The Teensy 'setup' function - runs once when the Teensy is booted or reset

void setup()
{
  //Start serial for debugging
  Serial.begin (115200);
  //Wait until serial is ready
  while (!Serial) {};

  //Start Skywriter object
  Skywriter.begin (PIN_FLICK_TRFR, PIN_FLICK_RESET);

  //Assign callback functions to the different types of gestures that the Flick board can detect
  Skywriter.onXYZ (handleXyzPosition);
  Skywriter.onTouch (handleTouch);
  Skywriter.onAirwheel (handleAirwheel);
  Skywriter.onGesture (handleGesture);

  //initialise the LED pins as outputs
  pinMode (PIN_LED_RED, OUTPUT);
  pinMode (PIN_LED_GREEN, OUTPUT);

  //Turn on the red LED to signify that the board is on and in 'gesture' mode
  digitalWrite (PIN_LED_RED, HIGH);

  Serial.println("3D gesture MIDI controller started!");

}

//=====================================================================
//=====================================================================
//=====================================================================
//The Teensy 'loop' function - runs continuously once the setup function has finished

void loop()
{
  //Check for new data from the Flick sensor
  Skywriter.poll();

  // Teensy MIDI Controllers should check for incoming MIDI messages,
  // even if it doesn't want to respond to them -
  // http://forum.pjrc.com/threads/24179-Teensy-3-Ableton-Analog-CC-causes-midi-crash
  while (usbMIDI.read())
  {
    // ignore/discard incoming messages
  }

}

//=====================================================================
//=====================================================================
//=====================================================================
//Callback function for handling XYZ position/tracking data from the Flick sensor

void handleXyzPosition (unsigned int x, unsigned int y, unsigned int z)
{
  //if the device is currently in 'tracking' mode
  if (xyzModeEnabled)
  {
#ifdef DEBUG_XYZ
    char buf[17];
    sprintf(buf, "%05u:%05u:%05u", x, y, z);
    Serial.println(buf);
#endif //DEBUG_XYZ

    //Convert each axis (XYZ) value (0-65535) into a MIDI CC value (0-127),
    //and send the value as a particular MIDI CC message.
    //Only send the MIDI message if it is different from the last one sent
    //for each dimension.

    //=====================================================================
    //X-axis...

    //Convert a certain range of the X axis value to the MIDI CC range of 0-127.
    //You may want to adjust these range values.
    int cc_x_val = map (x, 10000, 60000, 0, 127);

    //Constrain the MIDI CC value to a valid range
    cc_x_val = constrain (cc_x_val, 0, 127);

    //If the MIDI CC value is different from the last sent X axis MIDI CC value
    if (cc_x_val != prevXMidiVal)
    {
      //Send the MIDI CC message
      usbMIDI.sendControlChange (MIDI_CC_X, cc_x_val, MIDI_CHANNEL);

      //Store the new value
      prevXMidiVal = cc_x_val;

#ifdef DEBUG_X_MIDI
      char buf[17];
      sprintf(buf, "X MIDI val: %d", cc_x_val);
      Serial.println(buf);
#endif //DEBUG_X_MIDI

    }

    //=====================================================================
    //Y-axis...

    //Convert a certain range of the Y axis value to the MIDI CC range of 0-127.
    //You may want to adjust these range values.
    int cc_y_val = map (y, 0, 25000, 0, 127);

    //Constrain the MIDI CC value to a valid range
    cc_y_val = constrain (cc_y_val, 0, 127);

    //If the MIDI CC value is different from the last sent Y axis MIDI CC value
    if (cc_y_val != prevYMidiVal)
    {
      //Send the MIDI CC message
      usbMIDI.sendControlChange (MIDI_CC_Y, cc_y_val, MIDI_CHANNEL);

      //Store the new value
      prevYMidiVal = cc_y_val;

#ifdef DEBUG_Y_MIDI
      char buf[17];
      sprintf(buf, "Y MIDI val: %d", cc_y_val);
      Serial.println(buf);
#endif //DEBUG_Y_MIDI

    }

    //=====================================================================
    //Z-axis...

    //Convert a certain range of the Z axis value to the MIDI CC range of 0-127.
    //You may want to adjust these range values.
    int cc_z_val = map (z, 0, 65535, 0, 127);

    //Constrain the MIDI CC value to a valid range
    cc_z_val = constrain (cc_z_val, 0, 127);

    //If the MIDI CC value is different from the last sent Z axis MIDI CC value
    if (cc_z_val != prevZMidiVal)
    {
      //Send the MIDI CC message
      usbMIDI.sendControlChange (MIDI_CC_Z, cc_z_val, MIDI_CHANNEL);

      //Store the new value
      prevZMidiVal = cc_z_val;

#ifdef DEBUG_Z_MIDI
      char buf[17];
      sprintf(buf, "Z MIDI val: %d", cc_z_val);
      Serial.println(buf);
#endif //DEBUG_Z_MIDI

    }

  } //if (xyzModeEnabled)

}

//=====================================================================
//=====================================================================
//=====================================================================
//Callback function for handling touch data from the Flick sensor

void handleTouch (unsigned char type)
{
#ifdef DEBUG_TOUCH
  Serial.println("Got touch ");
  Serial.print(type, DEC);
  Serial.print('\n');
#endif //DEBUG_TOUCH

  //if touched the bottom of the board (and over a second since the last touch)
  if (type == SW_TOUCH_SOUTH && (millis() - prevPressTime > 100))
  {
    //switch the mode of the device ('gesture' mode or 'tracking' mode)
    xyzModeEnabled = !xyzModeEnabled;

    //If changed to 'gesture' mode
    if (!xyzModeEnabled)
    {
      //reset tracking dimension CC values
      usbMIDI.sendControlChange (MIDI_CC_X, 0, MIDI_CHANNEL);
      usbMIDI.sendControlChange (MIDI_CC_Y, 0, MIDI_CHANNEL);
      usbMIDI.sendControlChange (MIDI_CC_Z, 127, MIDI_CHANNEL);
    }

    //Turn on the green LED if now in 'tracking' mode, else turn the red LED on.
    digitalWrite (PIN_LED_GREEN, xyzModeEnabled);
    digitalWrite (PIN_LED_RED, !xyzModeEnabled);

    //store the press time
    prevPressTime = millis();

  } //if (type == SW_TOUCH_SOUTH)

  //If a tap on another of the other sides, send a single x/y/z CC.
  //This is useful when wanting to send a single value of a specific axis
  //for mapping the CC to a parameter in MIDI software, which is very hard
  //to do when you've got 3 simultaneous streams of CCs being sent.
  else if (type == SW_TAP_WEST)
    usbMIDI.sendControlChange (MIDI_CC_X, 0, MIDI_CHANNEL);
  else if (type == SW_TAP_NORTH)
    usbMIDI.sendControlChange (MIDI_CC_Y, 0, MIDI_CHANNEL);
  else if (type == SW_TAP_EAST)
    usbMIDI.sendControlChange (MIDI_CC_Z, 0, MIDI_CHANNEL);
}

//=====================================================================
//=====================================================================
//=====================================================================
//Callback function for handling airwheel gesture data from the Flick sensor

void handleAirwheel (int delta)
{
  //if the device is currently in 'gesture' mode
  if (!xyzModeEnabled)
  {

#ifdef DEBUG_GESTURES
    Serial.println("Got airwheel ");
    Serial.print(delta, DEC);
    Serial.print('\n');
#endif //DEBUG_GESTURES

    //Create a new airwheel MIDI CC value from the previous value + new airwheel delta value
    int cc_val = prevAirwheelMidiVal + delta;
    //Make sure the new CC value is within valid bounds
    cc_val = constrain (cc_val, 0, 127);

    //Send the MIDI CC message
    usbMIDI.sendControlChange (MIDI_CC_AIRWHEEL, cc_val, MIDI_CHANNEL);

    //Store the new value
    prevAirwheelMidiVal = cc_val;

  } //if (!xyzModeEnabled)

}

//=====================================================================
//=====================================================================
//=====================================================================
//Callback function for handling other gesture data from the Flick sensor

void handleGesture (unsigned char type)
{
  //if the device is currently in 'gesture' mode
  if (!xyzModeEnabled)
  {
#ifdef DEBUG_GESTURES
    Serial.println("Got gesture ");
    Serial.print(type, DEC);
    Serial.print('\n');
#endif //DEBUG_GESTURES

    //For every potential gesture (garbage and flick gestures)
    for (uint8_t i = 0; i < 5; i++)
    {
      //if we have got a gesture of type i+1
      if (type == i + 1)
      {
        uint8_t cc_val;

        //toggle a CC value based on the previous value
        if (prevGestureMidiVals[i] == 0)
          cc_val = 127;
        else
          cc_val = 0;

        //Send the MIDI CC message
        usbMIDI.sendControlChange (gestureMidiCcNumbers[i], cc_val, MIDI_CHANNEL);

        //Store the new value
        prevGestureMidiVals[i] = cc_val;

      } //if (type == i + 1)

    } //for (uint8_t i = 0; i < 5; i++)

  } //if (!xyzModeEnabled)

}
