#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
// pin 17: RX - connect to ODrive TX GPIO 1
// pin 16: TX - connect to ODrive RX GPIO 2
HardwareSerial& odrive_serial = Serial2;

// ODrive object
ODriveArduino odrive(odrive_serial);

void odrive_calibration()
{
  int motornum0 = 0;
  int motornum1 = 1;
  int requested_state0;
  int requested_state1;

  requested_state0 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum0, requested_state0, false /*don't wait*/)) return;
  
  requested_state1 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum1, requested_state1, false /*don't wait*/)) return;
}

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  
  odrive_calibration();
  delay(1000);
}

void loop() {
    Serial.println("Executing test position move");
    delay(2000);
    float pos_m0 = 2.0f;
    float pos_m1 = 2.0f;
    odrive.SetPosition(0, pos_m0);
    odrive.SetPosition(1, pos_m1);
}