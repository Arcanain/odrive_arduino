// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 5000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to test velocity move");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
    }

    // test velocity moves
    /*
    Serial.println("Executing test velocity move");
    Serial.println("Send the character 'u' velocity move");
    Serial.println("Send the character 'd' velocity move");
    Serial.println("Send the character 's' stop velocity move");
    */
        
    if (c == 's') {
        /*
        delay(2000);

        char p = Serial.read();
        
        float vel0 = 2.0f;
        float vel1 = 2.0f;
        
        if (p == 'u') {
            odrive.SetVelocity(0, vel0);
            odrive.SetVelocity(1, vel0);

            //odrive.SetVelocity(0, vel1, 20);
            //odrive.SetVelocity(1, vel1, 20);
        } else if (p == 'd') {
            odrive.SetVelocity(0, -vel0);
            odrive.SetVelocity(1, -vel0);

            //odrive.SetVelocity(0, -vel1, 20);
            //odrive.SetVelocity(1, -vel1, 20);
        } else if (p == 's') {
            odrive.SetVelocity(0, 0);
            odrive.SetVelocity(1, 0);
        }
        */
        
        /*
        if (mydata_remote.down == 1 && endstop == 0) {
            odrive.SetVelocity(0, -vel0, 20);
            odrive.SetVelocity(0, -vel1, 20);
            moving = 1;
        } else if (mydata_remote.up == 1 && currentPos >= farEndStop) {
            odrive.SetVelocity(0, vel0, 20);
            odrive.SetVelocity(0, vel1, 20);
            moving = 1; 
        } else {
            odrive.SetVelocity(0, 0);
            moving = 0;
        }
        */

        delay(2000);

        float vel0 = 2.0f;
        float vel1 = 2.0f;
        
        odrive.SetVelocity(0, vel0);
        odrive.SetVelocity(1, vel1);
    }

  }
}