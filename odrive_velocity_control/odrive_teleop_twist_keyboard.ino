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

// golbal variable
int cnt = 0;
bool calb_flag = true;
float vel0 = 2.0f;
float vel1 = 2.0f;
        
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
}

void loop() {
  
  // loop初回時のみcalibrationを一回行う
  if (calb_flag == true) {
      calb_flag = false;
      // axis0のみcalibration
      char c = '0';
      //int motornum = c-'0';
      int motornum = 0;
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
  
  /*
  --------------------------------------------------
  Moving around:
    q   w   e
    a   s   d
    z   x   c
  T/B :   increase/decrease max speeds 10%
  Y/N :   increase/decrease only linear speed 10%
  U/M :   increase/decrease only angular speed 10%
  anything else : stop
  --------------------------------------------------
  */
  char cmd = Serial.read();
  
  switch (cmd) {
    case 'w':
      odrive.SetVelocity(0, vel0);
      odrive.SetVelocity(1, vel1);
      break;
    case 'x':
      odrive.SetVelocity(0, -vel0);
      odrive.SetVelocity(1, -vel1);
      break;
    case 'd':
      odrive.SetVelocity(0, vel0);
      odrive.SetVelocity(1, 0);
      break;
    case 'a':
      odrive.SetVelocity(0, 0);
      odrive.SetVelocity(1, vel1);
      break;
    case 'e':
      odrive.SetVelocity(0, 0.5 * vel0);
      odrive.SetVelocity(1, vel1);
      break;
    case 'q':
      odrive.SetVelocity(0, vel0);
      odrive.SetVelocity(1, 0.5 * vel1);
      break;
    case 'c':
      odrive.SetVelocity(0, -0.5 * vel0);
      odrive.SetVelocity(1, -vel1);
      break;
    case 'z':
      odrive.SetVelocity(0, -vel0);
      odrive.SetVelocity(1, -0.5 * vel1);
      break;
    case 's':
      odrive.SetVelocity(0, 0);
      odrive.SetVelocity(1, 0);
      break;
    default:
      printf("error");
  }

  /*
  if (cmd == 'w') {
      odrive.SetVelocity(0, vel0);
      odrive.SetVelocity(1, vel1);
  } else if (cmd == 'x') {
      odrive.SetVelocity(0, -vel0);
      odrive.SetVelocity(1, -vel1);
  } else if (cmd == 's') {
      odrive.SetVelocity(0, 0);
      odrive.SetVelocity(1, 0);
  }
  */

  cnt++;
}