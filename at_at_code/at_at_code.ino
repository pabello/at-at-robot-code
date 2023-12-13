#include <Dynamixel2Arduino.h>
#include <math.h>
#include "headers.h"
// #include <Ramp.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// Bluetooth communication buffer
String messageBuffer = "";

//initiating the motors as numbers
const uint8_t DXL_ID1 = 1;  //motor 1 (lower part)
const uint8_t DXL_ID2 = 2;  //motor 2 (upper part)
const uint8_t DXL_ID3 = 3;  //motor 1 (lower part)
const uint8_t DXL_ID4 = 4;  //motor 2 (upper part)
const uint8_t DXL_ID5 = 5;  //motor 1 (lower part)
const uint8_t DXL_ID6 = 6;  //motor 2 (upper part)
const uint8_t DXL_ID7 = 7;  //motor 1 (lower part)
const uint8_t DXL_ID8 = 8;  //motor 2 (upper part)
const float DXL_PROTOCOL_VERSION = 2.0;

//fixed length of robot legs (later used)
double LEG_LENGTH = 120;  //fixed length leg

Dynamixel2Arduino dxl(DXL_SERIAL);

double x_desired, y_desired, q1d_l, q2d_l, q1d_r, q2d_r;
int walking_state = 4;
bool flag_var = false;
unsigned long tf = millis() + 500;


int legsOrder[4] = { 1, 2, 3, 4 };  // thanks to this list we can try different setups of leg movement order
int legOrderId = 0;

int typeOfMovement = 1;    // 0: none; 1: forward; 2: backward; 3: left; 4: right;
int whichLegIsMoving = 1;  // 0: none; 1: front-left; 2: front-right; 3: back-left; 4: back-right;
int stepDuration = 500;


//configuring the motors and stuff for pawel
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Arduino builtin led for debugging
  Serial.begin(115200);          // Uncomment this line if you want to use serial communication for debugging
  Serial2.begin(9600);

  messageBuffer.reserve(32);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println("Hello");

  for (int i = 1; i <= 8; ++i) {
    // Check if the motor with ID i responds to ping
    // ^???
    servoDiscovery(i);
  }

  RobotStartingPosition();
}

void RobotStartingPosition() {
  LegAngles angles;

  Coordinates coords_stand = Coordinates(0, -240);
  angles = singleLegAngles(coords_stand, false);
  for (int legId : legsOrder) {
    testMoveSingleLeg(legId, angles);
  }
  delay(2000);

  Coordinates coords_ready = Coordinates(0, -230);
  for (int legId : legsOrder) {
    bool invertedAngles = ((legId + 1) % 2);
    angles = singleLegAngles(coords_ready, invertedAngles);
    testMoveSingleLeg(legId, angles);
  }
  delay(1000);
}


// TODO: What do these lines do? If not important then remove.
bool servoDiscovery(int servoId) {
  dxl.torqueOff(servoId);
  dxl.setOperatingMode(servoId, OP_CURRENT_BASED_POSITION);
  //dxl.writeControlTableItem(CURRENT_LIMIT, i, 100);

  //dxl.writeControlTableItem(POSITION_P_GAIN, i, 900);
  //dxl.writeControlTableItem(POSITION_I_GAIN, i, 100);
  //dxl.writeControlTableItem(POSITION_D_GAIN, i, 200);
  dxl.writeControlTableItem(DRIVE_MODE, servoId, 2);              //drive mode 2 for the trapezoid input (see more info in the pdf of the motors) also for the rest
  dxl.writeControlTableItem(PROFILE_VELOCITY, servoId, 500);      //trajectory time in ms
  dxl.writeControlTableItem(PROFILE_ACCELERATION, servoId, 100);  //acceleraction time in ms

  dxl.torqueOn(servoId);
  dxl.setGoalCurrent(servoId, 200, UNIT_MILLI_AMPERE);
}



// bool moveLeg(int legId, int position[2]) {
//   Serial.print("");
//   // return true/false to indicate if the movement has finished
// }



double getParabolaY(double x) {
  return (-0.012 * (x*x)) + 30;
}

// do not remove start and stop - it will be usefull for making turns
Coordinates getMovementTrajectory(long startTime, Coordinates start, Coordinates stop, int duration, bool isLeading) {
  long currentTime = millis();
  double multiplicationFactor = (currentTime - startTime) / duration;

  Coordinates vector = stop-start;
  vector.multiply(multiplicationFactor);

  Coordinates newCoordinates = start + vector;
  if (isLeading) {
    newCoordinates.y = getParabolaY(newCoordinates.x);
  }

  return newCoordinates;
}


LegAngles angles;
Coordinates coords[8] = {
  Coordinates(0, -200),
  Coordinates(-51, -230),
  Coordinates(-34, -230),
  Coordinates(-17, -230),
  Coordinates(0, -230),
  Coordinates(17, -230),
  Coordinates(34, -230),
  Coordinates(51, -230),
};

int coordinateId = -1;
void loop() {
  if (++coordinateId >= sizeof(coords)) {
    coordinateId = 0;
  }

  // NEW CODE
  // if (typeOfMovement > 0) {
  //   bool movementFinished = true;
  //   // bool movementFinished = moveLeg();
  //   if (movementFinished) {
  //     if (++legOrderId >= sizeof(legsOrder))
  //       legOrderId = 0;
  //     whichLegIsMoving = legsOrder[legOrderId];
  //   }
  // }
  //================================================


  // Naive test for all 4 legs (without taking order into account)
  for (int legId = 0; legId < 4; legId++) {
    int coordsId = (coordinateId + 2*legId) % 8;
    bool invertAngles = (legId + 1) % 2;  // gives 0 for left side and 1 for right side
    angles = singleLegAngles(coords[coordsId], invertAngles);
    testMoveSingleLeg(legId, angles);
    delay(1000);
  }

  // Bluetooth signals handling
  while (Serial2.available()) {
    char inputChar = Serial2.read();
    messageBuffer += inputChar;
    delay(2);

    // Serial.print(messageBuffer + "  |  ");
    if (inputChar == ';') {
      Serial.println(messageBuffer);
      dumpDataSerial2();
      if (messageBuffer == "LED ON;") {
        digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
        Serial2.print("LED ON;");
      }
      if (messageBuffer == "LED OFF;") {
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (LOW is the voltage level)
        Serial2.print("LED OFF;");
      }
      break;
    }
  }
  messageBuffer = "";
}

LegAngles singleLegAngles(Coordinates coords, bool inverted) {
  double C_factor = 3.8298;
  double q1, q2;
  int invertionFactor = 1;
  LegAngles angles;

  if (inverted) {
    invertionFactor = -1;
  }

  q2 = acos((coords.x * coords.x + coords.y * coords.y - LEG_LENGTH * LEG_LENGTH - LEG_LENGTH * LEG_LENGTH) / (2 * LEG_LENGTH * LEG_LENGTH));
  q1 = atan2(coords.y, coords.x) - atan2(LEG_LENGTH * sin(q2), LEG_LENGTH + LEG_LENGTH * cos(q2));

  angles.upperAngle = 180 - (invertionFactor * ((90 + degrees(q1)) * C_factor));
  angles.lowerAngle = 180 - (invertionFactor * ((90 + degrees(q1) + degrees(q2)) * C_factor));

  return angles;
}

void testMoveSingleLeg(int legId, LegAngles legAngles) {
  dxl.setGoalPosition(2 * legId - 1, legAngles.upperAngle, UNIT_DEGREE);
  dxl.setGoalPosition(2 * legId, legAngles.lowerAngle, UNIT_DEGREE);
}

// Debugging bluetooth signals using serial monitor
void dumpDataSerial2() {
  while (Serial2.available()) Serial2.read();
}


