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
}

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



int legsOrder[4] = { 1, 2, 3, 4 };  // thanks to this list we can try different setups of leg movement order
int legOrderId = 0;

int typeOfMovement = 1;    // 0: none; 1: forward; 2: backward; 3: left; 4: right;
int whichLegIsMoving = 1;  // 0: none; 1: front-left; 2: front-right; 3: back-left; 4: back-right;

bool moveLeg(int legId, int position[2]) {
  Serial.print("");
  // return true/false to indicate if the movement has finished
}


LegAngles angles;
Coordinates coords;

void loop() {
  //main loop idea was if walking state changes it goes either straight backwards, left/right (only straight is working atm)

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

  // long time = millis()
  coords = Coordinates(0, -240);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(5000);


  coords = Coordinates(0, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(0, -200);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(-51, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(-34, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(-17, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(0, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(17, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(34, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);

  coords = Coordinates(51, -230);
  angles = singleLegAngles(coords);
  testMoveSingleLeg(1, angles);
  delay(1000);




  // while (!flag_var){
  //   //the straight walking movement. it works by changing the position every half a second in a circle.
  //   //so the position of every motor is changed after 500ms
  //   //there are 8 position in totals so 1 complete cycle takes 4s in total
  //   //the loop goes as following back left, front left, back right front right and over again.
  //   //the legs are spaced 1 second apart for the right walking movement in the same order as said earlier
  //   //so imaging back left=1, front left=2 etc.
  //   //the 8 coordinates are the following (the code is also base upon this circle)
  //   //x,  y
  //   //0,-230
  //   //17, -230
  //   //34, -230
  //   //51, -230
  //   //0, -200    //lifting the leg
  //   //-51, -230
  //   //-34, -230
  //   //-17, -230
  //   // and over againnn

  //   double angles[8]; //calculated angles put in here
  //   int coords[8] = {0, -240, 0, -240 ,0 ,-240, 0, -240}; //coords
  //   coords[0]= 0;  //x
  //   coords[1]= -240;  //y
  //   coords[2]= 0;
  //   coords[3]= -240;
  //   coords[4]= 0;
  //   coords[5]= -240;
  //   coords[6]= 0;
  //   coords[7]= -240;

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= 0;
  //     coords[5]= -230;
  //     coords[0]= 34;
  //     coords[1]= -230;
  //     coords[6]= 0;
  //     coords[7]= -200;
  //     coords[2]= -34;
  //     coords[3]= -230;

  //     calculateAngles(coords, angles);
  //     writing_legs(angles);

  //     Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
  //     Serial.print(",");
  //     Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
  //     Serial.print(",");
  //     Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
  //     Serial.print(",");
  //     Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {

  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= 17;
  //     coords[5]= -230;
  //     coords[0]= 51;
  //     coords[1]= -230;
  //     coords[6]= -51;
  //     coords[7]= -230;
  //     coords[2]= -17;
  //     coords[3]= -230;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= 34;
  //     coords[5]= -230;
  //     coords[0]= 0;
  //     coords[1]= -200;
  //     coords[6]= -34;
  //     coords[7]= -230;
  //     coords[2]= 0;
  //     coords[3]= -230;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= 51;
  //     coords[5]= -230;
  //     coords[0]= -51;
  //     coords[1]= -230;
  //     coords[6]= -17;
  //     coords[7]= -230;
  //     coords[2]= 17;
  //     coords[3]= -230;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= 0;
  //     coords[5]= -200;
  //     coords[0]= -34;
  //     coords[1]= -230;
  //     coords[6]= 0;
  //     coords[7]= -230;
  //     coords[2]= 34;
  //     coords[3]= -230;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= -51;
  //     coords[5]= -230;
  //     coords[0]= -17;
  //     coords[1]= -230;
  //     coords[6]= 17;
  //     coords[7]= -230;
  //     coords[2]= 51;
  //     coords[3]= -230;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }

  //   tf=millis()+500;
  //   while(millis()<tf)
  //   {
  //     coords[4]= -34;
  //     coords[5]= -230;
  //     coords[0]= 0;
  //     coords[1]= -230;
  //     coords[6]= 34;
  //     coords[7]= -230;
  //     coords[2]= 0;
  //     coords[3]= -200;
  //     calculateAngles(coords, angles);
  //     writing_legs(angles);
  //   }


  //   // Serial.println("angles: ");
  //   // Serial.print(angles[0]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[1]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[2]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[3]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[4]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[5]);
  //   // Serial.print(" ");
  //   // Serial.print(angles[6]);
  //   // Serial.print(" ");
  //   // Serial.println(angles[7]);


  //   // while(millis()<tf)
  //   // // {
  //   //   Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
  //   //   Serial.print(",");
  //   //   Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
  //   //   Serial.print(",");
  //   //   Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
  //   //   Serial.print(",");
  //   //   Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
  //   // }



  //   // tf=millis()+4000;
  //   // while(millis()<tf)
  //   // {
  //   //   Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
  //   //   Serial.print(",");
  //   //   Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
  //   //   Serial.print(",");
  //   //   Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
  //   //   Serial.print(",");
  //   //   Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
  //   // }


  // }
  //stuff for pawel
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


// the algorithm for calculating the motor angles based upon inverse kinematics
void calculateAngles(int inp_coords[], double out_angles[]) {

  double C_factor = 3.8298;
  double q1, q2;

  for (int i = 0; i <= 6; i += 2) {
    q2 = acos((inp_coords[i] * inp_coords[i] + inp_coords[i + 1] * inp_coords[i + 1] - 2 * (LEG_LENGTH * LEG_LENGTH)) / (2 * LEG_LENGTH * LEG_LENGTH));
    q1 = atan2(inp_coords[i + 1], inp_coords[i]) - atan2(LEG_LENGTH * sin(q2), LEG_LENGTH + LEG_LENGTH * cos(q2));

    q1 = degrees(q1);
    q2 = degrees(q2);

    if (i == 4 || i == 0) {
      out_angles[i] = 180 - ((90 + q1) * C_factor);
      out_angles[i + 1] = 180 - ((90 + q1 + q2) * C_factor);
    }
    if (i == 6 || i == 2) {
      out_angles[i] = 180 + ((90 + q1) * C_factor);
      out_angles[i + 1] = 180 + ((90 + q1 + q2) * C_factor);
    }
  }
}



LegAngles singleLegAngles(Coordinates coords) {
  double C_factor = 3.8298;
  double q1, q2;
  LegAngles angles;

  q2 = acos((coords.x * coords.x + coords.y * coords.y - LEG_LENGTH * LEG_LENGTH - LEG_LENGTH * LEG_LENGTH) / (2 * LEG_LENGTH * LEG_LENGTH));
  q1 = atan2(coords.y, coords.x) - atan2(LEG_LENGTH * sin(q2), LEG_LENGTH + LEG_LENGTH * cos(q2));

  angles.upperAngle = 180 - ((90 + degrees(q1)) * C_factor);
  angles.lowerAngle = 180 - ((90 + degrees(q1) + degrees(q2)) * C_factor);

  return angles;
}

void testMoveSingleLeg(int legId, LegAngles legAngles) {
  dxl.setGoalPosition(2 * legId - 1, legAngles.upperAngle, UNIT_DEGREE);
  dxl.setGoalPosition(2 * legId, legAngles.lowerAngle, UNIT_DEGREE);
}



//this function updates all the motor angles when giving the array of 8 new positions.
void writing_legs(double angles[8]) {
  dxl.setGoalPosition(1, angles[0], UNIT_DEGREE);
  dxl.setGoalPosition(2, angles[1], UNIT_DEGREE);
  dxl.setGoalPosition(3, angles[2], UNIT_DEGREE);
  dxl.setGoalPosition(4, angles[3], UNIT_DEGREE);
  dxl.setGoalPosition(5, angles[4], UNIT_DEGREE);
  dxl.setGoalPosition(6, angles[5], UNIT_DEGREE);
  dxl.setGoalPosition(7, angles[6], UNIT_DEGREE);
  dxl.setGoalPosition(8, angles[7], UNIT_DEGREE);
}
//stuff for pawel
void dumpDataSerial2() {
  while (Serial2.available()) Serial2.read();
}
