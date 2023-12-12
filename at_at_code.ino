#include <Dynamixel2Arduino.h>
#include <math.h>
#include <Ramp.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial

//This namespace is required to use Control table item names
using namespace ControlTableItem;
//for pawel
String messageBuffer = "";

//initiating the motors as numbers
const uint8_t DXL_ID1 = 1; //motor 1 (lower part)
const uint8_t DXL_ID2 = 2; //motor 2 (upper part)
const uint8_t DXL_ID3 = 3; //motor 1 (lower part)
const uint8_t DXL_ID4 = 4; //motor 2 (upper part)
const uint8_t DXL_ID5 = 5; //motor 1 (lower part)
const uint8_t DXL_ID6 = 6; //motor 2 (upper part)
const uint8_t DXL_ID7 = 7; //motor 1 (lower part)
const uint8_t DXL_ID8 = 8; //motor 2 (upper part)
const float DXL_PROTOCOL_VERSION = 2.0; 

//fixed length of robot legs (later used)
double a1=120; //fixed length leg
double a2=120; //fixed length leg

Dynamixel2Arduino dxl(DXL_SERIAL);


//configuring the motors and stuff for pawel
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial2.begin(9600);
  messageBuffer.reserve(32);
  Serial.begin(115200); // Uncomment this line if you want to use serial communication for debugging
  Serial.println("Hello");
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 1; i <= 8; ++i) {
    // Check if the motor with ID i responds to ping
  
    Serial.print("Motor ");
    Serial.print(i);
    Serial.println(" found.");
    
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_CURRENT_BASED_POSITION);
    //dxl.writeControlTableItem(CURRENT_LIMIT, i, 100);
  
    //dxl.writeControlTableItem(POSITION_P_GAIN, i, 900);
    //dxl.writeControlTableItem(POSITION_I_GAIN, i, 100);
    //dxl.writeControlTableItem(POSITION_D_GAIN, i, 200);
    dxl.writeControlTableItem(DRIVE_MODE, i, 2); //drive mode 2 for the trapezoid input (see more info in the pdf of the motors) also for the rest
    dxl.writeControlTableItem(PROFILE_VELOCITY, i, 500);  //trajectory time in ms
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 100);  //acceleraction time in ms

    dxl.torqueOn(i);
    dxl.setGoalCurrent(i,200,UNIT_MILLI_AMPERE);
  }
}


void loop() {
  //main loop idea was if walking state changes it goes either straight backwards, left/right (only straight is working atm)
  double x_desired, y_desired, q1d_l, q2d_l, q1d_r, q2d_r;
  int walking_state=4;
  bool flag_var=false;
  unsigned long tf=millis()+500;
  
  if (walking_state == 1) { //walking straight ahead
   //1
  } else if (walking_state == 2) {  //walking right
    //2
  
    }else if (walking_state == 3) {  //walking left
    //3
    
  } else {  //walking stationairy
  
    while (!flag_var){
      //the straight walking movement. it works by changing the position every half a second in a circle. 
      //so the position of every motor is changed after 500ms
      //there are 8 position in totals so 1 complete cycle takes 4s in total
      //the loop goes as following back left, front left, back right front right and over again.
      //the legs are spaced 1 second apart for the right walking movement in the same order as said earlier 
      //so imaging back left=1, front left=2 etc.
      //the 8 coordinates are the following (the code is also base upon this circle)
      //x,  y
      //0,-230
      //17, -230
      //34, -230
      //51, -230
      //0, -200    //lifting the leg
      //-51, -230
      //-34, -230
      //-17, -230
      // and over againnn
  
    double angles[8]; //calculated angles put in here
    int coords[8] = {0, -240, 0, -240 ,0 ,-240, 0, -240}; //coords
    coords[0]= 0;  //x
    coords[1]= -240;  //y
    coords[2]= 0;
    coords[3]= -240;
    coords[4]= 0;
    coords[5]= -240;
    coords[6]= 0;
    coords[7]= -240;
    
    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= 0;
      coords[5]= -230;
      coords[0]= 34;
      coords[1]= -230;
      coords[6]= 0;
      coords[7]= -200;
      coords[2]= -34;
      coords[3]= -230;

      calculateAngles(coords, angles);
      writing_legs(angles);
      
      Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
      Serial.print(",");
      Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
      Serial.print(",");
      Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
      Serial.print(",");
      Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
    }

    tf=millis()+500;
    while(millis()<tf)
    {

      calculateAngles(coords, angles);
      writing_legs(angles);
    }

    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= 17;
      coords[5]= -230;
      coords[0]= 51;
      coords[1]= -230;
      coords[6]= -51;
      coords[7]= -230;
      coords[2]= -17;
      coords[3]= -230;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }

    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= 34;
      coords[5]= -230;
      coords[0]= 0;
      coords[1]= -200;
      coords[6]= -34;
      coords[7]= -230;
      coords[2]= 0;
      coords[3]= -230;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }
        
    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= 51;
      coords[5]= -230;
      coords[0]= -51;
      coords[1]= -230;
      coords[6]= -17;
      coords[7]= -230;
      coords[2]= 17;
      coords[3]= -230;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }

    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= 0;
      coords[5]= -200;
      coords[0]= -34;
      coords[1]= -230;
      coords[6]= 0;
      coords[7]= -230;
      coords[2]= 34;
      coords[3]= -230;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }    
    
    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= -51;
      coords[5]= -230;
      coords[0]= -17;
      coords[1]= -230;
      coords[6]= 17;
      coords[7]= -230;
      coords[2]= 51;
      coords[3]= -230;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }

    tf=millis()+500;
    while(millis()<tf)
    {
      coords[4]= -34;
      coords[5]= -230;
      coords[0]= 0;
      coords[1]= -230;
      coords[6]= 34;
      coords[7]= -230;
      coords[2]= 0;
      coords[3]= -200;
      calculateAngles(coords, angles);
      writing_legs(angles);
    }


    // Serial.println("angles: ");
    // Serial.print(angles[0]);
    // Serial.print(" ");
    // Serial.print(angles[1]);
    // Serial.print(" ");
    // Serial.print(angles[2]);
    // Serial.print(" ");
    // Serial.print(angles[3]);
    // Serial.print(" ");
    // Serial.print(angles[4]);
    // Serial.print(" ");
    // Serial.print(angles[5]);
    // Serial.print(" ");
    // Serial.print(angles[6]);
    // Serial.print(" ");
    // Serial.println(angles[7]);


    // while(millis()<tf)
    // // {
    //   Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
    //   Serial.print(",");
    //   Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
    //   Serial.print(",");
    //   Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
    //   Serial.print(",");
    //   Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
    // }
    

  
    // tf=millis()+4000;
    // while(millis()<tf)
    // {
    //   Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
    //   Serial.print(",");
    //   Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
    //   Serial.print(",");
    //   Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
    //   Serial.print(",");
    //   Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
    // }
    

    }
  }
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
void calculateAngles(int inp_coords[], double out_angles[]){

  double C_factor = 3.8298;
  double q1, q2;

  for (int i = 0; i <= 6; i += 2) {    
    q2 = acos((inp_coords[i] * inp_coords[i] + inp_coords[i+1] * inp_coords[i+1] - a1 * a1 - a2 * a2) / (2 * a1 * a2));
    q1 = atan2(inp_coords[i+1], inp_coords[i]) - atan2(a2 * sin(q2), a1 + a2 * cos(q2));
  
    q1 = degrees(q1);
    q2 = degrees(q2);

    if(i == 4 || i == 0){
      out_angles[i] = 180 - ((90 + q1) * C_factor);
      out_angles[i+1] = 180 - ((90 + q1 + q2) * C_factor);
    }
    if(i == 6 || i == 2){
      out_angles[i] = 180 + ((90 + q1) * C_factor);
      out_angles[i+1] = 180 + ((90 + q1 + q2) * C_factor);
   }
   
  }
  
}

//this function updates all the motor angles when giving the array of 8 new positions.
void writing_legs(double angles[8]){
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
