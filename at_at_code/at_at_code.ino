#include "headers.h"
// #include <Ramp.h>

//This namespace is required to use Control table item names
using namespace ControlTableItem;


Coordinates standPosition = Coordinates(0, -240);
Coordinates readyPosition = Coordinates(0, -230);
Coordinates frontPosition = Coordinates(-50, -230);
Coordinates backPosition = Coordinates(50, -230);
Coordinates turnFrontPosition = Coordinates(-20, -230);
Coordinates turnBackPosition = Coordinates(20, -230);

Leg legs[4] = {
  Leg(1, readyPosition, readyPosition, 0),  // [0] -> 1: front-left
  Leg(2, readyPosition, readyPosition, 0),  // [1] -> 2: front-right
  Leg(3, readyPosition, readyPosition, 0),  // [2] -> 3: back-left
  Leg(4, readyPosition, readyPosition, 0),  // [3] -> 4: back-right
};


//configuring the motors and stuff for pawel
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Arduino builtin led for debugging
  Serial.begin(115200);          // Uncomment this line if you want to use serial communication for debugging
  Serial2.begin(9600);

  messageBuffer.reserve(32);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);


  for (int i = 1; i <= 8; ++i) {
    // Check if the motor with ID i responds to ping
    // ^???
    servoDiscovery(i);
  }

  RobotStartingPosition();
}

void RobotStartingPosition() {
  LegAngles angles;

  delay(2000);
  Coordinates coords_stand = Coordinates(0, -240);
  for (Leg &leg : legs) {
    leg.setToPosition(coords_stand);
  }
  delay(2000);

  Coordinates coords_ready = Coordinates(0, -230);
  for (Leg &leg : legs) {
    leg.setToPosition(coords_ready);
  }
  delay(1000);
}

bool servoDiscovery(int servoId) {  // TODO: What do these lines do? If not important then remove.
  dxl.torqueOff(servoId);
  dxl.setOperatingMode(servoId, OP_CURRENT_BASED_POSITION);

  //dxl.writeControlTableItem(POSITION_P_GAIN, i, 900);
  //dxl.writeControlTableItem(POSITION_I_GAIN, i, 100);
  //dxl.writeControlTableItem(POSITION_D_GAIN, i, 200);

  dxl.writeControlTableItem(DRIVE_MODE, servoId, 2);                   //drive mode 2 for the trapezoid input (see more info in the pdf of the motors) also for the rest
  dxl.writeControlTableItem(PROFILE_VELOCITY, servoId, stepDuration);  //trajectory time in ms
  dxl.writeControlTableItem(PROFILE_ACCELERATION, servoId, 100);       //acceleraction time in ms

  dxl.torqueOn(servoId);
  dxl.setGoalCurrent(servoId, 200, UNIT_MILLI_AMPERE);
}



void loop() {
  bool leadingLegUpdated = false;
  if (typeOfMovement != 0 ) {
    for (Leg &leg : legs) {
      delay(10);
      if (updatedInPreviousLoop && (leg.legId == leaderUpdatingLegId)) {
        updatedInPreviousLoop = false;
      }
      if (millis() > leg.movementFinishTime) {
        if (!leadingLegUpdated && !updatedInPreviousLoop) {
          leadingLegUpdated = true;
          updatedInPreviousLoop = true;
          leaderUpdatingLegId = leg.legId;
          legOrderId = (legOrderId + 1) % 4;
          leadingLegId = legsOrder[legOrderId];
        }
        leg.updateMovementParameters();
      }
      leg.continueMovement();

      if (leg.legId == 1) {
        bool isLeading = (leg.legId == leadingLegId);
        float fullMovementTime = (stepDuration + (2 * stepDuration * !isLeading));
        Coordinates movementVector = (leg.goalPosition - leg.startPosition);

        // Serial.print("LegId: ");
        // Serial.print(leg.legId);
        // Serial.print(" >>> ");
        // leg.startPosition.print(false);
        // Serial.print(" -> ");
        // leg.goalPosition.print(false);
        // Serial.print(" => ");
        // movementVector.print(false);
        // Serial.print(" =/= ");
        // leg.getCurrentLegPosition().print(false);
        // Serial.print(" || Lead: ");
        // Serial.print(isLeading);
        // Serial.print("(");
        // Serial.print(leadingLegId);
        // Serial.print("); Rem: ");
        // Serial.print(leg.getRemainingTime());
        // Serial.print(" / ");
        // Serial.print(fullMovementTime);
        // Serial.println(" |;");
      }
    }
  }

  printToSerialPlotter();

  // Bluetooth signals handling
  while (Serial2.available()) {
    char inputChar = Serial2.read();
    messageBuffer += inputChar;
    delay(2);

    if (inputChar == ';') {
      // Serial.println(messageBuffer);
      // Serial2.println(messageBuffer);
      dumpDataSerial2();
      if (messageBuffer == "LED ON;") {
        digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      }
      if (messageBuffer == "LED OFF;") {
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED off (LOW is the voltage level)
      }
      if (messageBuffer == "GO FORWARD;") {
        typeOfMovement = 1;
        forwardTurn = 0;
        actionStartTime = millis();
        discoverLegPositions();
      }
      if (messageBuffer == "GO BACKWARD;") {
        typeOfMovement = 2;
        forwardTurn = 0;
        actionStartTime = millis();
        discoverLegPositions();
      }
      if (messageBuffer == "TURN LEFT;") {
        typeOfMovement = 3;
        forwardTurn = -1;
        actionStartTime = millis();
        discoverLegPositions();
      }
      if (messageBuffer == "TURN RIGHT;") {
        typeOfMovement = 4;
        forwardTurn = 1;
        actionStartTime = millis();
        discoverLegPositions();
      }
      if (messageBuffer == "BULL MODE;") {
        typeOfMovement = 1111;
        forwardTurn = 0;
        actionStartTime = millis();
      }
      if (messageBuffer == "STOP;") {
        typeOfMovement = 0;
      }
      if (messageBuffer == "RESET;") {
        typeOfMovement = 0;
        setup();
      }
      break;
    }
  }
  messageBuffer = "";
}

//####====----====####====----====####====----====####====----====####====----====####====----====####====----====####

// Debugging bluetooth signals using serial monitor
void dumpDataSerial2() {
  while (Serial2.available()) Serial2.read();
}


void printToSerialPlotter() {
  Serial.print(dxl.getPresentPosition(3,UNIT_DEGREE));
  Serial.print(",");
  Serial.print(dxl.readControlTableItem(POSITION_TRAJECTORY,3)*360/4096);
  Serial.print(",");
  Serial.print(dxl.readControlTableItem(GOAL_POSITION,3)*360/4096);
  Serial.print(",");
  Serial.println(dxl.getPresentCurrent(3,UNIT_MILLI_AMPERE));
}


void discoverLegPositions() {
  Coordinates fullMovement, currentPosition, remainingMovement;
  float leaderElapsedTime;

  for (Leg &leg : legs) {
    bool isLeading = (leg.legId == leadingLegId);

    if (typeOfMovement == 1) { // go forward
      if (isLeading) {
        leg.startPosition = backPosition;
        leg.goalPosition = frontPosition;
      } else {
        leg.startPosition = frontPosition;
        leg.goalPosition = backPosition;
      }
    }
    if (typeOfMovement == 2) { // go backward
      if (isLeading) {
        leg.startPosition = frontPosition;
        leg.goalPosition = backPosition;
      } else {
        leg.startPosition = backPosition;
        leg.goalPosition = frontPosition;
      }
    }
    if ((typeOfMovement == 3) || (typeOfMovement == 4)) {  // turn left OR turn right
      bool isTurning = (((typeOfMovement == 3) && ((leg.legId % 2) == 1)) || ((typeOfMovement == 4) && ((leg.legId % 2) == 0)));

      if (isTurning) {
        if (isLeading) {
          leg.startPosition = turnBackPosition;
          leg.goalPosition = turnFrontPosition;
        } else {
          leg.startPosition = turnFrontPosition;
          leg.goalPosition = turnBackPosition;
        }
      } else {
        if (isLeading) {
          leg.startPosition = backPosition;
          leg.goalPosition = frontPosition;
        } else {
          leg.startPosition = frontPosition;
          leg.goalPosition = backPosition;
        }
      }
    }

    if (isLeading) {
      fullMovement = leg.goalPosition - leg.startPosition;
      currentPosition = leg.getCurrentLegPosition();
      remainingMovement = leg.goalPosition - currentPosition;

      leaderElapsedTime = ((remainingMovement.x / fullMovement.x) * stepDuration);
      float baseMovementTime = stepDuration;
      float remainingMovementTime = baseMovementTime - leaderElapsedTime;
      leg.movementFinishTime = actionStartTime + remainingMovementTime;
    }
  }

  for (Leg &leg : legs) {
    float baseMovementTime, remainingMovementTime;
    bool isLeading = (leg.legId == leadingLegId);

    if (!isLeading) {
      baseMovementTime = stepDuration * howManyLegsUntilLeader(leg.legId);
      remainingMovementTime = baseMovementTime - leaderElapsedTime;
      // float baseMovementTime = stepDuration * howManyLegsUntilLeader(leg.legId);
      // float remainingMovementTime = baseMovementTime - leaderElapsedTime;
      leg.movementFinishTime = actionStartTime + remainingMovementTime;
    } else {
      baseMovementTime = stepDuration;
      remainingMovementTime = baseMovementTime - leaderElapsedTime;
    }

    // Serial.print("LegId: ");
    // Serial.print(leg.legId);
    // Serial.print(" | baseTime: ");
    // Serial.print(baseMovementTime);
    // Serial.print(" | remainingTime: ");
    // Serial.print(remainingMovementTime);
    // Serial.print(" | leading: ");
    // Serial.println(isLeading ? "true" : "false");
  }
}
