/////////////////////////////////////// ---=== INCLUDES ===--- ////////////////////////////////////////
#include <Dynamixel2Arduino.h>
#include <math.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial

/////////////////////////////////////// ---=== CONSTANTS ===--- ///////////////////////////////////////
const float DXL_PROTOCOL_VERSION = 2.0;

// Initiating the motors as numbers
const uint8_t DXL_ID1 = 1;  //motor 1 (upper part)
const uint8_t DXL_ID2 = 2;  //motor 2 (lower part)
const uint8_t DXL_ID3 = 3;  //motor 1 (upper part)
const uint8_t DXL_ID4 = 4;  //motor 2 (lower part)
const uint8_t DXL_ID5 = 5;  //motor 1 (upper part)
const uint8_t DXL_ID6 = 6;  //motor 2 (lower part)
const uint8_t DXL_ID7 = 7;  //motor 1 (upper part)
const uint8_t DXL_ID8 = 8;  //motor 2 (lower part)

float LEG_LENGTH = 120;  //fixed length leg
int stepDuration = 500;

Dynamixel2Arduino dxl(DXL_SERIAL);

//////////////////////////////////// ---=== GLOBAL VARIABLES ===--- ///////////////////////////////////

int legsOrder[4] = { 1, 3, 2, 4 };  // thanks to this list we can try different setups of leg movement order
int legOrderId = 0;
int leadingLegId = legsOrder[legOrderId];  // 0: none; 1: front-left; 2: front-right; 3: back-left; 4: back-right;
int typeOfMovement = 0;    // 0: none; 1: forward; 2: backward; 3: left; 4: right;
float actionStartTime;
int leaderUpdatingLegId;
bool updatedInPreviousLoop = false;
float turningRate = 0.5;  // between <0; 1> where 0.0 does not move legs at all and 1.0 is going straight
float forwardTurn = 0;  // -1: turn left; 0: go straight; 1: turn right;
String messageBuffer = "";

/////////////////////////////////////// ---=== FUNCTIONS ===--- ///////////////////////////////////////

float getParabolaY(float x, float maxX) {
  float a = (-30 / (maxX * maxX));
  return (a * (x*x)) - 200;
}

int howManyLegsUntilLeader(int legId) {
  int remainingLegs = 0;
  int index;
  while (true) {
    index = (legOrderId + remainingLegs) % 4;
    if (legsOrder[index] == legId) {
      return remainingLegs;
    }
    ++remainingLegs;
  }
}

//////////////////////////////////// ---=== DATA STRUCTURES ===--- ////////////////////////////////////

struct LegAngles {
  float upperAngle;
  float lowerAngle;
};

//-----------------------------------------------------------------------------------------------------

struct Coordinates {
  float x;
  float y;

  Coordinates(float x_, float y_) {
    x = x_;
    y = y_;
  };

  Coordinates() {
    x = 0;
    y = 0;
  };

  Coordinates operator-(const Coordinates& other) {
    Coordinates result;
    result.x = this->x - other.x;
    result.y = this->y - other.y;
    return result;
  }

  Coordinates operator+(const Coordinates& other) {
    Coordinates result;
    result.x = this->x + other.x;
    result.y = this->y + other.y;
    return result;
  }

  void increaseBy(const Coordinates& other) {
    this->x += other.x;
    this->y += other.y;
  }

  Coordinates multiply(float factor) {
    Coordinates result;
    result.x = this->x * factor;
    result.y = this->y * factor;
    return result;
  }

  Coordinates multiplyX(float factor) {
    Coordinates result;
    result.x = this->x * factor;
    return result;
  }
  
  void print(bool newline) {
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(")");
    if (newline) {
      Serial.println();
    }
  }
};

//-----------------------------------------------------------------------------------------------------

struct Leg {
  int legId;
  
  Coordinates startPosition;
  Coordinates goalPosition;

  float fullMovementTime;
  float movementFinishTime;
  bool invertedAngles;
  bool previousLeader;

  Leg(int legId, Coordinates startPosition, Coordinates goalPosition, float movementFinishTime) {
    this->legId = legId;
    this->startPosition = startPosition;
    this->goalPosition = goalPosition;
    this->movementFinishTime = movementFinishTime;
    this->invertedAngles = ((legId % 2) == 0);
    this->previousLeader = (legId == leadingLegId);
  }
  

  void updateMovementParameters() {
    bool isLeading = (this->legId == leadingLegId);
    if (previousLeader != isLeading) {
      // Serial.print(legId);
      // Serial.print(" ==> ");
      // startPosition.print(false);
      // Serial.print("->");
      // goalPosition.print(false);
      // Serial.print(" --> ");
      Coordinates temp = startPosition;
      startPosition = goalPosition;
      goalPosition = temp;
      // startPosition.print(false);
      // Serial.print("->");
      // goalPosition.print(true);

    }
    previousLeader = isLeading;
    if (isLeading) {
      fullMovementTime = stepDuration;
    } else {
      fullMovementTime = (3 * stepDuration);
    }
    movementFinishTime += fullMovementTime;
    // Serial.print("LegId: ");
    // Serial.print(legId);
    // Serial.print(" | Movement time: ");
    // Serial.print(fullMovementTime);
    // Serial.print(" | Leading: ");
    // Serial.println(isLeading);

  }

  // if not used, then delete
  float getRemainingTime() {
    return movementFinishTime - millis();
  }


  Coordinates getNextLegPosition() {
    bool isLeading = (legId == leadingLegId);
    float fullMovementTime = (stepDuration + (2 * stepDuration * !isLeading));

    float remainingTime = movementFinishTime - millis();
    float requiredMovementPercent = 1 - (remainingTime / fullMovementTime);

    Coordinates movementVector = (goalPosition - startPosition);
    movementVector = movementVector.multiply(requiredMovementPercent);

    Coordinates newPosition = startPosition + movementVector;

    if (isLeading) {
      newPosition.y = getParabolaY(newPosition.x, goalPosition.x);
    }
    return newPosition;
  }


  LegAngles calculateNewAngles(Coordinates newPosition) {
    float C_factor = 3.8298;
    float q1, q2;
    int invertionFactor = 1;
    LegAngles angles;

    if (invertedAngles) {
      invertionFactor = -1;
    }

    q2 = acos((newPosition.x * newPosition.x + newPosition.y * newPosition.y - 2 * (LEG_LENGTH * LEG_LENGTH)) / (2 * LEG_LENGTH * LEG_LENGTH));
    q1 = atan2(newPosition.y, newPosition.x) - atan2(LEG_LENGTH * sin(q2), LEG_LENGTH + LEG_LENGTH * cos(q2));

    angles.upperAngle = 180 - (invertionFactor * ((90 + degrees(q1)) * C_factor));
    angles.lowerAngle = 180 - (invertionFactor * ((90 + degrees(q1) + degrees(q2)) * C_factor));
    return angles;
  }


  void setToPosition(Coordinates newPosition) {
    LegAngles newAngles = calculateNewAngles(newPosition);

    dxl.setGoalPosition(2 * legId - 1, newAngles.upperAngle, UNIT_DEGREE);
    dxl.setGoalPosition(2 * legId, newAngles.lowerAngle, UNIT_DEGREE);
  }


  void continueMovement() {
    Coordinates newPosition = getNextLegPosition();
    setToPosition(newPosition);
  }
  

  // Gets current leg position from angles reading of servos
  Coordinates getCurrentLegPosition() {
    float C_factor = 3.8298;
    bool inverted = ((legId % 2) == 0);

    // Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);
    float lowerMotorAngle = dxl.getPresentPosition(2 * legId - 1, UNIT_DEGREE); //use angle in degree
    float upperMotorAngle = dxl.getPresentPosition(2 * legId, UNIT_DEGREE);

    int invertionFactor = 1;
    if (inverted) {
      invertionFactor = -1;
    }
      
    float q1 = radians((180 - upperMotorAngle) / (invertionFactor * C_factor) - 90);
    float q2 = radians((180 - lowerMotorAngle) / (invertionFactor * C_factor) - 90) - q1;

    float x = LEG_LENGTH * (cos(q1) + cos(q1 + q2));
    float y = LEG_LENGTH * (sin(q1) + sin(q1 + q2));

    return Coordinates(x, y);
  }
  
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
