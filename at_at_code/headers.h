struct LegAngles {
  double upperAngle;
  double lowerAngle;
};

struct Coordinates {
  int x;
  int y;

  Coordinates(int x_, int y_) {
    x = x_;
    y = y_;
  };

  Coordinates() {
    x = 0;
    y = 0;
  };
};