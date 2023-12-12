struct LegAngles {
  double upperAngle;
  double lowerAngle;
};

struct Coordinates {
  double x;
  double y;

  Coordinates(double x_, double y_) {
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

  void multiply(double factor) {
    this->x *= factor;
    this->y *= factor;
  }
};