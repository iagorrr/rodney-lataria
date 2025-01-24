#define DEBUG_MOTOR
#define DEBUG_PRINT
#define DEBUG_ULTRASONIC

const uint8_t PIN_LED = 2;

namespace Ultrasonic {
const uint8_t PIN_TRIG = 33;
const uint8_t PIN_ECHO = 35;

//Half sound speed in cm / us
const double HALF_SOUND_SPEED = 0.017;

inline void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}


inline double get_distance() {
  static double duration_us, distance_cm;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  duration_us = pulseIn(PIN_ECHO, HIGH);
  distance_cm = HALF_SOUND_SPEED * duration_us;
  return distance_cm;
}

}

namespace Motors {
const uint8_t PINS[2][2] = {
  { 12, 14 },
  { 27, 26 },
};


enum Side {
  left,
  right
};

enum Direction {
  foward,
  backward
};

void move(int32_t t, Side s, Direction d) {
  auto a = d ? HIGH : LOW;
  auto b = d ? LOW : HIGH;
  digitalWrite(PINS[s][0], a),
    digitalWrite(PINS[s][1], b);

  delayMicroseconds(t);

  digitalWrite(PINS[s][0], LOW),
    digitalWrite(PINS[s][1], LOW);
}

void move(int32_t t, Direction d) {
  digitalWrite(PINS[0][0], d),
    digitalWrite(PINS[0][1], !d);

  digitalWrite(PINS[1][0], d),
    digitalWrite(PINS[1][1], !d);

  delayMicroseconds(t);

  digitalWrite(PINS[0][0], LOW),
    digitalWrite(PINS[0][1], LOW);

  digitalWrite(PINS[1][0], LOW),
    digitalWrite(PINS[1][1], LOW);
}

inline void setup() {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pinMode(PINS[i][j], OUTPUT);
      digitalWrite(PINS[i][j], LOW);
    }
  }



#ifdef DEBUG_MOTOR
  Serial.printf("MOTOR: test started\n");
  digitalWrite(PIN_LED, HIGH);

  move(2 * 1e6, Side::left, Direction::foward);
  move(2 * 1e6, Side::left, Direction::backward);

  move(2 * 1e6, Side::right, Direction::foward);
  move(2 * 1e6, Side::right, Direction::backward);

  move(2 * 1e6, Direction::foward);
  move(2 * 1e6, Direction::backward);

  digitalWrite(PIN_LED, LOW);
  Serial.printf("MOTOR: Test finished\n");
#endif
}
}

namespace Infrared {

enum Infrared_position {
  FRONT_LEFT = 0,
  FRONT_RIGHT,
  BACK_RIGHT,
  BACK_LEFT,

};
const uint8_t PIN_OUT[]{ 15, 4, 19, 22 };


inline void setup() {
  pinMode(PIN_OUT[Infrared_position::FRONT_LEFT], INPUT);
  pinMode(PIN_OUT[Infrared_position::FRONT_RIGHT], INPUT);
  pinMode(PIN_OUT[Infrared_position::BACK_RIGHT], INPUT);
  pinMode(PIN_OUT[Infrared_position::BACK_LEFT], INPUT);
}

enum Arena_position {
  OUT = 0,
  IN
};

Arena_position get_arena_position(Infrared_position p) {
  return (Arena_position)digitalRead(PIN_OUT[p]);
}

}


void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);

  Infrared::setup();
  Ultrasonic::setup();
  Motors::setup();
  pinMode(2, OUTPUT);
}


void debug() {
#ifdef DEBUG_PRINT
  static int idx;
  Serial.printf("%4d \t IRFL %d \t IRFR %d \t IRBR %d \t IRBL %d \t US %0.2f\n",
                idx++,
                Infrared::get_arena_position(Infrared::FRONT_LEFT),
                Infrared::get_arena_position(Infrared::FRONT_RIGHT),
                Infrared::get_arena_position(Infrared::BACK_RIGHT),
                Infrared::get_arena_position(Infrared::BACK_LEFT),
                Ultrasonic::get_distance());
  Serial.flush();
#endif

#ifdef DEBUG_ULTRASONIC
  if (Ultrasonic::get_distance() >= 10) {
    digitalWrite(PIN_LED, HIGH);
  } else digitalWrite(PIN_LED, LOW);
#endif
}

void loop() {
#ifdef PROTOTYPE
  debug();
#endif
}

