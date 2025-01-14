namespace Ultrasonic {

const uint8_t PIN_TRIG = 26; // D26 / 10
const uint8_t PIN_ECHO = 27; // D27 / 11
// Half sound speed in cm / us
const double HALF_SOUND_SPEED = 0.017;

inline void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

double duration_us, distance_cm;
inline double get_distance() {
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  duration_us = pulseIn(PIN_ECHO, HIGH);
  distance_cm = HALF_SOUND_SPEED * duration_us;
  return distance_cm;
}

} // namespace Ultrasonic

namespace Motors {
const uint8_t PIN_IN_1 = 33; // D33 / 8
const uint8_t PIN_IN_2 = 32; // D32 / 7
const uint8_t PIN_IN_3 = 35; // D35 / 6
const uint8_t PIN_IN_4 = 34; // D34 / 5

inline void setup() {
  pinMode(PIN_IN_1, OUTPUT);
  pinMode(PIN_IN_2, OUTPUT);
  pinMode(PIN_IN_3, OUTPUT);
  pinMode(PIN_IN_4, OUTPUT);
}
} // namespace Motors

namespace Infrared {

enum Infrared_position {
  FRONT_LEFT = 0,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT,
};
const uint8_t PIN_OUT[]{21, 19, 18, 17};

inline void setup() {
  pinMode(PIN_OUT[Infrared_position::FRONT_LEFT], INPUT);
  pinMode(PIN_OUT[Infrared_position::FRONT_RIGHT], INPUT_PULLUP);
  pinMode(PIN_OUT[Infrared_position::BACK_RIGHT], INPUT_PULLUP);
  pinMode(PIN_OUT[Infrared_position::BACK_LEFT], INPUT_PULLUP);
}

enum Arena_position { OUT = 0, IN };

Arena_position get_arena_position(Infrared_position p) {
  return (Arena_position)digitalRead(PIN_OUT[p]);
}

} // namespace Infrared

void setup() {
  Serial.begin(9600);
  Infrared::setup();
  Ultrasonic::setup();
  Motors::setup();
}

void loop() {}
