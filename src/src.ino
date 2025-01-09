
namespace Ultrasonic {
  const uint8_t PIN_TRIG = 26; // D26 / 10
  const uint8_t PIN_ECHO = 27; // D27 / 11

  inline void setup() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
  }

}

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
}

namespace Infrared {
  const uint8_t PIN_FRONT_LEFT = 21; // D21 /25
  const uint8_t PIN_FRONT_RIGHT = 19; // D19 /27
  const uint8_t PIN_BACK_LEFT = 18; // D18 /28
  const uint8_t PIN_BACK_RIGHT = 17; // D17 /30

  inline void setup() {
    pinMode(PIN_FRONT_LEFT, INPUT);
    pinMode(PIN_FRONT_RIGHT, INPUT);
    pinMode(PIN_BACK_RIGHT, INPUT);
    pinMode(PIN_BACK_LEFT, INPUT);
  }
}

void setup() {
  Infrared::setup();
  Ultrasonic::setup();
  Motors::setup();
}

void loop() {

}