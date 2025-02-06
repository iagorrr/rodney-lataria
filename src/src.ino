/*
 * TODO: implementar PWM pros motores ao invés de tudo torado
 * TODO: tratar caso em que o ultrasonic devolve lixo ?
 */

// #define DEBUG_MOTOR
// #define DEBUG_PRINT
#define DEBUG_ULTRASONIC
// #define PROTOTYPE

const uint8_t PIN_LED = 2;

namespace Constants {
const double ARENA_RADIUS_CM = 400;
const double MIN_ATK_DIST_CM = ARENA_RADIUS_CM * 2;
};        // namespace Constants

namespace Ultrasonic {
const uint8_t PIN_TRIG = 33;
const uint8_t PIN_ECHO = 35;

// Half sound speed in cm / us
const double HALF_SOUND_SPEED = 0.017;

inline void setup() {
        pinMode(PIN_TRIG, OUTPUT);
        pinMode(PIN_ECHO, INPUT);
}

inline double get_distance() {
        static double duration_us, distance_cm;
        digitalWrite(PIN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_TRIG, LOW);
        duration_us = pulseIn(PIN_ECHO, HIGH);
        distance_cm = HALF_SOUND_SPEED * duration_us;

#ifdef DEBUG_ULTRASONIC

        if (distance_cm > Constants::MIN_ATK_DIST_CM) {
                digitalWrite(PIN_LED, HIGH);
        } else
                digitalWrite(PIN_LED, LOW);
#endif

        return distance_cm;
}

}        // namespace Ultrasonic

namespace Motors {
const uint8_t PINS[2][2] = {
        {12, 14},
        {26, 27},
};

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_RES = 8;

enum Side { left, right };

enum Direction { foward, backward };

void stop(Side s) { ledcWrite(PINS[s][0], 0), ledcWrite(PINS[s][1], 0); }

void stop() {
        // if (set_state(State::stoped)) return;
        stop(Side::left), stop(Side::right);
}

const uint8_t MIN_MOVE_SPEED = 255 * 0.8;
void move(Side s, Direction d, uint8_t speed = MIN_MOVE_SPEED) {
        ledcWrite(PINS[s][d], 0), ledcWrite(PINS[s][!d], speed);
}

void move(Direction d, uint8_t speed = MIN_MOVE_SPEED) {
        move(Side::right, d, min(speed, (uint8_t)(speed * 0.7))),
                move(Side::left, d, speed);
}

/*
 * Simply rotates clockwise :D
 * */
const uint8_t MIN_ROTATE_SPEED = 255 * 0.35;
void rotate(uint8_t speed = MIN_ROTATE_SPEED) {
        // if (set_state(State::rotating)) return;
        move(Side::left, Direction::foward, speed),
                move(Side::right, Direction::backward, speed);
}

inline void setup() {
        for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                        pinMode(PINS[i][j], OUTPUT);
                        digitalWrite(PINS[i][j], LOW);
                        ledcAttach(PINS[i][j], PWM_FREQ, PWM_RES);
                }
        }

#ifdef DEBUG_MOTOR
        Serial.printf("MOTOR: test started\n");
        digitalWrite(PIN_LED, HIGH);

        const double t = 2 * 1e3;

        move(Side::left, Direction::foward), delay(t);

        move(Side::left, Direction::backward), delay(t);

        stop(Side::left);

        move(Side::right, Direction::foward), delay(t);

        move(Side::right, Direction::backward), delay(t);

        stop(Side::right);

        move(Direction::foward), delay(t);

        move(Direction::backward), delay(t);

        rotate(), delay(t);

        stop();

        digitalWrite(PIN_LED, LOW);
        Serial.printf("MOTOR: Test finished\n");
#endif
        Motors::stop();
}
}        // namespace Motors

namespace Infrared {

bool parameufi = false;

enum Infrared_position {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_RIGHT,
        BACK_LEFT,

};
const uint8_t PIN_OUT[]{15, 4, 19, 22};

void IRAM_ATTR fallback() { parameufi = true; }

inline void setup() {
        for (int i = 0; i < 2; i++) {
                pinMode(PIN_OUT[i], INPUT_PULLUP);
                attachInterrupt(digitalPinToInterrupt(PIN_OUT[i]), fallback,
                                RISING);
        }
}

enum Arena_position { OUT, IN };

Arena_position get_arena_position(Infrared_position p) {
        return (Arena_position)!digitalRead(PIN_OUT[p]);
}

Arena_position get_front_position() {
        auto ret = (Arena_position)(get_arena_position(
                                            Infrared_position::FRONT_LEFT) and
                                    get_arena_position(
                                            Infrared_position::FRONT_RIGHT));
        if (!ret) {
                digitalWrite(PIN_LED, HIGH);
        } else
                digitalWrite(PIN_LED, LOW);
        return ret;
}

}        // namespace Infrared

hw_timer_t *timer = NULL;
void setup() {
        pinMode(PIN_LED, OUTPUT);
        Motors::setup();
        Infrared::setup();
        Ultrasonic::setup();
        delay(5000);
}

// int c = 0;
void loop() {
#ifdef DEBUG_PRINT
        static int idx;
        Serial.printf(
                "%4d \t IRFL %d \t IRFR %d \t IRBR %d \t IRBL %d \t US %0.2f \t MS
                        % d\n ",
                        idx++,
                Infrared::get_arena_position(Infrared::FRONT_LEFT),
                Infrared::get_arena_position(Infrared::FRONT_RIGHT),
                Infrared::get_arena_position(Infrared::BACK_RIGHT),
                Infrared::get_arena_position(Infrared::BACK_LEFT),
                Ultrasonic::get_distance(), (int)Motors::current);
        Serial.flush();
#endif
        if (Infrared::parameufi) {
                Infrared::parameufi = false;
                Motors::stop();
                delay(500);
                Motors::move(Motors::Direction::backward);
                delay(300);
                Motors::rotate();
                delay(250);
                Infrared::parameufi = false;
        } else if (Ultrasonic::get_distance() <= Constants::MIN_ATK_DIST_CM) {
                Motors::stop();
                delay(500);
                Motors::move(Motors::Direction::foward);
                delay(200);
        } else {
                Motors::rotate();
                delay(250);
        }

        Motors::stop();
        delay(200);
}

// oi como faz isso aqui
