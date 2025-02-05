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
const double arena_ratio_cm = 20;
const double min_atk_dist = arena_ratio_cm * 2;
const double micro_backward_time_ms = 200;
const double micro_rotation_time_ms = 100;
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

        if (distance_cm > Constants::min_atk_dist) {
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

const uint8_t MIN_SPEED = 40;

const uint32_t PWM_FREQ = 20000;
const uint8_t PWM_RES = 8;

enum Side { left, right };

enum Direction { foward, backward };

/*
 * GAMBIARRA to don't set a state twice
 * This may be a mutex
 * */
enum State { stoped, rotating, moving_foward, moving_backward };
State current = State::stoped;
bool set_state(State st) { return current == st ? true : current = st, false; }

void stop(Side s) {
        ledcWrite(PINS[s][0], 0), ledcWrite(PINS[s][1], 0);
}

void stop() {
        // if (set_state(State::stoped)) return;
        stop(Side::left), stop(Side::right);
}

void move(Side s, Direction d, uint8_t speed=MIN_SPEED) {
        ledcWrite(PINS[s][d], 0), ledcWrite(PINS[s][!d], speed);
}

void move(Direction d, uint8_t speed=MIN_SPEED) {
        // if (set_state(d == Direction::foward ? State::moving_foward
        //                                      : State::moving_backward))
        //         return;

        move(Side::left, d, speed), move(Side::right, d, speed);
}

/*
 * Simply rotates clockwise :D
 * */
void rotate(uint8_t speed=MIN_SPEED) {
        // if (set_state(State::rotating)) return;
        move(Side::left, Direction::foward, speed), move(Side::right, Direction::backward, speed);
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
}
}        // namespace Motors

namespace Infrared {

enum Infrared_position {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_RIGHT,
        BACK_LEFT,

};
const uint8_t PIN_OUT[]{15, 4, 19, 22};

inline void setup() {
        pinMode(PIN_OUT[Infrared_position::FRONT_LEFT], INPUT);
        pinMode(PIN_OUT[Infrared_position::FRONT_RIGHT], INPUT);
        pinMode(PIN_OUT[Infrared_position::BACK_RIGHT], INPUT);
        pinMode(PIN_OUT[Infrared_position::BACK_LEFT], INPUT);
}

enum Arena_position { OUT, IN };

Arena_position get_arena_position(Infrared_position p) {
        return (Arena_position)digitalRead(PIN_OUT[p]);
}

Arena_position get_front_position() {
        return (Arena_position)(get_arena_position(
                                        Infrared_position::FRONT_LEFT) and
                                get_arena_position(
                                        Infrared_position::FRONT_RIGHT));
}

}        // namespace Infrared

// void IRAM_ATTR onTimer(){
//         ledcWrite(Motors::PINS[Motors::left][0], 0);
//         ledcWrite(Motors::PINS[Motors::left][1], 160);

//         ledcWrite(Motors::PINS[Motors::right][0], 0);
//         ledcWrite(Motors::PINS[Motors::right][1], 160);
//         delay(1000);
// }

hw_timer_t *timer = NULL;
void setup() {
        Serial.begin(9600);
        Serial.println("Setup starting...");

        pinMode(PIN_LED, OUTPUT);

        Infrared::setup();
        Ultrasonic::setup();
        Motors::setup();

        // timer = timerBegin(1000000);
        // timerAttachInterrupt(timer, &onTimer);
        // timerAlarm(timer, 8000000, true, 0);

        Serial.println("Setup starting...");
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
        // if (!Infrared::get_front_position()) {
        //          /*
        //           * Any infrared out of arena
        //           */
        //           Motors::stop();
        //         delay(100);
        //          Motors::move(Motors::Direction::backward);
        //          delay(100);
        //          Motors::rotate();
        //          delay(100);
        // } else
        if (Ultrasonic::get_distance() <= Constants::min_atk_dist) {
                /*
                 * Start engage or keep may it
                 */
                move(Motors::Direction::foward);
        } else {
                /*
                 * Seek and destroy !
                 * */
                Motors::rotate();
        }
}