// MAZE RUNNERS
// hamza, zena, roya, anas

// USEFUL RESOURCES: -------
//  - https://docs.arduino.cc/learn/programming/memory-guide/
//  - https://docs.arduino.cc/language-reference/
//  - https://arduino-doc.readthedocs.io/en/stable/3.Arduino%20Projects/8.Ultrasonic%20Sensor/
// -------------------------

// DATA TYPE SIZES ---------
//           char: 1 byte
//  unsigned char: 1 byte
//           byte: 1 byte
//            int: 2 bytes
//   unsigned int: 2 bytes
//           long: 4 bytes
//  unsigned long: 4 bytes
//          float: 4 bytes
// -------------------------

// UNITS -------------------
// - time: us
// - distance: cm
// -------------------------

#include "stem.h"

// HELPER FUNCTIONS
f32 lerpf(f32 a, f32 b, f32 t) {
    return a + t * (b - a);
}

cardinal_t relative_to_cardinal(cardinal_t forward, dir_t dir) {
    return (forward + dir) % 4;
}

// GLOBAL VARIABLES
path_s          paths[MAX_PATHS] = {0};
intersection_s  intersections[MAX_INTERSECTIONS] = {0};

state_s state = {0};
robot_s robot = {0};

// INITIALIZATION
void init_pins() {
    // motor enables
    pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
    pinMode(MOTOR_LEFT_ENABLE,  OUTPUT);

    // motor ins
    pinMode(MOTOR_RIGHT_IN1,    OUTPUT);
    pinMode(MOTOR_RIGHT_IN2,    OUTPUT);
    pinMode(MOTOR_LEFT_IN1,     OUTPUT);
    pinMode(MOTOR_LEFT_IN2,     OUTPUT);

    // ultrasonic triggers
    pinMode(US_LEFT_TRIGGER,    OUTPUT);
    pinMode(US_FRONT_TRIGGER,   OUTPUT);
    pinMode(US_RIGHT_TRIGGER,   OUTPUT);

    // ultrasonic echos
    pinMode(US_LEFT_ECHO,       INPUT);
    pinMode(US_FRONT_ECHO,      INPUT);
    pinMode(US_RIGHT_ECHO,      INPUT);

    // ir in
    pinMode(IR_IN, INPUT);
}

void init_state() {
    state.elapsed_time = 0.0f;

    state.curr_inter = NO_INTERSECTION;
    state.curr_path  = NO_PATH;

    state.first_free_intersection = 0;
    state.first_free_path = 0;
}

void init_robot() {
    // pins
    robot.us_left.trigger       = US_LEFT_TRIGGER;
    robot.us_front.trigger      = US_FRONT_TRIGGER;
    robot.us_right.trigger      = US_RIGHT_TRIGGER;

    robot.us_left.echo          = US_LEFT_ECHO;
    robot.us_front.echo         = US_FRONT_ECHO;
    robot.us_right.echo         = US_RIGHT_ECHO;

    robot.motor_right.enable    = MOTOR_RIGHT_ENABLE;
    robot.motor_left.enable     = MOTOR_LEFT_ENABLE;

    robot.motor_right.in1       = MOTOR_RIGHT_IN1;
    robot.motor_right.in2       = MOTOR_RIGHT_IN2;
    robot.motor_left.in1        = MOTOR_LEFT_IN1;
    robot.motor_left.in2        = MOTOR_LEFT_IN2;

    robot.ir.pin                = IR_IN;

    // robot parameters
    robot.speed = 64;
    robot.dir = DIR_NORTH;
    robot.movement = MOVE_STILL;

    robot.motor_right.rot = ROT_ACW;
    robot.motor_left.rot = ROT_CW;
    robot.motor_right.running = true;
    robot.motor_left.running = true;
}

void setup() {
    Serial.begin(9600);

    init_pins();

    init_state();
    init_robot();
}

// PATHS & INTERSECTIONS
path_s* create_path() {
    u16 handle = state.first_free_path ++;
    path_s* path = &paths[handle];

    path->handle = handle;
    path->flags = PATH_NONE;
    path->intersections[0] = NO_INTERSECTION;
    path->intersections[1] = NO_INTERSECTION;

    path->length = 0.0f;
    path->width = 0.0f;
    path->bias = 0;

    return path;
}

intersection_s* create_intersection() {
    u16 handle = state.first_free_intersection ++;
    intersection_s* inter = &intersections[handle];
    inter->handle = handle;

    inter->paths[0] = NO_PATH;
    inter->paths[1] = NO_PATH;
    inter->paths[2] = NO_PATH;
    inter->paths[3] = NO_PATH;

    inter->flags = INTER_NONE;
    inter->age = 0;

    return inter;
}

#define MIN_PATH_DIST (8.0f)

void intersection_sense_path(intersection_s* inter, ultrasonic_s* sensor, cardinal_t dir) {
    if(inter->paths[dir] != NO_PATH) return;

    if(sensor->dist >= MIN_PATH_DIST) {
        path_s* path = create_path();
        inter->paths[dir] = path->handle;

        if(dir == DIR_NORTH || dir == DIR_SOUTH) path->flags |= PATH_ORIENTATION;
        else path->flags &= ~PATH_ORIENTATION;
    }
}

void intersection_sense_available_paths(intersection_s* inter) {
    intersection_sense_path(inter, &robot.us_left,  relative_to_cardinal(robot.dir, DIR_LEFT));
    intersection_sense_path(inter, &robot.us_front, relative_to_cardinal(robot.dir, DIR_FORWARD));
    intersection_sense_path(inter, &robot.us_right, relative_to_cardinal(robot.dir, DIR_RIGHT));
}

// SENSOR INTERFACES
void ultrasonic_update(ultrasonic_s* sensor) {
    // ok so i think i understood this wrong when i first wrote it
    // after some lookup, it seems the ultrasonic sends out pulses of 37kHz sound waves
    // while it is set to HIGH, so delays are necessary
    digitalWrite(sensor->trigger, HIGH);
    // according to sensor specifications this should be 10
    delayMicroseconds(10);

    digitalWrite(sensor->trigger, LOW);
    u32 duration = pulseIn(sensor->echo, HIGH);
    sensor->dist = duration * 0.0170145f;

    // keep the trigger at LOW for 1 microsecond
    // to ensure the next cycle is a clean high
    // from tested sources it takes 4us for it to fully go back to LOW
    // NOTE(anas): this *could* be removed, if we know that the rest of the program
    //             takes ~4us to execute but that would be risky
    delayMicroseconds(4);
}

void motor_update(motor_s* motor) {
    if(motor->rot == ROT_CW) {
        digitalWrite(motor->in1, HIGH);
        digitalWrite(motor->in2, LOW);
    } else {
        digitalWrite(motor->in1, LOW);
        digitalWrite(motor->in2, HIGH);
    }

    analogWrite(motor->enable, motor->running ? robot.speed : 0);
}

void infrared_update(ir_s* sensor) {
    sensor->val = digitalRead(sensor->pin);
}

// ROBOT STATE
void log_robot_state() {
    /*
    Serial.println(F("Ultrasonic:"));
    Serial.print(F("Left: "));
    Serial.println(robot.us_left.dist);
    Serial.print(F("Front: "));
    Serial.println(robot.us_front.dist);
    Serial.print(F("Right: "));
    Serial.println(robot.us_right.dist);
    Serial.println(F("-----------"));
    */

    /*
    Serial.println(F("Motor: "));
    Serial.print(F("Speed: "));
    Serial.println(robot.speed);
    Serial.print(F("Right: "));
    Serial.println(robot.motor_right.rot);
    Serial.print(F("Left: "));
    Serial.println(robot.motor_left.rot);
    Serial.println(F("-----------"));
    */

    /*
    Serial.print(F("IR: "));
    Serial.println(robot.ir.val);
    Serial.println(F("-----------"));
    */
}

void robot_update_motors() {
    motor_update(&robot.motor_right);
    motor_update(&robot.motor_left);
}

void robot_set_movement(movement_t move) {
    robot.motor_right.running = move != MOVE_STILL;
    robot.motor_left.running = move != MOVE_STILL;

    if(move == MOVE_FORWARD) {
        robot.motor_right.rot = ROT_ACW;
        robot.motor_left.rot = ROT_ACW;
    } else if(move == MOVE_BACK) {
        robot.motor_right.rot = ROT_CW;
        robot.motor_left.rot = ROT_CW;
    } else if(move == MOVE_TURN_RIGHT) {
        robot.motor_right.rot = ROT_CW;
        robot.motor_left.rot = ROT_ACW;
    } else if(move == MOVE_TURN_LEFT) {
        robot.motor_right.rot = ROT_ACW;
        robot.motor_left.rot = ROT_CW;
    }

    robot_update_motors();
}

// TODO(anas): measure this
#define ROT_RADIUS      (6.5f)
#define WHEEL_DIAMETER  (6.5f)

// angle in radians
f32 robot_get_turn_delay(f32 angle) {
    f32 velocity = (WHEEL_DIAMETER / 60.0f) * lerpf(95.0f, 175.0f, robot.speed / 255.0f);
    f32 distance = angle * ROT_RADIUS;
    return distance / velocity;
}

void robot_turn(movement_t move) {
    if(move != MOVE_TURN_RIGHT && move != MOVE_TURN_LEFT) return;
    robot.turning = true;
    robot_set_movement(move);
    delay(robot_get_turn_delay(90.0f));
    robot_set_movement(MOVE_STILL);
}

// EXECUTION LOOP
void loop() {
    // update the global state
    state.elapsed_time = micros();

    // update the sensor interfaces
    ultrasonic_update(&robot.us_left);
    ultrasonic_update(&robot.us_front);
    ultrasonic_update(&robot.us_right);
    robot_update_motors();
    infrared_update(&robot.ir);

    robot_turn(MOVE_TURN_RIGHT);

    // log_robot_state();
}
