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

// NOTE(anas): width of path is always 20cm

#include "stem.h"

// HELPER FUNCTIONS
f32 lerpf(f32 a, f32 b, f32 t) {
    return a + t * (b - a);
}

cardinal_t relative_to_cardinal(cardinal_t forward, dir_t dir) {
    return (forward + dir) % 4;
}

// GLOBAL VARIABLES
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
    robot.speed = 100;
    robot.rot_speed = 255;
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

    init_robot();
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

    if(!motor->running)
        analogWrite(motor->enable, 0);
    else if(robot.turning)
        analogWrite(motor->enable, robot.rot_speed);
    else
        analogWrite(motor->enable, robot.speed);
}

void infrared_update(ir_s* sensor) {
    sensor->val = digitalRead(sensor->pin);
}

// ROBOT STATE
void log_robot_state() {
    Serial.println(F("Ultrasonic:"));
    Serial.print(F("Left: "));
    Serial.println(robot.us_left.dist);
    Serial.print(F("Front: "));
    Serial.println(robot.us_front.dist);
    Serial.print(F("Right: "));
    Serial.println(robot.us_right.dist);
    Serial.println(F("-----------"));

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

void robot_turn(movement_t move, u8 turns) {
    if(move != MOVE_TURN_RIGHT && move != MOVE_TURN_LEFT) return;
    robot.turning = true;
    robot_set_movement(move);

    // after testing, we found 102ms to be the approximate time needed
    // for the robot to do a 90 degree turn at max speed
    // its not exact but its good enough
    delay(turns * 102.0f * 255.0f / robot.rot_speed);

    robot.turning = false;
    robot_set_movement(MOVE_STILL);
}

// LOGIC
boolean at_intersection() {
    return robot.us_front.dist <= MIN_TURN_DIST;
}

movement_t detect_robot_movement() {
    if(robot.us_right.dist >= MIN_WALL_DIST) return MOVE_TURN_RIGHT;
    else if(robot.us_left.dist >= MIN_WALL_DIST) return MOVE_TURN_LEFT;
    else return MOVE_STILL;
}

// EXECUTION LOOP
void loop() {
    // update the sensor interfaces
    ultrasonic_update(&robot.us_left);
    ultrasonic_update(&robot.us_front);
    ultrasonic_update(&robot.us_right);
    robot_update_motors();
    infrared_update(&robot.ir);

    robot_set_movement(MOVE_FORWARD);

    if(at_intersection()) {
        movement_t movement = detect_robot_movement();
        robot_turn(movement, 1);

        if(movement == MOVE_STILL) robot_set_movement(movement);
    }

    // log_robot_state();
    // delay(200);
}
