#ifndef STEM_H
#define STEM_H

// MAZE RUNNERS
// hamza, zena, roya, anas

// NOTE: the reason that i had to make a separate header file
//       is because of the stupid decisions made by the arduino IDE devs
//       they (for some reason) generate function prototypes
//       before sending the code to the compiler
//       and ALL function prototypes are generated above the first declared function,
//       so if you had a function declared above a typedef or macro,
//       then the compiler will complain about the type/macro not being defined
//       thank you arduino IDE devs very cool

// MACROS
#define MIN_TURN_DIST (6.0f)
#define MIN_WALL_DIST (6.0f)

// PINS
// NOTE(anas): right motor -> A
//             left  motor -> B
#define MOTOR_RIGHT_ENABLE  (7)
#define MOTOR_LEFT_ENABLE   (2)

#define MOTOR_RIGHT_IN1     (5)
#define MOTOR_RIGHT_IN2     (6)
#define MOTOR_LEFT_IN1      (4)
#define MOTOR_LEFT_IN2      (3)

#define US_LEFT_TRIGGER     (13)
#define US_LEFT_ECHO        (12)

#define US_FRONT_TRIGGER    (11)
#define US_FRONT_ECHO       (10)

#define US_RIGHT_TRIGGER    (9)
#define US_RIGHT_ECHO       (8)

#define IR_IN               (22)

// DATA TYPES
typedef char i8;
typedef unsigned char u8;
typedef int i16;
typedef unsigned long u32;
typedef long i32;
typedef float f32;

typedef enum {
    ROT_CW  = 0,
    ROT_ACW = 1,
} rotation_t;

// NOTE(anas): directions are defined in clockwise order
//             which lets us do some tricks with for loops
//             and arithmetic that makes handling rotations easier
typedef enum {
    DIR_FORWARD = 0,
    DIR_RIGHT   = 1,
    DIR_BACK    = 2,
    DIR_LEFT    = 3,
} dir_t;

typedef enum {
    DIR_NORTH   = 0,
    DIR_EAST    = 1,
    DIR_SOUTH   = 2,
    DIR_WEST    = 3,
} cardinal_t;

typedef enum {
    MOVE_STILL      = 0,
    MOVE_FORWARD    = 1,
    MOVE_BACK       = 2,
    MOVE_TURN_RIGHT = 3,
    MOVE_TURN_LEFT  = 4,
} movement_t;

// SENSOR INTERFACES
typedef struct {
    u16 enable;
    u16 in1;
    u16 in2;

    boolean running;
    rotation_t rot;
} motor_s;

typedef struct {
    u16 trigger;
    u16 echo;

    f32 dist;
} ultrasonic_s;

typedef struct {
    u16 pin;
    u8 val; // 0 when white object
            // 1 when black object
} ir_s;

// ROBOT
typedef struct {
    ultrasonic_s us_left;
    ultrasonic_s us_front;
    ultrasonic_s us_right;

    ir_s ir;

    motor_s motor_right;
    motor_s motor_left;

    u8 speed;     // speed when moving forward/backward
    u8 rot_speed; // speed when turning

    cardinal_t dir;

    boolean turning;
    movement_t movement;
} robot_s;

#endif
