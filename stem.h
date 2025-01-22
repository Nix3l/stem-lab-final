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
#define LERP(_a, _b, _t) ((_a) + t * ((_b) - (_a)))
#define CLAMP(_x, _min, _max) ((_x) <= (_min) ? (_min) : (_x) >= (_max) ? (_max) : (_x))

#define MIN_FRONT_TURN_DIST   (14.5f)
#define MIN_TURN_WALL_DIST    (14.0f)
#define ADJUST_WALL_DIST      (8.0f)

#define TURN_DELAY            (90)

// PINS
// NOTE(anas): right motor -> A
//             left  motor -> B
#define MOTOR_RIGHT_ENABLE  (6)
#define MOTOR_LEFT_ENABLE   (3)

#define MOTOR_RIGHT_IN1     (7)
#define MOTOR_RIGHT_IN2     (5)
#define MOTOR_LEFT_IN1      (4)
#define MOTOR_LEFT_IN2      (2)

#define US_LEFT_TRIGGER     (13)
#define US_LEFT_ECHO        (12)

#define US_FRONT_TRIGGER    (11)
#define US_FRONT_ECHO       (10)

#define US_RIGHT_TRIGGER    (9)
#define US_RIGHT_ECHO       (8)

#define MOTOR_K             (60)

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
    u8 speed;
    rotation_t rot;
} motor_s;

typedef struct {
    u16 trigger;
    u16 echo;

    f32 dist;
} ultrasonic_s;

// ROBOT
typedef struct {
    ultrasonic_s us_left;
    ultrasonic_s us_front;
    ultrasonic_s us_right;

    motor_s motor_right;
    motor_s motor_left;

    u8 base_speed;
    u8 slow_speed;
    u8 max_adjust_speed;
    u8 min_adjust_speed;
    u8 parent_rot_speed;
    u8 child_rot_speed;

    boolean turning;
    movement_t movement;
} robot_s;

#endif