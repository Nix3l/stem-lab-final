#ifndef STEM_H
#define STEM_H

// MAZE RUNNERS
// hamza, zena, roya, anas

// MACROS
#define PI   (3.14159265358979323f)
#define PI_2 (1.570796327f)

#define MAX_INTERSECTIONS   (128)
#define MAX_PATHS           (128)

// pins
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
    u8 speed;

    cardinal_t dir;

    boolean turning;
    movement_t movement;
} robot_s;

// GLOBAL STATE
typedef enum {
    AT_INTERSECTION = 0,
    ALONG_PATH      = 1,
} task_t;

typedef struct {
    // NOTE(anas): will overflow after ~70 minutes
    //             but shouldnt be a problem
    u32 elapsed_time;

    task_t task;
    u16 curr_inter;
    u16 curr_path;

    u16 goal_inter;

    f32 last_us_left;
    f32 last_us_front;
    f32 last_us_right;

    u16 first_free_intersection;
    u16 first_free_path;
} state_s;

// ARENA
enum {
    INTER_NONE              = (0x00),
    INTER_FULLY_EXPLORED    = (0x01),
    INTER_UP_PATH           = (0x02),
    INTER_RIGHT_PATH        = (0x04),
    INTER_DOWN_PATH         = (0x08),
    INTER_LEFT_PATH         = (0x10),
};

#define NO_INTERSECTION (-1)

typedef struct {
    u16 handle; // index in intersections array
    u16 paths[4]; // indices go clockwise, north first
    u8 flags;
} intersection_s;

#define NO_PATH (-1)

enum {
    PATH_NONE           = (0x00),
    PATH_EXPLORED       = (0x01),
    PATH_ORIENTATION    = (0x02), // 1 is vertical, 0 is horizontal
    PATH_HAS_DEAD_END   = (0x04),
};

typedef struct {
    u16 handle; // index in paths array
    u16 intersections[2];
    u8 flags;
} path_s;

#endif
