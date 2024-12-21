// MAZE RUNNERS
// hamza, zena, roya, anas

// USEFUL RESOURCES: -------
//  - https://docs.arduino.cc/learn/programming/memory-guide/
//  - https://docs.arduino.cc/language-reference/en/functions/time/micros/
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

// MACROS
#define MAX_INTERSECTIONS   (128)
#define MAX_PATHS           (128)

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
} rotation_s;

typedef enum {
    DIR_NORTH   = 0,
    DIR_EAST    = 1,
    DIR_SOUTH   = 2,
    DIR_WEST    = 3,
} cardinal_dir_s;

typedef enum {
    DIR_FORWARD = 0,
    DIR_RIGHT   = 1,
    DIR_BACK    = 2,
    DIR_LEFT    = 3,
} dir_s;

typedef struct {
    u8 motor_right_enable;
    u8 motor_left_enable;

    u8 motor_right_in1;
    u8 motor_right_in2;
    u8 motor_left_in1;
    u8 motor_left_in2;

    u8 us_left_trigger;
    u8 us_left_echo;
    u8 us_forward_trigger;
    u8 us_forward_echo;
    u8 us_right_trigger;
    u8 us_right_echo;

    u8 ir_in;
} pins_s;

typedef struct {
    u16 enable_pin;

    u16 input1;
    u16 input2;
} motor_s;

typedef struct {
    u16 trigger;
    u16 echo;

    f32 val;
} ultrasonic_s;

typedef struct {
    u16 pin;

    f32 val;
} ir_s;

typedef struct {
    ultrasonic_s left_us;
    ultrasonic_s front_us;
    ultrasonic_s right_us;

    ir_s ir;

    motor_s right_wheel;
    motor_s left_wheel;
    u8 speed;
} robot_s;

typedef struct {
    u32 elapsed_time;

    u16 first_free_intersection;
    u16 first_free_path;
} state_s;

typedef struct {
    u16 handle;
    u16 paths[4];
    u8 flags;

    u8 age;
} intersection_s;

typedef struct {
    u16 handle;
    u16 intersections[2];
    u8 flags;

    f32 length;
    f32 width;

    u8 bias;
} path_s;

// GLOBAL VARIABLES
const PROGMEM pins_s pins = (pins_s) {
    .motor_right_enable     = 0,
    .motor_left_enable      = 1,

    .motor_right_in1        = 5,
    .motor_right_in2        = 6,
    .motor_left_in1         = 3,
    .motor_left_in2         = 4,

    .us_left_trigger        = 13,
    .us_left_echo           = 12,
    .us_forward_trigger     = 11,
    .us_forward_echo        = 10,
    .us_right_trigger       = 9,
    .us_right_echo          = 8,

    .ir_in                  = 7,
};

path_s          paths[MAX_PATHS] = {0};
intersection_s  intersections[MAX_INTERSECTIONS] = {0};

state_s state;
robot_s robot;

// INITIALIZATION
void init_pins() {
    pinMode(pins.motor_right_enable, OUTPUT);
    pinMode(pins.motor_left_enable,  OUTPUT);

    pinMode(pins.motor_right_in1,    OUTPUT);
    pinMode(pins.motor_right_in2,    OUTPUT);
    pinMode(pins.motor_left_in1,     OUTPUT);
    pinMode(pins.motor_left_in2,     OUTPUT);

    pinMode(pins.us_left_trigger,    OUTPUT);
    pinMode(pins.us_left_echo,       OUTPUT);
    pinMode(pins.us_forward_trigger, OUTPUT);
    pinMode(pins.us_forward_echo,    OUTPUT);
    pinMode(pins.us_right_trigger,   OUTPUT);
    pinMode(pins.us_right_echo,      OUTPUT);

    pinMode(pins.ir_in, INPUT);
}

void init_state() {
    state.elapsed_time = 0.0f;

    state.first_free_intersection = 0;
    state.first_free_path = 0;
}

void init_robot() {

}

void setup() {
    Serial.begin(9600);

    init_pins();

    init_state();
    init_robot();
}

// PATHS & INTERSECTIONS
path_s* create_path() {
    u16 handle = state.first_free_path;
    path_s* path = &paths[handle];
    path->handle = handle;
    // TODO(anas): init path
    return path;
}

intersection_s* create_intersection() {
    u16 handle = state.first_free_intersection ++;
    intersection_s* inter = &intersections[handle];
    inter->handle = handle;
    // TODO(anas): init intersection
    return inter;
}

// EXECUTION LOOP
void loop() {

}
