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

// NOTE(anas): ALL time is in microseconds

// MACROS
#define MAX_INTERSECTIONS   (128)
#define MAX_PATHS           (128)

#define ULTRASONIC_DELAY    (5)

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

// TODO(anas): cardinal to relative and relative to cardinal dir

// pins
// NOTE(anas): right motor -> A
//             left  motor -> B
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

// sensor interaces
typedef struct {
    u16 enable;
    u16 in1;
    u16 in2;

    rotation_t rot;
} motor_s;

typedef struct {
    u16 trigger;
    u16 echo;

    u32 trigger_time;
    f32 dist;
} ultrasonic_s;

typedef struct {
    u16 pin;
    u8 val;
} ir_s;

// robot
typedef struct {
    ultrasonic_s left_us;
    ultrasonic_s front_us;
    ultrasonic_s right_us;

    ir_s ir;

    motor_s right_wheel;
    motor_s left_wheel;
    u8 speed;

    dir_t dir;
} robot_s;

// global state
typedef struct {
    u32 elapsed_time;

    u16 first_free_intersection;
    u16 first_free_path;
} state_s;

// arena
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
// NOTE(anas): this is stored in the flash memory as to not waste ram
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

state_s state = {0};
robot_s robot = {0};

// INITIALIZATION
void init_pins() {
    pinMode(pins.motor_right_enable, OUTPUT);
    pinMode(pins.motor_left_enable,  OUTPUT);

    pinMode(pins.motor_right_in1,    OUTPUT);
    pinMode(pins.motor_right_in2,    OUTPUT);
    pinMode(pins.motor_left_in1,     OUTPUT);
    pinMode(pins.motor_left_in2,     OUTPUT);

    pinMode(pins.us_left_trigger,    OUTPUT);
    pinMode(pins.us_left_echo,       INPUT);
    pinMode(pins.us_forward_trigger, OUTPUT);
    pinMode(pins.us_forward_echo,    INPUT);
    pinMode(pins.us_right_trigger,   OUTPUT);
    pinMode(pins.us_right_echo,      INPUT);

    pinMode(pins.ir_in, INPUT);
}

void init_state() {
    state.elapsed_time = 0.0f;

    state.first_free_intersection = 0;
    state.first_free_path = 0;
}

void init_robot() {
    robot.left_us.trigger = pins.us_left_trigger;
    robot.front_us.trigger = pins.us_forward_trigger;
    robot.right_us.trigger = pins.us_right_trigger;
    robot.left_us.echo = pins.us_left_echo;
    robot.front_us.echo = pins.us_forward_echo;
    robot.right_us.echo = pins.us_right_echo;
    robot.right_wheel.enable = pins.motor_right_enable;
    robot.left_wheel.enable = pins.motor_left_enable;
    robot.right_wheel.in1 = pins.motor_right_in1;
    robot.right_wheel.in2 = pins.motor_right_in2;
    robot.left_wheel.in1 = pins.motor_left_in1;
    robot.left_wheel.in2 = pins.motor_left_in2;
    robot.ir.pin = pins.ir_in;
    robot.speed = 255;
    robot.dir = DIR_LEFT;
    robot.right_wheel.rot = ROT_CW;
    robot.left_wheel.rot = ROT_ACW;
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

// SENSOR INTERFACES
void ultrasonic_update(ultrasonic_s* sensor) {
    if(state.elapsed_time >= sensor->trigger_time + ULTRASONIC_DELAY) {
        digitalWrite(sensor->trigger, LOW);
        u32 duration = pulseIn(sensor->echo, HIGH);
        sensor->dist = duration * 0.034f / 2.0f;

        digitalWrite(sensor->trigger, HIGH);
        sensor->trigger_time = state.elapsed_time;
    }
}

void motor_set_direction(motor_s* motor) {
    if(motor->rot == ROT_CW) {
        digitalWrite(motor->in1, HIGH);
        digitalWrite(motor->in2, LOW);
    } else {
        digitalWrite(motor->in1, LOW);
        digitalWrite(motor->in2, HIGH);  
    }
}

void motor_set_speed(motor_s* motor) {
    // TODO(anas): convert motor speed to RPM
    analogWrite(motor->enable, robot.speed);
}

void infrared_get(ir_s* sensor) {
    sensor->val = digitalRead(sensor->pin);
}

void log_robot_state() {
    Serial.println(F("Ultrasonic:"));
    Serial.print(F("Left: "));
    Serial.println(robot.left_us.dist);
    Serial.print(F("Front: "));
    Serial.println(robot.front_us.dist);
    Serial.print(F("Right: "));
    Serial.println(robot.right_us.dist);
    Serial.println(F("-----------"));
    Serial.println(F("Motor: "));
    Serial.print(F("Speed: "));
    Serial.println(robot.speed);
    Serial.print(F("Right"));
    Serial.println(robot.right_wheel.rot);
    Serial.print(F("Left"));
    Serial.println(robot.left_wheel.rot);
    Serial.println(F("-----------"));
    Serial.print(F("IR: "));
    Serial.println(robot.ir.val);
    Serial.println(F("-----------"));
}

// EXECUTION LOOP
void loop() {
    // update the global state
    state.elapsed_time = micros();

    // update the sensor interfaces
    ultrasonic_update(&robot.left_us);
    ultrasonic_update(&robot.front_us);
    ultrasonic_update(&robot.right_us);
    motor_set_direction(&robot.left_wheel);
    motor_set_direction(&robot.right_wheel);
    infrared_get(&robot.ir);

    log_robot_state();
}
