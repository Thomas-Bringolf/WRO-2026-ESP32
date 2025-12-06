#ifndef MOTOR_H
#define MOTOR_H

#include <string.h>
#include <stdbool.h>

typedef struct {
    int pwm_pin;         // GPIO for PWM signal
    int dir_pin;         // GPIO for direction
    float current_speed; // [-1.0 .. 1.0]
    float step_size;     // increment per ramp step
    int step_delay_ms;   // delay per step in ms
} Motor;

typedef struct {
    int pwm_pin;         // GPIO for PWM signal
    int angle_range;       // maximum angle in degrees
    int center_duty;  // center offset in degrees
    int min_duty;
    int max_duty;
} Servo;


// Initialize motor hardware
void motor_init(Motor *m);

// Initialize servo hardware
void servo_init(Servo *s);

// Set target speed (-1.0 .. 1.0), ramps automatically
void motor_set_speed(Motor *m, float target_speed, bool debug);

// Set target angle (-max_angle .. max_angle)
void servo_sets_angle(Servo *s, int angle_deg);

// Stop motor immediately (no ramp down)
void motor_stop(Motor *m);

// Enable motors via relay
void motors_enable(int PWRrelaiPin);

// Disable motors via relay
void motors_disable(int PWRrelaiPin);

#endif // MOTOR_H