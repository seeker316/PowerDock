#include <stdio.h>

class PIDController {
public:

float Kp, Ki, Kd, output_min, output_max;

float prev_error = 0, derivative = 0, output = 0;
float dt = 0.1;

PIDController(float Kp, float Ki, float Kd, float output_min, float output_max){

    Kp = Kp;
    Ki = Ki;
    Kd = Kd;
    output_min = output_min;
    output_max = output_max; 
};

float PID_Output(float error){

    derivative = error - prev_error;

    output = ((Kp * error) + (Kd * derivative)) + 1500;
    
    prev_error = error;

    // Apply Ceiling
    if (output < output_min) {output = output_min;}
    else if (output > output_max) {output = output_max;}

    return output;
};

};

int main() {

// Initialize PID Controllers with new ranges
PIDController roll_pid(0.3, 0.01, 0.3, 1400, 1600);
PIDController pitch_pid(0.5, 0.01, 0.5, 1400, 1600);
PIDController yaw_pid(1.5, 0.02, 0.7, 1200, 1800);
PIDController throttle_pid(1.5, 0.01, 0.5, 1300, 1800);

float error_x = 0, error_y = 0, error_yaw = 0, error_throttle = 0;
float roll_error, pitch_error, yaw_error, throttle_error;

roll_error = roll_pid.PID_Output(error_x);
pitch_error = pitch_pid.PID_Output(error_y);
yaw_error = yaw_pid.PID_Output(error_yaw);
throttle_error = throttle_pid.PID_Output(error_throttle);

printf("Roll error = %d\n", roll_error);
}