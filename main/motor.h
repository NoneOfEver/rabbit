#ifndef MAIN_MOTOR_H_
#define MAIN_MOTOR_H_

void motor_init();
void set_motor_target(float target);
void control_task(void *arg);
#endif // !MAIN_MOTOR_H_