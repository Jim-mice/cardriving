#ifndef __BSP_MOTOR_L9110_H
#define __BSP_MOTOR_L9110_H

void motor_init(void);
void motor_left_set(int speed);
void motor_right_set(int speed);
void motor_stop(void);

#endif
