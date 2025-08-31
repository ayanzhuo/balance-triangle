#include <Arduino.h>
#include <SimpleFOC.h>

extern float LQR_K3_1;
extern float LQR_K3_2;
extern float LQR_K3_3;
extern float LQR_K4_1;
extern float LQR_K4_2;
extern float LQR_K4_3;

extern float v_p_1;
extern float v_i_1;
extern float v_p_2;
extern float v_i_2;

extern float target_angle;
extern float swing_up_voltage;
extern float swing_up_angle;

extern float err_angle;
extern bool blance_swingup;

extern MotionControlType mode; // torque or velocity
extern float motion_target;    // voltage if torque, velocity if velocity
extern bool isStable;          // 当前是否稳态，用于外部切换PI参数
void blance_compute();
void swingup_compute();
void move_update();
void testmode();