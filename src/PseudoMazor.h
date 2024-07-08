#include <Arduino.h>

//pin definitions: long names but I am trying to improve bad naming habits for readability
#define right_motor_pwm_pin 2
#define right_motor_negative_pin 49 //PL0
#define right_motor_positive_pin 48 //PL1
#define right_motor_encoder_pin 18

#define left_motor_pwm_pin 3
#define left_motor_negative_pin 47 //PL2 
#define left_motor_positive_pin 46 //PL3
#define left_motor_encoder_pin 19

#define motor_standby_pin 50

#define color_sensor_s0_pin 45 //PL4
#define color_sensor_s1_pin 44 //PL5
#define color_sensor_s2_pin 43 //PL6
#define color_sensor_s3_pin 42 //PL7
#define color_sensor_output_pin 4

#define right_ultrasonic_sensor_trigger_pin 34
#define right_ultrasonic_sensor_echo_pin 35

#define left_ultrasonic_sensor_trigger_pin 32
#define left_ultrasonic_sensor_echo_pin 33

#define center_ultrasonic_sensor_trigger_pin 30
#define center_ultrasonic_sensor_echo_pin 31

#define Buzzer 51

#define line_follower_sensor_0 A0 //4 is left, 0 is right 
#define line_follower_sensor_1 A1
#define line_follower_sensor_2 A2
#define line_follower_sensor_3 A3
#define line_follower_sensor_4 A4
#define Line_follower_sensor_5 A5
#define VBat_pin A6

#define R_LED 13 //RGB LED stuff (Has PWM)
#define G_LED 5
#define B_LED 6

// #define using_TFT 1
#define using_OLED 1

#ifdef using_OLED
#define LCD_SDA_pin
#define LCD_SCL_pin
#elif using_TFT
#define TFT_SCL_pin
#define TFT_SDA_pin
#define TFT_RST_pin
#define TFT_DC_pin
#define TFT_CS_pin
#endif

#define left_button_pin
#define right_button_pin
#define middle_button_pin
#define red_LED_pin 

#define MAX_DISTANCE 60 //60cm



void getWallDistances(); //retrieves the distances 
void getIntersectionFromDistance();

void getLinePosition(); //retrieves the line position in the form of a byte 
void getIntersectionFromLine();
void tuneLinePID();

void getSpeed(); //retrives the speed of both motors
void tuneMotorSpeed();

void moveDistance();
void moveToNextBlock();
void optimizePath();
void floodFill();

//motor defs
#define dir_forward 0
#define dir_U_left  1
#define dir_U_right 2 //turns in place (U shaped demi-tour stuff)
#define dir_C_left  3 //pivots around one of the wheels (good for C turns), mess with this stuff to gain time or something
#define dir_C_right 4
#define dir_reverse 5
#define dir_break   6 //ki tkon mat3rfch tfrini
#define dir_stop    7

#define robot_width  173 //in mm
#define robot_length 166
#define block_width  300
#define block_length 300
#define ultrasonic_mount_width 75
#define wheel_diameter 66 //in mm
#define encoder_resolution 11 //4 magnets per wheel for now
#define encoder_unit (wheel_diameter*3.14/encoder_resolution) //for a resolution of 4 we get 16.5mm per count 
#define encoder_counts_to_distance(counts) (encoder_unit*counts) //we could add the count in the ISR directly but eh, I wanna keep it as simple as just incrimenting ++ style so the atmega isn't blown like it naturally is ffs
#define distance_to_encoder_counts(distance) (distance/encoder_unit) //however, I'll see if this takes a serious toll on the µcontroller speed, if it does, it doesn't differ to put everything in the ISR bruh

#define max_speed  255