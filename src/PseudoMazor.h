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

#define R_LED 6 //RGB LED stuff (Has PWM)
#define G_LED 4
#define B_LED 5

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

#define left_button_pin A13
#define right_button_pin A14
#define middle_button_pin A15
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
#define angle_to_encoder_counts(angle) ( (((angle*2*3.14)/360)*75)/encoder_unit)

#define max_speed  255


const unsigned char Mhabsi_ASCE [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0x98, 0x06, 0x08, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xf0, 0xaf, 0x04, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0x01, 0xe9, 0x2d, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xa0, 0x50, 0xd1, 0x80, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xc1, 0x80, 0x01, 0x80, 0x00, 0x07, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0x8e, 0xff, 0xc1, 0x81, 0x83, 0xc0, 0x00, 0x07, 0xff, 0xff, 0xff, 
	0x86, 0x1f, 0xff, 0xff, 0xf1, 0x8e, 0x0f, 0x80, 0x50, 0x81, 0x80, 0x00, 0x02, 0xff, 0xff, 0xf8, 
	0xbd, 0xff, 0xff, 0xff, 0xc1, 0xd7, 0x0f, 0x80, 0xc0, 0x40, 0x80, 0x00, 0x00, 0x7f, 0xfe, 0x70, 
	0xed, 0x7c, 0x00, 0x00, 0x00, 0x6f, 0x1f, 0x80, 0x33, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xed, 0x7f, 0xff, 0x83, 0x81, 0x8f, 0x9c, 0x00, 0x38, 0x00, 0x00, 0x20, 0x00, 0x7c, 0x3e, 0x63, 
	0xfe, 0x3f, 0x07, 0x03, 0x87, 0x83, 0xcc, 0x00, 0x00, 0x04, 0x08, 0x00, 0x00, 0x7c, 0x03, 0xe7, 
	0xfe, 0x3e, 0x07, 0x03, 0x8f, 0xc7, 0x18, 0x00, 0x00, 0xfb, 0x08, 0x00, 0x00, 0x3c, 0x81, 0xc7, 
	0xfc, 0x3c, 0x06, 0x03, 0x8f, 0x8f, 0x1e, 0x40, 0x01, 0xbc, 0x00, 0x00, 0x00, 0x3f, 0xf1, 0xcf, 
	0xfc, 0x1c, 0x1e, 0x1f, 0x80, 0xbf, 0xee, 0x00, 0x07, 0x9f, 0x80, 0x00, 0x00, 0x3f, 0xe3, 0xcf, 
	0xf8, 0x18, 0x7e, 0x3f, 0x80, 0xef, 0xf6, 0x00, 0x3f, 0xdd, 0x00, 0x00, 0x00, 0x3f, 0xc4, 0xcf, 
	0xf8, 0x08, 0x00, 0x20, 0x00, 0x07, 0xf2, 0x20, 0x7f, 0xdf, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xf8, 0x88, 0x7c, 0x3f, 0x81, 0xc3, 0xfe, 0x41, 0xff, 0xff, 0x40, 0x00, 0x00, 0x3f, 0x0c, 0xc7, 
	0xf0, 0x88, 0x0c, 0x3f, 0x83, 0xdf, 0xf9, 0xc1, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x3e, 0x3f, 0xe3, 
	0xf0, 0x0c, 0x00, 0x7f, 0x88, 0xff, 0xff, 0x81, 0xff, 0xfc, 0xe0, 0x00, 0x00, 0x3c, 0x3f, 0xe0, 
	0xe0, 0x06, 0x00, 0x71, 0x80, 0xef, 0xff, 0x01, 0xbf, 0x30, 0xf0, 0x00, 0x00, 0x3c, 0x7f, 0xf0, 
	0x40, 0x07, 0x80, 0x01, 0x80, 0xc7, 0xfd, 0xc0, 0x7f, 0x30, 0x78, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xc1, 0x80, 0x04, 0x01, 0x80, 0xff, 0xb0, 0x40, 0x7f, 0xf0, 0x14, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0x41, 0x80, 0x04, 0x03, 0x81, 0xfa, 0x18, 0x00, 0x7f, 0xfb, 0x04, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xc3, 0x80, 0x00, 0x00, 0x00, 0x00, 0x10, 0x90, 0x7f, 0xfd, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x83, 0xc3, 0xff, 0xff, 0xff, 0x90, 0x28, 0x10, 0x7f, 0xff, 0x80, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0x04, 0xc3, 0xff, 0xe7, 0xff, 0x81, 0x14, 0x18, 0x0c, 0x00, 0xa0, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0x87, 0xe3, 0xff, 0xe7, 0xff, 0x99, 0xf6, 0x4c, 0x20, 0x61, 0x08, 0x00, 0x03, 0xff, 0xff, 0xff, 
	0xff, 0xe3, 0xff, 0xe7, 0xff, 0xfd, 0x80, 0x34, 0x05, 0xff, 0x80, 0x00, 0x02, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x8f, 0xc7, 0xff, 0xfb, 0xc4, 0xb8, 0x8e, 0xf1, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x8f, 0xcf, 0xff, 0xe2, 0x03, 0x78, 0x5c, 0xc3, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0x87, 0xcf, 0xff, 0xe7, 0x73, 0xfc, 0xf3, 0xb3, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 
	0xe0, 0x00, 0x00, 0x00, 0x00, 0x61, 0x53, 0xfd, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xe0, 0x03, 0x87, 0xcf, 0xff, 0xbd, 0xe3, 0xce, 0x7f, 0xff, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 
	0xe6, 0x3f, 0x83, 0x8f, 0xff, 0xfe, 0x21, 0xf7, 0x1f, 0xfe, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xfe, 0x3f, 0xc3, 0x9f, 0xff, 0xff, 0x48, 0x25, 0xf3, 0xfc, 0x10, 0x00, 0x00, 0x3f, 0xff, 0xff, 
	0xfe, 0x3f, 0xc1, 0x9f, 0xff, 0xff, 0x30, 0x2d, 0xa6, 0xc0, 0x30, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xd6, 0x1f, 0xc1, 0x1f, 0xff, 0xfc, 0x64, 0xaf, 0xbe, 0x00, 0x30, 0x40, 0x00, 0x7f, 0xff, 0xff, 
	0xbc, 0x1f, 0xc8, 0x3f, 0xff, 0xf9, 0xf0, 0x7e, 0x90, 0x80, 0xf0, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0x8c, 0x1f, 0xc8, 0x3f, 0xff, 0xe0, 0x30, 0x5f, 0xb4, 0x01, 0xf0, 0x00, 0x00, 0x07, 0xff, 0xff, 
	0xfe, 0x1f, 0xcc, 0x7c, 0x7f, 0x80, 0x20, 0xff, 0xe7, 0x21, 0xf0, 0xc0, 0x00, 0x00, 0xff, 0xff, 
	0xff, 0x9f, 0xcf, 0xf8, 0x7e, 0x00, 0x00, 0x4f, 0xf0, 0x03, 0xf0, 0x40, 0x00, 0x00, 0x3f, 0xff, 
	0xff, 0x90, 0x4f, 0xf8, 0x70, 0x00, 0x02, 0x5f, 0xa0, 0x87, 0xf0, 0x20, 0x00, 0x00, 0x3f, 0xff, 
	0xfc, 0x00, 0x4f, 0x00, 0x40, 0x00, 0x00, 0xc7, 0xe0, 0x13, 0xe0, 0x00, 0x00, 0x00, 0x0f, 0xff, 
	0xc0, 0x06, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x1d, 0xe0, 0x10, 0x00, 0x00, 0x0f, 0xff, 
	0xc1, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x1f, 0xff, 
	0xcf, 0xff, 0xfc, 0x63, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x7f, 0xe0, 0x00, 0x00, 0x00, 0x0f, 0xff, 
	0xff, 0xff, 0xfc, 0x63, 0x00, 0x00, 0x00, 0x09, 0xfc, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0xff, 0xff, 0xfc, 0x62, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0x5c, 0x9f, 0xff, 0xe6, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0x5b, 0x9f, 0xff, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0xfb, 0xff, 0xff, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 
	0xfb, 0xfc, 0xff, 0x8e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x3b, 0xfc, 0xf3, 0x8e, 0x00, 0x00, 0x00, 0x18, 0x46, 0x30, 0x9c, 0x00, 0x00, 0x00, 0x02, 0x01, 
	0xfc, 0xe4, 0x73, 0x0e, 0x00, 0x00, 0x00, 0x35, 0xcf, 0x4b, 0x96, 0x00, 0x00, 0x00, 0x03, 0xff, 
	0xff, 0xe0, 0x70, 0x1e, 0x00, 0x00, 0x00, 0x05, 0x49, 0x53, 0x92, 0x00, 0x00, 0x00, 0x01, 0xff, 
	0xfe, 0x20, 0x60, 0x1e, 0x00, 0x00, 0x00, 0x0c, 0x49, 0x71, 0x96, 0x00, 0x00, 0x00, 0x01, 0xff, 
	0xfe, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x10, 0x49, 0x49, 0x82, 0x00, 0x00, 0x00, 0x01, 0xff, 
	0xfe, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x3c, 0x4f, 0x59, 0x94, 0x00, 0x00, 0x00, 0x00, 0xff, 
	0xff, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x3c, 0x46, 0x71, 0x9c, 0x00, 0x00, 0x00, 0x00, 0xff, 
	0xff, 0x04, 0x00, 0x04, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 
	0xff, 0x07, 0xff, 0x3c, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0xff, 
	0x00, 0x1f, 0xff, 0x3c, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0xff, 
	0x00, 0x7f, 0xff, 0x3c, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x3d, 0xfe, 0x00, 0x00, 0x7f, 
	0xff, 0xff, 0xff, 0xfc, 0x00, 0xff, 0xdf, 0x01, 0x80, 0x03, 0x80, 0x7b, 0xff, 0x00, 0x00, 0x7f
};

const unsigned char Mhabsi_BILAL [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xe0, 0x3f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xe1, 0xfc, 0x3f, 0xff, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xc7, 0xfc, 0x3f, 0xfe, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xfc, 0x0f, 0x87, 0xdf, 0xfe, 0xff, 0xfe, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xfe, 0x0e, 0x07, 0xe0, 0xff, 0xff, 0xfe, 0x19, 0xf1, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0x0c, 0xe3, 0xf9, 0xff, 0xff, 0xfe, 0x10, 0x71, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0x8f, 0xc3, 0xff, 0xff, 0xff, 0xfe, 0x30, 0x31, 0xff, 
	0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0x8e, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x01, 0xff, 
	0xff, 0xff, 0xe3, 0xfc, 0x7f, 0xff, 0xff, 0xde, 0x7f, 0xff, 0xff, 0xff, 0xff, 0x38, 0x31, 0xff, 
	0xff, 0xff, 0xe1, 0xfc, 0x40, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x11, 0xc1, 
	0xff, 0xe3, 0xe1, 0x8c, 0x40, 0x3f, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x11, 0xc1, 
	0xff, 0xe2, 0x31, 0x88, 0x40, 0x07, 0x81, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x01, 0xc1, 
	0xff, 0xe2, 0x31, 0x88, 0x78, 0x01, 0x01, 0xff, 0xff, 0xdf, 0xfd, 0xfe, 0x0f, 0x00, 0x00, 0x43, 
	0xff, 0xe2, 0x31, 0x88, 0xfe, 0x00, 0x01, 0xff, 0xf9, 0xdf, 0xfd, 0xfe, 0x00, 0x00, 0x00, 0x03, 
	0x3f, 0xe0, 0x31, 0x88, 0xfe, 0x00, 0x01, 0xff, 0xf8, 0xff, 0xfc, 0xfe, 0x00, 0x00, 0x00, 0x03, 
	0x36, 0x00, 0x10, 0x08, 0x00, 0x00, 0x01, 0xff, 0xf8, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x3e, 0x00, 0x10, 0x08, 0x00, 0x00, 0x01, 0xf7, 0xf9, 0xfe, 0xfc, 0x7e, 0x00, 0x00, 0x00, 0x03, 
	0x3e, 0x00, 0x00, 0x08, 0x00, 0x60, 0x11, 0xf3, 0xff, 0xf8, 0xfc, 0x3f, 0x80, 0x1f, 0xff, 0xff, 
	0x3e, 0x00, 0x00, 0x38, 0x01, 0xe1, 0x01, 0xfb, 0xe6, 0x1a, 0xf8, 0x1f, 0x80, 0x00, 0x07, 0xff, 
	0x3e, 0x1f, 0x03, 0xfe, 0x1f, 0xff, 0x01, 0xfb, 0xc1, 0xff, 0xf0, 0x0f, 0xa0, 0x00, 0x0f, 0x8f, 
	0x06, 0x1f, 0x83, 0xff, 0xff, 0xff, 0x81, 0xfb, 0xc3, 0xff, 0x80, 0x07, 0xf0, 0x01, 0xff, 0x8f, 
	0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x81, 0xf9, 0x1f, 0x9f, 0x80, 0x01, 0xf0, 0x00, 0xff, 0x8f, 
	0x00, 0x0f, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xf9, 0x3e, 0x1f, 0x00, 0x00, 0x30, 0x01, 0xff, 0x8f, 
	0xe0, 0x0f, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 
	0x84, 0x1f, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xfe, 0x04, 0x00, 0x00, 0x00, 0x03, 0xff, 
	0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xb8, 0x03, 0xf8, 0x0e, 0x00, 0x00, 0x00, 0x1f, 0xff, 
	0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x79, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x63, 0xff, 
	0xc0, 0x3f, 0xfc, 0xff, 0xff, 0xff, 0xf0, 0xf9, 0x80, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xf0, 0x3f, 0xf8, 0xff, 0xff, 0xfe, 0x09, 0xf9, 0xc0, 0x00, 0x3e, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0xfc, 0x3f, 0xf8, 0xff, 0xff, 0xff, 0x8b, 0xf9, 0xe0, 0x00, 0x3e, 0x80, 0x00, 0x00, 0x3f, 0xff, 
	0xff, 0xff, 0xf8, 0xff, 0xff, 0xfe, 0x13, 0xf8, 0xf0, 0x00, 0x7e, 0x80, 0x00, 0x00, 0x07, 0xff, 
	0xff, 0xff, 0xf0, 0xff, 0xff, 0xfc, 0x23, 0xf8, 0x78, 0x00, 0xff, 0xd2, 0x00, 0x00, 0x03, 0xff, 
	0xff, 0xff, 0xf0, 0x3f, 0xff, 0xc4, 0x27, 0xe0, 0x7f, 0xbf, 0xff, 0xc2, 0x00, 0x00, 0x01, 0xff, 
	0xff, 0xff, 0xe2, 0x3f, 0xff, 0xa8, 0x47, 0x70, 0x7f, 0xff, 0xfe, 0x5a, 0x00, 0x00, 0x01, 0xff, 
	0xff, 0xff, 0xc2, 0x3f, 0xfe, 0xd9, 0xcf, 0xe0, 0x7f, 0xff, 0xff, 0x5a, 0x00, 0x00, 0x00, 0xff, 
	0xc7, 0x1f, 0xc6, 0x3f, 0xfc, 0xd9, 0x8f, 0xc0, 0x7f, 0xff, 0xfe, 0x4b, 0x00, 0x00, 0x00, 0x7f, 
	0xc3, 0x00, 0x8e, 0x31, 0xf3, 0xf1, 0x9f, 0x80, 0x3f, 0xff, 0xfe, 0x4b, 0x00, 0x08, 0x00, 0x7f, 
	0xc3, 0x00, 0x8e, 0x01, 0xef, 0xf1, 0x9f, 0x00, 0x1f, 0xff, 0xfe, 0x4a, 0x00, 0x08, 0x00, 0x3f, 
	0xc0, 0x00, 0x8c, 0x00, 0xd7, 0xf1, 0x00, 0x00, 0x1f, 0xff, 0xfe, 0x7e, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0xd7, 0xf3, 0x80, 0x7a, 0x0f, 0xff, 0xfe, 0x17, 0x00, 0x00, 0x00, 0x00, 
	0xe0, 0x0f, 0xf8, 0x07, 0xff, 0xd3, 0x8f, 0xf2, 0x0f, 0xff, 0xfe, 0x17, 0x00, 0x00, 0x00, 0x07, 
	0x80, 0x07, 0xf0, 0x0f, 0xff, 0xf7, 0xb7, 0xfe, 0x07, 0xff, 0xfc, 0x17, 0x00, 0x00, 0x00, 0x07, 
	0x00, 0x81, 0xe0, 0x1c, 0x7d, 0xff, 0xbf, 0xfe, 0x1f, 0xff, 0xfe, 0x3e, 0x00, 0x00, 0x00, 0x02, 
	0x00, 0x41, 0xe0, 0x7c, 0x3c, 0xb0, 0x00, 0x00, 0x0f, 0xff, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x18, 0x01, 0xe1, 0xfc, 0x1c, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
	0x18, 0x01, 0xe1, 0xfa, 0x1c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
	0x00, 0x03, 0xe3, 0xfa, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
	0x00, 0x63, 0xff, 0xf4, 0x0c, 0x10, 0x18, 0x00, 0x18, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x63, 0xff, 0xee, 0x30, 0x20, 0x78, 0x00, 0x1c, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 
	0x0c, 0x7f, 0xff, 0xee, 0x00, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x1c, 0x7f, 0xff, 0xc0, 0x80, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 
	0x0c, 0x87, 0xff, 0x40, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xff, 0xff, 0xff, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x00, 
	0xff, 0xff, 0xff, 0xc4, 0x7f, 0xc0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 
	0xff, 0xff, 0xff, 0x95, 0xfe, 0xf8, 0x78, 0x00, 0x65, 0x20, 0x04, 0x40, 0x00, 0x3c, 0x78, 0x60, 
	0xff, 0xff, 0xff, 0xff, 0xfe, 0x3a, 0x00, 0x00, 0x4d, 0x35, 0xf8, 0xc0, 0x00, 0x31, 0xf8, 0x00, 
	0xff, 0xff, 0xff, 0xef, 0xfc, 0xc1, 0x00, 0x00, 0x6d, 0x14, 0x92, 0xc0, 0x00, 0x23, 0xf8, 0x00, 
	0xff, 0xff, 0xff, 0xe3, 0xf9, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 
	0xff, 0xff, 0xfc, 0x13, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 
	0xff, 0xff, 0xf8, 0x07, 0xfc, 0x02, 0x27, 0x00, 0x00, 0x05, 0xe0, 0x00, 0x00, 0x00, 0xfc, 0x00
};