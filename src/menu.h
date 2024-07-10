#include <Arduino.h>

#define page_index_count 10
#define menu_page_index_count 8 //Two pages don't have elements but they display images and such;
#define menu_index_count 8

byte index_name_counts[menu_page_index_count]={1,7,5,6,6,4,3,3}; //including 0
byte elements_index;
byte page_index;
bool isModifying;


String index_name[menu_page_index_count][menu_index_count] = 
{
    {"run","configure"}, //First page has only 2 elements
    {"Operation Mode","PID","Distances","Telemetry","Color","End","Sensors","Back"}, //Second page has 8 elements
    {"Adjusted","PID1","PID2","Telemetry","PID+Telemetry","Back"}, //6 elements
    {"Kp","Ki","Kd","Base SPD:","Left SPD","Right SPD:","Back"}, //7 elements
    {"Left Minima:","Left Maxima","Right Minima","Right Maxima","Front Minima","Front Maxima","Back"},//7 elements
    {"Motor Bias Left","Motor Bias Right","Test Motors","Block Size","Back"}, //5 elements, For the motor Bias, test their encoder counts for equal times (ex: 3 secs)
    {"Red Threshhold","Blue Threshold","Test Color","Back"}, //4 elements
    {"Calibrate IR","Test IR","Left Threshold","Right Threshold"}, //4 elements
};

