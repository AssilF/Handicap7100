#include <Arduino.h>

#define menu_page_index_count 17 //Two pages don't have elements but they display images and such;
#define menu_index_count 8

byte index_name_counts[menu_page_index_count]={6,7,6,6,6,5,3,4,4,3,0,6,0,7,2  , 5 , 5}; //including 0
int elements_index;
byte page_index;
bool isModifying;
byte variable_index; //Left minima, Left Maxima, Right Minima, Right Maxima, Front Minima, Front Maxima, 
//Red Threshhold, Blue Threshhold.
double double_buffer;
int int_buffer;
byte last_element_index;
byte last_page_index;


String index_name[menu_page_index_count][menu_index_count] = 
{
    {"run","configure","save config","load config","Debug Readings","Debug Raw","Line Config"}, //First page has only 4 elements 0
    {"Operation Mode","PID","Distances","Telemetry","Color","Line","Encoders","Back"}, //Second page has 8 elements 1
    {"Adjusted","PID1","PID2","Telemetry","PID+Telemetry","Line","Back"}, //6 elements 2
    {"Kp","Ki","Kd","Base SPD:","Left SPD","Right SPD:","Back"}, //7 elements 3
    {"Left Minima:","Left Maxima","Right Minima","Right Maxima","Front Minima","Front Maxima","Back"},//7 elements 4
    {"Motor Bias Left","Motor Bias Right","Test Motors","Block Size","Calibrate Motors","Back"}, //6 elements, For the motor Bias, test their encoder counts for equal times (ex: 3 secs) 5
    {"Red Threshhold","Blue Threshold","Test Color","Back"}, //4 elements 6
    {"Calibrate Line","Toggle Line","Test Line","Calibration Mode:","Back"}, //3 elements 7
    {"Left Counts: ","Right Counts: ","Left Travel: ","Right Travel: ","Angel: "}, //8 
    {"+","rec","-","back"}, //9
    {}, //10
    {"LinePoseB:","LinePoseW:","LineMode:","DistanceL:","DistanceR:","DistanceF:","Intersection:"} //7 Elements. 11
    ,{}, //12
    {"IR0: ","IR1: ","IR2: ","IR3: ","IR4: ","IR5: ","IR6: ","IR7: "}, //13
    {"Line Calibration","Line PID","Back"}, //14 add intersection and junction options here
    {"Threshold Mode: "/*Normal/Raw*/,"Weight Mode: "/*QTR/TCRTase*/,"Hold Time: "/*Phantom Shi*/,"Threshold: "/*Normalt threshold*/,"Sample count: ","back"}, //15 Weight to whether to use the QTR Library or my own Approach, for now I'll only care about hold time LoL
    {"L-Kp:","L-Kd: ","L-Ki: ","L-BaseS: ","PID interval: ","back"} //16 Added Interval
};//note on adding an experimental speed dependency on the phantom time.

