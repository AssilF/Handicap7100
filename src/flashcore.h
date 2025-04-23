#include <Arduino.h>

#define R_LED 6 //RGB LED stuff (Has PWM)
#define G_LED 4
#define B_LED 5


// Light Core 
#define rgb_partitions 8
#define rgb_IDs 11

// ID mapping
#define light_zone1       0
#define light_zone2       1
#define light_zone3       2
#define light_zone4       3
#define light_zone5       4
#define light_center_turn 5 // loops
#define light_right_turn  6 // loops
#define light_left_turn   7 // loops
#define light_modifying   8 // loops
#define light_recording   9
#define light_off        10 // no light

static const byte red[rgb_IDs][rgb_partitions] = {
  // zone1: static white
  {255,255,255,255,255,255,255,255},
  // zone2: static green
  {0,128,0,128,0,128,0,128},
  // zone3: static purple
  {128,0,128,0,128,0,128,0},
  // zone4: running blue dot
  {0,0,0,255,0,0,0,0},
  // zone5: rainbow gradient
  {255,0,0,255,255,0,0,255},
  // center_turn: blinking red+blue
  {255,0,255,0,255,0,255,0},
  // right_turn: blinking red
  {255,0,255,0,255,0,255,0},
  // left_turn: blinking blue
  {0,0,0,0,0,0,0,0}, // blue channel controls blink
  // modifying: slow fade white
  {255,200,150,100,50,25,10,0},
  // recording: red flash once
  {255,0,255,0,255,0,255,0},
  {0}
};

static const byte green[rgb_IDs][rgb_partitions] = {
  // zone1
  {255,255,255,255,255,255,255,255},
  // zone2
  {0,128,0,128,0,128,0,128},
  // zone3
  {0,0,0,0,0,0,0,0},
  // zone4
  {0,0,0,0,0,0,0,0},
  // zone5
  {0,255,0,255,0,255,0,255},
  // center_turn
  {0,0,0,0,0,0,0,0},
  // right_turn
  {0,0,0,0,0,0,0,0},
  // left_turn
  {0,0,0,0,0,0,0,0},
  // modifying
  {0,0,0,0,0,0,0,0},
  // recording
  {0,0,0,0,0,0,0,0},
  {0}
};

static const byte blue[rgb_IDs][rgb_partitions] = {
  // zone1
  {255,255,255,255,255,255,255,255},
  // zone2
  {0,0,0,0,0,0,0,0},
  // zone3
  {128,0,128,0,128,0,128,0},
  // zone4
  {0,0,255,0,0,0,0,0},
  // zone5
  {0,0,255,128,0,128,255,128},
  // center_turn
  {255,0,255,0,255,0,255,0},
  // right_turn
  {0,0,0,0,0,0,0,0},
  // left_turn
  {255,0,255,0,255,0,255,0},
  // modifying
  {255,200,150,100,50,25,10,0},
  // recording
  {0,0,0,0,0,0,0,0},
  {0}
};

static const unsigned long rgbMillis[rgb_IDs][rgb_partitions] = {
  // static: no timing
  {0,0,0,0,0,0,0,0},
  // green pulse speed
  {250,250,250,250,250,250,250,250},
  // purple pulse speed
  {300,300,300,300,300,300,300,300},
  // chase speed
  {100,100,100,100,100,100,100,100},
  // gradient: static
  {10,10,10,10,10,10,10,10},
  // center blink
  {250,250,250,250,250,250,250,250},
  // right blink
  {250,250,250,250,250,250,250,250},
  // left blink
  {250,250,250,250,250,250,250,250},
  // modifying fade
  {300,500,300,500,300,500,300,500},
  // recording flash
  {200,200,200,200,200,200,200,200},
  {0}
};

// Loop control per ID (true = loop, false = run once)
static const bool rgbLooper[rgb_IDs] = {
  false, // zone1
  true,  // zone2
  true,  // zone3
  true,  // zone4
  false, // zone5
  true,  // center_turn
  true,  // right_turn
  true,  // left_turn
  true,  // modifying
  false  // recording
,0
};

static const byte rgb_set_width[rgb_IDs] = {
  8, // zone1
  8, // zone2
  8, // zone3
  1, // zone4 dot
  8, // zone5 gradient
  8, // center blink
  8, // right blink
  8, // left blink
  8, // modifying fade
  8,  // recording flash
  0
};


bool isFlashing;

byte rgb_partition_index;
byte rgb_ID;

unsigned long rgb_time_reference;

void writeRGB(byte r, byte g, byte b)
{
  analogWrite(R_LED, 255 - r);
  analogWrite(G_LED, 255 - g);
  analogWrite(B_LED, 255 - b);
}

void light(byte rgb_pattern)
{
    rgb_partition_index=0;
    rgb_ID=rgb_pattern;
    if(rgb_ID!=light_off){
    isFlashing=1;
    rgb_time_reference=millis();}else
    {
        isFlashing=0;
    }
}

void performLightFeedback()
{
    if(isFlashing)
    {
        if(millis()-rgb_time_reference>=rgbMillis[rgb_ID][rgb_partition_index])
        {
            rgb_partition_index++;
            writeRGB(red[rgb_ID][rgb_partition_index],green[rgb_ID][rgb_partition_index],blue[rgb_ID][rgb_partition_index]);
            rgb_time_reference=millis();
        }
        if(rgb_partition_index>rgb_set_width[rgb_ID])
        {
            isFlashing=0;
            if(rgbLooper[rgb_ID])
            {
                light(rgb_ID);
            }
        }
    }else
    {
        writeRGB(0,0,0);
    }
}


