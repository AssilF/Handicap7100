#include <Arduino.h>
#include <PseudoMazor.h>
#include <U8g2lib.h>
#include <NewTone.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <NewPing.h> //questionable library but this will do for now

/* 
This code is our "last resort" entry for polymaze 2024, hence the name "pseudomazor", later called handicap7100, the real mazor is based on an 
entirely different architecture and processor and framework (STM32) while this uses the arduino framework  and an arduino mega board with minimal 
hardware design to experiment with code and behaviour before improving hardware and optimizations. . .                                            ~Asce.il Fer 
*/

/*Todo:
-Test the magnetic encoder idea !DONE!
-test a basic block mapping algorithm
-test motor functions !DONE!
-test screen feedback !DONE!
-don't forget to add a buzzer !DONE!
-don't forget to add some RED LED arrays to assert dominance !CANCELED, added RGB LED instead because I can't assert dominance with a handicap . . .!
-add an LCD and menu control via 3-buttons and a potentiometer. . . 
-maybe read the battery ? keep the entire thing on breadboards, can't afford to solder shit during these times !DONE!
-normalment ctt ??? ig ?
-experiment with flood filling algorithms and visualizing mazes and such . . . !iih 3ilm kbir hadi!
-is there such thing as an analog interrupt ??? !lmfao no!
*/

//

//Command Defs=================================================================
#define oprint(text) oled.print(text)
#define ocursor(x,y) oled.setCursor(x,y)
#define oimage(x,y,w,h,img) oled.drawBitmap(0,0,w,h,img)

//Other =======================================================================
void writeRGB(byte r,byte g,byte b);
//Telemetry magic
int left_encoder_counts;
unsigned long last_Left_Count_Millis;
void leftISR()
{  
  if(millis()-last_Left_Count_Millis>=30)
    {
    left_encoder_counts++;
    last_Left_Count_Millis=millis();
    }
}

int right_encoder_counts;
unsigned long last_Right_Count_Millis;
void rightISR()
{
  if(millis()-last_Right_Count_Millis>=30)
    {
    right_encoder_counts++;
    last_Right_Count_Millis=millis();
    }
}


//Distance retrieval ======================================================================
#define moving_average_iterations 3

void getDistancesRaw();

int left_distance;
int right_distance;
int front_distance;

int front_threshold; //minima
int left_threshold; //maxima
int right_threshold; //maxima

int front_maxima;
int left_minima;
int right_minima;

unsigned long distance_buffer;

byte distance_buffer_state=1; //can't be null
byte median_iterations=3; //to be used later

int left_distance_buffer[moving_average_iterations];
int right_distance_buffer[moving_average_iterations];
int front_distance_buffer[moving_average_iterations];

unsigned long left_distance_sum;
unsigned long right_distance_sum;
unsigned long front_distance_sum;

byte moving_average_index = 0;
byte moving_average_start = 0;
byte iterable_index=0;

NewPing frontSonar(center_ultrasonic_sensor_trigger_pin,center_ultrasonic_sensor_echo_pin,120);
NewPing rightSonar(right_ultrasonic_sensor_trigger_pin,right_ultrasonic_sensor_echo_pin,120);
NewPing leftSonar(left_ultrasonic_sensor_trigger_pin,left_ultrasonic_sensor_echo_pin,120);



//Wheels magic ==============================================================================
void setDirection(byte dir=0);
void setSpeed(); //PID differential adjustments
void travelToTarget(int distance_in_mm);
void turn(byte direction); //helpful for when the maze has been optimized ig ? 
void performTravel();

void turnUntilClear();
void moveUntilClear();


bool isTraveling;
int travel_target=0;

int base_speed=70;
unsigned long speed_time_instance; //let's hope the calculations don't throw this off balance . . . 
unsigned long PID_interval;
double speed_Kp=3 ; //maybe turn these variables to floats if it takes a gazillion seconds to compute ?
double speed_Ki=0.00; //probably won't be used ?
double speed_Kd=-0.00;
double speed_integral;

int speed_error;
int speed;
int adjustment;

//Testing debuggy days ===============================================================================
byte testing_counter;
bool isTesting;
bool verbose=0;

unsigned long compute_time;
unsigned long distance_time;
unsigned long display_interval;

String serial_buffer;

void analyzeSerial();


//IO Dayz ====================================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0);

unsigned long beep_time_reference;
byte beep_count;
byte beep_target;
int beep_delay;
bool is_beeping;
bool beep_state;

//Audio Core
#define sound(freq) NewTone(Buzzer,freq)

#define sound_partitions 6
#define sound_IDs 16

#define audio_scroll_up 0
#define audio_scroll_down 1
#define audio_value_inc 2
#define audio_value_dec 3
#define audio_boot 4
#define audio_shut_down 5
#define audio_home 6
#define audio_value_edit 7
#define audio_button_push 8
#define audio_cycle 9
#define audio_fail 10
#define audio_start 11
#define audio_finished 12
#define audio_connected 13
#define audio_disconnected 14
#define audio_webCMD 15

int sound_sets[sound_IDs][sound_partitions]={
{760},                  //scroll up 0
{560},                  //scroll down 1
{780,1290},             //inc 2
{1290,780},             //dec 3
{1000,760,800},             //boot 4
{1000,840},             //shut down 5
{560},                  //home 6
{1033,0,1033},          //edit 7
{450,670},              //Button 8
{310,0,310,0,310},      //cycle 9
{80,0,80},              //fail 10
{840,770,0,840,1000},   //start 11
{445,660,890}           //finished 12
,{660,890}              //Connected 13
,{445,660},             //Disconnected 14
{890}                   //Coms 15
}; 

unsigned long soundMillis[sound_IDs][sound_partitions]={
{180}                      //scroll up 0      
,{180}                      //scroll down 1
,{120,80}                   //inc 2
,{120,80}                   //dec 3
,{300,200,300}                  //boot 4
,{200,300}                  //shut down 5
,{160}                      //home 6
,{150,150,150}              //edit 7
,{200,50}                   //button 8
,{200,200,200,200,200}      //cycle 9
,{200,200,350}              //fail 10
,{200,100,300,100,200}      //start 11
,{500,400,300}              //finished 12
,{500,400,300}              //Connceted 13
,{500,400,300}              //Disconnected 14
,{200}                      //Coms 15
};                
                                  
byte sound_set_width[sound_IDs] = {1,1,2,2,2,2,1,3,2,5,3,5,3,2,2,1};
byte sound_priority[sound_IDs]; //later to be implemented, which sound cancels which :)
bool isSounding;
unsigned long sound_partition_reference;
byte sound_partition_index; //3 stages of sound or beyond, each stage acts like an index for the array of type of sounds, reset this index when overriding a sound
byte sound_ID;
bool willPlay=1;
bool audioConflict=1;

//Graphics Core
#define mute 0
#define alarms 1
#define minimal 2
#define all 3
byte audio_mode = 3;

void play_audio(byte sound_to_play)
{
    sound_partition_index=0;
    isSounding=1;
    sound_ID=sound_to_play;
    sound_partition_reference=millis();

    switch (audio_mode)
    {
    case mute:
    willPlay=0;
    break;

    case alarms:
    sound_ID >10? willPlay=1:willPlay=0; 
    break;

    case minimal:
    sound_ID>6?willPlay=1:willPlay=0;
    if(sound_ID!=audio_button_push){audioConflict=0;} //we giving the priority to these sounds except button push
    break;
    
    default:
    willPlay=1;
    break;
    }

    if(willPlay){sound(sound_sets[sound_ID][sound_partition_index]);}
}

void performAudioFeedback()
{
    if(isSounding && willPlay)
    {
             //this is quite unoptimized as it'll keep filling the PWM buffer with this freq each loop cycle but eh, can optimize later when this becomes a problem
            if(millis()-sound_partition_reference>=soundMillis[sound_ID][sound_partition_index])
            {
                sound_partition_index++;
                sound(sound_sets[sound_ID][sound_partition_index]);
                sound_partition_reference=millis();
            }
            
            if(sound_partition_index>sound_set_width[sound_ID]){isSounding=0;}
    }else
    {
        noNewTone(Buzzer);
        audioConflict=1;
    }
}




// Functions=======================
void getDistancesRaw() //remove that distance buffer stuff, it's useless now . . . 
{
  distance_time=millis();
  if(moving_average_index==moving_average_start){moving_average_start++;}

  delay(3);
  right_distance_buffer[moving_average_index] = (int)rightSonar.ping()*0.17;
  delay(3);
  left_distance_buffer[moving_average_index]= (int)leftSonar.ping()*0.17;
  delay(3);

  front_distance_buffer[moving_average_index]= (int)frontSonar.ping()*0.17;

  left_distance_sum+=-left_distance_buffer[moving_average_start]+left_distance_buffer[moving_average_index];
  left_distance = left_distance_sum/moving_average_iterations;

  right_distance_sum+=-right_distance_buffer[moving_average_start]+right_distance_buffer[moving_average_index];
  right_distance = right_distance_sum/moving_average_iterations;

  front_distance_sum+=-front_distance_buffer[moving_average_start]+front_distance_buffer[moving_average_index];
  front_distance = front_distance_sum/moving_average_iterations;

  moving_average_start==(moving_average_iterations-1)?moving_average_start=0:moving_average_start++;
  moving_average_index==(moving_average_iterations-1)?moving_average_index=0:moving_average_index++;
  distance_time=millis()-distance_time;
} 


void setDirection(byte dir=dir_forward)
{
  switch (dir)
  {
  case dir_forward://the four last bits are as follows <---r+-l+-
    digitalWrite(left_motor_positive_pin,1);
    digitalWrite(left_motor_negative_pin,0);
    digitalWrite(right_motor_positive_pin,1);
    digitalWrite(right_motor_negative_pin,0);
    break;
  case dir_U_left:
    digitalWrite(left_motor_positive_pin,0);
    digitalWrite(left_motor_negative_pin,1);
    digitalWrite(right_motor_positive_pin,1);
    digitalWrite(right_motor_negative_pin,0);
    break;
  case dir_U_right:
    digitalWrite(left_motor_positive_pin,1);
    digitalWrite(left_motor_negative_pin,0);
    digitalWrite(right_motor_positive_pin,0);
    digitalWrite(right_motor_negative_pin,1);
    break;
  case dir_C_left:
    digitalWrite(left_motor_positive_pin,0);
    digitalWrite(left_motor_negative_pin,0);
    digitalWrite(right_motor_positive_pin,1);
    digitalWrite(right_motor_negative_pin,0);
    break;
  case dir_C_right:
    digitalWrite(left_motor_positive_pin,1);
    digitalWrite(left_motor_negative_pin,0);
    digitalWrite(right_motor_positive_pin,0);
    digitalWrite(right_motor_negative_pin,0);
    break;
  case dir_reverse:
    digitalWrite(left_motor_positive_pin,0);
    digitalWrite(left_motor_negative_pin,1);
    digitalWrite(right_motor_positive_pin,0);
    digitalWrite(right_motor_negative_pin,1);
    break;
  case dir_break:
    digitalWrite(left_motor_positive_pin,1);
    digitalWrite(left_motor_negative_pin,1);
    digitalWrite(right_motor_positive_pin,1);
    digitalWrite(right_motor_negative_pin,1);
    break;
  case dir_stop:
    digitalWrite(left_motor_positive_pin,0);
    digitalWrite(left_motor_negative_pin,0);
    digitalWrite(right_motor_positive_pin,0);
    digitalWrite(right_motor_negative_pin,0);
    break;
  }
}


void setSpeed() //positive is right negative is left for direction and vice versa for the error
{//Btw, replace the constrains with some #defined calculations ig ? the US offset from the center of the robot is 7.5/2 and subtract the block width/2 from it to get the desired distance (centered) 
 double d_error = (micros()-speed_time_instance)*0.000001;

 double error = map(constrain(left_distance,0,200/*12*/) - constrain(right_distance,0,200/*12*/),-200,200,-base_speed,base_speed); //optimally, 15 cancels 15 so the robot is trying to keep at least a distance of 15 between the two points
 
 speed_integral = speed_integral + error*d_error;
 d_error = (error-speed_error)/d_error;

 adjustment = constrain((int)(error*speed_Kp+ d_error*speed_Kd + speed_integral*speed_Ki),-base_speed,base_speed);

 analogWrite(left_motor_pwm_pin,constrain(base_speed+adjustment,0,255));
 analogWrite(right_motor_pwm_pin,constrain(base_speed-adjustment,0,255));

 speed_error = error;
 speed_time_instance = micros();
}

int left_speed=80;
double speed_error_left;
unsigned long speed_time_instance_left;
int left_maxima=150;
int adjustment_left;
double speed_integral_left;

void setSpeedLeft()
{
  double d_error_left = (micros()-speed_time_instance_left)*0.000001;

  double error_left = map(constrain(left_distance,left_minima,left_maxima/*12*/),left_minima,left_maxima,-left_speed,left_speed);

  speed_integral_left = speed_integral_left + error_left*d_error_left;
  d_error_left = (error_left-speed_error_left)/d_error_left;

  if(adjustment_left<0)
  {
    digitalWrite(left_motor_positive_pin,0);
    digitalWrite(left_motor_negative_pin,1);
  }else
  {
    digitalWrite(left_motor_positive_pin,1);
    digitalWrite(left_motor_negative_pin,0);
  }
  adjustment_left = abs(adjustment_left);
  analogWrite(left_motor_pwm_pin,constrain(adjustment,0,255));


  speed_error_left = error_left;
  speed_time_instance_left = micros();
} 

int right_speed=80;
double speed_error_right;
unsigned long speed_time_instance_right;
int right_maxima=150;
int adjustment_right;
double speed_integral_right;

void setSpeedRight()
{
  double d_error_right = (micros()-speed_time_instance_right)*0.000001;

  double error_right = map(constrain(right_distance,right_minima,right_maxima/*12*/),right_minima,right_maxima,-right_speed,right_speed);

  speed_integral_right = speed_integral_left + error_right*d_error_right;
  d_error_right = (error_right-speed_error_right)/d_error_right;

  if(adjustment_right<0)
  {
    digitalWrite(right_motor_positive_pin,0);
    digitalWrite(right_motor_negative_pin,1);
  }else
  {
    digitalWrite(right_motor_positive_pin,1);
    digitalWrite(right_motor_negative_pin,0);
  }
  adjustment_left = abs(adjustment_left);
  analogWrite(right_motor_pwm_pin,constrain(adjustment,0,255));


  speed_error_right = error_right;
  speed_time_instance_right = micros();
} 

void analyzeSerial()
{
  if(Serial.available()>=2)
  {
    char *command; 
    Serial.readBytes(command,1);
    switch (*command)
    {
    case 'p':
      speed_Kp=Serial.parseFloat();
      Serial.print("New Kp is:");
      Serial.println(speed_Kp);
      break;

    case 'd':
      speed_Kd=Serial.parseFloat();
      Serial.print("New Kd is:");
      Serial.println(speed_Kd);
      break;

    case 'S':
      base_speed=Serial.parseInt(),0,255;
      base_speed = constrain(base_speed,0,255);
      Serial.print("New speed is:");
      Serial.println(base_speed);
      break;

    case 'M':
      verbose=!verbose;
      break;
    
    default:
      Serial.println("Didn't understand command");
      break;
    }
    Serial.flush();
  }
  if(verbose)
  {
    unsigned long t =millis(); //plotting the PID response . . .
    Serial.print(t);
    Serial.print(" ");
    Serial.print(adjustment);
    Serial.print(" ");
    Serial.print(left_distance);
    Serial.print(" ");
    Serial.print(right_distance);
    Serial.println();
  }
}

void display()
{
  if(millis()-display_interval>=250){
  oled.clearBuffer();
  oled.setCursor(5,7);
  oled.print("Front Sonar:");
  oled.print(front_distance);
  oled.setCursor(5,15);
  oled.print("Right Sonar:");
  oled.print(right_distance);
  oled.setCursor(5,23);
  oled.print("Left Sonar:");
  oled.print(left_distance);

  oled.setCursor(5,30);
  oled.print("Battery mV:");
  oled.print(analogRead(VBat_pin)*0.009786-1.4);

  oled.setCursor(5,37);
  oled.print("PID error:");
  oled.print(speed_error);

  oled.setCursor(5,50);
  oled.print("Left Encoder");
  oled.print(left_encoder_counts);

  oled.setCursor(5,58);
  oled.print("Right Encoder:");
  oled.print(right_encoder_counts);

  oled.setCursor(96,62);
  oled.print(compute_time);

  oled.setCursor(96,7);
  oled.print(distance_time);

  oled.sendBuffer();
  display_interval = millis();
  }
}

#define turning_mode 1
#define forward_mode 0
bool travel_mode;

void travelToTarget(int distance_in_mm)
{
  if(!isTraveling)
  {
    travel_mode=forward_mode;
    isTraveling=1;
    left_encoder_counts=0;
    right_encoder_counts=0;
    travel_target=distance_to_encoder_counts((int)distance_in_mm);
  }
}

void turnToTarget(int target_angle)
{
  if(!isTraveling)
  {
    travel_mode=turning_mode;
    isTraveling=1;
    left_encoder_counts=0;
    right_encoder_counts=0;
    travel_target=angle_to_encoder_counts((int)target_angle);
  }
}

#define move_until_clear_mode 0
#define block_mapping_mode 1

byte robot_mode=1;
bool isOffsetting;
void performTravel()
{

  switch (robot_mode)
  {
  case move_until_clear_mode:
  if(travel_mode==forward_mode){
    setSpeed();
    if(front_distance<=front_threshold||left_distance>=left_threshold||right_distance>=right_threshold) //move until clear lol
    {
      setDirection(dir_break);
      isTraveling=0;
    }
  }else
  {
    analogWrite(left_motor_pwm_pin,base_speed);
    analogWrite(right_motor_pwm_pin,base_speed);
    turnUntilClear();
  }
    break;
  
  default:
    if(isTraveling)
  {
    if((!travel_mode?  ( left_encoder_counts>=travel_target  ||  right_encoder_counts>=travel_target ) : (((left_encoder_counts+right_encoder_counts)/2)   >=travel_target))/* || (!isOffsetting&&travel_mode==forward_mode&&((front_distance<front_threshold&&front_distance>10)||(left_distance>left_threshold&&left_distance>15)||(right_distance>right_threshold&&right_distance>20)))*/)
    {
      //beep(beep_single_long);
      setDirection(dir_break);
      isTraveling=0;
    }
  }
    break;
  }

}

#define turning_left 0
#define turning_right 1

byte turning_direction;

void turnUntilClear() //travel mode must be turning
{
  if(front_distance>=front_maxima)
  { 
  setDirection(dir_break);
  isTraveling=0;
  }else
  if(turning_direction == turning_left && left_distance<=left_minima)
  {
  setDirection(dir_break);
  isTraveling=0;
  }else
  if(turning_direction == turning_right && right_distance<=right_minima)
  {
  setDirection(dir_break);
  isTraveling=0;
  }
  
}

void writeRGB(byte r,byte g,byte b)
{
  analogWrite(R_LED,255-r);
  analogWrite(G_LED,255-g);
  analogWrite(B_LED,255-b);
}

bool isInConfiguration;
byte configuration_index;
void configureThresholds()
{
  isInConfiguration=1;
  while(isInConfiguration)
  {
    getDistancesRaw();
    oled.clearBuffer();
    switch (configuration_index)
    {
    case 0:
    ocursor(5,7);
    oprint("front Distance:");
    oprint(front_distance);

      break;
    
        case 1:
    ocursor(5,7);
    oprint("right Distance:");
    oprint(right_distance);

      break;

        case 2:
    ocursor(5,7);
    oprint("left Distance:");
    oprint(left_distance);
      break;
    
            case 3:
    ocursor(5,7);
    oprint("left minima:");
    oprint(left_minima);
          break;
            case 4:
    ocursor(5,7);
    oprint("right minima:");
    oprint(right_minima);
          break;
            case 5:
    ocursor(5,7);
    oprint("front maxima:");
    oprint(front_maxima);
          break;
    
    default:


    isInConfiguration=0;
      break;
    } 
    if(isInConfiguration){
    ocursor(5,20);
    oprint("press green button to configure");}else
    {
          oled.clearBuffer();
          ocursor(5,7);
          oprint("The configured thresholds:");
          ocursor(5,16);
          oprint("front:");
          oprint(front_threshold);
          ocursor(5,26);
          oprint("right:");
          oprint(right_threshold);
          ocursor(5,36);
          oprint("left:");
          oprint(left_threshold);
          delay(3000);
    }
    oled.sendBuffer();
    performAudioFeedback();
  }
}

bool run;

unsigned long right_button_interval;
unsigned long left_button_interval;
unsigned long center_button_interval;

void left_button_ISR(void)
{
    if(millis()-left_button_interval>=100)
    {
      writeRGB(0,100,100);
      play_audio(audio_button_push);
      left_button_interval = millis();
    }
}
void right_button_ISR(void)
{
    if(millis()-right_button_interval>=100)
    {
      writeRGB(100,100,0);
      if(isInConfiguration)
      {
        
       switch (configuration_index)
       {
       case 0:
        front_threshold=front_distance;
        break;
       case 1:
        right_threshold=right_distance;
        break;
        case 2:
        left_threshold=left_distance;
        break;
                case 3:
        left_minima=left_distance;
        break;
                case 4:
        right_minima=right_distance;
        break;
                case 5:
        front_maxima=front_distance;
        break;
       }
       configuration_index++; 
      }
      play_audio(audio_button_push);
      right_button_interval = millis();
    }
}

void middle_button_ISR(void)
{
    if(millis()-center_button_interval>=100)
    {
      if(run){run=0;}else{run=1;}
      writeRGB(80,100,200);
      play_audio(audio_button_push);
      center_button_interval = millis();
    }
}


void detectColor()
{

}

int distance_counts;

//boring hardcoded dayz ======================================================================
void setup()  
{
  //pin direction Note: for the color sensor and motor directions we can manipulate registers directly, this is to be optimized later ig . . .  
  pinMode(left_motor_encoder_pin,INPUT_PULLUP);    
  pinMode(right_motor_encoder_pin,INPUT_PULLUP);
  pinMode(left_motor_negative_pin,OUTPUT);
  pinMode(left_motor_positive_pin,OUTPUT);
  pinMode(right_motor_negative_pin,OUTPUT);
  pinMode(right_motor_positive_pin,OUTPUT);
  pinMode(motor_standby_pin,OUTPUT);
  pinMode(right_motor_pwm_pin,OUTPUT);
  pinMode(left_motor_pwm_pin,OUTPUT);
  pinMode(Buzzer,OUTPUT);
  pinMode(R_LED,OUTPUT);
  pinMode(G_LED,OUTPUT);
  pinMode(B_LED,OUTPUT);
  pinMode(right_button_pin,INPUT_PULLUP);
  pinMode(left_button_pin,INPUT_PULLUP);
  pinMode(middle_button_pin,INPUT_PULLUP);

  //interrupts and stuff
  attachInterrupt(digitalPinToInterrupt(left_motor_encoder_pin),leftISR,FALLING);
  attachInterrupt(digitalPinToInterrupt(right_motor_encoder_pin),rightISR,FALLING);
  attachPCINT(digitalPinToPCINT(left_button_pin),left_button_ISR,FALLING);
  attachPCINT(digitalPinToPCINT(right_button_pin),right_button_ISR,FALLING);
  attachPCINT(digitalPinToPCINT(middle_button_pin),middle_button_ISR,FALLING);
  
  writeRGB(0,0,0);
  //Serial.begin(19200);   //intialize the serial monitor baud rate
  //Serial.println("Serial initialized");
  oled.begin();
  oled.setFont(u8g2_font_squeezed_b6_tr);

  //do{testYALL();}while(isTesting);
  while(isSounding){
  play_audio(audio_boot);
  performAudioFeedback();}

  //configureThresholds();
}


bool turnB;

int lastLeftDistance;
int lastRightDistance;
int lastFrontDistance;

void loop()
{
  if(run){
  digitalWrite(motor_standby_pin,1);

  if(travel_mode==forward_mode){setSpeedRight();setSpeedLeft();}

  }else
  {
    digitalWrite(motor_standby_pin,0);
  }
  
  performTravel();
  display();
  performAudioFeedback();

}