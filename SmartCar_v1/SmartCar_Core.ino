/*
   @Description: In User Settings Edi
   @Author: your name
   @Date: 2019-08-12 18:00:25
   @LastEditTime: 2020-09-04 14:40:48
   @LastEditors: Changhua
*/
#include <IRremote.h>
#include <Servo.h>
#include <stdio.h>
#include "HardwareSerial.h"
#include "ArduinoJson-v6.11.1.h" //Use ArduinoJson Libraries
//#include "NewTone.h" //AMA using this because the timer2 conflict of tone() and IRremote.h libs. But NewTimer uses timer1 and conflicts with Servo.h lib. :(
#include <TimerFreeTone.h>
#include "Music.h" //AMA

#define f 0xFF629D //16736925 // FORWARD
#define b 0xFFA857 //16754775 // BACK
#define l 0xFF22DD //16720605 // LEFT
#define r 0xFFC23D //16761405 // RIGHT
#define s 0xFF02FD //16712445 // STOP

#define UNKNOWN_F 5316027    // FORWARD
#define UNKNOWN_B 2747854299 // BACK
#define UNKNOWN_L 1386468383 // LEFT
#define UNKNOWN_R 553536955  // RIGHT
#define UNKNOWN_S 3622325019 // STOP

//#define KEY1 16738455 //Line Teacking mode
//#define KEY2 16750695 //Obstacles Avoidance mode

//AMA
#define KEY0 0xFF4AB5 //play music
#define KEY1 0xFF6897 //Line Teacking mode
#define KEY2 0xFF9867 //Obstacles Avoidance mode
#define KEY3 0xFFB04F
#define KEY4 0xFF30CF
#define KEY5 0xFF18E7
#define KEY6 0xFF7A85
#define KEY7 0xFF10EF //toggle RGB LED
#define KEY8 0xFF38C7 //toggle RGB LED
#define KEY9 0xFF5AA5 //toggle RGB LED

#define KEY_STAR 0xFF42BD //16728765 //REDUCE CARSPEED //AMA
#define KEY_HASH 0xFF52AD //16732845 //INCREASE CARSPEED //AMA

/*Arduino pin is connected to the IR Receiver*/
#define RECV_PIN 12

/*Arduino pin is connected to the Ultrasonic sensor module*/
#define ECHO_PIN A4
#define TRIG_PIN A5
const int ObstacleDetection = 35;
#define BEEP_PIN A3 //AMA
#define RGB1_PIN A2 //AMA
#define RGB2_PIN A1 //AMA
#define RGB3_PIN A0 //AMA

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define LED_Pin 13

/*Arduino pin is connected to the IR tracking module*/
#define LineTeacking_Pin_Right 10
#define LineTeacking_Pin_Middle 4
#define LineTeacking_Pin_Left 2

#define LineTeacking_Read_Right !digitalRead(10) //Right  AMA: right-hand side if looking at the front of the car
#define LineTeacking_Read_Middle !digitalRead(4) //Middle ELEGOO ORIGINAL HW
//#define LineTeacking_Read_Middle digitalRead(4) //Middle AMA HW is inverted
#define LineTeacking_Read_Left !digitalRead(2)   //Left  AMA: left-hand side if looking at the front of the car

//#define carSpeed 250 //PWM(Motor speed/Speed)
unsigned int carSpeed = 100; //PWM(Motor speed/Speed)  AMA used for these modes: IR control, line tracking, obstacle avoidance

//unsigned int carSpeed_rocker = 250;
unsigned int carSpeed_rocker = carSpeed; // AMA

// Minimum PWM value of analogWrite to spin motor when robot is on the ground.
int MINIMUM_MOTOR_SPEED = 40; //175; //100; //40; //AMA
int MAXIMUM_MOTOR_SPEED = 250; //255; //AMA

// Melody (liberated from the toneMelody Arduino example sketch by Tom Igoe).
//int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
//int duration[] = { 250, 125, 125, 250, 250, 250, 250, 250 };

#define PIN_Servo 3
Servo servo;             //  Create a DC motor drive object
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object

unsigned long IR_PreMillis;
unsigned long LT_PreMillis;

int rightDistance = 0;  //Right distance
int leftDistance = 0;   //left Distance
int middleDistance = 0; //middle Distance

/*CMD_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
uint8_t CMD_MotorSelection;
uint8_t CMD_MotorDirection;

uint16_t CMD_MotorSpeed;
unsigned long CMD_leftMotorControl_Millis;
unsigned long CMD_rightMotorControl_Millis;

/*CMD_CarControl: Car Control：Car moving direction、Car Speed、Car moving time*/
uint8_t CMD_CarDirection;
uint8_t CMD_CarSpeed;
uint16_t CMD_CarTimer; //AMA was in [sec], I changed the code to make it [ms]
unsigned long CMD_CarControl_Millis;

uint8_t CMD_CarDirectionxxx;
uint8_t CMD_CarSpeedxxx;
uint16_t CMD_Distance;

String CommandSerialNumber; //

enum SERIAL_mode
{
  Serial_rocker,
  Serial_programming,
  Serial_CMD,
} Serial_mode = Serial_programming;

enum FUNCTIONMODE
{
  IDLE,                  /*free*/
  LineTeacking,          /*Line Teacking Mode*/
  ObstaclesAvoidance,    /*Obstacles Avoidance Mode*/
  Follow_mode,            /*跟随模式*/ //AMA
  Bluetooth,             /*Bluetooth Control Mode*/
  IRremote,              /*Infrared Control Mode*/
  CMD_MotorControl,      /*Motor Control Mode*/
  CMD_CarControl,        /*Car Control Mode*/
  CMD_CarControlxxx,     /*Car Control Mode*/
  CMD_ClearAllFunctions, /*Clear All Functions Mode*/
} func_mode = IDLE;      /*Functional mode*/

enum MOTIONMODE
{
  LEFT,    /*left*/
  RIGHT,   /*right*/
  FORWARD, /*forward*/
  BACK,    /*back*/
  STOP,    /*stop*/
  LEFT_FORWARD,
  LEFT_BACK,
  RIGHT_FORWARD,
  RIGHT_BACK,
} mov_mode = STOP; /*move mode*/

bool music_mode = false; //playing music when true
//int thisNote = 0; //current music note

// plays the music //AMA
void music_play() {
  if (music_mode == true)
  {
    // iterate over the notes of the melody.
    // Remember, the array is twice the number of notes (notes + durations)
    //for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    if (thisNote < notes * 2) {
      // calculates the duration of each note
      divider = pgm_read_word_near(melody + thisNote + 1);
      if (divider > 0) {
        // regular note, just proceed
        noteDuration = (wholenote) / divider;
      } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration = (wholenote) / abs(divider);
        noteDuration *= 1.5; // increases the duration in half for dotted notes
      }
      //RGB LED fun
      digitalWrite(RGB1_PIN, random() % 2);
      digitalWrite(RGB2_PIN, random() % 2);
      digitalWrite(RGB3_PIN, random() % 2);

      //TimerFreeTone(BEEP_PIN, pgm_read_word_near(melody+thisNote), noteDuration * 0.9, 10); // Play melody. volume = 10 is max
      TimerFreeTone(BEEP_PIN, pgm_read_word_near(melody + thisNote), noteDuration * 0.9); // Play melody
      delay(noteDuration * 0.1);

      //getBTData_Plus(); //Bluetooth Communication Data Acquisition
      //getIRData();      //Infrared Communication Data Acquisition
      //if (music_mode == false) return;
      thisNote = thisNote + 2;
    }
    else
    {
      //RGB LED fun off
      digitalWrite(RGB1_PIN, LOW);
      digitalWrite(RGB2_PIN, LOW);
      digitalWrite(RGB3_PIN, LOW);
      thisNote = 0; //rewind
      music_mode = false;
    }
  }
}

void delays(unsigned long t)
{
  for (unsigned long i = 0; i < t; i++)
  {
    getBTData_Plus(); //Bluetooth Communication Data Acquisition
    getIRData();      //Infrared Communication Data Acquisition
    delay(1);
  }
}

/*ULTRASONIC*/
unsigned int getDistance(void)
{ //Getting distance
  static unsigned int tempda = 0;
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  //tempda = tempda_x;
  if (tempda_x > 150)
  {
    tempda_x = 150;
  }
  // return tempda;
  return tempda_x;
}

/*
  Control motor：Car movement forward
*/
void forward(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go forward!");
}
/*
  Control motor：Car moving backwards
*/
void back(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go back!");
}
/*
  Control motor：The car turns left and moves forward
*/
void left(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, in_carSpeed +20); //AMA it needs more force to turn
  analogWrite(ENB, in_carSpeed +20); //AMA
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go left!");
}
/*
  Control motor：The car turns right and moves forward
*/
void right(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed +20); //AMA it needs more force to turn
  analogWrite(ENB, in_carSpeed +20); //AMA
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go right!");
}

void forward_left(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed / 2);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go right!");
}

void forward_right(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed / 2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go right!");
}

void back_left(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed / 2);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go right!");
}

void back_right(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed / 2);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go right!");
}

/*
  Stop motor control：Turn off the motor drive
*/
void stop(bool debug = false)
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if (debug)
    Serial.println("Stop!");
}
/*
  Servo Control angle Setting
*/
void ServoControl(uint8_t angleSetting)
{
  if (angleSetting > 175)
  {
    angleSetting = 175;
  }
  else if (angleSetting < 5)
  {
    angleSetting = 5;
  }
  servo.attach(3);
  servo.write(angleSetting); //sets the servo position according to the  value
  delays(500);
  servo.detach();
}

/*
  Infrared Communication Data Acquisition
*/
void getIRData(void)
{
  if (irrecv.decode(&results))
  {
    digitalWrite(LED_Pin, HIGH);   // turn the LED on AMA
    IR_PreMillis = millis();
    switch (results.value)
    {
      case f:
      case UNKNOWN_F:
        func_mode = IRremote;
        mov_mode = FORWARD;
        break; /*forward*/
      case b:
      case UNKNOWN_B:
        func_mode = IRremote;
        mov_mode = BACK;
        break; /*back*/
      case l:
      case UNKNOWN_L:
        func_mode = IRremote;
        mov_mode = LEFT;
        break; /*left*/
      case r:
      case UNKNOWN_R:
        func_mode = IRremote;
        mov_mode = RIGHT;
        break; /*right*/
      case s:
      case UNKNOWN_S:
        func_mode = IRremote;
        //func_mode = CMD_ClearAllFunctions; //AMA is this ok?
        music_mode = false;
        //RGB LED fun off
        digitalWrite(RGB1_PIN, LOW);
	    digitalWrite(RGB2_PIN, LOW);
	    digitalWrite(RGB3_PIN, LOW);		
        mov_mode = STOP;
        break; /*stop*/
      case KEY1:
        func_mode = LineTeacking;
        break; /*Line Teacking Mode*/
      case KEY2:
        func_mode = ObstaclesAvoidance;
        break; /*Obstacles Avoidance Mode*/
      //AMA added cases
      case KEY3:
        func_mode = Follow_mode;
        break; /*Follow Mode*/
      case KEY_STAR:
        if (carSpeed > MINIMUM_MOTOR_SPEED) carSpeed -= 10;
        TimerFreeTone(BEEP_PIN, 150 + carSpeed, 200); //feedback
        break; /*Reduce carSpeed*/
      case KEY_HASH:
        if (carSpeed < MAXIMUM_MOTOR_SPEED) carSpeed += 10;
        TimerFreeTone(BEEP_PIN, 150 + carSpeed, 200); //feedback
        break; /*increase carSpeed*/
      case KEY0:
        //tone(BEEP_PIN, 440, 250); // Hz, ms //conflicts with IRremote.h for timer2
        //NewTone(BEEP_PIN, 440, 250); // Hz, ms //conflicts with Servo.h for timer1
        //digitalWrite(BEEP_PIN, HIGH);   // turn the buzzer on AMA
        //delay(250); // ms
        //digitalWrite(BEEP_PIN, LOW);   // turn the buzzer off
        //TimerFreeTone(BEEP_PIN, 440, 250); // Hz, ms // Play
        /*for (int thisNote = 0; thisNote < 8; thisNote++) // Loop through the notes in the array.
          {
          TimerFreeTone(BEEP_PIN, melody[thisNote], duration[thisNote]); // Play melody[thisNote] for duration[thisNote].
          delay(50); // Short delay between notes.
          }*/
        music_select(0u);
        music_mode = !music_mode;
        //play();
        break; /*beep*/
      case KEY4:
        music_select(1u);
        music_mode = !music_mode;
        break; 
      case KEY5:
        music_select(2u);
        music_mode = !music_mode;
        break; 		
      case KEY6:
        music_select(3u);
        music_mode = !music_mode;
        break; 
      case KEY7:
        digitalWrite(RGB1_PIN, !digitalRead(RGB1_PIN)); //toggle the pin state
        TimerFreeTone(BEEP_PIN, NOTE_C5, 200); //feedback
        break; /*RGB LED*/
      case KEY8:
        digitalWrite(RGB2_PIN, !digitalRead(RGB2_PIN)); //toggle the pin state
        TimerFreeTone(BEEP_PIN, NOTE_E5, 200); //feedback
        break; /*RGB LED*/
      case KEY9:
        digitalWrite(RGB3_PIN, !digitalRead(RGB3_PIN)); //toggle the pin state
        TimerFreeTone(BEEP_PIN, NOTE_G5, 200); //feedback
        break; /*RGB LED*/
      default:
        break;
    }
    irrecv.resume();
  }
  else
  {
    digitalWrite(LED_Pin, LOW);   // turn the LED off AMA
  }
}
/*
  Bluetooth remote control mode
*/
void bluetooth_mode()
{
  if (func_mode == Bluetooth)
  {
    switch (mov_mode)
    {
      case LEFT:
        left(false, carSpeed_rocker);
        break;
      case RIGHT:
        right(false, carSpeed_rocker);
        break;
      case FORWARD:
        forward(false, carSpeed_rocker);
        break;
      case BACK:
        back(false, carSpeed_rocker);
        break;
      case STOP:
        stop();
        break;
      case LEFT_FORWARD:
        forward_left(false, carSpeed_rocker);
        break;
      case LEFT_BACK:
        back_left(false, carSpeed_rocker);
        break;
      case RIGHT_FORWARD:
        forward_right(false, carSpeed_rocker);
        break;
      case RIGHT_BACK:
        back_right(false, carSpeed_rocker);
        break;
      default:
        break;
    }
  }
}
/*
  Infrared remote control mode
*/
void irremote_mode(void)
{
  if (func_mode == IRremote)
  {
    switch (mov_mode)
    {
      case FORWARD:
        forward(false, carSpeed);
        break;
      case BACK:
        back(false, carSpeed);
        break;
      case LEFT:
        left(false, carSpeed);
        break;
      case RIGHT:
        right(false, carSpeed);
        break;
      case STOP:
        stop();
        break;
      default:
        break;
    }
    if (millis() - IR_PreMillis > 500)
    {
      mov_mode = STOP;
      IR_PreMillis = millis();
    }
  }
}

/*
  Line Teacking Mode
*/
void line_teacking_mode(void) //with 3 tracking sensors, native HW
  {
  if (func_mode == LineTeacking)
  {
    if (LineTeacking_Read_Middle)
    { //Detecting in the middle infrared tube

      forward(false, carSpeed); //Control motor：the car moving forward //180
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Right)
    { //Detecting in the right infrared tube

      right(false, carSpeed); //Control motor：the car moving right
      while (LineTeacking_Read_Right)
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Left)
    {                   //Detecting in the left infrared tube
      left(false, carSpeed); //Control motor：the car moving left
      while (LineTeacking_Read_Left)
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      LT_PreMillis = millis();
    }
    else
    {
      if (millis() - LT_PreMillis > 150)
      {
        stop(); //Stop motor control：Turn off motor drive mode
      }
    }
  }
}
  
/*void line_teacking_mode(void) //AMA with 2 tracking sensors, adopted from "Elegoo smart car mini".
{
  if (func_mode == LineTeacking)
  {
    //if (LineTeacking_Read_Middle)
    if (LineTeacking_Read_Right && LineTeacking_Read_Left) //AMA
    { // both sensors sense black line

      //forward(false, carSpeed); //Control motor：the car moving forward
      stop(); //Stop motor control：Turn off motor drive mode
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      //LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Right)
    { //Detecting in the right infrared tube

      right(false, carSpeed); //Control motor：the car moving right
      //while (LineTeacking_Read_Right)
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      //LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Left)
    { //Detecting in the left infrared tube
      left(false, carSpeed); //Control motor：the car moving left
      //while (LineTeacking_Read_Left)
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      //LT_PreMillis = millis();
    }
    else
    {
      forward(false, carSpeed); //Control motor：the car moving forward
      {
        getBTData_Plus(); //Bluetooth data acquisition
        getIRData();      //Infrared data acquisition
      }
      //if (millis() - LT_PreMillis > 150)
      //{
      //  stop(); //Stop motor control：Turn off motor drive mode
      //}
    }
  }
}*/

/*f(x) int */
static boolean function_xxx(long xd, long sd, long ed) //f(x)
{
  if (sd <= xd && xd <= ed)
    return true;
  else
    return false;
}

/*Obstacle avoidance*/

void obstacles_avoidance_mode(void)
{
  static boolean first_is = true;
  uint8_t switc_ctrl = 0;
  if (func_mode == ObstaclesAvoidance)
  {
    if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      ServoControl(90);
      first_is = false;
    }
    uint8_t get_Distance = getDistance();
    if (function_xxx(get_Distance, 0, 20))
    {
      stop();
      /*
        ------------------------------------------------------------------------------------------------------
        ServoControl(30 * 1): 0 1 0 1 0 1 0 1
        ServoControl(30 * 3): 0 0 1 1 0 0 1 1
        ServoControl(30 * 5): 0 0 0 0 1 1 1 1
        1 2 4 >>>             0 1 2 3 4 5 6 7
        1 3 5 >>>             0 1 3 4 5 6 5 9
        ------------------------------------------------------------------------------------------------------
        Truth table of obstacle avoidance state
      */
      for (int i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
      {
        ServoControl(30 * i);
        get_Distance = getDistance();
        delays(200);
        if (function_xxx(get_Distance, 0, 5))
        {
          switc_ctrl = 10;
          break;
        }
        else if (function_xxx(get_Distance, 0, 20)) //How many cm in the front have obstacles?
        {
          switc_ctrl += i;
        }
      }
      ServoControl(90);
    }
    else //if (function_xxx(get_Distance, 20, 50))
    {
      //forward(false, 150); //Control car forwar
      forward(false, carSpeed); //Control car forwar
    }
    while (switc_ctrl)
    {
      switch (switc_ctrl)
      {
        case 1:
        case 5:
        case 6:
          //forward(false, 150); //Control car forwar
          forward(false, carSpeed); //Control car forwar
          switc_ctrl = 0;
          break;
        //case 3:
        //left(false, 250); //Control car left
        //switc_ctrl = 0;
        //break;
        case 3:
        case 4:
          //left(false, 250); //Control car left
          left(false, carSpeed); //Control car left
          switc_ctrl = 0;
          break;
        case 8:
        case 11:
          //right(false, 250); //Control car right
          right(false, carSpeed); //Control car right
          switc_ctrl = 0;
          break;
        case 9:
        case 10:
          //back(false, 150); //Control car Car backwards
          back(false, carSpeed); //Control car Car backwards
          switc_ctrl = 11;
          break;
      }
      ServoControl(90);
    }
  }
  else
  {
    first_is = true;
  }
}

// AMA ====================================================================
void ApplicationFunctionSet_Follow(void)
{
  static boolean first_is2 = true;
  static uint8_t get_Distance;
  const uint8_t distanceKeepMin = 15; //[cm] to keep
  const uint8_t distanceKeepMax = 30; //[cm] to keep
  const uint8_t distanceFollowMax = 50; //[cm] to follow
  
  if (func_mode == Follow_mode)
  {
    if (first_is2 == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      ServoControl(90);
      first_is2 = false;
    }
	
	get_Distance = getDistance();
	
	// if the distance object is fitted then keep stop
    if (true == function_xxx(get_Distance, distanceKeepMin, distanceKeepMax)) //AMA
    {
      stop();
      return;
    }

    // if the distance object is too close then move backward
    if(get_Distance < distanceKeepMin)
	{
	  back(false, carSpeed);
      return;
    } 

	// follow 
    if (true == function_xxx(get_Distance, distanceKeepMax, distanceFollowMax)) //AMA
    {
	  forward(false, carSpeed);
      return;
    }
	
	//search for the object
	if (get_Distance > distanceFollowMax)
    {
	  right(false, carSpeed);
    }
  }
  else
  {
    first_is2 = true;
  }  
}

/*****************************************************Begin@CMD**************************************************************************************/

/*
  N21:command
  CMD mode：Ultrasonic module：App controls module status, module sends data to app
*/
void CMD_UltrasoundModuleStatus_Plus(uint8_t is_get) //Ultrasonic module processing
{
  //uint16_t
  CMD_Distance = getDistance(); //Ultrasonic module measuring distance

  if (1 == is_get) // is_get Start  true：Obstacles / false:No obstacles
  {
    if (CMD_Distance <= 50)
    {
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (2 == is_get) //Ultrasonic is_get data
  {
    char toString[10];
    sprintf(toString, "%d", CMD_Distance);
    // Serial.print(toString);
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
  }
}
/*
  N22:command
   CMD mode：Teacking module：App controls module status, module sends data to app
*/
void CMD_TraceModuleStatus_Plus(uint8_t is_get) //Tracking module processing
{
  if (0 == is_get) /*Get traces on the left*/
  {
    if (LineTeacking_Read_Left)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (1 == is_get) /*Get traces on the middle*/
  {
    if (LineTeacking_Read_Middle)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (2 == is_get)
  { /*Get traces on the right*/

    if (LineTeacking_Read_Right)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
}

/*
  N1:command
  CMD mode：Sport mode <motor control> Control motor by app
  Input：uint8_t is_MotorSelection,  Motor selection   1：left  2：right  0：all
        uint8_t is_MotorDirection,   Motor steering  1：Forward  2：Reverse 0：stop
        uint8_t is_MotorSpeed,       Motor speed   0-250
*/
void CMD_MotorControl_Plus(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
  static boolean MotorControl = false;

  if (func_mode == CMD_MotorControl) //Motor control mode
  {
    MotorControl = true;
    if (is_MotorSelection == 1 || is_MotorSelection == 0) //Left motor
    {
      if (is_MotorDirection == 1) //Positive rotation
      {
        analogWrite(ENA, is_MotorSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      }
      else if (is_MotorDirection == 2) //Reverse
      {
        analogWrite(ENA, is_MotorSpeed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      }
      else if (is_MotorDirection == 0)
      {
        digitalWrite(ENA, LOW); //Turn off the motor enable pin
      }
    }
    if (is_MotorSelection == 2 || is_MotorSelection == 0) //Right motor
    {
      if (is_MotorDirection == 1) //Positive rotation
      {
        analogWrite(ENB, is_MotorSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      else if (is_MotorDirection == 2) //Reverse
      {
        analogWrite(ENB, is_MotorSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }
      else if (is_MotorDirection == 0)
      {
        digitalWrite(ENB, LOW); //Turn off the motor enable pin
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
    }
  }
}

/*
  N4：command
  CMD mode：<Car control> APP control car
  Time limited
*/
//void CMD_CarControl_Plus(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint8_t is_Timer)
void CMD_CarControl_Plus(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint16_t is_Timer)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; //Have time to spend
  static boolean CarControl_return = false;
  if (func_mode == CMD_CarControl) //Car Control Mode
  {
    CarControl = true;
    if (is_Timer != 0) //Setting time cannot be empty
    {
      //if ((millis() - CMD_CarControl_Millis) > (is_Timer * 1000)) //check the time
      if ((millis() - CMD_CarControl_Millis) > is_Timer) //check the time
      {
        CarControl_TE = true;
        digitalWrite(ENA, LOW); //Turn off the motor enable pin
        digitalWrite(ENB, LOW);
        if (CarControl_return == false)
        {
          Serial.print('{' + CommandSerialNumber + "_ok}");
          delay(1);
          Serial.print('{' + CommandSerialNumber + "_ok}");
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //Have time to spend
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      switch (is_CarDirection)
      {
        case 1: /*Left-Forward Motion Mode*/
          left(false, is_CarSpeed);
          break;
        case 2: /*Right-Forward Motion Mode*/
          right(false, is_CarSpeed);
          break;
        case 3: /*Sport mode forward*/
          forward(false, is_CarSpeed);
          break;
        case 4: /*Sport mode back*/
          back(false, is_CarSpeed);
          break;
        default:
          break;
      }
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
      CMD_CarControl_Millis = 0;
    }
  }
}

/*
  N40：command
  CMD mode：<Car control> APP control car
  No time limit
*/
void CMD_CarControl_Plusxxx(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  static boolean CarControl = false;
  if (func_mode == CMD_CarControlxxx) //Car Control Mode
  {
    CarControl = true;
    switch (is_CarDirection)
    {
      case 1: /*Left-Forward Motion Mode*/
        left(false, is_CarSpeed);
        break;
      case 2: /*Right-Forward Motion Mode*/
        right(false, is_CarSpeed);
        break;
      case 3: /*Sport mode forward*/
        forward(false, is_CarSpeed);
        break;
      case 4: /*Sport mode back*/
        back(false, is_CarSpeed);
        break;
      default:
        break;
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
    }
  }
}

/*
  N5:command
  CMD mode：
*/
void CMD_ClearAllFunctionsXXX(void)
{
  if (func_mode == CMD_ClearAllFunctions)
  {

    mov_mode = STOP;
    func_mode = IDLE;
    digitalWrite(ENA, LOW); //Turn off the motor enable pin
    digitalWrite(ENB, LOW);

    /*CMD_MotorControl:Motor Control： Motor Speed、Motor Direction、Motor Time*/
    CMD_MotorSelection = NULL;
    CMD_MotorDirection = NULL;

    CMD_MotorSpeed = NULL;
    CMD_leftMotorControl_Millis = NULL;
    CMD_rightMotorControl_Millis = NULL;

    /*CMD_CarControl:Car Control：Car moving direction、Car Speed、Car moving time*/
    CMD_CarDirection = NULL;
    CMD_CarSpeed = NULL;
    CMD_CarTimer = NULL;
    CMD_CarControl_Millis = NULL;
	
	/*music_mode = false; //AMA . Keep music and light.
    //RGB LED fun off
    digitalWrite(RGB1_PIN, LOW);
	digitalWrite(RGB2_PIN, LOW);
	digitalWrite(RGB3_PIN, LOW);*/
  }
}

void getDistance_xx(void)
{
  CMD_Distance = getDistance(); //Ultrasonic measurement distance
}

/*****************************************************End@CMD**************************************************************************************/
/*
  Bluetooth serial port data acquisition and control command parsing
*/
#include "hardwareSerial.h"
void getBTData_Plus(void)
{
  static String SerialPortData = "";
  uint8_t c = "";
  if (Serial.available() > 0)
  {
    digitalWrite(LED_Pin, HIGH);   // turn the LED on AMA
    while ((c != '}') && Serial.available() > 0) //Forcibly wait for a frame of data to finish receiving
    {
      // while (Serial.available() == 0)
      //   ;
      c = Serial.read();
      SerialPortData += (char)c;
    }
  }
  else
  {
    digitalWrite(LED_Pin, LOW);    // turn the LED off AMA
  }
  if (c == '}')
  {
    //Serial.println(SerialPortData);
    StaticJsonDocument<200> doc;                                       //Create a JsonDocument object
    DeserializationError error = deserializeJson(doc, SerialPortData); //Deserialize JSON data
    SerialPortData = "";
    if (!error) //Check if deserialization is successful
    {
      int control_mode_N = doc["N"];
      char buf[3];
      uint8_t temp = doc["H"];
      sprintf(buf, "%d", temp);
      CommandSerialNumber = buf; //Get the serial number of the new command
      switch (control_mode_N)
      {
        case 1: /*Motion module  processing <command：N 1>*/
          {
            Serial_mode = Serial_programming;
            func_mode = CMD_MotorControl;
            CMD_MotorSelection = doc["D1"];
            CMD_MotorDirection = doc["D2"];
            CMD_MotorSpeed = doc["D3"];
            Serial.print('{' + CommandSerialNumber + "_ok}");
          }
          break;
        case 2: /*Remote switching mode  processing <command：N 2>*/
          {
            Serial_mode = Serial_rocker;
            int SpeedRocker = doc["D2"];
            if (SpeedRocker != 0)
            {
              carSpeed_rocker = SpeedRocker;
            }
            if (1 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = LEFT;
              // Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (2 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = RIGHT;
              // Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (3 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = FORWARD;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (4 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = BACK;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (5 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = STOP;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (6 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = LEFT_FORWARD;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (7 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = LEFT_BACK;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (8 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = RIGHT_FORWARD;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (9 == doc["D1"])
            {
              func_mode = Bluetooth;
              mov_mode = RIGHT_BACK;
              //Serial.print('{' + CommandSerialNumber + "_ok}");
            }
          }
          break;
        case 3: /*Remote switching mode  processing <command：N 3>*/
          {
            Serial_mode = Serial_rocker;
            if (1 == doc["D1"]) // Line Teacking Mode
            {
              func_mode = LineTeacking;
              Serial.print('{' + CommandSerialNumber + "_ok}");
            }
            else if (2 == doc["D1"]) //Obstacles Avoidance Mode
            {
              func_mode = ObstaclesAvoidance;
              Serial.print('{' + CommandSerialNumber + "_ok}");
            }
          }
          break;
        case 4: /*Motion module  processing <command：N 4>*/
          {
            Serial_mode = Serial_programming;
            func_mode = CMD_CarControl;
            CMD_CarDirection = doc["D1"];
            CMD_CarSpeed = doc["D2"];
            //CMD_CarTimer = doc["T"]; //AMA
            CMD_CarTimer = 1000u * (uint16_t)(doc["T"]); //comes in sec, transform to ms //AMA
            CMD_CarControl_Millis = millis(); //Get the timestamp
            //Serial.print("{ok}");
          }
          break;
        case 5: /*Clear mode  processing <command：N 5>*/
          {
            func_mode = CMD_ClearAllFunctions;
            Serial.print('{' + CommandSerialNumber + "_ok}");
          }

          break;
        case 6: /*CMD mode：angle Setting*/
          {
            uint8_t angleSetting = doc["D1"];
            ServoControl(angleSetting);
            Serial.print('{' + CommandSerialNumber + "_ok}");
          }

          break;
        case 21: /*Ultrasonic module  processing <command：N 21>*/
          {
            Serial_mode = Serial_programming;
            CMD_UltrasoundModuleStatus_Plus(doc["D1"]);
          }

          break;
        case 22: /*Trace module data processing <command：N 22>*/
          {
            Serial_mode = Serial_programming;
            CMD_TraceModuleStatus_Plus(doc["D1"]);
          }
          break;
        case 40:
          {
            Serial_mode = Serial_programming;
            func_mode = CMD_CarControlxxx;
            CMD_CarDirectionxxx = doc["D1"];
            CMD_CarSpeedxxx = doc["D2"];
            Serial.print('{' + CommandSerialNumber + "_ok}");
          }
          break;
        default:
          break;
      }
    }
  }
  else if (SerialPortData != "")
  {
    if (true == SerialPortData.equals("f"))
    {
      /*func_mode = CMD_CarControlxxx;
        CMD_CarDirectionxxx = 3;
        CMD_CarSpeedxxx = carSpeed; //180; //AMA*/
      func_mode = CMD_CarControl; //AMA better move for a limited time
      CMD_CarDirection = 3;
      CMD_CarSpeed = carSpeed; //180; //AMA
      CMD_CarTimer = 500; //ms //1; //s uint16
      CMD_CarControl_Millis = millis(); //Get the timestamp
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("b"))
    {
      /*func_mode = CMD_CarControlxxx;
        CMD_CarDirectionxxx = 4;
        CMD_CarSpeedxxx = carSpeed; //180; //AMA*/
      func_mode = CMD_CarControl; //AMA better move for a limited time
      CMD_CarDirection = 4;
      CMD_CarSpeed = carSpeed; //180; //AMA
      CMD_CarTimer = 500; //ms //1; //s uint16
      CMD_CarControl_Millis = millis(); //Get the timestamp
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("l"))
    {
      /*func_mode = CMD_CarControlxxx;
        CMD_CarDirectionxxx = 1;
        CMD_CarSpeedxxx = carSpeed; //180; //AMA*/
      func_mode = CMD_CarControl; //AMA better move for a limited time
      CMD_CarDirection = 1;
      CMD_CarSpeed = carSpeed; //180; //AMA
      CMD_CarTimer = 500; //ms //1; //s uint16
      CMD_CarControl_Millis = millis(); //Get the timestamp
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("r"))
    {
      /*func_mode = CMD_CarControlxxx;
        CMD_CarDirectionxxx = 2;
        CMD_CarSpeedxxx = carSpeed; //180; //AMA*/
      func_mode = CMD_CarControl; //AMA better move for a limited time
      CMD_CarDirection = 2;
      CMD_CarSpeed = carSpeed; //180; //AMA
      CMD_CarTimer = 500; //ms //1; //s uint16
      CMD_CarControl_Millis = millis(); //Get the timestamp
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("s"))
    {
      func_mode = Bluetooth;
      //func_mode = CMD_ClearAllFunctions; //AMA is this ok?
      music_mode = false;
      //RGB LED fun off
      digitalWrite(RGB1_PIN, LOW);
	  digitalWrite(RGB2_PIN, LOW);
	  digitalWrite(RGB3_PIN, LOW);
	
      mov_mode = STOP;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("1"))
    {
      func_mode = LineTeacking;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("2"))
    {
      func_mode = ObstaclesAvoidance;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("3")) //AMA
    {
      func_mode = Follow_mode;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("*")) //AMA
    {
      if (carSpeed > MINIMUM_MOTOR_SPEED) carSpeed -= 10; //Reduce carSpeed
      TimerFreeTone(BEEP_PIN, 150 + carSpeed, 200); //feedback
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("#")) //AMA
    {
      if (carSpeed < MAXIMUM_MOTOR_SPEED) carSpeed += 10; //increase carSpeed
      TimerFreeTone(BEEP_PIN, 150 + carSpeed, 200); //feedback
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("0")) //AMA
    {
      /*digitalWrite(BEEP_PIN, HIGH);   // turn the buzzer on
        delay(250); // ms
        digitalWrite(BEEP_PIN, LOW);   // turn the buzzer off */
      /*for (int thisNote = 0; thisNote < 8; thisNote++) // Loop through the notes in the array.
        {
        TimerFreeTone(BEEP_PIN, melody[thisNote], duration[thisNote]); // Play melody[thisNote] for duration[thisNote].
        delay(50); // Short delay between notes.
        }*/
      //play();
      music_select(0u);
      music_mode = !music_mode;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("4")) //AMA
    {
      music_select(1u);
      music_mode = !music_mode;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("5")) //AMA
    {
      music_select(2u);
      music_mode = !music_mode;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("6")) //AMA
    {
      music_select(3u);
      music_mode = !music_mode;
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("7")) //AMA
    {
      digitalWrite(RGB1_PIN, !digitalRead(RGB1_PIN)); //toggle the pin state
      TimerFreeTone(BEEP_PIN, NOTE_C5, 200); //feedback
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("8")) //AMA
    {
      digitalWrite(RGB2_PIN, !digitalRead(RGB2_PIN)); //toggle the pin state
      TimerFreeTone(BEEP_PIN, NOTE_E5, 200); //feedback
      SerialPortData = "";
    }
    else if (true == SerialPortData.equals("9")) //AMA
    {
      digitalWrite(RGB3_PIN, !digitalRead(RGB3_PIN)); //toggle the pin state
      TimerFreeTone(BEEP_PIN, NOTE_G5, 200); //feedback
      SerialPortData = "";
    }
  }
}

void setup(void)
{
  Serial.begin(9600); //initialization
  ServoControl(90);
  irrecv.enableIRIn(); //Enable infrared communication NEC

  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(IN1, OUTPUT); //Motor-driven port configuration
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LineTeacking_Pin_Right, INPUT); //Infrared tracking module port configuration
  pinMode(LineTeacking_Pin_Middle, INPUT);
  pinMode(LineTeacking_Pin_Left, INPUT);

  //AMA
  pinMode(LED_Pin, OUTPUT);
  pinMode(BEEP_PIN, OUTPUT);
  pinMode(RGB1_PIN, OUTPUT);
  pinMode(RGB2_PIN, OUTPUT);
  pinMode(RGB3_PIN, OUTPUT);
}

void loop(void)
{
  music_play(); //AMA

  getBTData_Plus();           //Bluetooth data acquisition
  getIRData();                //Infrared data acquisition
  bluetooth_mode();           //Bluetooth remote mode
  irremote_mode();            //Infrared NEC remote control mode
  line_teacking_mode();       //Line Teacking Mode
  obstacles_avoidance_mode(); //Obstacles Avoidance Mode
  ApplicationFunctionSet_Follow(); //Follow_mode AMA

  CMD_Distance = getDistance(); //Ultrasonic measurement distance
  /*CMD_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
  CMD_MotorControl_Plus(CMD_MotorSelection, CMD_MotorDirection, CMD_MotorSpeed); //Control motor steering
  /*  CMD mode：<Car control> APP control car*/
  CMD_CarControl_Plus(CMD_CarDirection, CMD_CarSpeed, CMD_CarTimer); //Control the direction of the car<Time limited>
  CMD_CarControl_Plusxxx(CMD_CarDirectionxxx, CMD_CarSpeedxxx);      //Control the direction of the car<No Time limited>
  CMD_ClearAllFunctionsXXX();
}
