
//----------------------------------------------------------------
//-- Zowi basic firmware v2 adapted to Otto
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Original project ZOWI Authors:  Anita de Prado: ana.deprado@bq.com
//--                  Jose Alberca:   jose.alberca@bq.com
//--                 Javier Isabel:  javier.isabel@bq.com
//--       Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--                   Irene Sanz : irene.sanz@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Otto has thanks to Zowi!
//-----------------------------------------------------------------
//-- MARCH 2018: modified for OTTO with DC motors (SMARS tracks - designed by Kevin Thomas)
//-- written for two N20 150rpm 6v DC motors via motor controllers, A new Ultrasonic library has been used
//-- This sketch created by Jason Snow
//-- OTTO created by Camilo Parra Palacio

#include "MaxMatrix.h"
MaxMatrix ledmatrix=MaxMatrix(12,10,11, 1); //PIN  12=DIN, PIN 10=CS, PIN 11=CLK

//-- Otto Library

#include "OttoDC_sounds.h"
#include "OttoDC_mouths.h"
#include "OttoDC_gestures.h"
#include "BatReader.h"
#include <Ultrasonic.h>  // library by J.Rodrigo 
#include <EEPROM.h>

Ultrasonic ultrasonic(8,9); // (Trig PIN,Echo PIN)

BatReader battery;
//-- Library to manage serial commands
#include <OttoSerialCommand.h>
OttoSerialCommand SCmd;  //The SerialCommand object

//---------------------------------------------------------
//--Configure the pins where the motor drivers are attached
/*
          --------------- 
         |     O   O     |
         |---------------|
PWM 3==>||               || <== PWM 5
DIR 2==>| ------  ------  | <== DIR 4
        |                 |
*/

// buzzer is now connected to pin 13

///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////
const char programID[]="Otto_Wheels"; //Each program will have a ID
//---------------------------------------------------------
//-- Otto has 5 modes:
//--    * MODE = 0: Otto is awaiting  
//--    * MODE = 1: Dancing mode!  
//--    * MODE = 2: Obstacle detector mode  
//--    * MODE = 3: Noise detector mode   
//--    * MODE = 4: Otto APP mode (listening SerialPort). 
//---------------------------------------------------------
volatile int MODE=0; //State of Otto in the principal state machine. 
int modeId=0;       //Number of mode
unsigned long previousMillis=0;
unsigned long int getMouthShape(int number);
unsigned long int getAnimShape(int anim, int index);
int distance = 0; // ultrasonic distance measured variable
int randomDance=0;
int randomSteps=0;
int gesture = 0;
int gesture1 = 0;
bool obstacleDetected = false;// ultrasonic object detected logic
int newmouth = 0;
bool goingforward = false; // motor direction logic
bool goingreverse = false; // motor direction logic
bool goingleft = false; // motor direction logic
bool goingright = false; // motor direction logic
bool APPleftFORWARD = false; // motor direction logic for APP mode 4
bool APPleftREVERSE = false; // motor direction logic for APP mode 4
bool APPrightFORWARD = false; // motor direction logic for APP mode 4
bool APPrightREVERSE = false; // motor direction logic for APP mode 4
int leftSPEED = 0; //  left motor speed variable
int rightSPEED = 0; //  right motor speed variable
int mDelay = 10; // motor delay before changing movement
int lSpeed = 150; // motor speed (higher is faster)
int rSpeed = 150; // motor speed (higher is faster)
int APPleftSPEED = 0; //  left motor speed variable from APP mode 4
int APPrightSPEED = 0; //  right motor speed variable from APP mode 4
bool AUXcontrol1 = false; // logic for aux control switch 1 from app
bool AUXcontrol2 = false; // logic for aux control switch 2 from app 
int noiseLevel = 0; // noise variable to hold the average measured noise level
int noiseReadings = 0; // noise raw reading variable for actual measured noise level
int numReadings = 2; // number of readings for the average noise reading
#define mE1 6 // motor 1 speed PWM pin
#define mE2 5 // motor 2 speed PWM pin
#define mI1 2 // motor 1 direction pin
#define mI2 4 // motor 2 direction pin
#define OUT1 A1 // Aux output 1 pin A1
#define OUT2 A2 // Aux output 1 pin A2
#define pinNoiseSensor A6 //noise analog input pin
int calibration = 0; // used if one motor is faster than the other due to mechanical differences
int melody[] = {
  note_C4, note_G3, note_G3, note_A3, note_G3, 0, note_B3,note_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};
///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){
  rSpeed = (rSpeed - calibration); // make calibration adjustment
  pinMode(mE1, OUTPUT);
  pinMode(mE2, OUTPUT);
  pinMode(mI1, OUTPUT);
  pinMode(mI2, OUTPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  digitalWrite (OUT1, LOW);
  digitalWrite (OUT2, LOW);
  Serial.begin(19200);  // for default Bluetooth module use:  Serial.begin(9600);  
  newmouth = 0;
// set up Matrix display
  ledmatrix.init();
  ledmatrix.setIntensity(1);
  //Set a random seed
  randomSeed(analogRead(A6));
//Setup callbacks for SerialCommand commands 
  SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
  SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
  SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
  SCmd.addCommand("E", requestName);
  SCmd.addCommand("D", requestDistance);
  SCmd.addCommand("T", receiveAUX);     // receive aux switches
//SCmd.addCommand("N", requestNoise);
  SCmd.addCommand("B", requestBattery); // 
  SCmd.addCommand("I", requestProgramId);
   SCmd.addCommand("J", requestMode);
 //SCmd.addCommand("P", requestRGB);
  SCmd.addDefaultHandler(receiveStop);

  //Otto wake up!
  delay(500);
  
  //Battery voltage is measured on pin A7 (MAX 5Volt DO NOT EXCEED THIS VOLTAGE)
  // Designed for a 3v7 LIPO or L-ION rechargable batteries
  // BAT_MAX  4.2   - 100% battery level
  // BAT_MIN  3.25 - 0% battery level
  //OttoLowBatteryAlarm(); // check battery level
//  OTTO is ALIVE!!!!!!!  
// Animation Uuuuuh - A little moment of initial surprise
  for(int i=0; i<2; i++){
      for (int i=0;i<8;i++){
        putAnimationMouth(littleUuh,i);
        delay(250);
      }
  }
  putMouth(smile);
  delay(250);
  previousMillis = millis();
// OTTO sings a little tune of start up
// notes in the melody:

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(13, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(13);
  }
  //OTTO has these animations, gestures, sounds, mouths
  //putAnimationMouth(littleUuh); //8 parts
  //putAnimationMouth(dreamMouth); //4 parts
  //putAnimationMouth(adivinawi); //6 parts
  //putAnimationMouth(wave); //10 parts
  //playGesture(OttoHappy);
  //playGesture(OttoSuperHappy);
  //playGesture(OttoSad);
  //playGesture(OttoSleeping);
  //playGesture(OttoFart);
  //playGesture(OttoConfused);
  //playGesture(OttoLove);
  //playGesture(OttoAngry);
  //playGesture(OttoFretful);
  //playGesture(OttoMagic);
  //playGesture(OttoWave);
  //playGesture(OttoVictory);
  //playGesture(OttoFail);
  //soundcmd(S_connection);
  //soundcmd(S_disconnection2);
  //soundcmd(S_buttonPushed);
  //soundcmd(S_mode1);
  //soundcmd(S_mode2);
  //soundcmd(S_mode3);
  //soundcmd(S_surprise);
  //soundcmd(S_OhOoh);
  //soundcmd(S_OhOoh2);  
  //soundcmd(S_cuddly);
  //soundcmd(S_sleeping);
  //soundcmd(S_happy);
  //soundcmd(S_superHappy);
  //soundcmd(S_happy_short);
  //soundcmd(S_sad);
  //soundcmd(S_fart1);
  //soundcmd(S_fart2);
  //soundcmd(S_fart3);
  //soundcmd(S_track_1);
  //putMouth(zero);
  //putMouth(one);
  //putMouth(two);
  //putMouth(three);
  //putMouth(four);
  //putMouth(five);
  //putMouth(six);
  //putMouth(seven);
  //putMouth(eight);
  //putMouth(nine);
  //putMouth(smile);
  //putMouth(happyOpen);
  //putMouth(happyClosed);
  //putMouth(heart);
  //putMouth(bigSurprise);
  //putMouth(smallSurprise);
  //putMouth(tongueOut);
  //putMouth(vamp1);
  //putMouth(vamp2);
  //putMouth(lineMouth);
  //putMouth(confused);
  //putMouth(diagonal);
  //putMouth(sad);
  //putMouth(sadOpen);
  //putMouth(sadClosed);
  //putMouth(okMouth);
  //putMouth(xMouth);
  //putMouth(interrogation);
  //putMouth(thunder);
  //putMouth(culito);
  //putMouth(angry);
  delay(1000);
}
 
///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop()
{
    if (Serial.available()>0 && MODE!=4)
    {
    SCmd.readSerial();
    putMouth(happyOpen);
   }
 
    switch (MODE) 
     {
            //-- MODE 0 - Otto is awaiting
      //---------------------------------------------------------
    case 0:
      
        //Every 80 seconds in this mode, Otto falls asleep as he is bored!
        if (millis()-previousMillis>=80000)
        {
            OttoSleeping_withInterrupts(); //ZZzzzzz...
            previousMillis=millis();         
        }

        break;
      

      //-- MODE 1 - Dance Mode!
      //---------------------------------------------------------
      case 1:
        // THIS Otto can not dance - he has no legs, so here he displays a random mouth
        // change this for what ever you would like Otto to do in mode 1
        // This would be ideal for a line following mode if you can add some additional
        // sensors to Otto, 
        putMouth(random(10,21));
        delay(1000);
        break;


      //-- MODE 2 - Obstacle detector mode
      //---------------------------------------------------------
      case 2:
        if(obstacleDetected)
        {
          motorstop(); // stop motors
            soundcmd(S_OhOoh);
            delay(100);
              putMouth(bigSurprise);
              delay(500);
              putMouth(confused);
              delay(500);
              soundcmd(S_OhOoh2);
            //Otto drives back
            putMouth(xMouth);
             //delay(1000);
             motormoveBackward(); // moveBackwards
              delay(1500); 
              motorstop();
              delay(500);           
              putMouth(smile);
              delay(1500);
                motorturnLeft(); // turn small amount left
                delay(300);
                motorstop();
                obstacleDetector();
                delay(250);
                
           
            //If there are no obstacles, Otto is happy
            if(obstacleDetected==true){break;}           
            
                obstacleDetected=false;
                putMouth(happyOpen);
                soundcmd(S_happy_short);
                delay(500);
                
            }
        else
            {

    //select a random number between 1 and 5
      if (newmouth > 20)
          {
      gesture = random(1,5);
      // display the relevant gesture for the random number selected
      switch (gesture) {
      case 1: //H 1 
        putMouth(happyOpen);
        soundcmd(S_happy);
        putMouth(happyClosed);
        delay(500);
        putMouth(happyOpen);
        soundcmd(S_superHappy);
        putMouth(happyClosed);
        break;
      case 2: //H 2 
        putMouth(smile);
        soundcmd(S_cuddly);
        break;
      case 3: //H 3 
        putMouth(happyClosed);
        break;
      case 4: //H 4 
         putMouth(tongueOut);  
         soundcmd(13);  
        break;
      case 5: //H 5  
        putMouth(lineMouth);
        break;
        
          }
      newmouth = 0;
          }

            //Otto Drive straight
            motormoveForward(); // moveForward
            obstacleDetector(); // check for obstacles
            delay(100);
            obstacleDetector(); // check for obstacles
            delay(100);
            newmouth = (newmouth + 1);

      }   

        break;
        //-- MODE 3 - Noise detector mode
      //--------------------------------------------------------- 
        case 3:
        noiseLevel = 0;
        getNoise();
        if (noiseLevel >=650){ //740
         // if we detect noise then we do something - change this for what ever you want Otto to do if there
         // is a noise, change the value for noise levels
          delay(50); 
            putMouth(bigSurprise);
            soundcmd(S_OhOoh);
            delay(500); //
            putMouth(random(10,21));
            delay(500); //
            randomDance=random(5,21);
            delay(500); //
            putMouth(happyOpen);
        }
        break;
      //-- MODE 4 - Otto APP mode (listening SerialPort) 
      //---------------------------------------------------------
      case 4:

        SCmd.readSerial();

        break;      

      default:
          MODE=0;
          break;
 }
      
}

///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////

//-- Function to read distance sensor & to actualize obstacleDetected variable
void obstacleDetector(){
  distance = (ultrasonic.Ranging(CM));  
 //Serial.println(distance);
        if(distance<29){ // if object detected closer than 29 then take action
          obstacleDetected = true;
        }else{
          obstacleDetected = false;
        }
}

//-- Functions with animatics
//--------------------------------------------------------

void OttoLowBatteryAlarm(){

     double batteryLevel = getBatteryLevel();
    //A7 = 880 = 4.2v = 100%
    //A7 = 758 = 3.7v = 50%
    if(batteryLevel<45){
        
          putMouth(thunder);
          bendTones (880, 2000, 1.04, 8, 3);  
          delay(30);
          bendTones (2000, 880, 1.02, 8, 3);  
          delay(1000);
          clearMouth();
          delay(500);
     } 
}

void motormoveForward() {
// STOP motors before direction change to help protect motor drivers
if (goingforward == false) motorstop();
 // DIR motor
    digitalWrite(mI1, LOW);//left motor direction
    digitalWrite(mI2, LOW);//right motor direction
     // PWM motor
    analogWrite(mE1, lSpeed);//left motor speed
    analogWrite(mE2, rSpeed);//right motor speed
    goingreverse = false;
    goingforward = true;
    goingleft = false;
    goingright = false; 
  }

/**
 * Move backward
 */
void motormoveBackward() {
// STOP motors before direction change to help protect motor drivers
if (goingreverse == false) motorstop();  
    // DIR motor
    digitalWrite(mI1, HIGH);//left motor direction
    digitalWrite(mI2, HIGH);//right motor direction
    // PWM motor
    analogWrite(mE1, 255 - lSpeed);
    analogWrite(mE2, 255 - rSpeed);
    goingreverse = true;
    goingforward = false;
    goingleft = false;
    goingright = false; 
}

/**
 * Turn Left
 */
void motorturnLeft() {
// STOP motors before direction change to help protect motor drivers
if (goingleft == false) motorstop();
    // DIR motor   
    digitalWrite(mI1, HIGH);
    digitalWrite(mI2, LOW);
    // PWM motor
    analogWrite(mE1, 255 - lSpeed);
    analogWrite(mE2, rSpeed);
    goingreverse = false;
    goingforward = false;
    goingleft = true;
    goingright = false; 

}

/**
 * Turn Right
 */
void motorturnRight() {
 // STOP motors before direction change to help protect motor drivers
if (goingright == false) motorstop();   
    // DIR motor 
    digitalWrite(mI1, LOW);
    digitalWrite(mI2, HIGH);
     // PWM motor
    analogWrite(mE1, lSpeed);
    analogWrite(mE2, 255 - rSpeed);
    goingreverse = false;
    goingforward = false;
    goingleft = false;
    goingright = true;  
}

/**
 * Stop Bot
 */
void motorstop() {
    // PWM motor
    digitalWrite(mE1, LOW);
    digitalWrite(mE2, LOW);
    // DIR motor 
    digitalWrite(mI1, LOW);
    digitalWrite(mI2, LOW);
}
// OTTO SOUNDS
void tonenew (float noteFrequency, long noteDuration, int silentDuration) {  

  if (silentDuration == 0) {silentDuration = 1;} 
  tone (13, noteFrequency, noteDuration); 

  delay (noteDuration); // milliseconds to microseconds 
  delay (silentDuration); 
} 

void bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration) { 

  if (silentDuration == 0) {silentDuration = 1;} 

  if (initFrequency <finalFrequency) 
    { 
    for (int i = initFrequency; i <finalFrequency; i = i * prop) { 
    tonenew (i, noteDuration, silentDuration); 
    } 

  } 
else { 

    for (int i = initFrequency; i> finalFrequency; i = i / prop) { 
    tonenew (i, noteDuration, silentDuration); 
    } 
  } 
} 

void soundcmd (int cmd) 
{ 
  switch (cmd) 
  { 
  case 1: //connection
  tonenew (note_E5, 50, 30); 
  tonenew (note_E6, 55, 25); 
  tonenew (note_A6, 60, 10); 
  break; 
  case 2: //disconnection
  tonenew (note_E5, 50, 30); 
  tonenew (note_A6, 55, 25); 
  tonenew (note_E6, 50, 10); 
  break; 
  case 3: //buttonPushed
  bendTones (note_E6, note_G6, 1.03, 20, 2); 
  delay (30); 
  bendTones (note_E6, note_D7, 1.04, 10, 2); 
  break; 
  case 4: //mode1
  bendTones (note_E6, note_A6, 1.02, 30, 10); //1318.51 to 1760 
  break; 
  case 5: //mode2
  bendTones (note_G6, note_D7, 1.03, 30, 10); //1567.98 to 2349.32 
  break; 
  case 6: //mode3
  tonenew (note_E6, 50, 100); // D6 
  tonenew (note_G6, 50, 80); // E6 
  tonenew (note_D7, 300, 0); // G6 
  break; 
  case 7: //surprise
  bendTones (800, 2150, 1.02, 10, 1); 
  bendTones (2149, 800, 1.03, 7, 1); 
  break; 
  case 8: //OhOoh
  bendTones (880, 2000, 1.04, 8, 3); // A5 = 880 
  delay (200); 

  for (int i = 880; i <2000; i = i * 1.04) { 
    tonenew (note_B5, 5, 10); 
  } 
  break;
  case 9: //OhOoh2
  bendTones (1880, 3000, 1.03, 8, 3); 
  delay (200); 

  for (int i = 1880; i <3000; i = i * 1.03) { 
    tonenew (note_C6, 10, 10); 
  } 
  break; 
  case 10: //cuddly
  bendTones (700, 900, 1.03, 16, 4); 
  bendTones (899, 650, 1.01, 18, 7); 
  break; 
  case 11: //sleeping
  bendTones (100, 500, 1.04, 10, 10); 
  delay (500); 
  bendTones (400, 100, 1.04, 10, 1); 
  break; 
  case 12: //happy
  bendTones (1500, 2500, 1.05, 20, 8); 
  bendTones (2499, 1500, 1.05, 25, 8); 
  break; 
  case 13: //superHappy
  bendTones (2000, 6000, 1.05, 8, 3); 
  delay (50); 
  bendTones (5999, 2000, 1.05, 13, 2); 
  break; 
  case 14: //happy_short
  bendTones (1500, 2000, 1.05, 15, 8); 
  delay (100); 
  bendTones (1900, 2500, 1.05, 10, 8); 
  break; 
  case 15: //sad
  bendTones (880, 669, 1.02, 20, 200); 
  break; 
  case 16: //confused
  bendTones (1000, 1700, 1.03, 8, 2); 
  bendTones (1699, 500, 1.04, 8, 3); 
  bendTones (1000, 1700, 1.05, 9, 10); 
  break; 
  case 17: //fart1
  bendTones (1600, 3000, 1.02, 2, 15); 
  break; 
  case 18: //fart2
  bendTones (2000, 6000, 1.02, 2, 20); 
  break; 
  case 19: //fart3
  bendTones (1600, 4000, 1.02, 2, 20); 
  bendTones (4000, 3000, 1.02, 2, 20); 
  break; 
  case 20: 
  tonenew (note_A6,30,5); 
  break; 
  default: 

  break; 
  } 
} 
// OTTO MOUTHS
unsigned long int getMouthShape(int number){
unsigned long int types []={zero_code,one_code,two_code,three_code,four_code,five_code,six_code,seven_code,eight_code,
  nine_code,smile_code,happyOpen_code,happyClosed_code,heart_code,bigSurprise_code,smallSurprise_code,tongueOut_code,
  vamp1_code,vamp2_code,lineMouth_code,confused_code,diagonal_code,sad_code,sadOpen_code,sadClosed_code,
  okMouth_code, xMouth_code,interrogation_code,thunder_code,culito_code,angry_code};

  return types[number];
}

void putMouth(unsigned long int mouth){

    ledmatrix.writeFull(getMouthShape(mouth));
 
}

void clearMouth(){

  ledmatrix.clearMatrix();
}

//  OTTO ANIMATIONS
unsigned long int getAnimShape(int anim, int index){

  unsigned long int littleUuh_code[]={
     0b00000000000000001100001100000000,
     0b00000000000000000110000110000000,
     0b00000000000000000011000011000000,
     0b00000000000000000110000110000000,
     0b00000000000000001100001100000000,
     0b00000000000000011000011000000000,
     0b00000000000000110000110000000000,
     0b00000000000000011000011000000000  
  };

  unsigned long int dreamMouth_code[]={
     0b00000000000000000000110000110000,
     0b00000000000000010000101000010000,  
     0b00000000011000100100100100011000,
     0b00000000000000010000101000010000           
  };

  unsigned long int adivinawi_code[]={
     0b00100001000000000000000000100001,
     0b00010010100001000000100001010010,
     0b00001100010010100001010010001100,
     0b00000000001100010010001100000000,
     0b00000000000000001100000000000000,
     0b00000000000000000000000000000000
  };

  unsigned long int wave_code[]={
     0b00001100010010100001000000000000,
     0b00000110001001010000100000000000,
     0b00000011000100001000010000100000,
     0b00000001000010000100001000110000,
     0b00000000000001000010100100011000,
     0b00000000000000100001010010001100,
     0b00000000100000010000001001000110,
     0b00100000010000001000000100000011,
     0b00110000001000000100000010000001,
     0b00011000100100000010000001000000    
  };

  switch  (anim){

    case littleUuh:
        return littleUuh_code[index];
        break;
    case dreamMouth:
        return dreamMouth_code[index];
        break;
    case adivinawi:
        return adivinawi_code[index];
        break;
    case wave:
        return wave_code[index];
        break;    
  }   
}


void putAnimationMouth(unsigned long int aniMouth, int index){

     ledmatrix.writeFull(getAnimShape(aniMouth,index));
}

void putMouth(unsigned long int mouth, bool predefined){

  if (predefined)
      {
    ledmatrix.writeFull(getMouthShape(mouth));
      }
  else{
    ledmatrix.writeFull(mouth);
      }
}

////////////////////////////////////////////////////////
void playGesture(int gesture1){
switch(gesture1){

    case OttoHappy: 
        tonenew(note_E5,50,30);
        putMouth(smile);
        soundcmd(S_happy_short); 
        soundcmd(S_happy_short);
        putMouth(happyOpen);
    break;


    case OttoSuperHappy:
        putMouth(happyOpen);
        soundcmd(S_happy);
        putMouth(happyClosed);
        delay(1000);
        putMouth(happyOpen);
        soundcmd(S_superHappy);
        putMouth(happyClosed); 
        delay(1000);
        putMouth(happyOpen);
    break;


    case OttoSad: 
        putMouth(sad);
        bendTones(880, 830, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(830, 790, 1.02, 20, 200);  
        putMouth(sadOpen);
        bendTones(790, 740, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(740, 700, 1.02, 20, 200);
        putMouth(sadOpen);
        bendTones(700, 669, 1.02, 20, 200);
        putMouth(sad);
        delay(1500);
        putMouth(happyOpen);
    break;


    case OttoSleeping:

        for(int i=0; i<4;i++){
          putAnimationMouth(dreamMouth,0);
          bendTones (100, 200, 1.04, 10, 10);
          putAnimationMouth(dreamMouth,1);
          bendTones (200, 300, 1.04, 10, 10);  
          putAnimationMouth(dreamMouth,2);
          bendTones (300, 500, 1.04, 10, 10);   
          delay(500);
          putAnimationMouth(dreamMouth,1);
          bendTones (400, 250, 1.04, 10, 1); 
          putAnimationMouth(dreamMouth,0);
          bendTones (250, 100, 1.04, 10, 1); 
          delay(500);
        } 

        putMouth(lineMouth);
        soundcmd(S_cuddly);

   
        putMouth(happyOpen);
    break;


    case OttoFart:
        delay(300);     
        putMouth(lineMouth);
        soundcmd(S_fart1);  
        putMouth(tongueOut);
        delay(500);
        putMouth(lineMouth);
        soundcmd(S_fart2); 
        putMouth(tongueOut);
        delay(500);
        putMouth(lineMouth);
        soundcmd(S_fart3);
        putMouth(tongueOut);    
        delay(1000); 
        putMouth(happyOpen);
    break;


    case OttoConfused:
        putMouth(confused);
        soundcmd(S_confused);
        delay(1000);
        putMouth(happyOpen);
    break;


    case OttoLove:
        putMouth(heart);
        soundcmd(S_cuddly);
        soundcmd(S_happy_short);  
        putMouth(happyOpen);
    break;


    case OttoAngry: 
        putMouth(angry);
        tonenew(note_A5,100,30);
        bendTones(note_A5, note_D6, 1.02, 7, 4);
        bendTones(note_D6, note_G6, 1.02, 10, 1);
        bendTones(note_G6, note_A5, 1.02, 10, 1);
        delay(15);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(400);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        putMouth(happyOpen);
    break;

    case OttoFretful: 
        putMouth(angry);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(750);
        putMouth(lineMouth);
        delay(750);
        putMouth(angry);
        delay(1000);
        putMouth(happyOpen);
    break;


    case OttoMagic:
       
        // Reproduce the animation four times
        for(int i = 0; i<4; i++){ 

          int noteM = 400; 

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //400 -> 1000 
              noteM+=100;
            }

            clearMouth();
            bendTones(noteM-100, noteM+100, 1.04, 10, 10);  //900 -> 1100

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //1000 -> 400 
              noteM-=100;
            }
        } 
 
        delay(1000);
        putMouth(happyOpen);
    break;


    case OttoWave:
        
        // Reproduce the animation four times
        for(int i = 0; i<2; i++){ 

            int noteW = 500; 

            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
        }    

        clearMouth();
        delay(100);
        putMouth(happyOpen);
    break;

    case OttoVictory:
        
        putMouth(smallSurprise);
        for (int i = 0; i < 60; ++i){
                    tonenew(1600+i*20,15,1);
        }

        putMouth(bigSurprise);
        for (int i = 0; i < 60; ++i){
                    tonenew(2800+i*20,15,1);
        }

        putMouth(happyOpen);
        soundcmd(S_superHappy);
        putMouth(happyClosed);
        delay(1000);
        clearMouth();
        putMouth(happyOpen);

    break;

    case OttoFail:

        putMouth(sadOpen);
        tonenew(900,200,1);
        putMouth(sadClosed);
        tonenew(600,200,1);
        putMouth(confused);
        tonenew(300,200,1);
        putMouth(xMouth);
        tonenew(150,2200,1);
        
        delay(750);
        clearMouth();
        putMouth(happyOpen);

    break;

  }
}    

//-- Function to send program ID
void requestProgramId(){

    Serial.print(F("&&"));
    Serial.print(F("I "));
    Serial.print(programID);
    Serial.println(F("%%"));
    Serial.flush();
}

//-- Function to send Ack comand (A)
void sendAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("A"));
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send final Ack comand (F)
void sendFinalAck(){

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("F"));
  Serial.println(F("%%"));
  Serial.flush();
}

//-- Function to send ultrasonic sensor measure (distance in "cm")
void requestDistance(){

    //Otto.home();  //stop if necessary  
     obstacleDetector();
    Serial.print(F("&&"));
    Serial.print(F("D "));
    Serial.print(distance);
    Serial.println(F("%%"));
    Serial.flush();
}
//-- Function to send Otto's name
void requestName(){

    //Otto.home(); //stop if necessary

    char actualOttoName[11]= "";  //Variable to store data read from EEPROM.
    int eeAddress = 5;            //EEPROM address to start reading from

    //Get the float data from the EEPROM at position 'eeAddress'
    EEPROM.get(eeAddress, actualOttoName);

    Serial.print(F("&&"));
    Serial.print(F("E "));
    Serial.print(actualOttoName);
    Serial.println(F("%%"));
    Serial.flush();
}
//-- Function to receive sing commands
void receiveSing(){

    //sendAck & stop if necessary
    sendAck();
    //Definition of Sing Bluetooth commands
    //K  SingID    
    int sing = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {sing=atoi(arg);}
    else 
    {
     putMouth(xMouth);
      delay(2000);
      clearMouth();
    }

    switch (sing) {
      case 1: //K 1 
        soundcmd(S_connection);
        break;
      case 2: //K 2 
        soundcmd(S_disconnection);
        break;
      case 3: //K 3 
        soundcmd(S_surprise);
        break;
      case 4: //K 4 
        soundcmd(S_OhOoh);
        break;
      case 5: //K 5  
        soundcmd(S_OhOoh2);
        break;
      case 6: //K 6 
        soundcmd(S_cuddly);
        break;
      case 7: //K 7 
        soundcmd(S_sleeping);
        break;
      case 8: //K 8 
        soundcmd(S_happy);
        break;
      case 9: //K 9  
        soundcmd(S_superHappy);
        break;
      case 10: //K 10
       soundcmd(S_happy_short);
        break;  
      case 11: //K 11
        soundcmd(S_sad);
        break;   
      case 12: //K 12
        soundcmd(S_confused);
        break; 
      case 13: //K 13
        soundcmd(S_fart1);
        break;
      case 14: //K 14
        soundcmd(S_fart2);
        break;
      case 15: //K 15
        soundcmd(S_fart3);
        break;    
      case 16: //K 16
        soundcmd(S_mode1);
        break; 
      case 17: //K 17
        soundcmd(S_mode2);
        break; 
      case 18: //K 18
        soundcmd(S_mode3);
        break;   
      case 19: //K 19
        soundcmd(S_buttonPushed);
        break;                      
      default:
        break;
    }

    sendFinalAck();
}
//-- Function to receive movement commands
void receiveMovement(){

    sendAck();
    //Otto.home();
    //Definition of Motor Bluetooth commands
    //M ModeID    
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) 
    {
      APPleftSPEED=atoi(arg);
      }
      else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
      APPleftSPEED=0; //stop
    }
    
    arg = SCmd.next(); 
    if (arg != NULL) {APPrightSPEED=atoi(arg);}
    else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
      APPrightSPEED=0;
    }
// D0 PWM motor control
// first map the value received from the app to a value between 0 and full speed
// the value from the app is positive for forward and negative for reverse
 if (APPleftSPEED < -1 ){
  leftSPEED = map(APPleftSPEED, 0, -100, 0, 255);
 }
  else
  {
   leftSPEED = map(APPleftSPEED, 0, 100, 0, 255);
  }
 if (APPrightSPEED < -1 ){
  rightSPEED = map(APPrightSPEED, 0, -100, 0, 255);
 }
 else 
 {
  rightSPEED = map(APPrightSPEED, 0, 100, 0, 255);
 }

// just a little deadband

if (APPleftSPEED > 5 ){
    if (APPleftFORWARD == false){;
      digitalWrite(mE1, LOW);
      digitalWrite(mI1, LOW);
      delay(250);
    }
    // DIR motor
    digitalWrite(mI1, LOW);
    // PWM motor
    analogWrite(mE1,leftSPEED);
    APPleftREVERSE = false;
    APPleftFORWARD = true;
  }
  else  {
  if (APPleftSPEED < -5 ){
    if (APPleftREVERSE == false){;
      digitalWrite(mE1, LOW);
      digitalWrite(mI1, LOW);
      delay(250);
    }
    // DIR motor
    digitalWrite(mI1, HIGH);
    // PWM motor
    analogWrite(mE1, 255 - lSpeed);
    APPleftREVERSE = true;
    APPleftFORWARD = false;
  }
  else{
  digitalWrite(mI1, LOW);
    // PWM motor
    analogWrite(mE1, 0);
    goingreverse = false;
    goingforward = true;
  }
}
if (APPrightSPEED > 5 ){
  if (APPrightFORWARD == false){;
      digitalWrite(mE2, LOW);
      digitalWrite(mI2, LOW);
      delay(250);
    }
    // DIR motor
    digitalWrite(mI2, LOW);
    // PWM motor
    analogWrite(mE2,rightSPEED);
    APPrightREVERSE = false;
    APPrightFORWARD = true;
  }
  else  {
  if (APPrightSPEED < -5 ){
    if (APPrightREVERSE == false){;
      digitalWrite(mE2, LOW);
      digitalWrite(mI2, LOW);
      delay(250);
    }
    digitalWrite(mI2, HIGH);
    // PWM motor
     analogWrite(mE2, 255 - rSpeed);
    APPrightREVERSE = true;
    APPrightFORWARD = false;
  }
  else{
    digitalWrite(mI2, LOW);
    // PWM motor
    analogWrite(mE2, 0);
    APPrightFORWARD = false;
    APPrightFORWARD = true;

    }
  }

}

//-- Function to receive LED commands
void receiveLED(){  

    //sendAck & stop if necessary
    sendAck();
    unsigned long int matrix;
    char *arg;
    char *endstr;
    arg=SCmd.next();
    //Serial.println (arg);
    if (arg != NULL) {
      matrix=strtoul(arg,&endstr,2);    // Converts a char string to unsigned long integer
      putMouth(matrix,false);
    }else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
    }

    sendFinalAck();

}
//-- Function to receive Stop command.
void receiveStop(){

    sendAck();
    //Otto.home();
    sendFinalAck();

}

//-- Function to send battery voltage percent
void requestBattery(){

//The first read of the batery is often a wrong reading, so we will discard this value. 
     double batteryLevel = getBatteryLevel();

    Serial.print(F("&&"));
    Serial.print(F("B "));
    Serial.print(batteryLevel);
    Serial.println(F("%%"));
    Serial.flush();
}

double getBatteryLevel(){

  //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = battery.readBatPercent();
    double batteryReadings = 0;
    int numReadings = 10;

    for(int i=0; i<numReadings; i++){
        batteryReadings += battery.readBatPercent();
        delay(1); // delay in between reads for stability
    }

    batteryLevel = batteryReadings / numReadings;

    return batteryLevel;
}

void OttoSleeping_withInterrupts(){

  for(int i=0; i<4;i++){
      putAnimationMouth(dreamMouth,0);
      bendTones (100, 200, 1.04, 10, 10);
      putAnimationMouth(dreamMouth,1);
      bendTones (200, 300, 1.04, 10, 10);  
      putAnimationMouth(dreamMouth,2);
      bendTones (300, 500, 1.04, 10, 10);   
    delay(500);
      putAnimationMouth(dreamMouth,1);
      bendTones (400, 250, 1.04, 10, 1); 
      putAnimationMouth(dreamMouth,0);
      bendTones (250, 100, 1.04, 10, 1); 
    delay(500);
  } 

    putMouth(lineMouth);
    soundcmd(S_cuddly);
  
}
//-- Function to receive mode selection.
void requestMode(){

    sendAck();
    //Definition of Mode Bluetooth commands
    //J ModeID    
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) 
    {
      modeId=atoi(arg);
      putMouth(heart);
      }
    else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
      modeId=0; //stop
    }
switch (modeId) {
      case 0: //
        MODE = 0;
        break;
      case 1: //
        MODE = 1;
        soundcmd(S_mode1);
        putMouth(one);
        delay(1000);
    delay(200);
        break;
        case 2: //
        MODE = 2;
        soundcmd(S_mode2);
        putMouth(two);
        delay(1000);
        break;
        case 3: //
        MODE = 3;
        soundcmd(S_mode3);
        putMouth(three);
        delay(1000);        
        break;
        case 4: //
        soundcmd(S_mode1);
        putMouth(four);
        delay(1000);       
        MODE = 4;
        motorstop();
        break;
     default:
        MODE = 0;
        break;
   }
sendFinalAck();
}
//-- Function to receive gesture commands
void receiveGesture(){

    //sendAck & stop if necessary
    sendAck(); 
    
    int gesture = 0;
    char *arg; 
    arg = SCmd.next(); 
    if (arg != NULL) {gesture=atoi(arg);}
    else 
    {
      putMouth(xMouth);
      delay(2000);
      clearMouth();
    }

    switch (gesture) {
      case 1: //H 1 
        playGesture(OttoHappy);
        break;
      case 2: //H 2 
        playGesture(OttoSuperHappy);
        break;
      case 3: //H 3 
        playGesture(OttoSad);
        break;
      case 4: //H 4 
        playGesture(OttoSleeping);
        break;
      case 5: //H 5  
        playGesture(OttoFart);
        break;
      case 6: //H 6 
        playGesture(OttoConfused);
        break;
      case 7: //H 7 
        playGesture(OttoLove);
        break;
      case 8: //H 8 
        playGesture(OttoAngry);
        break;
      case 9: //H 9  
        playGesture(OttoFretful);
        break;
      case 10: //H 10
        playGesture(OttoMagic);
        break;  
      case 11: //H 11
        playGesture(OttoWave);
        break;   
      case 12: //H 12
        playGesture(OttoVictory);
        break; 
      case 13: //H 13
        playGesture(OttoFail);
        break;         
      default:
        break;
    }

    sendFinalAck();
}
void receiveAUX()
{
sendAck();
    //Definition of Mode Bluetooth commands
    //T aux switches from app   
    char *arg; 
    int Aux1;
    int Aux2;
    arg = SCmd.next(); 
    if (arg != NULL) 
    {
      Aux1=atoi(arg);
      }
      else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
      Aux1=0; //stop
    }
    
    arg = SCmd.next(); 
    if (arg != NULL) {Aux2=atoi(arg);}
    else{
      putMouth(xMouth);
      delay(2000);
      clearMouth();
      Aux2=0;
    }
putMouth(okMouth);
delay(750);
clearMouth();
if (Aux1 == 1){
    AUXcontrol1 = true;
  }
  else{
    AUXcontrol1 = false;
  }
if (Aux2 == 1){
    AUXcontrol2 = true;
  }
  else{
    AUXcontrol2 = false;
  }
    sendFinalAck();
digitalWrite (OUT1, AUXcontrol1); // turn ON OFF Aux 1 output
digitalWrite (OUT2, AUXcontrol2); // turn ON OFF Aux 2 output
}

void getNoise(){  

    noiseLevel = analogRead(pinNoiseSensor);

    for(int i=0; i<numReadings; i++){
        noiseReadings += analogRead(pinNoiseSensor);
        delay(4); // delay in between reads for stability
    }

    noiseLevel = noiseReadings / numReadings;

    return noiseLevel;
}



