#include <QTRSensors.h>

//QTRA
#define NUM_SENSORS              8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR   4  // average 8 analog samples per sensor reading (For noise suppression during calliberation)(Not Used ,later ,ill explain)
#define NUM_SAMPLES_PER_SENSORC  8  // average 4 analog samples per sensor reading (For running)
#define EMITTER_PIN              11 // emitter is controlled by digital pin 11(beacause same side on nano)
//-------------------------------------------------

//QTRA Sensor
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5, A6, A7},
                                        NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[8];
void drive_left();
void drive_right();
void drive_st();

//Motor Driver
// motor left
int LDIR1 =48;
int LDIR2 =49;
int LENB = 3;
// motor right
int RDIR1 =52;
int RDIR2 =53;
int RENB = 2;
//-------------------------------------------------
//PID CONTROL DEFINATIONS
//FOR IR.........
#define Kp  0.8//to be determined experimentally 
#define Kd  6.2//to be determined experimentally(Kp<Kd)
#define Ki 0 //to be determined experimentally
double last_error=0;
double pos;
double error;
double correction;
double integral;
double last_integral;

//....................................................
//MOTOR SPEED CONTROL PARAMETERS
#define basespeed 100
int rgtspdIR;
int lftspdIR;
//int rgtspdUSS;
//int lftspdUSS; 
//..........................................................

//PATH DEFINATIONS
int found_left;
int found_right;
int found_st;
int found_y;
int found_half_y;
int found_bent_y;
int found_black;
int found_white;
//...........................................................

//PATH GLOBALS...............................................
/*bool path11=false;
bool path22=false;
bool path33=false;*/
int Button1=24; //ps1
int Button2=25; //ps2
int Button3=26; //ps3
*/
//TIME DEFINATIONS
long current_time;
long start_time_left;
long start_time_right;
long start_time_st;
long duration;
//.............................................................


void setup() 
{
  pinMode(Button1,INPUT_PULLUP);
  pinMode(Button2,INPUT_PULLUP);
  pinMode(Button3,INPUT_PULLUP);
  //digitalWrite(push,LOW);
  //pinMode(push,INPUT);
  // put your setup code here, to run once:
  //QTRA Setup----------------------------------------------
  digitalWrite(13, HIGH);        // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 300; i++)  // make the calibration take about 10 seconds
    {
      /*if(i<150)
      {
        digitalWrite(LDIR1,HIGH);
        digitalWrite(LDIR2,LOW);
        digitalWrite(RDIR1,LOW);
        digitalWrite(RDIR2,HIGH);
        analogWrite(LENB,150);
        analogWrite(RENB,150);
      }
      else
      {
        digitalWrite(LDIR1,LOW);
        digitalWrite(LDIR2,HIGH);
        digitalWrite(RDIR1,HIGH);
        digitalWrite(RDIR2,LOW);
        analogWrite(LENB,150);
        analogWrite(RENB,150);
      }*/
      qtra.calibrate();// reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call) (Rotates at 100/255 % speed of motor)
      //int B1Value=digitalRead(Button1);
      //int B2Value=digitalRead(Button2);
      //int B3Value=digitalRead(Button3);
      /*if(B1Value==LOW)
        {
        path11=true;  
        }
      else
        {
        path11=false;
        }
      if(B2Value=LOW)
        {
        path22=true;
        }
      else
        {
        path22=false;
        }
      if(B3Value=LOW)
        {
        path33=true;
        }
      else
        {
        path33=false;
        } 
      }*/
  digitalWrite(13, LOW);         // turn off Arduino's LED to indicate we are through with calibration

 // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
  Serial.println();
   
  // print the calibration maximum values measured when emitters were on
   for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtra.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
   Serial.println();
   Serial.println();
  //---------------------------------------------------------- 
 }
}

//BASIC LINE FOLLOWING MOVEMENT FUNCTION---------------------------------------------------------------------
void Move()
{
  pos=qtra.readLine(sensorValues,QTR_EMITTERS_ON,1); //qtra.readLine(sensorValues,QTR_EMITTERS_ON,TRUE/1);for white on black
  Serial.print("POSITION");
  Serial.print("=");
  Serial.print(pos);
  Serial.println();
  if(pos>=3350 && pos<=3650)
  {
    digitalWrite(RDIR1,LOW);
    digitalWrite(LDIR1,LOW);
    digitalWrite(RDIR2,HIGH);
    digitalWrite(LDIR2,HIGH);
    analogWrite(RENB,basespeed);
    analogWrite(LENB,basespeed);
  }
  else
  {
    error=pos-3500; //*
    integral=error+last_integral;
    Serial.print("ERROR");
    Serial.print("=");
    Serial.print(error);
    Serial.println();
    correction=(Kp*error) + (Kd*(error-last_error)) + Ki*integral;
    last_error=error;
    last_integral=integral;
    rgtspdIR=constrain((basespeed-correction),0,100);
    lftspdIR=constrain((basespeed+correction),0,100);
    digitalWrite(RDIR1,LOW);
    digitalWrite(RDIR2,HIGH);
    digitalWrite(LDIR1,LOW);
    digitalWrite(LDIR2,HIGH);
    analogWrite(RENB,rgtspdIR);
    analogWrite(LENB,lftspdIR);
   }
}

//LANE RECOGNITION FUNCTION-----------------------------------------------------------------------------------------------------
void laneRecognition()
{
  if(sensorValues[0]>800 && sensorValues[1]>800 && sensorValues[2]<700 && sensorValues[3]<700 && sensorValues[4]<700 && sensorValues[5]<700 && sensorValues[6]<700 && sensorValues[7]<700)
  {
    found_left=1;
  }
  if(sensorValues[6]>800 && sensorValues[7]>800 && sensorValues[0]<700 && sensorValues[1]<700 && sensorValues[2]<700 && sensorValues[3]<700 && sensorValues[4]<700 && sensorValues[5]<700)
  {
    found_right=1;
  }
  if(sensorValues[0]>700 && sensorValues[1]>700 && sensorValues[2]<700 && sensorValues[3]<700 && sensorValues[4]<700 && sensorValues[5]<700 && sensorValues[6]<700 && sensorValues[7]>600)
  {
    found_half_y=1;
  }
  if(sensorValues[0]<600 && sensorValues[1]<600 && sensorValues[2]<600 && sensorValues[3]<600 && sensorValues[4]<600 && sensorValues[5]<600 && sensorValues[6]<600 && sensorValues[7]<600)
  {
    found_left=1;
    found_right=1;
  }
  /*    digitalWrite(LDIR1, LOW);
      digitalWrite(LDIR2, HIGH);
      digitalWrite(RDIR2, HIGH);
      digitalWrite(RDIR1, LOW);  
      analogWrite(RENB, 100);  
      analogWrite(LENB, 100);
      delay(300);
      */
  if(sensorValues[0]>800 && sensorValues[1]>800 && sensorValues[2]<700 && sensorValues[3]<700 && sensorValues[4]<700 && sensorValues[5]<700 && sensorValues[6]>800 && sensorValues[7]>800)
  {
    found_st=1;
  }
  if(sensorValues[0]<600 && sensorValues[1]<600 && sensorValues[2]>600 && sensorValues[3]>600 && sensorValues[4]>600 && sensorValues[5]<600 && sensorValues[6]<600 && sensorValues[7]<600)
  {
    found_y=1;
  }
  if(sensorValues[0]<800 && sensorValues[1]<800 && sensorValues[2]>700 && sensorValues[3]>700 && sensorValues[4]>700 && sensorValues[5]>700 && sensorValues[6]<800 && sensorValues[7]<800)
  {
    found_black=1;
  }
}

void path1()
{
  int i=1;
  int j=1;
  if(found_left==1)
  {
   drive_left();
   found_left=0;
  }
  else if(found_right==1)
  {
    if(i==2)
    {
      drive_st();
    }
    else
    {
     drive_right();
     i++; 
    }
   found_right=0;
  }
  else if(found_right==1 && found_left==1)
  {
    if(j==4)
    {
      drive_left();
    }
    else
    {
      drive_st();
      j++;
    }
    found_right=0;
    found_left=0;
  }
  else if(found_black==1)
  {
    drive_left();
    found_black=0;
  }
}

void path2()
{
  int k=1;
  int i=1;
  int j=1;
  if(found_left==1 && found_right==1)
  {
    if(i==1 || i==3 || i==5 || i==7)
    {
      drive_left();
      i++;
    }
    if(i==2)
    {
      drive_st();
      i++;
    }
    if(i==4 || i==6)
    {
      drive_right();
      i++;
    }
    found_left=0;
    found_right=0;
  }
  else if(found_left==1)
  {
    if(j==1)
    {
      drive_left();
      j++; 
    }
    else
    {
      drive_st();
    }
    found_left=0;
  }
  else if(found_right==1)
  {
    if(k==1 || k==3)
    {
      drive_st();
      k++;
    }
    else if(k==2)
    {
      drive_right();
      k++;
    }
    found_right=0;
  }
  /*if(found_white==1)
  {
    drive_left();
    found_white=0;
  }
}*/

void path3()
{
  int i=1;
  int j= 1;
  if(found_right==1)
  {
    drive_right();
    found_right=0;
  }
  if(found_left==1)
  {
    if(i==1)
    {
      drive_st();
      i++;
    }
    else
    {
      drive_left();
    }
    found_st=0;
    found_left=0;
  }
  if(found_right==1 && found_left==1)
  {
   if(j==4)
   {
    drive_right();
    j++;
   }
   if(<=4)
   {
    drive_st();
    j++;
   }
    found_right=0;
    found_left=0;
  }
  if(found_y==1)
  {
    if(k==1 || k==6)
    {
     drive_left();
     i++; 
    }
    if(k>=2 && k<6)
    {
      drive_right();
      i++;
    }
    found_y=0;
  }
  if(found_half_y==1)
  {
    drive_right();
    found_half_y=0;
  }
  /*if(found_bent_y==1)
  {
    if(j==5)
    {
      drive_left();
      j++;
    }
    else
    {
      drive_right();
    }
    found_bent_y=0;
  }*/
  if(found_black==1)
  {
    drive_left();
    found_black=0;
  }
}


//MOTOR CONTROL FUNCTIONS--------------------------------------------------------------------------------------------------------
void drive_right()       //will include parameter duration
{
    digitalWrite(LDIR1, HIGH);
    digitalWrite(LDIR2, LOW);
    digitalWrite(RDIR1, LOW);
    digitalWrite(RDIR2, HIGH); 
    analogWrite(LENB, basespeed); 
    analogWrite(RENB, basespeed);
      
    pos = qtra.readLine(sensorValues);
      
    while (sensorValues[1] <200)  // wait for outer most sensor to find the line
    {
      pos = qtra.readLine(sensorValues);
    }
      
      // find center
     while (pos < 3250)  // tune - wait for line position to find near center
     {
        pos = qtra.readLine(sensorValues);
     }
     
      // stop both motors 
      analogWrite(LENB, 0); 
      analogWrite(RENB, 0);// stop right motor first to better avoid over run  
      
  }  


void drive_left()     //will include parameter duration
{
      digitalWrite(LDIR1, HIGH);
      digitalWrite(LDIR2, LOW);
      digitalWrite(RDIR1, LOW);
      digitalWrite(RDIR2, HIGH); 
      analogWrite(LENB, basespeed); 
      analogWrite(RENB, basespeed);
      
      pos = qtra.readLine(sensorValues);
      
      while (sensorValues[6] <200)  // wait for outer most sensor to find the line
      {
        pos = qtra.readLine(sensorValues);
      }
      
      // find center
      while (pos > 4350)  // tune - wait for line position to find near center
      {
        pos = qtra.readLine(sensorValues);
      }
     
      // stop both motors 
      analogWrite(LENB, 0); 
      analogWrite(RENB, 0);// stop right motor first to better avoid over run  
       
}

void drive_st()     //will include parameter duration
{
    digitalWrite(LDIR1,LOW);
    digitalWrite(LDIR2,HIGH);
    digitalWrite(RDIR1,LOW);
    digitalWrite(RDIR2,HIGH);
    analogWrite(LENB,basespeed);
    analogWrite(RENB,basespeed);  
}
//END OF ALL DEFINED FUNCTIONS----------------------------------------------------------------------------------------------

void loop() 
{
  // put your main code here, to run repeatedly:
  //QTRA Sensor Calls
   unsigned int position = qtra.readLine(sensorValues);
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  //if(push==HIGH)
   laneRecognition();
   path1();
   Move(); 
  
  
  //delay(250);
}
