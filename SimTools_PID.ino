//Libraries
#include <AltSoftSerial.h>
#include <Sabertooth.h>
#include <PID_v1.h>

//Constants
//SoftwareSerial SWSerial(NOT_A_PIN, 8);   // RX on no pin (unused), TX on pin 8 (to S1)
AltSoftSerial altSerial;
Sabertooth ST(128, altSerial);            // Address 128, and use SWSerial as the serial port.
#define leftPot A0                       // pin A0 is Left Pot Input
#define rightPot A1                      // pin A1 is Right Pot Input

//Debug                                 
//#define DEBUG                            // Debug for pots and PID
//#define SERIALDEBUG                      // Debug for serial


//Variables
double leftPotValue;              //value read from Left Pot
double rightPotValue;             //value read from Right Pot
long previousMillis = 0;          //Previous Timer
//unsigned long currentMillis;      //New Timer
long interval = 10000;            //Time to wait for no serial data

//Serial
String data;
String dataL;
String dataR;
int leftPosValue =512;
int rightPosValue =512; 


//PID Variables
double SetpointL = 512.00;
double SetpointR = 512.00;
double InputL;
double InputR;
double OutputL;
double OutputR;
double slow = .30;
double minV = 50.00;
double maxV = 900.00;


//Specify the links and initial tuning parameters including slowdown for null serial
double Kp=.5, Ki=.1, Kd=0.05, slowdown = .30;
PID PIDleft(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID PIDright(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

void setup() {
//Serial//
  Serial.begin(115200);              // initialize serial communications with computer
  altSerial.begin(9600);             // initialize serial communications with Sabertooth
  ST.autobaud();                     // autobaud for sabertooth
  Serial.setTimeout(1);              // setTimeout for fast string reads
  
//PID ON//
  PIDleft.SetMode(AUTOMATIC);       //turn the PID on for leftMotor
  PIDright.SetMode(AUTOMATIC);      //turn the PID on for rightMotor
  
//PID Limits//
  PIDleft.SetOutputLimits(-127, 127);
  PIDright.SetOutputLimits(-127, 127);
  
}
void loop() {
//Start Timer
  unsigned long currentMillis = millis();  
  
//Assign Input Values//            
  InputL = analogRead(leftPot);     //assign pot A0 value to InputL in PIDleft 
  InputR = analogRead(rightPot);    //assign pot A1 value to InputL in PIDright 



  if (Serial.available() > 0) {
    previousMillis = millis();
    signalProcessing();
  }

   else if (millis() - previousMillis >= interval) {
      previousMillis = millis();
      idle();      
     }

  

    //Run motors
    PIDleft.Compute();                                   //compute PIDleft
    PIDright.Compute();                                  //compute PIDright
      ST.motor(1, OutputL * slow);                       //Send PIDleft value to Motor1 * slowdown rate
      ST.motor(2, OutputR * slow);                       //Send PIDright value to Motor2 * slowdown rate


#ifdef DEBUG
Serial.print (InputL);
Serial.print("\t");
Serial.print(SetpointL);
Serial.print("\t");
Serial.print(OutputL);
Serial.print("\t \t");
Serial.print (InputR);
Serial.print("\t");
Serial.print(SetpointR);
Serial.print("\t");
Serial.print(OutputR);
Serial.print("\t \t");
Serial.print(slow);
Serial.print("\t \t");
#endif
#ifdef SERIALDEBUG
//Serial.print (data);
//Serial.print("\t");      
Serial.print (leftPosValue);
Serial.print("\t");
Serial.print (rightPosValue);
Serial.print("\t \t");
Serial.print(previousMillis);
Serial.print("\t");
Serial.println(currentMillis);
#endif  

}
  
//////////////////////////////////IDLE PROCESSING///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void idle(){
    SetpointL = 512.00;                  //put Lmotor into mid position
    SetpointR = 512.00;                  //put Rmotor into mid position

    slow = .30;

    Serial.flush();
        }
//////////////////////////////////SIGNAL PROCESSING/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void signalProcessing() {
  slow = 1;
  data = Serial.readString();
   
   //Assign Variables to Index Position of string for L and R
   int LPosition = data.indexOf ("L");     
   int RPosition = data.indexOf ("R");
          
          //Extract position value from substring between "L" and "R"
          dataL = data.substring(LPosition + 1, RPosition);
          leftPosValue = dataL.toInt();
          
          //Extract position value from substring between "R" and end of string
          dataR = data.substring(RPosition + 1, 12);
          rightPosValue = dataR.toInt();
  
  SetpointL = leftPosValue + .00;                  //Turn returned left value into float
  SetpointR = rightPosValue + .00;                 //Turn returned right value into float
  
  //Constrain motors to minV and maxV (min-max of Potentiometer)
  SetpointL = constrain(SetpointL, minV, maxV);
  SetpointR = constrain(SetpointR, minV, maxV);

  Serial.flush();
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

  
