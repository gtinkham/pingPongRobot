/****************************************************************
 *
 *    Author Name:Grant Tinkam
 *    Date: 4/2/2014
 *    Sketch Name: launcher_control
 *    Sketch Description: This sketch will test the          linear encoder strips
 *                        used with the Makeblock platform.  The user is 
 *                        able to move the linear stage left and right 
 *                        with the onboard buttons.  The encoder output 
 *                        is displayed to the serial monitor.
 *                        
 *    Button Usage: Left/Right -  Move linear stage left and right
 *                  Select     -  Reset the encoder counts
 *                  
 *    Pin Usage:    A0         -  Button input (pin connected internally)
 *                  D2         -  Reflectance sensor for linear encoder
 *                  D4         -  Motor direction pin (internal)
 *                  D5         -  Motor speed pin (internal)
 *                   2         -  IR Read pin
 *                   
 *****************************************************************/

//*** #defines and #includes go here ***
#define NONE 0   //compiler will replace NONE with 0, etc...
#define LEFT 1  
#define RIGHT 2
#define SELECT 3
#define UP 4
#define DOWN 5
#define ENCODER_MAX 40
#define STAHP 6
#define IN 7
#define OUT 8
#define MOTOL 22
#define MOTOR 21
#include <Servo.h>
#include <math.h>//tell compiler to include the Servo library

//*** Global Variable Declarations ***

// Analog pin declarations
const int buttonPin = 0;   // Attached to Romeo's onboard buttons

// Digital pin declarations
const int IRPin = 2;
const int motorDirPin = 4;
const int motorSpeedPin = 5;
const int LeftSwitchPin = 11;
const int RightSwitchPin = 12;
const int LaunchServoPin = 9;
const int ReloaderServoPin = 10;
const int SolenoidDirPin = 7;
const int SolenoidSpeedPin = 6;
const int LEDPin = 13;

// Program variable declarations
int buttonPressed;
int motorDirection;
int positionLauncher;
int Speed = 255; // motor speed (valid range: 0 to 255)
boolean currentIRReading, lastIRReading;
long lastReadingTime;
Servo LaunchServo, ReloaderServo; // creates a pair of Servo structures
int encoderLine;   
int stopLoop = 0;
int angle;
int solenoidDir;
int target;

double OPTThetas[6];
int ThetaServo[6]; //{22, 35, 46, 52, 65, 70};
double position_actual[6];
double x2[6];

unsigned long time = millis();
unsigned long elapsedTime = 0;


////// BEGINNING OF GRANTS FUNCTION

const double pi = 3.14159265359;

double D2R(double num)
{
 return num*(pi/180);
}

double R2D(double num2){
	return num2*(180/pi);
}
  
  
double Compute_Launch_Angle(double targetDist){ 
      

    
    double optTheta; 
    int theta;  
    	double L1 = .125; //0.1325 
	double L4 = .045;
	double L3 = .105; //0.09;
	double L2 =  00.0875; //0.0975;
    double r; 
      
      
    double minError = 999999; 
    for (theta = 30; theta < 75; theta++){ 
	    double thetaR = D2R(theta); 
	      
	    double v0= 2.9/*2.87/*2.845/*2.84/*2.835/*2.83/*2.825x2/*2.825/*2.8/*2.85/*2.7*//*2.9236*/; 
	    double y0 = L1 + L2 * sin(thetaR)+ L3 * cos(thetaR);  
	    double x0 = L2 * cos(thetaR)-L3 * sin(thetaR);  
	    double g =  9.8;  
	    double a = -(g/2);  
	    double b = v0 * sin(thetaR); 
	    double c = y0; 
	    double t = (- b - sqrt ((b * b)-4 * a * c))/(2 * a);  
	    r = x0 + (v0 * cos(thetaR) * t);  
	      
	    double error = fabs((r-targetDist) / targetDist); 
        if (error < minError){ 
        	optTheta = theta; 
        	minError = error; 
        } 
  
    } 

    return optTheta;  
} 
     
int Compute_Servo_Angle (double Theta_Ldeg){
	
	
	
	double Theta_L = D2R(Theta_Ldeg);
	double L1 = .125; //0.1325 
	double L4 = .045;
	double L3 = .105; //0.09;
	double L2 =  00.0875; //0.0975;
	double crossed = 0;
	double offsets_SoDeg = 10; // degrees
	double offsets_So = D2R(offsets_SoDeg); //radians
	double offsets_LoDeg = 12;
	double offsets_Lo= D2R(offsets_LoDeg) ;
	double theta_2; 
	double K_1, K_2, K_3; 
	double A, B, C ; 
	double theta_4; 
	  
	  
	crossed = 0; 
	theta_2 = (Theta_L - offsets_Lo); 
	K_1 = L1 / L2; 
	K_2 = L1 / L4; 
	K_3 = (L1 * L1 + L2 * L2 - L3 * L3 + L4 * L4)/(2.0 * L2 * L4); 
	A = cos(theta_2) - K_1 - K_2 * cos(theta_2) + K_2; 
	B = -2.0 * sin(theta_2); 
	C = K_1 - ((K_2 + 1.0) * cos(theta_2)) + K_3; 
	double cond = (B * B - 4.0 * A * C); 
	
		  
	theta_4 = 2 * atan((-B - sqrt(cond)) / (2 * A)); 
		
	// 0} 
	 double ThetaSR= theta_4 + offsets_So;
	 int ThetaSD = (int)(R2D(ThetaSR));
	return ThetaSD; 
} 


////END OF GRANTS FUNCTION



void setup(void){
  // Pin initialization
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);

  // Serial communication initialization
  Serial.begin(9600);

  // Take an initial IR sensor reading
  lastIRReading = digitalRead(IRPin);

  // Take an initial time reading
  lastReadingTime = millis();

  // Declares pin of servos and attaches them 
  pinMode(LeftSwitchPin, INPUT_PULLUP);
  pinMode(RightSwitchPin, INPUT_PULLUP);
  LaunchServo.attach(LaunchServoPin);
  ReloaderServo.attach(ReloaderServoPin);
  LaunchServo.write(65); 
  ReloaderServo.write(35);
  
  
  int k;
  byte X_highbyte, X_lowbyte, Y_highbyte, Y_lowbyte;
  double x[6];
  int y[6];
  
  //Read the target locations

  for (k=0; k<6; k++){
  while (Serial.available()<2);
  X_highbyte = Serial.read();
  X_lowbyte = Serial.read();
  x[k] = X_highbyte * 256 + X_lowbyte;
  // delay(50);
  Serial.print("The X coordinate is ");
  Serial.println(x[k]);


}

  for (k=0; k<6; k++){
  while (Serial.available()<2);
  Y_highbyte = Serial.read();
  Y_lowbyte = Serial.read();
  y[k] = Y_highbyte * 256 + Y_lowbyte;
  // delay(50);
  Serial.print("The Y coordinate is ");
  Serial.println(y[k]);


}

//LE conversions factors
  for (target=0;target < 6;target++){
    //converts x coordinates from the image of the centroids to the actual enccoder based positions necessary. ;
    Serial.println("Calculating values");
    position_actual[target] = round((y[target]-45)/10);
     Serial.println(position_actual[target]);
  
  
  
  
  
    //computes the launch angle;
         x2[target] = (((x[target])/1000) + 0.5);
         Serial.println(x[target]);
        OPTThetas[target] = Compute_Launch_Angle(x2[target]);
        Serial.println(OPTThetas[target]);
          
       
  
  
    //compustes the launch angle;
  
        ThetaServo[target] = Compute_Servo_Angle(OPTThetas[target]);
        Serial.println(ThetaServo[target]);
         



    }
    target = 0;
    encoderLine = 0;
}

void readButton(){
  // Read button pin for input
  int buttonReading = analogRead(buttonPin);

  if (buttonReading < 5) {
    buttonPressed = UP;
    return;
  }
  if ((buttonReading > 320) && (buttonReading < 335)){
    buttonPressed = DOWN;
    return;
  }
  if ((buttonReading > 130) && (buttonReading < 160)){
    buttonPressed = LEFT;
    return;
  }
  if ((buttonReading > 500) && (buttonReading < 515)){
    buttonPressed = RIGHT;
    return;
  }
  if ((buttonReading > 740) && (buttonReading < 750)){
    buttonPressed = SELECT;
    return;
  }

  buttonPressed = NONE;   // this happens if none of the previous if statements is true

}

void runMotor(){
  switch (motorDirection){

  case MOTOL:

    digitalWrite(motorDirPin, HIGH);
    analogWrite(motorSpeedPin,Speed);
    break;
  case MOTOR:

    digitalWrite(motorDirPin, LOW);
    analogWrite(motorSpeedPin,Speed);

    break;
  case STAHP:

    analogWrite(motorSpeedPin,0);	
    break;
  default:
    break;

  }
}
void moveLauncher(){
  // moves motor left at the highest speed...for now
  // while loop to track encoder positions until specified one has been reached
  if ( (positionLauncher < encoderLine) ){
    Serial.println("Motor Moving Left...");
    motorDirection = MOTOL;
    runMotor();

  }

  if ((positionLauncher > encoderLine) ){
    Serial.println("Motor Moving Right...");
    motorDirection = MOTOR;
    runMotor();

  }

  while ((encoderLine != positionLauncher) ){
 //   Serial.println(positionLauncher);	

    boolean currentIRreading = 0;

    currentIRreading = digitalRead(IRPin);

    
    time = millis();
    
    elapsedTime = time - lastReadingTime;


    if((currentIRreading != lastIRReading) && (elapsedTime > 100)){


      if (motorDirection == MOTOL){
        encoderLine = encoderLine - 1; 
      }
      else{

        encoderLine = encoderLine + 1;
      }

      lastIRReading = currentIRreading;
      lastReadingTime = time;
      Serial.println(encoderLine);
    }

    // Reads left pin followed by the right pin 

    int LeftSwitch = 0;
    LeftSwitch = digitalRead(LeftSwitchPin);
    int RightSwitch = 0;
    RightSwitch = digitalRead(RightSwitchPin);



    if ((LeftSwitch == 1) && (motorDirection == MOTOL)){

      Serial.println("Loop has been broken");
      encoderLine = 0; 
      Serial.println(encoderLine);
      stopLoop = 1;
      break;
    }
    else if ((RightSwitch == 1) && (motorDirection == MOTOR)){

      encoderLine = ENCODER_MAX;
      Serial.println("Loop has been broken");
      Serial.println(encoderLine);
      stopLoop = 1;
      break;
    }
      
  }
  

    if (motorDirection == MOTOL){
   
      delay(30);
      motorDirection = MOTOR;
      runMotor();
      delay(30);
      motorDirection = MOTOL;
      runMotor();
      
    }
    else if  (motorDirection == MOTOR){

       delay(30);
        motorDirection = MOTOL;
        runMotor();
        delay(30);
        motorDirection = MOTOR;
        runMotor();
    }
  //stopLoop = 1;
  motorDirection = STAHP;
  runMotor();
  
 
  //Serial.println(encoderLine);
}



void servo(){
  
  LaunchServo.write(angle);
  delay(1000);
    
}

void solenoid(){

  if (solenoidDir == IN){ 
    digitalWrite(SolenoidDirPin, LOW);
    analogWrite(SolenoidSpeedPin,250);
    delay(100);
    analogWrite(SolenoidSpeedPin,0);

  }
  else {
    analogWrite(SolenoidSpeedPin,0 );
  }
}

void reloader(){
 Serial.println("reloading ");
 positionLauncher = 41; 
 LaunchServo.write(30); 
 moveLauncher();
 delay(1000);
 ReloaderServo.write(0);
 delay(500);
 ReloaderServo.write(35);
  
}

void et_go_home(){
  positionLauncher = -1; 
  moveLauncher();
  
}


void loop(){
  
  if (target < 5){
  positionLauncher = (int)position_actual[target]; 
   moveLauncher();
   delay(125);
   angle =  ThetaServo[target];
   servo();
   delay(62.5);
   solenoidDir = IN; 
   solenoid();
   delay(125);
   reloader();
   target++; 
    
    
  }
  else{
    
   positionLauncher = (int)position_actual[target]; 
   moveLauncher();
   delay(125);
   angle = ThetaServo[target] ;
   servo();
   delay(62.5);
   solenoidDir = IN; 
   solenoid();
   delay(125);
   et_go_home();
   while(target = 6);
    
  }
  
  
}












