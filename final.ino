#include <Servo.h>
#include <PID_v1.h>




int eqbAngle = 113;

int currentPosition = eqbAngle;


const int errorSize =  20  ;
float errorArray[errorSize];
int lastError = 0;
bool filled = false;
int v = 3;
int tolerance = 2*1;
Servo servo_test;  

double setPoint=0, error=0, output;
double consKp=0.0322, consKi=0, consKd=0.00015;

const int maxAngle =30;

PID irfan(&error, &output, &setPoint, consKp, consKi, consKd, DIRECT);
  
enum{
  RIGHT = 1,
  LEFT = 0
};

enum{
  TRIG = 0,
  ECHO = 1
};

int pingPin[2][2];



//const int trigPin = 12;
//const int echoPin = 11; 
const int motor = 9;               
int angle = 0;    
int equilibriumPosition = 20;
long duration, distance;
long prevDistance[2];


long getDistance(int PIN){
  digitalWrite(pingPin[PIN][TRIG], LOW);
  delayMicroseconds(2);

  digitalWrite(pingPin[PIN][TRIG], HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin[PIN][TRIG], LOW);

  duration = pulseIn(pingPin[PIN][ECHO], HIGH);
  distance = duration *0.0170;

  if(distance>100){
    return prevDistance[PIN];
  }

  else if(distance>39) distance=20;
  prevDistance[PIN] = distance;
  return distance;
}

long avgError(){
  long sum = 0;
  for(int i=0;i<errorSize;i++){
    sum+=errorArray[i];
  }

  return sum/errorSize;
}




void setup() 
{ 
  pingPin[RIGHT][TRIG] = 12;
  pingPin[RIGHT][ECHO] = 11;
  pingPin[LEFT][TRIG] = 6;
  pingPin[LEFT][ECHO] = 5;

  irfan.SetTunings(consKp, consKi, consKd);
  irfan.SetMode(AUTOMATIC);
  servo_test.attach(motor);      // attach the signal pin of servo to pin9 of arduino
  pinMode(pingPin[RIGHT][TRIG], OUTPUT);
  pinMode(pingPin[RIGHT][ECHO], INPUT);
  pinMode(pingPin[LEFT][TRIG], OUTPUT);
  pinMode(pingPin[LEFT][ECHO], INPUT);
  Serial.begin(9600);
  prevDistance[0]=prevDistance[1]=0;
} 


  
void loop() 
{ 

  distance =0;
  long leftDistance, rightDistance;
  rightDistance = getDistance(RIGHT);
//  delayMicroseconds(2);
  leftDistance = getDistance(LEFT);

  
  if(leftDistance<rightDistance){
    distance = leftDistance;
  }
  else{
    distance = rightDistance;
  }
  

  if(fabs(distance)>20){
    distance = 20;
  }

  distance = equilibriumPosition - fabs(distance);
//  prevDistance = distance;
//  
  error = (distance > 0)? -distance : +distance;

  errorArray[lastError] = error;

  if(++lastError >= errorSize){
    lastError = 0;
    filled = true;
  }

  error = -avgError()*avgError();
  if(fabs(error)<tolerance)error=0.0;
  
  if(filled)
  {
    
      
    irfan.Compute();
//    Serial.println(rightDist/ance);
  ////  
  ////  
  ////
    float target = 0.0;
  ////  
    if(output>maxAngle)output=maxAngle;
    else if(output<-maxAngle)output= -maxAngle;
  
  
  //  Serial.println(output);
    
    if(leftDistance>rightDistance)
    {
      target = eqbAngle-output;
//      Serial.print("RIGHT=");
//      Serial.println(rightDistance);
//      Serial.println(leftDistance);
    }
    else{
      target = eqbAngle+output;
//      Serial.print("LEFT=");
//      Serial.println(leftDistance);
//      Serial.println(rightDistance);
    }
    
    if(error==0)target=eqbAngle;

    if(currentPosition<target-v){
      currentPosition+=v;
    }
    else if(currentPosition>target+v){
      currentPosition-=v;
    }
    servo_test.write(currentPosition);
    Serial.print(error);
    
    Serial.print('\n');
    delayMicroseconds(1);
  }

}
