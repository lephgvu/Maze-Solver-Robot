#include <PID_v1.h>

//MOTOR A 
int enA = 5;
int in1 = 9;
int in2 = 8;

// Motor B
int enB = 10;
int in3 = 7;
int in4 = 6;

//sensor pins 
#define trig_pin A2 //analog input 1
#define echo_pin A3 //analog input 2  

#define trig_pin2 A0 //analog input 1
#define echo_pin2 A1 //analog input 2

#define trig_pin3 A4 //analog input 1
#define echo_pin3 A5 //analog input 2


long duration, distance, distanceFront, distanceRight,distanceLeft,r1,r2,x ;
long LastDistanceFront = 0 , LastDistanceRight =0 ,LastDistanceLeft =0; 
 
double Setpoint, Input1, Output=0.0;
  
  // first make KI and KD=0 and try Kp values that matches your robot 
  double Kp=1, Ki=1, Kd=1; 
  PID myPID(&Input1, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  
void setup() {

  pinMode(in3, OUTPUT);  //make initially all pins as output
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin2, OUTPUT);
  pinMode(echo_pin2, INPUT);
  pinMode(trig_pin3, OUTPUT);
  pinMode(echo_pin3, INPUT);
 
  Serial.begin (9600);
   
    Setpoint = 80;
   
    distanceRight=SonarSensor(trig_pin2, echo_pin2);
    distanceFront=SonarSensor(trig_pin, echo_pin);
    distanceLeft=SonarSensor(trig_pin3, echo_pin3);
   
   Input1 = distanceRight;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-200,200);
    myPID.SetTunings(Kp, Ki, Kd);
    

    delay(10);
    
    moveForward(170,Output);
    
   }

   
  void printSerial(){
    Serial.print("Distance Right: ");
    Serial.print(distanceRight);
    Serial.print(" cm\t");
    Serial.print("Distance Front: ");
    Serial.print(distanceFront);
    Serial.print(" cm\t");
    Serial.print("Distance Left: ");
    Serial.print(distanceLeft);
    Serial.println(" cm");
}  

void turnLeft(){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
 
      analogWrite(enA, 170); 
      analogWrite(enB, 130);
      delay(300); 

        //delay(5);
}
void stopmotor(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Tạm dừng trong 2 giây
  delay(1000);
  }

 
long SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;
 return distance ;
}


void moveForward(int i,double x){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    
    analogWrite(enA, (i-20)); //the speed left
    analogWrite(enB, (i-66)); //the speed  right
}

void moveBackward(int i,double x){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    
    analogWrite(enA, (i-20)); //the speed left
    analogWrite(enB, (i-66)); //the speed  right
}

void turnRight(){
     digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    

    analogWrite(enA, 170); //the speed
    analogWrite(enB, 130); //the speed  
    delay(300);
      }
      
  //delay(5);

void turnAround(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 170); // Tốc độ động cơ 1
  analogWrite(enB, 130); // Tốc độ động cơ 2
  delay(300);

  }

  void loop(){
    distanceRight=SonarSensor(trig_pin2, echo_pin2);
    distanceFront=SonarSensor(trig_pin, echo_pin);
    distanceLeft=SonarSensor(trig_pin3, echo_pin3);

   //printSerial();
    
    
    if(distanceRight <= 10 && distanceFront <=10 && distanceLeft <=10){
      Serial.println("turn around");
      stopmotor();
      turnAround();
      moveForward(170,0);
      
      }
    if(distanceRight <= 15 && distanceFront >= 15 || distanceLeft <=15 && distanceFront >8){
      Serial.println("move forward");
      distanceRight=SonarSensor(trig_pin2, echo_pin2);
      distanceFront=SonarSensor(trig_pin, echo_pin);
      distanceLeft=SonarSensor(trig_pin3, echo_pin3);
      
      Input1 = distanceRight;
      myPID.Compute();
      moveForward(170,0);
      
       //forward callibration
       if(distanceLeft>=20 && distanceRight >2){
            distanceRight = LastDistanceRight;
        }else{
            LastDistanceRight = distanceRight;
        }
        if(distanceRight>=20 && distanceLeft>2){
            distanceLeft = LastDistanceLeft;
          }else{
            LastDistanceLeft = distanceLeft;
               }    
 
    }
    
    //moving left 
    else if(distanceFront <= 8 && distanceRight <= 30 && distanceLeft >=30){
           r1=distanceLeft;
           r2=r1+12;
           x =r1/r2;
      do{
          stopmotor();

          turnLeft();
          stopmotor();
          moveForward(170,0);
          Serial.println(" moving left ");
          distanceRight=SonarSensor(trig_pin2, echo_pin2);
          distanceFront=SonarSensor(trig_pin, echo_pin);
          distanceLeft=SonarSensor(trig_pin3, echo_pin3);
            
        //callibration   
         if(distanceFront >= 70 || distanceRight >= 40 ){
               distanceFront = LastDistanceFront; 
               distanceRight = LastDistanceRight;
                
          }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                }
          
      }while(distanceFront < 45 && distanceRight > 13&&distanceFront!=0);
            
  }else if(distanceRight > 25 && distanceFront <= 8 && distanceLeft<30){      
                r1=distanceRight;
                r2=r1+12;
                x =r1/r2;
              do{
                 distanceRight=SonarSensor(trig_pin2, echo_pin2);
                 distanceFront=SonarSensor(trig_pin, echo_pin);
                 distanceLeft=SonarSensor(trig_pin3, echo_pin3);
                 stopmotor();
                 turnRight();
                 moveForward(170,0);
                 Serial.println(" moving right");

                 //callibration
                if(distanceFront >= 70 || distanceRight >= 40  ){
                distanceFront = LastDistanceFront; 
                distanceRight = LastDistanceRight;
             
                }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                 }
                }while(distanceFront<45 && distanceRight > 13 && !(distanceLeft > 13));
    }
    
    else if(distanceLeft>=30 &&distanceRight>=30 && distanceFront<=8){
                r1=distanceLeft;
                r2=r1+12;
                x =r1/r2;
         do{
            stopmotor();

            turnLeft();
            stopmotor();
            moveForward(170,0);
            distanceRight=SonarSensor(trig_pin2, echo_pin2);
            distanceFront=SonarSensor(trig_pin, echo_pin);
            distanceLeft=SonarSensor(trig_pin3, echo_pin3);
            
            Serial.println(" center turning left ");
             //callibration
         if(distanceFront >= 70 || distanceRight >= 40 ){
               distanceFront = LastDistanceFront; 
               distanceRight = LastDistanceRight;
          }else {
                LastDistanceFront = distanceFront ;
                LastDistanceRight = distanceRight;
                }        
      }while(distanceFront < 45 && distanceRight > 13 && !(distanceLeft > 13)&&distanceFront!=0);
    
    }
    if(distanceRight >= 30 && distanceFront >=30 && distanceLeft >=10){
      Serial.println("Move stop");
      stopmotor();
      
      
      }
      
  }// end void loop