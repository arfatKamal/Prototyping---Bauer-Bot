#include <Arduino.h>



/* Loop */
const byte IR = 20;
const byte ultraSonic = 21;
const byte decider = 22;
const byte Forward = 23;
const byte turnRight = 24;
const byte turnLeft = 25;
const byte object = 26; 
volatile byte Mode = 20; 


/*IR sensors */
const byte IR1_pin= 2; // Left
const byte IR2_pin= 4; // Right
byte IR1, IR2;           


/* distance */
const byte trigPin= 12;
const byte echoPin= 13;
int currentDistance; 


/* Motors */
const byte enA = 10;   // Motor Right connections
const byte in1 = 9;
const byte in2 = 8;                      
const byte enB = 5;   // Motor Left connections
const byte in3 = 7;
const byte in4 = 6;
const byte speed = 255;
const byte turnSpeed = 150; 

/* functions */
void distance();
void forward();
void backward();
void forward_right();
void forward_left();
void stop();
void map_IRsensor();


void setup() {
  pinMode(9,OUTPUT); 
  pinMode(8,OUTPUT);   
  pinMode(7,OUTPUT);   
  pinMode(6,OUTPUT);   
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  analogWrite(enA, speed);
  analogWrite(enB, speed);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); 

  

  Serial.begin(9600);
}


void loop(){

  
   switch (Mode)
   {
    
   case IR: 
    
     IRsensor();
    
     Mode = ultraSonic;
     
   break;
   case ultraSonic:
   //distance();
   
   Mode = decider;
   break;
   case decider:

     if (IR1 == 1 && IR2 == 0)
     {
       Serial.println("Turning left");
       Mode = turnRight;
     }else if (IR1 == 0 && IR2 == 1) 
     {
       Serial.println("Turning right");
       Mode = turnLeft;
     }else if (IR1 == 0 && IR2 == 0)
     {
       Serial.println("Going forward");
       Mode = Forward; 
     }else if (IR1 == 1 && IR2 == 1)
     {
       Serial.println("Object detected or Both sensor on white");
       Mode = object;  
     }
   break;
   case Forward:
   forward();

   Mode = IR;  
   break;
     
   case turnLeft:
    forward_left(); 

  Mode = IR; 
  break;

  case turnRight: 
    forward_right();

    Mode = IR;
    break;
  case object:
     stop();

     Mode = IR;
   break;

   default:
     break;
   }
}



//Motor Commands
void forward() {
 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);   
}

void backward() {
 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, HIGH);
 digitalWrite(in4, LOW);
}

void forward_right() {
 
 analogWrite(enA, 0);
 analogWrite(enB, turnSpeed);

 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);
}

void forward_left() {
 
 analogWrite(enA, turnSpeed);
 analogWrite(enB, 0);

 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);
}

void stop() {
 digitalWrite(in1, LOW);
 digitalWrite(in3, LOW);
 digitalWrite(in2, LOW);
 digitalWrite(in4, LOW);
}


void distance(){
   
  digitalWrite(trigPin,LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW); 
  int travelTime = pulseIn(echoPin, HIGH);
  delay(20);

  currentDistance = 0.0343*travelTime/2;
  
  Serial.print("Distance:");
  Serial.print(currentDistance);
  Serial.println("cm");
  delay(100); 
}



void IRsensor() {
 IR1= digitalRead(IR1_pin); 
 IR2= digitalRead(IR2_pin); 

 Serial.println(IR1);
 Serial.println(IR2); 

  delay(100);
}
