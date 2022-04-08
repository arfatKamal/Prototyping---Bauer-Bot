//#include <Arduino.h>
#include <IRremote.h>


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
const byte virtualIR = 2;
int IR1, IR2;           // Sensor 1 is on the left and Sensor 2 is on the right
IRrecv irrecv(virtualIR);
decode_results virtualIR_read;

/* distance */
const byte trigPin= 12;
const byte echoPin= 13;
int currentDistance; 


/* Motors */
const byte enA = 10;   // Motor A connections
const byte in1 = 9;
const byte in2 = 8;                      
const byte enB = 5;   // Motor B connections
const byte in3 = 7;
const byte in4 = 6;
const byte speed = 150;
const byte turnSpeed = 80; 

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

  irrecv.enableIRIn();

  Serial.begin(9600);
}


void loop(){

  
   switch (Mode)
   {
    
   case IR: 
    
     map_IRsensor(); 
     
     Mode = ultraSonic;
     
   break;
   case ultraSonic:
   distance();
   delay(500);
   Mode = decider;
   break;
   case decider:

     if (IR1 == 0 && IR2 == 1 && currentDistance >= 40)
     {
       Serial.println("Turning left");
       Mode = turnLeft;
     }else if (IR1 == 1 && IR2 == 0 && currentDistance >= 40) 
     {
       Serial.println("Turning right");
       Mode = turnRight;
     }else if (IR1 == 0 && IR2 == 0 && currentDistance >= 40)
     {
       Serial.println("Going forward");
       Mode = Forward; 
     }else if ((IR1 == 1 && IR2 == 1) || currentDistance <= 40)
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
 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
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
 
 analogWrite(enA, turnSpeed);
 analogWrite(enB, turnSpeed);

 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);
}

void forward_left() {
 
 analogWrite(enA, turnSpeed);
 analogWrite(enB, turnSpeed);

 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 digitalWrite(in3, HIGH);
 digitalWrite(in4, LOW);
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
  delay(10); 
}

void map_IRsensor() {

/*_This function Maps the IR remote value to IR Variables. 
 * It uses the reading of one virtual IR sensor and maps it to two Physical IR sensors namely IR1 & IR 2.  
 *IR1 and IR2 can be used in the main code for IR sensor reads.
 *Just call this function in the void loop before calling IR1 & IR2.  
 *This way there will be less adjustment necessary while using the code on professor's platform  _*/
 
  
  if (irrecv.decode(&virtualIR_read)) {
    //Serial.println(virtualIR_read.value, HEX);
    irrecv.resume(); // Receive the next value
    
    if(virtualIR_read.value==0xFD08F7){
      IR1=0;            //both sensors are on the black line.
      IR2=0;            //Remote Button= 1
      Serial.println("Both on black");
      }
       else if(virtualIR_read.value==0xFD8877){
        IR1=0;         //Sensor 1 is on the black line & sensor 2 is on the White surface. 
        IR2=1;         //Remote Button= 2
        Serial.println("White and Black");
     
    }
       else if(virtualIR_read.value==0xFD48B7){
        IR1=1;         //Sensor 2 is on the black line & sensor 1 is On the White Surface. 
        IR2=0 ;        //Remote Button= 3    
        Serial.println("White and Black");  
    }

     else if(virtualIR_read.value==0xFD28D7){
        IR1=1;      //Both Sensors are on the White Surface.
        IR2=1;       //Remote Button= 4  
        Serial.println("Both on White"); 
        }
  }
  delay(100);
}