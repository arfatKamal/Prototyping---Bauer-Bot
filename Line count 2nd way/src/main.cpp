#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

/*IR sensors */
const uint8_t IR1_pin = A0;   // Left
const uint8_t IR2_pin = A1;   // Right
const uint8_t IR3_pin = 12;   // Third IR
uint8_t IR1, IR2, count;

/* distance */
const uint8_t trigPin = 12;
const uint8_t echoPin = 13;
uint16_t currentDistance;


/* Position Count */
uint8_t targetX = 2;
uint8_t currentX= 0;
uint8_t targetY = 2;
uint8_t currentY= 3;
uint8_t secondX = 3;
uint8_t secondY = 3; 
uint8_t destinationX = abs(targetX - currentX);
uint8_t destinationY = abs(targetY - currentY);

/* Position count States */ 
const uint8_t highStateX = 0;
const uint8_t lowStateX = 1;
const uint8_t turn = 2;
const uint8_t lowStateY = 3;
const uint8_t highStateY = 4;
const uint8_t destination = 5; 
volatile uint8_t State = highStateX;

/*Turn */

volatile uint8_t direction=1; 


/* Motors */
const uint8_t rightMotor = 10;   // Motor Right connections
const uint8_t in1 = 9;           // right motor ; for foward LOW
const uint8_t in2 = 8;           // right motor ; for forward HIGH

const uint8_t leftMotor = 5;   // Motor Left connections
const uint8_t in3 = 7;         // left motor ; for foward LOW
const uint8_t in4 = 6;         // left motor ; for forward HIGH

/* Speed */
uint8_t Speed = 150;
uint8_t error = 30;
uint8_t leftSpeed = Speed+error;
uint8_t rightSpeed = Speed;

uint8_t turnSpeed = 30;


void readings(void *argc);
void running(void *argc);
void setSpeed(void *argc);
void positionCount(void *argc);


TaskHandle_t Data, go, mSpeed, axis;

/* functions */
void distance();
void forward();
void backward();
void forward_right();
void forward_left();
void stop();
void ninetyTurn();
void turning();


void setup()
{
    
  Serial.begin(9600);

  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  analogWrite(leftMotor, leftSpeed);
  analogWrite(rightMotor, rightSpeed);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IR1_pin, INPUT);
  pinMode(IR2_pin, INPUT);
  pinMode(IR3_pin, INPUT);

  xTaskCreate(running, "Car running", configMINIMAL_STACK_SIZE, NULL,1 , &go);
  xTaskCreate(setSpeed, "Incresing Speed", configMINIMAL_STACK_SIZE, NULL,1 , &mSpeed);
  xTaskCreate(positionCount, "Counting position", configMINIMAL_STACK_SIZE+16, NULL,1 , &axis);
  

  vTaskStartScheduler();
}

void loop()
{  
   /*IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);
   count = digitalRead(IR3_pin);
   Serial.print("Left IR: ");
   Serial.println(IR1);
   Serial.print("Right IR: ");
   Serial.println(IR2);
   Serial.print("3rd IR: ");
   Serial.println(count);*/
  
}



void setSpeed(void *argc)
{

  (void)argc;

  while (1)
  {
     
  if (IR1 == 1 && IR2 == 0)    
     { 
       turnSpeed += 5;
       analogWrite(leftMotor, turnSpeed);
       analogWrite(rightMotor, 0);
       
       if (turnSpeed >= leftSpeed){

         turnSpeed = 10;
       }

     }else if (IR1 == 0 && IR2 == 1) 
     {
       turnSpeed += 5;
       analogWrite(rightMotor, turnSpeed);
       analogWrite(leftMotor, 0);
   
       if (turnSpeed >= rightSpeed){

         turnSpeed = 10;
       }


     }else if (IR1 == 0 && IR2 == 0)
     {
      analogWrite(leftMotor, leftSpeed);
      analogWrite(rightMotor, rightSpeed);
      turnSpeed= 10; 

     }else if (IR1 == 1 && IR2 == 1)
     {
        stop();
     }


  }

}


void positionCount(void *argc){

(void)argc;

  

  while (1)
  {

   IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);
   count = digitalRead(IR3_pin);

  
    switch (State)
    {
    case lowStateX:

      if (destinationX == 0 )
      {
        vTaskSuspend(go);
        vTaskSuspend(mSpeed);
        stop();
        vTaskDelay(200/portTICK_PERIOD_MS);

        State = turn; 
      }
     
    if (count == 1){

      State = highStateX; 
    }    
      break;
    case highStateX:
    
    if (count == 0 && IR1 == 0 && IR2 == 0)
    {
      currentX += 1; 
      Serial.println(currentX);
      State = lowStateX;
    }
      break;

      case turn:
      if ((destinationX == 0) && (destinationY == 0))
      {
        State = destination; 
      }else
      {
        turning();
      }
  
      break;

      case highStateY:
      

      if (destinationY == 0 )
      {
        vTaskSuspend(go);
        vTaskSuspend(mSpeed);
        stop();
        vTaskDelay(500/portTICK_PERIOD_MS); 
        State = turn;
      }


      if (count == 0 && IR1 == 0 && IR2 == 0)
      {
        currentY += 1; 
        Serial.println(currentY);
        State = lowStateY;
      }    
    break;

     case lowStateY:

      if (count == 1)
      {
        //Serial.println(currentY);
         State = highStateY;
      }      
      break;
    
    case destination:

    // reseting X and Y   
      targetX = secondX;
      targetY = secondY; 
      State = turn; 
      break;

    default:
       stop();
      break;
    }
    
  }
}

void running(void *argc)
{

  (void)argc;

  while (1)
  {

  IR1 = digitalRead(IR1_pin);
  IR2 = digitalRead(IR2_pin);
    
  if (IR1 == 1 && IR2 == 0)    
     { 
      forward_right();
      //stop();

     }else if (IR1 == 0 && IR2 == 1) 
     {
       forward_left();
       //stop();

     }else if (IR1 == 0 && IR2 == 0)
     {
       forward();
      // stop();

     }else if (IR1 == 1 && IR2 == 1)
     {
       stop();
     }

  }
}

void turning(){
switch (direction)
{
case 1:
  if((currentY<targetY)&&(destinationY!=0 )&&(destinationX==0))
  {
  forward_left();
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerY();
    direction = 2;
  }
  }
  else if ((currentY>targetY)&&(destinationY!=0 )&&(destinationX==0))
  {
  
  forward_right();
   if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerY();
    direction = 4;
  }
  }
  
  break;
case 2: 
if((currentX>targetX)&&(destinationX!=0 )&&(destinationY==0))
  {
  forward_left(); 

  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerX();
    direction = 3;
  }
   }
  else if ((currentX<targetX)&&(destinationX!=0 )&&(destinationY==0))
  {
   forward_right(); 
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerX();
    direction = 1;
  }
  }
  break;
  case 3:
  if((currentY>targetY)&&(destinationY!=0 )&&(destinationX==0))
  {
  forward_left(); 
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerY();
    direction = 4;
  }
   }
  else if ((currentY<targetY)&&(destinationY!=0 )&&(destinationX==0))
  {
   forward_right(); 
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerY();
    direction = 2;
  }
  }
  break;
case 4:
  if((currentX>targetX)&&(destinationX!=0 )&&(destinationY==0))
  {
  forward_right(); 
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerX();
    direction = 3;
  }
  }
  else if ((currentX<targetX)&&(destinationX!=0 )&&(destinationY==0))
  {
   forward_left(); 
  if (count == 0 && IR1 == 0 && IR2 == 1)
  {
    turnManagerX();
    direction = 1;
  }
  }
  break;

default:
  break;
}
  

}


// Motor Commands
void forward()
{

  digitalWrite(in1, LOW);  // right motor
  digitalWrite(in2, HIGH); // right motor

  digitalWrite(in3, LOW);  // left motor
  digitalWrite(in4, HIGH); // left motor
}

void backward()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward_right()
{
  // right motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  //left motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);    
}

void forward_left()
{

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
}

void ninetyTurn(){
   
   analogWrite(leftMotor, leftSpeed);
   analogWrite(rightMotor, rightSpeed);

   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH);
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW);
}

void distance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int travelTime = pulseIn(echoPin, HIGH);

  currentDistance = 0.0343 * travelTime / 2;

  Serial.print("Distance:");
  Serial.print(currentDistance);
  Serial.println("cm");
}
void turnManagerY(){
  
  stop();
  vTaskDelay(500/portTICK_PERIOD_MS);
  vTaskResume(mSpeed);  // resuming Speed
  vTaskResume(go);      // resuming line follow
  State = lowStateY; 
}
void turnManagerX(){

  stop();
  vTaskDelay(500/portTICK_PERIOD_MS);
  vTaskResume(mSpeed);  // resuming Speed
  vTaskResume(go);      // resuming line follow
  State = highStateX; 
}
