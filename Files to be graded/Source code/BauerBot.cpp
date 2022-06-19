#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

/*IR sensors */
const uint8_t IR1_pin = A0;   // Left
const uint8_t IR2_pin = A1;   // Right
const uint8_t IR3_pin = 12;   // Third IR for counting position
uint8_t IR1, IR2, count;

/* Position Count */
uint8_t currentX= 1;
uint8_t currentY= 2;

uint8_t targetX = 3;
uint8_t targetY = 3;

uint8_t secondX = 1;
uint8_t secondY = 2; 

uint8_t destinationX ;
uint8_t destinationY ;

/* Defining Position counting States */ 
const uint8_t highStateX = 0;
const uint8_t lowStateX = 1;
const uint8_t turn = 2;
const uint8_t lowStateY = 3;
const uint8_t highStateY = 4;
const uint8_t destination = 5; 
volatile uint8_t State = highStateX;

/* Turn */
volatile uint8_t direction=1; 

/* Motors */
const uint8_t rightMotor = 10;   // Motor Right connections
const uint8_t in1 = 9;           // right motor ; for foward LOW
const uint8_t in2 = 8;           // right motor ; for forward HIGH
const uint8_t leftMotor = 5;     // Motor Left connections
const uint8_t in3 = 7;           // left motor ; for foward LOW
const uint8_t in4 = 6;           // left motor ; for forward HIGH

/* Speed */
uint8_t Speed = 150;
uint8_t error = 30;
uint8_t leftSpeed = Speed+error;  // used for going forward
uint8_t rightSpeed = Speed;       // used for going forward
uint8_t turnSpeed = 30;           // This is incremented for turning


/* Tasks and task handles */
void setSpeed(void *argc);
void positionCount(void *argc);
void running(void *argc);
TaskHandle_t go, mSpeed, axis;

/* functions */
void forward();
void backward();
void forward_right();
void forward_left();
void stop();
void turning();
void turnManagerX();
void turnManagerY();


void setup()
{
    
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(IR1_pin, INPUT);
  pinMode(IR2_pin, INPUT);
  pinMode(IR3_pin, INPUT);

  analogWrite(leftMotor, leftSpeed);
  analogWrite(rightMotor, rightSpeed);

  xTaskCreate(running, "Following line", configMINIMAL_STACK_SIZE, NULL,1 , &go);
  xTaskCreate(setSpeed, "Incresing Speed", configMINIMAL_STACK_SIZE, NULL,1 , &mSpeed);
  xTaskCreate(positionCount, "Counting position", configMINIMAL_STACK_SIZE+16, NULL,1 , &axis);
  
  vTaskStartScheduler();
}

void loop()
{  
     // nothing happens in the main loop
}



void setSpeed(void *argc)
{

  (void)argc;
   /* This task is responsible for continiously changing the speed of the motors. Depending on IR sensors */
  
  while (1)
  {   
    if (IR1 == 1 && IR2 == 0)    
     { 
       turnSpeed += 10;
       analogWrite(leftMotor, turnSpeed);
       analogWrite(rightMotor, 0);
       
       if (turnSpeed >= leftSpeed){
         turnSpeed = 10;   // making sure left turn speed never exceed left motor speed
       }
     }else if (IR1 == 0 && IR2 == 1) 
     {
       turnSpeed += 10;
       analogWrite(rightMotor, turnSpeed);
       analogWrite(leftMotor, 0);
       if (turnSpeed >= rightSpeed){
         turnSpeed = 10;     // making sure right turn speed never exceed right motor speed 
       }

     }else if (IR1 == 0 && IR2 == 0)
     {
      analogWrite(leftMotor, leftSpeed);    // re-writing both motors initial speed with ofsets for going forward 
      analogWrite(rightMotor, rightSpeed);  
      turnSpeed= 10;                        // re-seting turn speed again     

     }else if (IR1 == 1 && IR2 == 1)
     {
        stop();
     }


  }

}

void running(void *argc)
{

  (void)argc;
  
  /* This task is responsible for following the lines on the grid */

  while (1)
  {
    IR1 = digitalRead(IR1_pin);
    IR2 = digitalRead(IR2_pin);
    
    if (IR1 == 1 && IR2 == 0)    
     { 
      forward_right();
     }else if (IR1 == 0 && IR2 == 1) 
     {
       forward_left();
     }else if (IR1 == 0 && IR2 == 0)
     {
       forward();
     }else if (IR1 == 1 && IR2 == 1)
     {
       stop();
     }
  }
}


void positionCount(void *argc){

(void)argc;

   /* This task is responsible for counting its position on the grid */
  while (1)
  {

   IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);
   count = digitalRead(IR3_pin);
   destinationX = abs(targetX - currentX);
   destinationY = abs(targetY - currentY);

    switch (State)
    {
    case lowStateX:

      if (destinationX == 0 )
      {
        vTaskSuspend(go);     /* When X axis destiination is reached, speed seting task and line following task must be suspended for turning*/
        vTaskSuspend(mSpeed);      
        stop();
        vTaskDelay(200/portTICK_PERIOD_MS);
        State = turn;         // going to turning
      }
     
    if (count == 1){
      State = highStateX;    // if the third IR sees a black surface it swithces to high state and wait.
    }    
      break;
    case highStateX:
    
    if (count == 0 && IR1 == 0 && IR2 == 0)
    {  
      /* when all 3 of the IR sees a white/colourful surface, this state adds or substrucs 1
        depending on its current position. Then Immidietly goes back to low state and wait until next junction arrives*/ 
      
      if(targetX < currentX){  // this comparison and substruction is important to make destinationX == 0 
      currentX--; }                  // if we keep adding 1 but the targetX is smaller than currentX destinationX will never be 0
      else if(targetX > currentX){
      currentX++; }
      State = lowStateX;
    }
      break;


    case turn:
      if ((destinationX == 0) && (destinationY == 0))
      {
        State = destination;   // When both X and Y is 0, we send it to destination mode for reseting
      }  
      else
      {
        turning();
      }     
      break;

      case highStateY:
      /* Y axis count works exactly same as X axis*/

      if (count == 0 && IR1 == 0 && IR2 == 0)
    {
      if(targetY<currentY){
      currentY--;}
      else if(targetY>currentY)
      {
      currentY++; }
      State = lowStateY;
    }
      break;

      case lowStateY:
      if (destinationY == 0 )
      {
        vTaskSuspend(go);
        vTaskSuspend(mSpeed);
        stop();
        vTaskDelay(200/portTICK_PERIOD_MS);
        State = turn; 
      }   
    if (count == 1){
      State = highStateY; 
    }    

    case destination:
    // reseting X and Y 
      targetX = secondX;   
      targetY = secondY; 
      vTaskSuspend(go);
      vTaskSuspend(mSpeed);
      stop();
      vTaskDelay(200/portTICK_PERIOD_MS);    
      State = turn;    // after reseting sending back to turn to determine heading for nexr turn
      break;

    default:
       stop();
      break;
    }
    
  }
}



void turning()
{

/* Car'S heading could be in one of the 4 direction. In every direction the car compares 
 its current position vs target position and makes the correct turning 
 At direction 1 and 3 the car is following and counting x axis. After reaching X axis destination
 the car turn needs to towards y axis. So only the comparison of Y axis is required In case 1 and 3

 The opposite happens in 2 and 4  
 */

switch (direction)
{

 case 1:  // initially car is looking at direction 1

  if(currentY < targetY)
  {
     if (count == 0 && IR1 == 0 && IR2 == 1)    // left turn has been completed after this conditions are met 
     {
      turnManagerY();    // This fuction resumes line follow and speed set task after completing the turn. it also exit turning state 
      direction = 2;     // seting the direction at 2 after turning left
     }else {
     forward_left();   
     }
  }
  else if (currentY > targetY)
  {
    if (count == 1 && IR1 == 1 && IR2 == 0)  // right turn has been completed after this conditions are met 
     {
      turnManagerY();
      direction = 4;
      }else {
      forward_right();
      }
  }
  break;

case 2: 

 if(currentX > targetX)
  {
   if (count == 0 && IR1 == 0 && IR2 == 1)
    {
      turnManagerX();
      direction = 3;
    }else
    {
    forward_left(); 
    }
  }
  else if (currentX < targetX)
  { 
    if (count == 1 && IR1 == 1 && IR2 == 0)
    {
      turnManagerX();
      direction = 1;
    }else {
      forward_right();  
    }
  }
  break;

  case 3:
  if(currentY > targetY)
  {
    if (count == 0 && IR1 == 0 && IR2 == 1)
     {
     turnManagerY();
     direction = 4;
     }else
     {
     forward_left();   
     }
  }
  else if (currentY < targetY)
  { 
     if(count == 1 && IR1 == 1 && IR2 == 0)
     {
      turnManagerY();
       direction = 2;
      }else
      {
       forward_right();
      }
  }
  break;

case 4:
  if(currentX > targetX)
  {
  
    if (count == 1 && IR1 == 1 && IR2 == 0)
    {
    turnManagerX();
    direction = 3;
    }else 
    {
    forward_right(); 
    }
  }
  else if (currentX < targetX)
  {
    if (count == 0 && IR1 == 0 && IR2 == 1)
    {
    turnManagerX();
    direction = 1;
    }else
    {
       forward_left();  
    }
  }
  break;
default:
  break;
}
}

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
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
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

void turnManagerY(){
  stop();
  vTaskDelay(500/portTICK_PERIOD_MS);
  vTaskResume(mSpeed);  // resuming Speed
  vTaskResume(go);      // resuming line follow
  State = highStateY; 
}
void turnManagerX(){
  stop();
  vTaskDelay(500/portTICK_PERIOD_MS);
  vTaskResume(mSpeed);  // resuming Speed
  vTaskResume(go);      // resuming line follow
  State = highStateX; 
}