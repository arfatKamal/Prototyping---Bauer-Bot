#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

/*IR sensors */
const uint8_t IR1_pin = A0;   // Left
const uint8_t IR2_pin = A1;   // Right
uint8_t IR1, IR2;

/* distance */
const uint8_t trigPin = 12;
const uint8_t echoPin = 13;
uint16_t currentDistance;

/* RGB color sensor declaration */
const uint8_t S0 = 0;
const uint8_t S1 = 1;
const uint8_t S2 = 2;
const uint8_t S3 = 3;
const uint8_t sensorOut = 4;

uint8_t destinationX; 
uint8_t targetX = 1;
uint8_t currentX= 0;
uint8_t y= 1;

volatile uint8_t colorState;  
const uint8_t cyanState = 0;
const uint8_t orangeState = 1;
const uint8_t turn = 2;
const uint8_t purpleState = 3;
const uint8_t greenState = 4;

uint8_t redPW = 0;
uint8_t greenPW = 0;
uint8_t bluePW = 0;


/* Motors */
const uint8_t rightMotor = 10;   // Motor Right connections
const uint8_t in1 = 9;           // right motor ; for foward LOW
const uint8_t in2 = 8;           // right motor ; for forward HIGH

const uint8_t leftMotor = 5;   // Motor Left connections
const uint8_t in3 = 7;         // left motor ; for foward LOW
const uint8_t in4 = 6;         // left motor ; for forward HIGH

uint8_t Speed = 85;

uint8_t error = 22;
uint8_t leftSpeed = Speed+error;
uint8_t rightSpeed = Speed;

uint8_t turnSpeed = 10;

bool test = true;



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

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(sensorOut, INPUT);

  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

 
  xTaskCreate(readings, "Sensor readings", configMINIMAL_STACK_SIZE, NULL, 1, &Data);
  xTaskCreate(running, "Car running", configMINIMAL_STACK_SIZE, NULL,1 , &go);
  xTaskCreate(setSpeed, "Incresing Speed", configMINIMAL_STACK_SIZE, NULL,1 , &mSpeed);
  xTaskCreate(positionCount, "Counting position", configMINIMAL_STACK_SIZE+16, NULL,1 , &axis);

  vTaskStartScheduler();
}

void loop()
{  
   /*IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);
   Serial.print("Left IR: ");
   Serial.println(IR1);
   Serial.print("Right IR: ");
   Serial.println(IR2);

  redPW = getRedPW();
  greenPW = getGreenPW(); 
  bluePW = getBluePW();

  Serial.print(" redPW: ");
  Serial.print(redPW);
  Serial.print(" greenPW:  ");
  Serial.print(greenPW);
  Serial.print(" bluePW:  ");
  Serial.print(bluePW);
  Serial.println();
  delay(500);

if(test == true){
  backward();
  delay(1000);
  stop();
  test = false;
}*/
  
}



void setSpeed(void *argc)
{

  (void)argc;

  while (1)
  {
     
  if (IR1 == HIGH && IR2 == LOW)    
     { 
       turnSpeed += 5;
       analogWrite(leftMotor, turnSpeed);
       analogWrite(rightMotor, 0);
       //vTaskDelay(100/portTICK_PERIOD_MS);
       if (turnSpeed >= leftSpeed){

         turnSpeed = 10;
       }

     }else if (IR1 == LOW && IR2 == HIGH) 
     {
       turnSpeed += 5;
       analogWrite(rightMotor, turnSpeed);
       analogWrite(leftMotor, 0);
       //vTaskDelay(100/portTICK_PERIOD_MS);
       if (turnSpeed >= rightSpeed){

         turnSpeed = 10;
       }


     }else if (IR1 == LOW && IR2 == LOW)
     {
      analogWrite(leftMotor, leftSpeed);
      analogWrite(rightMotor, rightSpeed);
      turnSpeed= 10; 

     }else if (IR1 == HIGH && IR2 == HIGH)
     {
        stop();
     }


  }

}

void readings(void *argc)
{

  (void)argc;

  while (1)
  {
   IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);
  
   /*Serial.print("Left IR: ");
   Serial.println(IR1);
   Serial.print("Right IR: ");
   Serial.println(IR2);*/

  redPW = getRedPW();
  greenPW = getGreenPW(); 
  bluePW = getBluePW();
  
  }
}


void positionCount(void *argc){

(void)argc;

  while (1)
  {

  
    switch (colorState)
    {
    case cyanState:
      
      Serial.print(" Destination : ");
      Serial.println(destinationX);
      destinationX = (targetX - currentX);

      if (destinationX == 0 )
      {
        vTaskSuspend(go);
        vTaskSuspend(mSpeed);
        stop();
        vTaskDelay(500/portTICK_PERIOD_MS);
        colorState = turn; 
      }
      

      if ((redPW >= 4 && redPW <= 5) && (greenPW >= 7  && greenPW <= 9) && ( bluePW >= 6 && bluePW <= 10))
        {      
            colorState = orangeState;      
        }

      break;
    case orangeState:

     if ((redPW >= 7 && redPW <= 8) && (greenPW >= 3  && greenPW <= 6) && ( bluePW >= 2 && bluePW <= 6))
        {
            currentX += 1;
            colorState = cyanState;      
        }     
      break;

      case turn:

      analogWrite(leftMotor, 102);
      analogWrite(rightMotor, 100);

      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

      vTaskDelay(1200/portTICK_PERIOD_MS);

      vTaskResume(mSpeed);
      vTaskResume(go);

      /*if ((IR1 == LOW && IR2 == HIGH) || )
      {
        stop();
        //vTaskDelay(500/portTICK_PERIOD_MS);
       // vTaskResume(go);  // resuming line follow
       // colorState = purpleState; 
      }*/
      break;

      case purpleState:

         stop();

        if ((redPW >= 7 && redPW <= 8) && (greenPW >= 3  && greenPW <= 6) && ( bluePW >= 2 && bluePW <= 6))
        {
            currentX += 1;
            colorState = greenState;      
        }   
       break;

      case greenState:
         
        stop();

         if ((redPW >= 7 && redPW <= 8) && (greenPW >= 3  && greenPW <= 6) && ( bluePW >= 2 && bluePW <= 6))
        {
            currentX += 1;
            colorState = purpleState;      
        }  
      break;
    
    default:
      break;
    }
    
  }
}

void running(void *argc)
{

  (void)argc;

  while (1)
  {
    
  if (IR1 == HIGH && IR2 == LOW)    
     { 
      forward_right();
      //stop();

     }else if (IR1 == LOW && IR2 == HIGH) 
     {
       forward_left();
       //stop();

     }else if (IR1 == LOW && IR2 == LOW)
     {
       forward();
      // stop();

     }else if (IR1 == HIGH && IR2 == HIGH)
     {
       stop();
     }

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



int getRedPW() {
 
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Green Pulse Widths
int getGreenPW() {
 
  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}
 
// Function to read Blue Pulse Widths
int getBluePW() {
 
  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Define integer to represent Pulse Width
  int PW;
  // Read the output Pulse Width
  PW = pulseIn(sensorOut, LOW);
  // Return the value
  return PW;
 
}