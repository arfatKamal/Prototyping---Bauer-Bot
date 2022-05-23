#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

/*IR sensors */
const byte IR1_pin = A0;   // Left
const byte IR2_pin = A1;   // Right
byte IR1, IR2;

/* distance */
const byte trigPin = 12;
const byte echoPin = 13;
int currentDistance;

/* Motors */
const byte rightMotor = 10;   // Motor Right connections
const byte in1 = 9;           // right motor ; for foward LOW
const byte in2 = 8;           // right motor ; for forward HIGH

const byte leftMotor = 5;   // Motor Left connections
const byte in3 = 7;         // left motor ; for foward LOW
const byte in4 = 6;         // left motor ; for forward HIGH

byte Speed = 200;

byte error = 50;
byte leftSpeed  = Speed;
byte rightSpeed = Speed + error;

byte turnSpeed;



void readings(void *argc);
void running(void *argc);

TaskHandle_t Data, go;

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

  xTaskCreate(readings, "Sensor readings", configMINIMAL_STACK_SIZE+16, NULL, 1, &Data);
  xTaskCreate(running, "Car running", configMINIMAL_STACK_SIZE+16, NULL,1 , &go);



  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
}

void readings(void *argc)
{

  (void)argc;

  while (1)
  {
   IR1 = digitalRead(IR1_pin);
   IR2 = digitalRead(IR2_pin);

   Serial.println(IR1);
   Serial.println(IR2);
  }
}



void running(void *argc)
{

  (void)argc;

  while (1)
  {
    
  if (IR1 == 1 && IR2 == 0)    
     { 
       turnSpeed += 5;
       analogWrite(leftMotor, 0);
       analogWrite(rightMotor, turnSpeed);
       forward_left();

     }else if (IR1 == 0 && IR2 == 1) 
     {
       turnSpeed += 5;
       analogWrite(leftMotor, turnSpeed);
       analogWrite(rightMotor, 0);
       forward_right();

     }else if (IR1 == 1 && IR2 == 1)
     {
       turnSpeed= 80;

       forward();

     }else if (IR1 == 0 && IR2 == 0)
     {
       
       turnSpeed += 5;
       analogWrite(leftMotor, 0);
       analogWrite(rightMotor, turnSpeed);

       forward_left();
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
