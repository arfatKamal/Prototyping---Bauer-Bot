int IR1_pin= 2;
int IR2_pin= 4;
 

int IR1, IR2; // Sensor 1 is on the left and Sensor 2 is on the right

void setup()
{
  pinMode(IR1_pin, INPUT); 
  pinMode(IR2_pin, INPUT); 
 }


void loop() {
 IRsensor(); 

}

void IRsensor() {
 IR1= digitalRead(IR1_pin); 
 IR2= digitalRead(IR2_pin);  

  delay(100);
}