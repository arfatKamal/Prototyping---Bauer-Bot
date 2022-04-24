#include <IRremote.h>

int virtualIR = 2;

int IR1, IR2; // Sensor 1 is on the left and Sensor 2 is on the right

IRrecv irrecv(virtualIR);

decode_results virtualIR_read;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); 
  irrecv.blink13(true);
}

void loop() {
  map_IRsensor(); 

}

void map_IRsensor() {

/*_This function Maps the IR remote value to IR Variables. 
 * It uses the reading of one virtual IR sensor and maps it to two Physical IR sensors namely IR1 & IR 2.  
 *IR1 and IR2 can be used in the main code for IR sensor reads.
 *Just call this function in the void loop before calling IR1 & IR2.  
 *This way there will be less adjustment necessary while using the code on professor's platform  _*/

  
  if (irrecv.decode(&virtualIR_read)) {
    Serial.println(virtualIR_read.value, HEX);
    irrecv.resume(); // Receive the next value
    
    if(virtualIR_read.value==0xFD08F7){
      IR1=0;            //both sensors are on the black line.
      IR2=0;            //Remote Button= 1
       
      }
       else if(virtualIR_read.value==0xFD8877){
        IR1=0;         //Sensor 1 is on the black line & sensor 2 is on the White surface. 
        IR2=1;         //Remote Button= 2
      
    }
       else if(virtualIR_read.value==0xFD48B7){
        IR1=1;         //Sensor 2 is on the black line & sensor 1 is On the White Surface. 
        IR2=0 ;        //Remote Button= 3

    }

     else if(virtualIR_read.value==0xFD28D7){
        IR1=1;      //Both Sensors are on the White Surface.
        IR2=1;       //Remote Button= 4

        }
  }
  delay(100);
}
