/*
A(Top botton in the app) - The robot moves forward for 2 seconds and rotates 90 degrees.This repeats
E(Centre botton in the app ) - Robot stops
if max_pulse == 75 , the robot rotates 90 degrees
*/

// basic motor control
// motor1 - PB'6 and PD'3
// motor2 - PB'7 and PD'4


char inByte;
int previous_sensorValue = 0;
int pulse = 0;
int max_pulse = 19;
int flag = -1;
void setup() {
  
  // set as outputs for controlling motors
  DDRB |= (1 << DDB6) | (1 << DDB7);
  DDRD |= (1 << DDD6) | (1 << DDD7);
  //Initialize Serial communnication
  Serial.begin(9600);  
}

void loop() {
  
  inByte = Serial.read();
  if(inByte == 'A'){
    flag = 1;
  }
  else if(inByte == 'E'){
    flag = 0;
  }
  
  if (flag == 1){
    // Move robot forward
    PORTB = (1<<DDB6);
    PORTD = (0<<DDD3); 
    delay(2000);
    // Rotate 90 degress
    while (pulse < max_pulse)
        {
         Serial.println(inByte);
         //Rotate motor in clock wise direction
         PORTB = (0<<DDB7);
         PORTD = (1<<DDD4);
        
         //read input from analog pin 0
         int current_sensorValue = analogRead(A0);
         Serial.print(current_sensorValue );
         Serial.print("\t");
         Serial.println(pulse);
  
         Serial.print("\n");
         if(current_sensorValue-previous_sensorValue >15)
           {
            pulse = pulse + 1;
           }
         previous_sensorValue = current_sensorValue;
        }
        pulse = 0;
  }
  
  else if(flag == 0){
    Serial.println(inByte);
    PORTB = (0<<DDB7)|(0<<DDB6);
    PORTD = (0<<DDD4)|(0<<DDD3);
  }
     // Stop the robot if code comes out of while loop 
  /*PORTB = (0<<DDB7)|(0<<DDB6);
  PORTD = (0<<DDD4)|(0<<DDD3);*/
    
}
