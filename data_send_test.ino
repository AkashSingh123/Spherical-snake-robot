int c=0;
char inByte=0;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!Serial.available()) {}
  while(Serial.available()) 
   {
    //delay(30);  //delay to allow buffer to fill
    inByte=Serial.read();
    
    if (inByte=='T')
     {
       c = Serial.parseInt();   //gets one byte from serial buffer
      // Serial.println(c);
       //int d = Serial.read();  //gets one byte from serial buffer
       //Serial.println(d);
       Serial.println(c);
     }
   
   }


}
