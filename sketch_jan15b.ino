void setup() {
  // put your setup code here, to run once:
PORTD|=(1<<DDD7);
}

void loop() {
  // put your main code here, to run repeatedly:

}

     if(inByte == 1)
    {  
        flag=2;
        //mpu.resetFIFO();
        flag_IMU_retaliate=0;
       
    }
    
         if(inByte == 'T')
    {  
        flag=2;
        //mpu.resetFIFO();
        flag_IMU_retaliate=0;
       
    }
