#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//sphere variables

char inByte;
int i=0;
int roll_initial = 0;
int roll_1=0;
int roll_2=0;
int yaw_prev=0;
int yaw_recent=0;
int yaw_recent1=0;
int yaw_recent2=0;
int previous_sensorValue = 0;
int pulse = 0;
int max_pulse = 19;
int flag = -1;
int diff=100;
int thresh=6;
int yaw_initial=0;
int count=0;
int roll_recent1=0;
int roll_recent2=0;
int diff2=0;
int roll_prev=0;
int a=0;
int OCRV=0;
int start=0;
int flag_IMUsend=0;
int yaw_val=0;
int c=0;
int diff_y=0;
int yaw_t=0;
int rotate=0;
int turn_complete=1;
// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t PORTB6mask = 0b10111111;
uint8_t PORTD3mask = 0b11110111;
uint8_t PORTD4mask = 0b11101111;
uint8_t PORTB7mask = 0b01111111;
// packet structure for InvenSense teapot demo
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {


  
    // set as outputs for controlling motors
  DDRB |= (1 << DDB6) | (1 << DDB7);
  DDRD |= (1 << DDD3) | (1 << DDD4);
  OCR2B=250;
  
  Serial.begin(9600);
  sei();
//  pwm_init();
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    
    while (!Serial); 
   // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);// 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
     //   Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
     //   Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
      //  Serial.print(F("DMP Initialization failed (code "));
       // Serial.print(devStatus);
       // Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}

void pwm_init()
{
  
  TIMSK2 =  (1<<TOIE2)|(1<<OCIE2B);
  TCCR2A =  _BV(WGM21) | _BV(WGM20);
  TCCR2B =  _BV(CS22)|_BV(CS21);
}

ISR(TIMER2_COMPB_vect)
{
 
 //  Serial.println(PORTD);
   PORTB = PORTB|(1<<DDB6);
//   
   PORTD = PORTD&(PORTD3mask);
}

ISR(TIMER2_OVF_vect)
{
     //Serial.println(PORTD);

   PORTB =PORTB&(PORTB6mask);
   PORTD =PORTD&(PORTD3mask);
}
  
int IMU_values(){
  if (!dmpReady) return 0;
  while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
     //   digitalWrite(LED_PIN, blinkState);
    }
return ypr[0] * 180/M_PI;
  }
  
  int IMU_values2(){
  if (!dmpReady) return 0;
  while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        #endif
        // blink LED to indicate activity
        blinkState = !blinkState;
    //    digitalWrite(LED_PIN, blinkState);
    }
return ypr[2] * 180/M_PI;
  }

int custom_delay(int no) 
 {
   int k;
   for (int i=0; i<no ;i++)
          {
            k = IMU_values();
          }
 }
 int custom_delay2(int no) 
 {
   int k;
   for (int i=0; i<no ;i++)
          {
            k = IMU_values2();
          }
 }
 int turn_caliberate()
 {
    mpu.resetFIFO();
    int flag2=0;
    int diff2_prev=diff2;
    while(flag2<1)
    {
     roll_recent1 = IMU_values2();
  //   Serial.print(roll_recent1);
     custom_delay2(20);
     roll_recent2 = IMU_values2();
  //   Serial.print(roll_recent2);
     diff2= roll_recent2 - roll_recent1;
  //   Serial.print("ROLLdifference is");
 //    Serial.print("\t");
 //    Serial.println(diff2);
       if(abs(diff2)<2)
       {
         if(abs(diff2_prev)<2)
         {
           flag2=1;
         }
       }
      diff2_prev=diff2;
    }
 }

void forward_decelerate(int OCR_value)
{
   OCR2B=OCR_value;
   while(OCR2B<250)
   {
      OCR2B=OCR2B+1;
      custom_delay(1);
   }
 }
int init_caliberate()
    
 {  
    int init_count=0;
    while(init_count<1)
     { 
      mpu.resetFIFO();
      yaw_recent1 = IMU_values();
      custom_delay(200);
      yaw_recent2 = IMU_values();
      diff= yaw_recent2 - yaw_recent1;
 //     Serial.print("difference is");
 //     Serial.print("\t");
 //     Serial.println(diff);
      if(abs(diff)<1)
       {
        yaw_prev=yaw_recent2;
        init_count=1;
       }
     }
     return yaw_prev;
  } 
  
  
void loop(){
  //PORTD=(1 << DDD5);
  //Serial.println(PORTD);
  if(count<1)
  { 
     //PORTD=(1 << DDD5);
     yaw_prev=init_caliberate();
 //    Serial.println("yaw_prev is");
 //    Serial.println(yaw_prev);
     count++;
     OCR2B=250;
     pwm_init();
     //OCR2B=250;
  }
  if(count>0)
  {
    //PORTD=PORTD|(1 << DDD4);
 //   Serial.println(PORTD);
   
   while(!Serial.available()) 
   {
     yaw_prev=IMU_values();
    // start=start+1;
   }
   while(1)
   {
      
    inByte = Serial.read();
    Serial.flush();

    if(inByte == 'A')
    
    {  
       OCR2B=250; 
        mpu.resetFIFO();
        yaw_prev=IMU_values();
 //      Serial.println("this yaw_prev is");
 //      Serial.println(yaw_prev);
       flag = 1;
    }
    
    if(inByte == 'B')
    {
      if(flag==1)
      {
         OCRV=OCR2B;
         forward_decelerate(OCRV);
      }
      flag = 2;
  
    }  
    if(inByte == 'C')
    {   
        if(flag==1)
        {
         OCRV=OCR2B;
         forward_decelerate(OCRV);
        }
        turn_caliberate();
        flag = 3;
    }
    if(inByte == 'D')
    {
      if(flag==1)
      {
         OCRV=OCR2B;
         forward_decelerate(OCRV);
      }
      turn_caliberate();  
      flag = 4;
    }
    if(inByte == 'E')
    {
      if(OCR2B<250)
      {
         OCRV=OCR2B;
         forward_decelerate(OCRV);
      }
      flag = 0;
    }
    if(inByte == 'I')
    
    {  
        flag=5;
      //mpu.resetFIFO();
        flag_IMUsend=0;
       
    }
    if(inByte == 'T')
    
    {  
        flag=6;
      //mpu.resetFIFO();
      //  while(!Serial.available()){}
      //  while(Serial.available())
          {
            c = Serial.parseInt();
            yaw_t=c;
            //Serial.print(c);
            //flag_IMU_retaliate=0;
          }
        turn_complete=0;
    
    }
    if(inByte == 'R')
    
    {  
        flag=7;
        OCR2B = 60;
      //mpu.resetFIFO();
      //  while(!Serial.available()){}
      //  while(Serial.available())
    
    }
    
   if (flag == 1){
     if(OCR2B>2)
     {
        OCR2B=OCR2B-1; 
     }
        custom_delay(1);
        PORTB = PORTB&(PORTB7mask);
        PORTD = PORTD&(PORTD4mask); 
        yaw_recent = IMU_values();
        diff = yaw_recent-yaw_prev;

    if( abs(diff) > thresh)
    {
      if(diff>0)
       {
       //Serial.println("turnleft");
       PORTB = PORTB&(PORTB7mask);
       PORTD = PORTD|(1<<DDD4);
       if(abs(diff)>180)
       {
         PORTB = PORTB|(1<<DDB7);
         PORTD = PORTD&(PORTD4mask);
       }
       //custom_delay(10);
      }
      if(diff<0)
      {
       PORTB = PORTB|(1<<DDB7);
       PORTD = PORTD&(PORTD4mask);
       //Serial.println("turnright");
       if(abs(diff)>180)
        {
          PORTB = PORTB&(PORTB7mask);
          PORTD = PORTD|(1<<DDD4); 

        }
        //custom_delay(10);
      }
    }
  }

  else if(flag == 2)
   {
     //Serial.println(inByte);
      PORTB = PORTB&(PORTB7mask);
      PORTD = PORTD&(PORTD4mask);
      turn_caliberate();
      //Serial.println("done caliberation");
      //custom_delay(1);
      // Rotate 90 degress
       while (pulse < max_pulse)
          {
          
           PORTB = PORTB&(PORTB7mask);
           PORTD = PORTD|(1<<DDD4);
           a=IMU_values();         //    read input from analog pin 0
           int current_sensorValue = analogRead(A0);
          

          if(current_sensorValue-previous_sensorValue >15)
             { 
               pulse = pulse + 1;
             }
             previous_sensorValue = current_sensorValue;
         }
           pulse = 0;
           flag=0;
           PORTB = PORTB&(PORTB7mask);
           PORTD = PORTD&(PORTD4mask);
           //Serial.println("it Happend!");
           
  }
  else if(flag == 3)       
  {
   
    
    PORTB = PORTB&(PORTB7mask);
    PORTD = PORTD|(1<<DDD4); 
    custom_delay(1);
    PORTB = PORTB&(PORTB7mask);
    PORTD = PORTD&(PORTD4mask);
  }
  else if(flag == 4)       
  { 
    
    PORTB = PORTB|(1<<DDB7);
    PORTD = PORTD&(PORTD4mask); 
    custom_delay(1);
    PORTB = PORTB&(PORTB7mask);
    PORTD = PORTD&(PORTD4mask);
  }
    
  else if(flag == 0)
  {
     //Serial.println("yobabe");
     a=IMU_values();
     PORTB = PORTB&(PORTB7mask);
     PORTD = PORTD&(PORTD4mask);
  }
  else if(flag == 5)
  {
      
        yaw_val=IMU_values();
       if(flag_IMUsend==0)
         {
          Serial.println(yaw_val);
          flag_IMUsend=1;
         }
  }
   else if(flag == 6)
    {
      //delay(30);
        
        yaw_val=IMU_values();
        diff_y=-((yaw_t)-yaw_val);
       while( abs(diff_y) > 5)
         {
           custom_delay(1);
           PORTB = PORTB&(PORTB7mask);
           PORTD = PORTD&(PORTD4mask); 
           yaw_val=IMU_values();
           diff_y=-((yaw_t)-yaw_val);
           if(diff_y>0)
           {   
              rotate = 1;
            //Serial.println("turnleft");
              PORTB = PORTB&(PORTB7mask);
              PORTD = PORTD|(1<<DDD4);
             if(abs(diff_y)>180)
             {
              rotate = 2;
              PORTB = PORTB|(1<<DDB7);
              PORTD = PORTD&(PORTD4mask);
             }
       //custom_delay(10);
           }
       if(diff_y<0)
          {
            rotate = 2;
            PORTB = PORTB|(1<<DDB7);
            PORTD = PORTD&(PORTD4mask);
        //Serial.println("turnright");
           if(abs(diff_y)>180)
            {
             rotate = 1;
             PORTB = PORTB&(PORTB7mask);
             PORTD = PORTD|(1<<DDD4); 
            }
         //custom_delay(10);
        }
    }   
        PORTB = PORTB&(PORTB7mask);
        PORTD = PORTD&(PORTD4mask); 
        if(turn_complete==0)
          {
            Serial.println('T');
            turn_complete=1;
          }   
       
    }
       else if(flag == 7)
       {
         
         if(rotate==1)
          {
            PORTB = PORTB&(PORTB7mask);
            PORTD = PORTD|(1<<DDD4);
            custom_delay(1);
            PORTB = PORTB&(PORTB7mask);
            PORTD = PORTD&(PORTD4mask);
            custom_delay(3);
 
          }
          if(rotate==2)
          {
            PORTB = PORTB|(1<<DDB7);
            PORTD = PORTD&(PORTD4mask);
            custom_delay(12);
            PORTB = PORTB&(PORTB7mask);
            PORTD = PORTD&(PORTD4mask);
           custom_delay(3); 
          }
                 
       }

 }
  }
}
