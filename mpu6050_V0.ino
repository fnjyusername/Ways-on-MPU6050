Enter file contents here

 /*
WAYS OF GETTING MPU6050
*/
//////////////////////////////
//  IMU Bank Orientation    //
//        -X  +X            //
//           |              //
//    -Y     |    -Y        //
//       ---------          //
//    +Y     |    +Y        //
//           |              //  
//        -X  +X            //
//       Pitch about Y      //
//       Rolls about X      //
//////////////////////////////
#include "Arduino.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


/*********IMU Definition**************/
MPU6050 mpu;
// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
float last_x_angle=0;  // These are the filtered angles
float last_y_angle=0;
float last_z_angle=0;  
float last_gyro_x_angle=0;  // Store the gyro angles to compare drift
float last_gyro_y_angle=0;
float last_gyro_z_angle=0;


//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

float    GYRO_FACTOR;// This global variable tells us how to scale gyroscope data
float    ACCEL_FACTOR;// This global varible tells how to scale acclerometer data

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Buffer for data output
char dataOut[256];

//ALL MY DEFINITIONS:
int Setpoint=0, n=0;  byte DelimRc, Delim; boolean ESC_ARMED=false, statusESC=false;

/*IMU STABILIZATION VARIABLES */
float Acc_x, Acc_y, Acc_z;
float Gyr_x, Gyr_y, Gyr_z;
float Rol_x, Pit_y, Yaw_z;


//Time Variable
volatile unsigned long  lastTime=0, now=0; 
volatile unsigned long  dt_loop=0; 


void setup() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif  
  
pinMode(LED_PIN, OUTPUT);pinMode(22, OUTPUT); pinMode(23, OUTPUT);
//Serial Initialization
Serial.begin(115200); Serial1.begin(115200); delay(500);//give time for serial to initialized 
while (Serial.available() && Serial.read()); delay(500);// empty buffer 
while (Serial1.available() && Serial1.read()); delay(500);// empty buffer 

initializeMotors(numberOfMotors);
commandAllMotors(MINPULSE);

/*********************START OF MPU 6050 CONFIG********************************/ 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
 MPU6050_GYRO_FS_250         0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_GYRO_FS_500         0x01
 MPU6050_GYRO_FS_1000        0x02
 MPU6050_GYRO_FS_2000        0x03
 
 MPU6050_ACCEL_FS_2          0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_ACCEL_FS_4          0x01
 MPU6050_ACCEL_FS_8          0x02
 MPU6050_ACCEL_FS_16         0x03
*/
    //A. GYRO SETTINGS
        // Set the full scale range of the GYROSCOPE (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above
        uint8_t FS_SEL = 0, LSB = 131;      
        mpu.setFullScaleGyroRange(FS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange(); delay(50); 
        Serial.print("GyroRange = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = LSB/(FS_SEL + 1);
               
     //B. ACCEL SETTINGS    
        // Set the full scale range of the ACCELEROMETER (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above     
        uint8_t AFS_SEL = 0;
        mpu.setFullScaleAccelRange(AFS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange(); delay(50); 
        Serial.print("AccelRange = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
      //C. Read High pass filter
        uint8_t READ_AFS_SEL = mpu.getDHPFMode(); delay(50);     
        Serial.print("DHPFMode = ");
        Serial.println(READ_AFS_SEL);     

      //C. Read Low pass filter
        uint8_t READ_AFS_SEL = mpu.getDLPFMode(); delay(50);     
        Serial.print("DLPFMode = ");
        Serial.println(READ_AFS_SEL);          

     //C. CALIBRATION
      calibrate_sensors();
/*******END OF MPU 6050 CONFIG********************************/  


// Configure LED Level Indicator
digitalWrite(LED_PIN, LOW);
digitalWrite(22, LOW);digitalWrite(23,LOW);
}


uint16_t pulseCounter=0; 
int start = 0;
void loop() 
{
       now=micros();
       dt_loop = (now- lastTime);
    
        if (dt_loop >= 2500)
           {//Start 400 Hz
            lastTime = micros();  pulseCounter++;    
              loopIMU(dt_loop); //Extract IMU           
         }//End 400 Hz loop    
}//End Main Loop



/****************************************************************************/
/****************************MPU FUNCTIONS***********************************/
/****************************************************************************/
float emaX=0, emaY=0, emaZ=0, emaXg=0, emaYg=0, emaZg=0;
const float alphax = 0.20; //0 to 1
void loopIMU(unsigned long  dt_imu)
{ float dt=dt_imu*0.000001; //micro to seconds
  
////////////////////////////////////START IMU DATA WXTRACTION////////////////////////////////////////
const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        unsigned long t_now = millis();
        
        //STORE IMU DATA TO FLOAT VARIABLE
        double axx = ax ;//Serial.print(ax); Serial.print("\t");
        double ayy = ay ;//Serial.print(ay); Serial.print("\t");
        double azz = az ;//Serial.println(az);              
        //Normalise the measurements
        double R = sqrt(axx*axx + ayy*ayy + azz*azz); //Serial.println(R/16384);// R^2 = Rx^2 + Ry^2 + Rz^2  Pythegorian  

        float  Ax = axx/R ;//Serial.print(axx); Serial.print("\t");
        float  Ay = ayy/R ;//Serial.print(ayy); Serial.print("\t");
        float  Az = azz/R ;//Serial.println(azz);
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = Ax; // - base_x_accel;
        float accel_y = Ay; // - base_y_accel;
        float accel_z = Az; // - base_z_accel;
        

              
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        
        float gyro_angle_x = gyro_x*dt + last_x_angle;
        float gyro_angle_y = gyro_y*dt + last_y_angle;
        float gyro_angle_z = gyro_z*dt + last_z_angle;
        
        
                           
         // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.98;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x; last_x_angle = angle_x; 
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y; last_y_angle = angle_y;
        float angle_z = gyro_angle_z; last_z_angle = angle_z;  //Accelerometer doesn't give z-angle
        angle_z = alpha*(angle_z + gyro_z*dt)+(1.0 - alpha)*gyro_angle_z;
              if (angle_z>0)angle_z=angle_z-0.00005; 
              if (angle_z<0)angle_z=angle_z+0.00003; 
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle; last_gyro_x_angle = unfiltered_gyro_angle_x;
        float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle; last_gyro_y_angle = unfiltered_gyro_angle_y;
        float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle; last_gyro_z_angle = unfiltered_gyro_angle_z;     
       
        

///////////////////////////////////////////END IMU DATA WXTRACTION///////////////////////////////////////////////

////////////////////////////////////START IMU DATA USAGE AND MANUPULATION////////////////////////////////////////

                    Acc_x= angle_x+0.00; Acc_x=xEMA(Acc_x);///Yaw
                    Acc_y=-angle_y-0.40; Acc_y=yEMA(Acc_y);//Pitch  (-)FORWARD   (+)BACKWARD                   
                    Acc_z=-angle_z;      Acc_z=zEMA(Acc_z); //RollS  (+)ROLLRIRGT (-)ROLLLEFT
                    if (Acc_z>0) {Acc_z=Acc_z-0.02;}
                    if (Acc_z<0) {Acc_z=Acc_z-0.02;}
                    
                    if       (Acc_x>45)  {Acc_x=45;}
                    else if  (Acc_x<-45) {Acc_x=-45;}
                    else     {Acc_x=Acc_x;}
                    
                    if       (Acc_y>45)  {Acc_y=45;}
                    else if  (Acc_y<-45) {Acc_y=-45;}
                    else     {Acc_y=Acc_y;}  
                    
                    if (Acc_x>=-0.20 && Acc_x<=0.20) {digitalWrite(23, HIGH);} else {digitalWrite(23, LOW);}
                    if (Acc_y>=-0.20 && Acc_y<=0.20) {digitalWrite(22, HIGH);} else {digitalWrite(22, LOW);}
                    
                    Gyr_x=gyro_x;  //Rolls Gyro + Right    -Left about X (See Axis top page)
                    Gyr_x=gxEMA(Gyr_x);
                    Gyr_y=-gyro_y; //Pitch Gyro + Forward   -Backward about Y (See Axis top page)                          
                    Gyr_y=gyEMA(Gyr_y);
                    Gyr_z=gyro_z; //Pitch Gyro -CW +CCW about Z (See Axis top page)                     
                    Gyr_z=gzEMA(Gyr_z);                   
                    /*DISPLAY
                      Serial.print(Gyr_x);  
                      Serial.print('\t');
                      Serial.print(Gyr_y); 
                      Serial.println(); */                  


////////////////////////////////////END IMU DATA USAGE AND MANUPULATION////////////////////////////////////////       
}






float xEMA(float new_value) {
  emaX += alphax*(new_value - emaX);
  return(emaX);
}

float yEMA(float new_value) {
  emaY += alphax*(new_value - emaY);
  return(emaY);
}

float zEMA(float new_value) {
  emaZ += alphax*(new_value - emaZ);
  return(emaZ);
}

float gxEMA(float new_value) {
  emaXg += alphax*(new_value - emaXg);
  return(emaXg);
}

float gyEMA(float new_value) {
  emaYg += alphax*(new_value - emaYg);
  return(emaYg);
}

float gzEMA(float new_value) {
  emaZg += alphax*(new_value - emaZg);
  return(emaZg);
}



// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 500; //Default 10

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}
