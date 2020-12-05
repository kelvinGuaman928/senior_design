
/*
  Kelvin Guaman 
  reading sensor data
 adafruit library was implemented to read sensor data
*/
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


Adafruit_Sensor *accel2,*accel;


const float RAD2DEG = 180.0f / PI;
const float G = 9.81; 

int k = 0;
float accx,accy,accz;
float accx_2,accy_2,accz_2;
float gyrox,gyroy,gyroz;
float gyrox_2,gyroy_2,gyroz_2;

float mag_acc;
float mag_acc_2;
float mag_gyro;
float mag_gyro_2;
/*
float theta_mea,phi_mea;
float thetanew,phinew; 
float thetaold = 0;
float phiold = 0;
*/
float current_mag_acc = 0;
float current_mag_acc_2 =0;   
float  current_mag_gyro =0;
float current_mag_gyro_2 =0;

float total_mag_acc ,total_mag_acc_2 ,total_mag_gyro,total_mag_gyro_2; 
long previousMill = 0;
long interval = 2500;



Adafruit_ICM20948 icm;
Adafruit_ICM20948 icm2;
//Adafruit_Sensor *icm_accel, *icm_gyro, *icm_mag;
//Adafruit_Sensor *icm_accel_2, *icm_gyro_2, *icm_mag_2;



void setup(void) {
  Serial.begin(115200);
  // pause while the serial monitor is not open 

  icm.begin_I2C(0x69);
  icm2.begin_I2C(0x68);

  // range can be change 2,4,8,16,2g sound resobale for human movments 
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm2.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm2.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  accel =icm2.getAccelerometerSensor();
  accel2 =icm2.getAccelerometerSensor();


  // lower the DPS gyro can capture slow rotation but fast saturation would be sturated
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);// range 250,500,1000,2000
  icm2.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  
  icm.setGyroRateDivisor(255);
  icm2.setGyroRateDivisor(255);


}

void loop() {
  //  /* Get a sensor event */
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  //  /* Get a new normalized sensor event */
  sensors_event_t accel2;
  sensors_event_t gyro2;
  sensors_event_t mag2;
  sensors_event_t temp2;
  icm2.getEvent(&accel2, &gyro2, &temp2, &mag2);

  accz = accel.acceleration.z/G; 
  accy = accel.acceleration.y/G;
  accx = accel.acceleration.x/G;
  
  accx_2 =(accel2.acceleration.x/G);
  accy_2 =accel2.acceleration.y/G;
  accz_2 = accel2.acceleration.z/G;

  gyrox =gyro.gyro.x;
  gyroy =gyro.gyro.y;
  gyroz =gyro.gyro.z;
  
  gyrox_2 = gyro2.gyro.x;
  gyroy_2 = gyro2.gyro.y;
  gyroz_2 = gyro2.gyro.z;


  mag_acc = sqrt(square(accz) + (square(accy))+ (square(accx)));
  mag_acc_2 = sqrt(square(accx_2) + (square(accy_2)) + (square(accz_2))); 
  mag_gyro= sqrt(square(gyrox) + (square(gyroy)) + (square(gyroz)));
  mag_gyro_2 = sqrt(square(gyrox_2) + (square(gyroy_2)) + (square(gyroz_2)));

   // timer: adds the value of magnitude unitl 2.6s passed then send it out to the serial port
   unsigned long currentMillis = millis();

  if (currentMillis - previousMill >interval){
    previousMill = currentMillis;
    Serial.print( current_mag_acc);
    Serial.print(",");
    Serial.print(current_mag_acc_2);
    Serial.print(",");
    Serial.print(current_mag_gyro);
    Serial.print(",");
    Serial.print(current_mag_gyro_2);
    Serial.println();
    current_mag_acc = 0;
    current_mag_acc_2 = 0 ; 
    current_mag_gyro = 0;
    current_mag_gyro_2 =0;
  }
  total_mag_acc = mag_acc + current_mag_acc;
  total_mag_acc_2 = mag_acc_2 + current_mag_acc_2;
  total_mag_gyro= mag_gyro+ current_mag_gyro;
  total_mag_gyro_2 = mag_gyro_2 + current_mag_gyro_2;
    
  current_mag_acc = total_mag_acc;
  current_mag_acc_2 = total_mag_acc_2 ; 
  current_mag_gyro = total_mag_gyro;
  current_mag_gyro_2 =total_mag_gyro_2;
   
  delay(100);
}
 /* 
  // measure angle displacment of user 
  theta_mea = (-atan2(accx_2,accz_2))* RAD2DEG;
  phi_mea = (atan2(-accx_2,accz_2))* RAD2DEG;
  
  phinew = .3* phiold+.7*phi_mea; // pitch 
  thetanew = .8 * thetaold+ .2*theta_mea; // row
  
  thetaold = thetanew;
  phiold = phinew ; 
 // Serial.print(phinew);
 
  ///thigh 
  Serial.print(accx);
  Serial.print(",");
  Serial.print(accy);
  Serial.print(",");
  Serial.print(accz);   
  Serial.print(",");

  // HIP
  Serial.print(accx_2);
  Serial.print(",");
  Serial.print(accy_2);
  Serial.print(",");
  Serial.print(accz_2);
  Serial.print(",");  
  
//thigh 
  Serial.print(gyrox);
  Serial.print(",");
  Serial.print(gyroy);
  Serial.print(",");
  Serial.print(gyroz);   
  Serial.print(",");
// HIP
  Serial.print(gyrox_2);
  Serial.print(",");
  Serial.print(gyroy_2);
  Serial.print(",");
  Serial.print(gyroz_2);  
  Serial.print(",");
// magnitude
  Serial.print(mag_acc);
  Serial.print(",");
  Serial.print(mag_acc_2);
  Serial.print(",");
  Serial.print(mag_gyro);
  Serial.print(",");
  Serial.print(mag_gyro_2);
  
  Serial.println();
  delay(100);
 */
  
  
