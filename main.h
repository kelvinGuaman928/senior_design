/*
icm 20948 from adafruit

*/
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

const float RAD2DEG = 180.0f / PI;
const float G = 9.81; 
float pitch,roll; 
float gyro_x_rotation, gyro_y_rotation, gyro_z_rotation;

float axg,ayg,azg;


Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_accel, *icm_gyro, *icm_mag;



void setup(void) {
  Serial.begin(115200);
  // pause while the serial monitor is not open 
  while (!Serial)
    delay(10);

  Serial.println("Adafruit ICM20948");

  if (!icm.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  // range can be change 2,4,8,16,2g sound resobale for human movments 
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  // lower the DPS gyro can capture slow rotation but fast saturation would be sturated
  
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);// range 250,500,1000,2000

  icm.setGyroRateDivisor(255);
  icm_accel = icm.getAccelerometerSensor();
  
  icm_gyro = icm.getGyroSensor();

  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ); 
  icm_mag = icm.getMagnetometerSensor();
  icm_mag->printSensorDetails();

  Serial.println();
  
}

void loop() {
  //  /* Get a sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;

  icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);
  icm_mag->getEvent(&mag);
  
 

  /* Display the results (acceleration is measured in m/s^2)
    acce.x = data[0] / range_sensti
    */
   
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println();
 
  /* Display the results (rotation is measured in rad/s)*/
  
  
  gyro_x_rotation= gyro.gyro.x*RAD2DEG; // pitch angle around x 
  gyro_y_rotation = gyro.gyro.y*RAD2DEG; // roll angle around y
  gyro_z_rotation = gyro.gyro.z*RAD2DEG;
  
  Serial.print(gyro_x_rotation);
  Serial.print(" \tY: ");
  Serial.print(gyro_y_rotation);
  Serial.print(" \tZ: ");
  Serial.print(gyro_z_rotation);
  Serial.println(" dergees ");
  Serial.println();
  
  /*
   * for calulating roll and pitch
   * 
   * 
   */
    axg = accel.acceleration.x / G; // acc in Gs 
    ayg = accel.acceleration.y / G;
    azg = accel.acceleration.z /G;
     
    roll  = (atan2(-axg,azg))* RAD2DEG;
    pitch = (atan2(ayg, sqrt(square(axg)+ square(azg))))*RAD2DEG;
    
    Serial.print("\tacceleration roll & pitch");
    Serial.print("\troll:");
    Serial.print(roll);
    Serial.print("\tpitch:");
    Serial.print(pitch);
  
  
  /*
   *   Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");
   */
  
  delay(100);
}
