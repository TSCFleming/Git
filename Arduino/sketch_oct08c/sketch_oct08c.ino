#include <Adafruit_MPU6050.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <KalmanFilter.h>

const int chipSelectSD = 4;
Adafruit_MPU6050 mpu;
Adafruit_LPS22 lps;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;
float totalYaw = 0;

float kalPitch = 0;
float kalRoll = 0;

unsigned long timer;

String dataStringSave = "";

File dataFile;

void setupSD() {

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectSD)) {
    // don't do anything more:
    while (1);
  }
}

void saveToSD(String dataString) {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  dataFile = SD.open("datalog.csv", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();// print to the serial port too:
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void setupMPU() {

  // Try to initialize!
  if (!mpu.begin(0x68)) {
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void setupLPS() {
  // Try to initialize!
  if (!lps.begin_I2C(0x5D)) {
    while (1) {
      delay(10);
    }
  }

  lps.setDataRate(LPS22_RATE_10_HZ);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //0x5D is pressure
  //0x86 is acc/gyro
  while (!Serial)
    delay(10);
  setupSD();
  setupMPU();
  setupLPS();
  timer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
    /* Get new sensor events with the readings */
  sensors_event_t a, g, tempMPU;
  mpu.getEvent(&a, &g, &tempMPU);
  
  sensors_event_t tempLPS, pressure;
  lps.getEvent(&pressure, &tempLPS);// get pressure

    // Calculate Pitch & Roll from accelerometer (deg)
  accPitch  = ((atan2(a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z))*180.0)/M_PI);
  accRoll = -((atan2(a.acceleration.y, sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z))*180.0)/M_PI);

   // Kalman filter
  kalPitch = kalmanY.update(accPitch, g.gyro.y);
  kalRoll = kalmanX.update(accRoll, g.gyro.x);
  
  if (abs(g.gyro.x + 0.01) > 0.015) {
    totalYaw += ((g.gyro.x + 0.01) * (kalPitch/90))*0.4;
  }
  if (abs(g.gyro.y + 0.05) > 0.015) {
    totalYaw += ((g.gyro.y + 0.05) * (kalRoll/90))*0.4;
  }
  if (abs(g.gyro.z - 0.01) > 0.015) {
    totalYaw += ((g.gyro.z - 0.01) * ((kalPitch/90) * -1 + 1) * ((kalRoll/90) * -1 + 1))/3.7444;
  }

  if (totalYaw >= 180) {
    totalYaw = -180;
  } else if (totalYaw <= -180){
    totalYaw = 180;
  }
  
  String dataString1 = "";
  /* Print out the values */
  dataString1 += accPitch;//accPitch
  dataString1 += ":";
  dataString1 += accRoll;//accRoll
  dataString1 += ":";
  dataString1 += kalPitch;//kalPitch
  dataString1 += ":";
  dataString1 += kalRoll;//kalRoll
  dataString1 += ":";
  dataString1 += totalYaw;//yaw
  dataString1 += ":";
  dataString1 += a.acceleration.x;
  dataString1 += ":";
  dataString1 += a.acceleration.y;
  dataString1 += ":";
  dataString1 += a.acceleration.z;
  dataString1 += ":";
  dataString1 += g.gyro.x;
  dataString1 += ":";
  dataString1 += g.gyro.y;
  dataString1 += ":";
  dataString1 += g.gyro.z;
  
  Serial.println(dataString1);
  dataStringSave += dataString1;
  dataStringSave += ":";
  dataStringSave += pressure.pressure;
  dataStringSave += "\n";
  //delay(500);
  if ((millis() - timer) > 5000) {
    timer = millis();
    saveToSD(dataStringSave);
    dataStringSave = "";
    dataFile.close();
    setupMPU();
    setupLPS();
  }
}
