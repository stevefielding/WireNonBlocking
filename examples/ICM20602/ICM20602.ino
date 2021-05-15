/*
===Contact & Support===
Website: http://eeenthusiast.com/
Youtube: https://www.youtube.com/EEEnthusiast
Facebook: https://www.facebook.com/EEEnthusiast/
Patreon: https://www.patreon.com/EE_Enthusiast
Revision: 1.0 (July 13th, 2016)
===Hardware===
- Arduino Uno R3
- MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
===Software===
- Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
- Arduino IDE v1.6.9
- Arduino Wire library
===Terms of use===
The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or 
copyright holders be liable for any claim, damages or other liability, whether in an action of contract, 
tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in 
the software.
*/

//#include <Wire.h>
#include <WireNonBlocking.h>

int16_t accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

int16_t gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float tempInC;

#define ICM_ADDR 0b1101000

enum icmStates {ICM_INIT1, ICM_INIT2,ICM_INIT3, ICM_INIT4, ICM_ACCEL1, ICM_ACCEL2, ICM_GYRO1, ICM_GYRO2, ICM_TEMP1, ICM_TEMP2};
int icmCurrSt;


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire1.begin();
  // Run at i2c @ 100KHz to avoid race condition in setting stop bit for last data bit rxed.
  // At 100KHz the i2c must be serviced from the main loop within 100uS
  Wire1.setClock(100000);
  setupMPU();
  delay(1000);
  icmCurrSt = ICM_INIT4;
}


void loop() {
  recordAccelRegisters();
  //recordGyroRegisters();
  //recordTempRegisters();
  //printData();
  //delay(100);
}

void setupMPU(){
  Wire1.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire1.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire1.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire1.endTransmission();  
  Wire1.beginTransmission(0b1101000); //I2C address of the MPU
  Wire1.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire1.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire1.endTransmission(); 
  Wire1.beginTransmission(0b1101000); //I2C address of the MPU
  Wire1.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire1.write(0b00000000); //Setting the accel to +/- 2g
  Wire1.endTransmission(); 
}

uint8_t bytesRead;
bool readSuccess;
uint8_t byteWriteError;
void recordAccelRegisters() {


uint8_t byteWriteError;
int16_t regVal;
bool readSuccess;
uint8_t bytesRead;

  switch (icmCurrSt) {
    case ICM_INIT4:
      byteWriteError = 0;
      readSuccess = false;
      icmCurrSt = ICM_ACCEL1;
      Wire1.beginTransmission(ICM_ADDR); //I2C address of the MPU
      Wire1.write(0x3B); //Starting register for Accel Readings
      Wire1.endTransmission_nb(true, &byteWriteError);
      break;
    case ICM_ACCEL1:
      if (Wire1.endTransmission_nb(false, &byteWriteError)) {
        icmCurrSt = ICM_ACCEL2;
        // Total 650uS for requestFrom @ 100KHz. 2.3uS per pass
        // Total 170uS for requestFrom @ 400KHz. 2.3uS per pass
        // Polling for RX bytes, so run at i2c @ 100KHz to allow 100uS to service RX bytes.
        Wire1.requestFrom_nb(true, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR , (uint8_t) 6, (uint32_t) 0, (uint8_t) 0, (uint8_t) true);
      }
      break;
    case ICM_ACCEL2:
      if (Wire1.requestFrom_nb(false, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR, (uint8_t) 6, (uint32_t) 0, (uint8_t) 0, (uint8_t) true)) {
        icmCurrSt = ICM_GYRO1;
        if (readSuccess && bytesRead >= 6) {
          //while(Wire1.available() < 6);
          accelX = Wire1.read()<<8|Wire1.read(); //Store first two bytes into accelX
          accelY = Wire1.read()<<8|Wire1.read(); //Store middle two bytes into accelY
          accelZ = Wire1.read()<<8|Wire1.read(); //Store last two bytes into accelZ
          processAccelData();
        }
        else {
          Serial.println("accell read failed");
          Serial.println(readSuccess);
          Serial.println(bytesRead);
        } 
        //printData();
        delay(1);
        Wire1.beginTransmission(ICM_ADDR); //I2C address of the MPU
        Wire1.write(0x43); //Starting register for Gyro Readings
        Wire1.endTransmission_nb(true, &byteWriteError); 
      }
      break;




    case ICM_GYRO1:
      if (Wire1.endTransmission_nb(false, &byteWriteError)) {
        icmCurrSt = ICM_GYRO2;
        Wire1.requestFrom_nb(true, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR , (uint8_t) 6, (uint32_t) 0, (uint8_t) 0, (uint8_t) true);
      }
      break;
    case ICM_GYRO2:
      if (Wire1.requestFrom_nb(false, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR, (uint8_t) 6, (uint32_t) 0, (uint8_t) 0, (uint8_t) true)) {
        icmCurrSt = ICM_TEMP1;
        if (readSuccess && bytesRead >= 6) {
          gyroX = Wire1.read()<<8|Wire1.read(); //Store first two bytes into accelX
          gyroY = Wire1.read()<<8|Wire1.read(); //Store middle two bytes into accelY
          gyroZ = Wire1.read()<<8|Wire1.read(); //Store last two bytes into accelZ
          processGyroData();
        }
        else {
          Serial.println("gyro read failed");
          Serial.println(readSuccess);
          Serial.println(bytesRead);
        }
        delay(1);
        Wire1.beginTransmission(ICM_ADDR); //I2C address of the MPU
        Wire1.write(0x41); //Starting register for Temp Readings
        Wire1.endTransmission_nb(true, &byteWriteError);
      }
      break;
    case ICM_TEMP1:
      if (Wire1.endTransmission_nb(false, &byteWriteError)) {
        icmCurrSt = ICM_TEMP2;
        Wire1.requestFrom_nb(true, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR , (uint8_t) 2, (uint32_t) 0, (uint8_t) 0, (uint8_t) true);
      }
      break;
     case ICM_TEMP2:
      if (Wire1.requestFrom_nb(false, &readSuccess, &bytesRead, (uint8_t) ICM_ADDR, (uint8_t) 2, (uint32_t) 0, (uint8_t) 0, (uint8_t) true)) {
        icmCurrSt = ICM_INIT4;
        if (readSuccess && bytesRead >= 2) {
          regVal = Wire1.read()<<8|Wire1.read(); //Store first two bytes into temp
          tempInC = processTempData(regVal);
          printData();
        }
        else
          Serial.println("temp read failed");
        printData();
        delay(100); 
        //Wire1.beginTransmission(ICM_ADDR); //I2C address of the MPU
        //Wire1.write(0x3B); //Starting register for Accel Readings
        //Wire1.endTransmission_nb(true, &byteWriteError);
      }
      break;






    default:
      break;
  }





}

void recordTempRegisters() {
  Wire1.beginTransmission(0b1101000); //I2C address of the MPU
  Wire1.write(0x41); //Starting register for Temp Readings
  Wire1.endTransmission();
  Wire1.requestFrom(0b1101000,2); //Request Accel Registers (3B - 40)
  while(Wire1.available() < 2);
  int16_t regVal = Wire1.read()<<8|Wire1.read(); //Store first two bytes into temp

  tempInC = processTempData(regVal);
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire1.beginTransmission(0b1101000); //I2C address of the MPU
  Wire1.write(0x43); //Starting register for Gyro Readings
  Wire1.endTransmission();
  Wire1.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire1.available() < 6);
  gyroX = Wire1.read()<<8|Wire1.read(); //Store first two bytes into accelX
  gyroY = Wire1.read()<<8|Wire1.read(); //Store middle two bytes into accelY
  gyroZ = Wire1.read()<<8|Wire1.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

float processTempData(int16_t regVal) {
  return ((regVal / 326.8) + 25.0);
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.print(" Temp=");
  Serial.println(tempInC);
}

