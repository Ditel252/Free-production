#include <Arduino.h>
#include <Wire.h>

class BNO055{
private:
  struct addressInRegister{
    uint8_t chipID; 
    uint8_t operatingMode;

    uint8_t acclX_L;
    uint8_t acclX_H;
    uint8_t acclY_L;
    uint8_t acclY_H;
    uint8_t acclZ_L;
    uint8_t acclZ_H;

    uint8_t gyroX_L;
    uint8_t gyroX_H;
    uint8_t gyroY_L;
    uint8_t gyroY_H;
    uint8_t gyroZ_L;
    uint8_t gyroZ_H;
    
    uint8_t magX_L;
    uint8_t magX_H;
    uint8_t magY_L;
    uint8_t magY_H;
    uint8_t magZ_L;
    uint8_t magZ_H;
  };

  struct{
    uint8_t module;
    addressInRegister reg;
  }adderss;

  struct{
    uint8_t chipID_OK;
    uint8_t operatingMode_ON;
  }reg;

  struct accelerationVector{
    double x;
    double y;
    double z;
  };

  struct gyroVector{
    double x;
    double y;
    double z;
  };

  struct magVector{
    double x;
    double y;
    double z;
  };

  bool debugShowing;

  const unsigned int sizeOfSendDataTo9DoF = 100;
  const unsigned int sizeOfReceiveDataFrom9DoF = 100;

  uint8_t *sendDataTo9DoF;
  uint8_t *receiveDataFrome9DoF;

  void debugDisplay(bool _result, char *_message);

public:
  struct measurementData{
    accelerationVector accl;
    gyroVector gyro;
    magVector mag;
  };

private:
    void readAcceleration(BNO055::measurementData *_measurementData);
    void readGyro(BNO055::measurementData *_measurementData);
    void readGeomagnetism(BNO055::measurementData *_measurementData);

public:

  BNO055(uint8_t _address, bool isShowDebug = false);

  bool begin();
  bool measuringStart();
  void read(BNO055::measurementData *_mesurementData, bool _isReadAcceleration, bool _isReadGyro, bool _isReadGeomagnetism);
};

BNO055::BNO055(uint8_t _address, bool isShowDebug = false){
    BNO055::adderss.module = _address;
    BNO055::debugShowing = isShowDebug;

    BNO055::adderss.reg.chipID = 0x00;
    BNO055::adderss.reg.operatingMode = 0x3D;

    BNO055::reg.chipID_OK = 0xA0;
    BNO055::reg.operatingMode_ON = 0x07;

    BNO055::adderss.reg.acclX_L = 0x08;
    BNO055::adderss.reg.acclX_H = 0x09;
    BNO055::adderss.reg.acclY_L = 0x0A;
    BNO055::adderss.reg.acclY_H = 0x0B;
    BNO055::adderss.reg.acclZ_L = 0x0C;
    BNO055::adderss.reg.acclZ_H = 0x0D;

    BNO055::adderss.reg.gyroX_L = 0x14;
    BNO055::adderss.reg.gyroX_H = 0x15;
    BNO055::adderss.reg.gyroY_L = 0x16;
    BNO055::adderss.reg.gyroY_H = 0x17;
    BNO055::adderss.reg.gyroZ_L = 0x18;
    BNO055::adderss.reg.gyroZ_H = 0x19;

    BNO055::adderss.reg.magX_L = 0x0E;
    BNO055::adderss.reg.magX_H = 0x0F;
    BNO055::adderss.reg.magY_L = 0x10;
    BNO055::adderss.reg.magY_H = 0x11;
    BNO055::adderss.reg.magZ_L = 0x12;
    BNO055::adderss.reg.magZ_H = 0x13;

    BNO055::sendDataTo9DoF = (uint8_t *)malloc(BNO055::sizeOfSendDataTo9DoF);
    BNO055::receiveDataFrome9DoF = (uint8_t *)malloc(BNO055::sizeOfReceiveDataFrom9DoF);
}

void BNO055::readAcceleration(BNO055::measurementData *_measurmentData){
    *(BNO055::sendDataTo9DoF + 0) = BNO055::adderss.reg.acclX_L;
    *(BNO055::sendDataTo9DoF + 1) = BNO055::adderss.reg.acclX_H;
    *(BNO055::sendDataTo9DoF + 2) = BNO055::adderss.reg.acclY_L;
    *(BNO055::sendDataTo9DoF + 3) = BNO055::adderss.reg.acclY_H;
    *(BNO055::sendDataTo9DoF + 4) = BNO055::adderss.reg.acclZ_L;
    *(BNO055::sendDataTo9DoF + 5) = BNO055::adderss.reg.acclZ_H;

    Wire.beginTransmission(BNO055::adderss.module);
    Wire.write(BNO055::sendDataTo9DoF, 6);
    Wire.endTransmission();
    Wire.requestFrom(BNO055::adderss.module, 6);

    for (int i = 0; i < 6; i++)
        *(BNO055::receiveDataFrome9DoF + i) = Wire.read();
        
    (_measurmentData->accl).x = (double)((*(BNO055::receiveDataFrome9DoF + 1) << 8) | (*(BNO055::receiveDataFrome9DoF + 0) << 0)) / 100;
    (_measurmentData->accl).y = (double)((*(BNO055::receiveDataFrome9DoF + 3) << 8) | (*(BNO055::receiveDataFrome9DoF + 2) << 0)) / 100;
    (_measurmentData->accl).z = (double)((*(BNO055::receiveDataFrome9DoF + 5) << 8) | (*(BNO055::receiveDataFrome9DoF + 4) << 0)) / 100;
}

void BNO055::readGyro(BNO055::measurementData *_measurmentData){
    *(BNO055::sendDataTo9DoF + 0) = BNO055::adderss.reg.gyroX_L;
    *(BNO055::sendDataTo9DoF + 1) = BNO055::adderss.reg.gyroX_H;
    *(BNO055::sendDataTo9DoF + 2) = BNO055::adderss.reg.gyroY_L;
    *(BNO055::sendDataTo9DoF + 3) = BNO055::adderss.reg.gyroY_H;
    *(BNO055::sendDataTo9DoF + 4) = BNO055::adderss.reg.gyroZ_L;
    *(BNO055::sendDataTo9DoF + 5) = BNO055::adderss.reg.gyroZ_H;

    Wire.beginTransmission(BNO055::adderss.module);
    Wire.write(BNO055::sendDataTo9DoF, 6);
    Wire.endTransmission();
    Wire.requestFrom(BNO055::adderss.module, 6);

    for (int i = 0; i < 6; i++)
        *(BNO055::receiveDataFrome9DoF + i) = Wire.read();
        
    (_measurmentData->gyro).x = (double)((*(BNO055::receiveDataFrome9DoF + 1) << 8) | (*(BNO055::receiveDataFrome9DoF + 0) << 0)) / 16;
    (_measurmentData->gyro).y = (double)((*(BNO055::receiveDataFrome9DoF + 3) << 8) | (*(BNO055::receiveDataFrome9DoF + 2) << 0)) / 16;
    (_measurmentData->gyro).z = (double)((*(BNO055::receiveDataFrome9DoF + 5) << 8) | (*(BNO055::receiveDataFrome9DoF + 4) << 0)) / 16;
}

void BNO055::readGeomagnetism(BNO055::measurementData *_measurmentData){
    *(BNO055::sendDataTo9DoF + 0) = BNO055::adderss.reg.magX_L;
    *(BNO055::sendDataTo9DoF + 1) = BNO055::adderss.reg.magX_H;
    *(BNO055::sendDataTo9DoF + 2) = BNO055::adderss.reg.magY_L;
    *(BNO055::sendDataTo9DoF + 3) = BNO055::adderss.reg.magY_H;
    *(BNO055::sendDataTo9DoF + 4) = BNO055::adderss.reg.magZ_L;
    *(BNO055::sendDataTo9DoF + 5) = BNO055::adderss.reg.magZ_H;

    Wire.beginTransmission(BNO055::adderss.module);
    Wire.write(BNO055::sendDataTo9DoF, 6);
    Wire.endTransmission();
    Wire.requestFrom(BNO055::adderss.module, 6);

    for (int i = 0; i < 6; i++)
        *(BNO055::receiveDataFrome9DoF + i) = Wire.read();
        
    (_measurmentData->mag).x = (double)((*(BNO055::receiveDataFrome9DoF + 1) << 8) | (*(BNO055::receiveDataFrome9DoF + 0) << 0)) / 16;
    (_measurmentData->mag).y = (double)((*(BNO055::receiveDataFrome9DoF + 3) << 8) | (*(BNO055::receiveDataFrome9DoF + 2) << 0)) / 16;
    (_measurmentData->mag).z = (double)((*(BNO055::receiveDataFrome9DoF + 5) << 8) | (*(BNO055::receiveDataFrome9DoF + 4) << 0)) / 16;
}

bool BNO055::begin(){
    if(BNO055::debugShowing){
        Serial.begin(9600);

        while(true){
            static uint8_t count = 0;

            if(Serial.available() && (Serial.read() == 13))
                break;

            if(count > 100){
                Serial.println("Please Press Enter Key");
                count = 0;
            }else{ 
                count++;
            }

            delay(10);
        }
    }

    Wire.begin();
    Wire.beginTransmission(BNO055::adderss.module);
    uint8_t startResult =  Wire.endTransmission();

    if(startResult == 0){
        if(BNO055::debugShowing)    debugDisplay(true, "Start Communication with BNO055");
    }else{
        if(BNO055::debugShowing)    debugDisplay(false, "Start Communication with BNO055");
        return false;
    }

    delay(800);

    Wire.beginTransmission(BNO055::adderss.module);
    Wire.write(BNO055::adderss.reg.chipID);
    Wire.endTransmission();
    Wire.requestFrom(BNO055::adderss.module, 1);
    
    uint8_t checkResult = Wire.read();

    if(checkResult == BNO055::reg.chipID_OK){
        if(BNO055::debugShowing)    debugDisplay(true, "Check Recive Data From 9DoF");
    }else{
        if(BNO055::debugShowing)    debugDisplay(false, "Check Recive Data From 9DoF");
        return false;
    }

    return true;
}

void BNO055::debugDisplay(bool _result, char *_message){
    if(_result){
        Serial.print("[  OK  ] ");
        Serial.println(_message);
    }else{
        Serial.print("[FAILED] ");
        Serial.println(_message);
    }
}

bool BNO055::measuringStart(){
    *(BNO055::sendDataTo9DoF + 0) = BNO055::adderss.reg.operatingMode;
    *(BNO055::sendDataTo9DoF + 1) = BNO055::reg.operatingMode_ON;

    Wire.beginTransmission(BNO055::adderss.module);
    Wire.write(BNO055::sendDataTo9DoF, 2);
    uint8_t startResult = Wire.endTransmission();

    if(startResult == 0){
        if(BNO055::debugShowing)    debugDisplay(true, "Start Mesuring");
    }else{
        if(BNO055::debugShowing)    debugDisplay(false, "Start Mesuring");
        return false;
    }

    return true;
}

void BNO055::read(BNO055::measurementData *_mesurementData, bool _isReadAcceleration, bool _isReadGyro, bool _isReadGeomagnetism){
    if(_isReadAcceleration) BNO055::readAcceleration(_mesurementData);
    if(_isReadGyro)         BNO055::readGyro(_mesurementData);
    if(_isReadGeomagnetism) BNO055::readGeomagnetism(_mesurementData);
}