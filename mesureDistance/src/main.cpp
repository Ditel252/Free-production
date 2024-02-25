#include <Arduino.h>
#include "BNO055_Controller.hpp"
#include <math.h>

#define PIN_DATA 5
#define PIN_LATCH 4
#define PIN_CLK 3

#define GYRO_POSITIVE_NEGATIVE_X 1
#define GYRO_POSITIVE_NEGATIVE_Y 1

const double ledsInterval = 22.857;

BNO055 gyro(0x28);
BNO055::measurementData readData;


struct INPUT_DATA{
  double initialAcclX;
  double initialAcclY;
  double initialAcclZ;

  double initialGyroX;
  double initialGyroY;
  double initialGyroZ;
};

struct DERIVATION_DATA{
  double veloX;
  double veloY;

  double disX;
  double disY;

  uint8_t veloModifyingCountX;
  uint8_t veloModifyingCountY;
  uint8_t veloModifyingCountZ;
};

INPUT_DATA inputData;
DERIVATION_DATA derivationData;

unsigned long int measurementTime = 0;
unsigned long int nowMeasurementTime = 0, lastMeasurementTime = 0;

ISR(TIMER1_COMPA_vect)
{
  measurementTime++;
}


void gyroSetup(){
  inputData.initialAcclX = 0;
  inputData.initialAcclY = 0;

  derivationData.veloModifyingCountX = 0;
  derivationData.veloModifyingCountY = 0;

  
  gyro.begin();
  gyro.measuringStart();


  Serial.println("Start Read Initial Condition");

  for(int i = 0; i < 1000; i++){
    gyro.read(&readData, true, false, false);
    
    inputData.initialAcclX += readData.accl.x;
    inputData.initialAcclY += readData.accl.y;
  }

  inputData.initialAcclX /= 1000;
  inputData.initialAcclY /= 1000;

  Serial.println("Finish Read Initial Condition");
  Serial.print("accX : ");
  Serial.print(inputData.initialAcclX);
  Serial.print(" accY : ");
  Serial.print(inputData.initialAcclY);


  derivationData.veloX = 0;
  derivationData.disX = 0;

  derivationData.veloY = 0;
  derivationData.disY = 0;
}

void gyroRead(){
  gyro.read(&readData, true, false, false);

  nowMeasurementTime = measurementTime;

  if((abs(readData.accl.x - inputData.initialAcclX) - abs((int)(readData.accl.x - inputData.initialAcclX))) > 0.1){
    derivationData.veloX += ((readData.accl.x - inputData.initialAcclX) * (nowMeasurementTime - lastMeasurementTime)) / 1000;
    derivationData.disX += derivationData.veloX * (nowMeasurementTime - lastMeasurementTime);
  }else{
    if(derivationData.veloModifyingCountX > 100){
      derivationData.veloModifyingCountX = 0;
      derivationData.veloX = 0;
    }else{
      derivationData.veloModifyingCountX++;
    }
  }
  
  if((abs(readData.accl.y - inputData.initialAcclY) - abs((int)(readData.accl.y - inputData.initialAcclY))) > 0.1){
    derivationData.veloY += ((readData.accl.y - inputData.initialAcclY) * (nowMeasurementTime - lastMeasurementTime)) / 1000;
    derivationData.disY += derivationData.veloY * (nowMeasurementTime - lastMeasurementTime);
  }else{
    if(derivationData.veloModifyingCountY > 100){
      derivationData.veloModifyingCountY = 0;
      derivationData.veloY = 0;
    }else{
      derivationData.veloModifyingCountY++;
    }
  }

  lastMeasurementTime = nowMeasurementTime;
}


void timerSetup(){
  TCCR1A = 0;
  TCCR1B = 0;                                         // いったん初期化しないと正しく動作しない可能性がある
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTCmode, prescaler to 1024
  OCR1A = 15.625;                                         // 15625 カウント
  TIMSK1 |= (1 << OCIE1A);                            // Compare A Match
}


void shiftSetup(){
  DDRB = (1 << PIN_DATA) | (1 << PIN_LATCH) | (1 << PIN_CLK);
}

void shiftWrite(uint8_t sendData){
  PORTB &= ~((1 << PIN_DATA) | (1 << PIN_LATCH) | (1 << PIN_CLK));
  
  for(int i = 0; i < 8; i++){
    PORTB &= ~(1 << PIN_DATA);
    PORTB |= ((!!(sendData & (1 << i))) << PIN_DATA);

    PORTB |= (1 << PIN_CLK);
    PORTB &= ~(1 << PIN_CLK);
  }

  PORTB |= (1 << PIN_LATCH);
  PORTB &= ~(1 << PIN_DATA);
}


int Position[2] = {0};
uint8_t leds[8] = {B00000000};

void BitTranslate(int Position[],uint8_t bitData[])
{
  uint8_t ForShift = B00000001;
  bitData[Position[0]] |= (ForShift << Position[1]);
}

void ToShiftRegister(uint8_t bitData[])
{
  for (int i = 0; i < 8; i++)
  {
    shiftWrite(bitData[i]);
  }
}

void PositionToLeds(){
  int lastPosition[2];

  lastPosition[0] = Position[0];
  lastPosition[1] = Position[1];

  double gyroX = derivationData.disX * GYRO_POSITIVE_NEGATIVE_X;
  double gyroY = derivationData.disX * GYRO_POSITIVE_NEGATIVE_Y;

  for(int i = 0; i < 7; i++){
    if((gyroX > ((ledsInterval * i) - (ledsInterval / 2))) && (gyroX < ((ledsInterval * i) + (ledsInterval / 2)))){
      Position[1] = 7 - i;
    }
  }

  for(int i = 0; i < 7; i++){
    if((gyroY > ((ledsInterval * i) - (ledsInterval / 2))) && (gyroY < ((ledsInterval * i) + (ledsInterval / 2)))){
      Position[0] = 7 - i;
    }
  }

  if((lastPosition[0] != Position[0]) || (lastPosition[1] != Position[1])){
    BitTranslate(Position, leds);

    ToShiftRegister(leds);
  }
}

void setup(){
  Serial.begin(9600);

  Serial.println("Program Start");

  gyroSetup();
  timerSetup();
  shiftSetup();

  lastMeasurementTime = measurementTime;
}

int count = 0; 

void loop(){
  gyroRead();

  PositionToLeds();

  if(count > 10){
    count = 0;

    Serial.print(">disX:");
    Serial.println(derivationData.disX * GYRO_POSITIVE_NEGATIVE_X);

    Serial.print(">disY:");
    Serial.println(derivationData.disY * GYRO_POSITIVE_NEGATIVE_Y);

  }else{
    count++;
  }
}

