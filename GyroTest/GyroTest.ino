#include<Wire.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int x,y,z;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
}
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  x += GyX/131;
  y += GyY/131;
  z += GyZ/131;

  Serial.print("Acc Clean: ");
  Serial.print("X = "); Serial.print(AcX/16.384); Serial.print("g");
  Serial.print(" | Y = "); Serial.print(AcY/16.384); Serial.print("g");
  Serial.print(" | Z = "); Serial.println(AcZ/16.384); Serial.print("g");
  
  Serial.print("Gyro Clean: ");
  Serial.print("X = "); Serial.print(GyX/131); Serial.println("°/s;"); Serial.println(x);
  Serial.print(" | Y = "); Serial.print(GyY/131); Serial.println("°/s"); Serial.println(y);
  Serial.print(" | Z = "); Serial.print(GyZ/131); Serial.println("°/s"); Serial.println(z);
  Serial.println(" ");
  delay(1000);
}
