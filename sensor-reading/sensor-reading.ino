//libraries
#include <Wire.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include <math.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_VL53L0X.h"

//variables
//I2C Multi
#define TCAADDR 0x70
/*Sensoren im Multi:
0 -> ToF0
1 -> ToF1
2 -> ToF2
3 -> ToF3
4 -> ToF4
5 -> ToF5
6 -> Col0
7 -> Col1
*/

//ToF
Adafruit_VL53L0X lox0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();

//IR Temp
#define SUR_TEMP_PIN0 A0 // Analog input pin connect to temperature sensor SUR pin
#define OBJ_TEMP_PIN0 A1 // Analog input pin connect to temperature sensor OBJ pin
#define SUR_TEMP_PIN1 A2
#define OBJ_TEMP_PIN1 A3
float temp_calibration=0;       //this parameter was used to calibrate the temperature
//float objt_calibration=0.000; //this parameter was used to calibrate the object temperature
float temperature_range=10;    //we make a map of temperature-voltage according to sensor datasheet. 10 is the temperature step when sensor and 
                               //object distance is 9CM.
float offset_vol=0.014;        //this parameter was used to set the mid level voltage,when put the sensor in normal environment after 10 min,
                               //the sensor output 0.For example,the surrounding temperature is 29℃，but the result is 27℃ via the sensor,
                               //you should set the reerence to 0.520 or more,according to your sensor to change.
                               //the unit is V
float tempValue = 0; 
float objtValue= 0;  
float current_temp=0;
float temp=0;
float temp1=0;
float temp2=0;
unsigned int temp3=0;
const float reference_vol=0.500;
unsigned char clear_num=0;//when use lcd to display
float R=0;
float voltage=0;
long res[100]={
                 318300,302903,288329,274533,261471,249100,237381,226276,215750,205768,
                 196300,187316,178788,170691,163002,155700,148766,142183,135936,130012,
                 124400,119038,113928,109059,104420,100000,95788,91775,87950,84305,
                 80830,77517,74357,71342,68466,65720,63098,60595,58202,55916,
                 53730,51645,49652,47746,45924,44180,42511,40912,39380,37910,
                 36500,35155,33866,32631,31446,30311,29222,28177,27175,26213,
                 25290,24403,23554,22738,21955,21202,20479,19783,19115,18472,
                 17260,16688,16138,15608,15098,14608,14135,13680,13242,12819,
                 12412,12020,11642,11278,10926,10587,10260,9945,9641,9347,
                 9063,8789,8525,8270,8023,7785,7555,7333,7118,6911}; 
float obj [13][12]={
/*0*/             { 0,-0.274,-0.58,-0.922,-1.301,-1.721,-2.183,-2.691,-3.247,-3.854,-4.516,-5.236}, //
/*1*/             { 0.271,0,-0.303,-0.642,-1.018,-1.434,-1.894,-2.398,-2.951,-3.556,-4.215,-4.931},  //→surrounding temperature,from -10,0,10,...100
/*2*/             { 0.567,0.3,0,-0.335,-0.708,-1.121,-1.577,-2.078,-2.628,-3.229,-3.884,-4.597},   //↓object temperature,from -10,0,10,...110
/*3*/             { 0.891,0.628,0.331,0,-0.369,-0.778,-1.23,-1.728,-2.274,-2.871,-3.523,-4.232},
/*4*/             { 1.244,0.985,0.692,0.365,0,-0.405,-0.853,-1.347,-1.889,-2.482,-3.13,-3.835},
/*5*/             { 1.628,1.372,1.084,0.761,0.401,0,-0.444,-0.933,-1.47,-2.059,-2.702,-3.403},
/*6*/             { 2.043,1.792,1.509,1.191,0.835,0.439,0,-0.484,-1.017,-1.601,-2.24,-2.936},
/*7*/             { 2.491,2.246,1.968,1.655,1.304,0.913,0.479,0,-0.528,-1.107,-1.74,-2.431},
/*8*/             { 2.975,2.735,2.462,2.155,1.809,1.424,0.996,0.522,0,-0.573,-1.201,-1.887},
/*9*/             { 3.495,3.261,2.994,2.692,2.353,1.974,1.552,1.084,0.568,0,-0.622,-1.301},
/*10*/            { 4.053,3.825,3.565,3.27,2.937,2.564,2.148,1.687,1.177,0.616,0,-0.673},
/*11*/            { 4.651,4.43,4.177,3.888,3.562,3.196,2.787,2.332,1.829,1.275,0.666,0},
/*12*/            { 5.29,5.076,4.83,4.549,4.231,3.872,3.47,3.023,2.527,1.98,1.379,0.72}
};


//Color
// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground
// set to false if using a common cathode LED
#define commonAnode true
// our RGB -> eye-recognized gamma color
byte gammatable[256];
Adafruit_TCS34725 tcs0 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//gyro
const int MPU=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

int ToF(int k) {
  VL53L0X_RangingMeasurementData_t measure;
  //Serial.print("Reading a measurement... ");
  switch(k) {
    case 0:
      lox0.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
    case 1:
      lox1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
    case 2:
      lox2.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
    case 3:
      lox3.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
    case 4:
      lox4.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
    case 5:
      lox5.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      break;
  }
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return(measure.RangeMilliMeter);
  } else {
    return(99999);
  }
    
  delay(100);
}

float Color(int k, int rgb) {
  uint16_t clear, red, green, blue;
  switch(k) {
    case 0:
      tcs0.setInterrupt(false);      // turn on LED
      delay(60);  // takes 50ms to read 
      tcs0.getRawData(&red, &green, &blue, &clear);
      tcs0.setInterrupt(true);  // turn off LED
      break;
    case 1:
      tcs1.setInterrupt(false);      // turn on LED
      delay(60);  // takes 50ms to read 
      tcs1.getRawData(&red, &green, &blue, &clear);
      tcs1.setInterrupt(true);  // turn off LED
      break;
  }
  //Serial.print("C:\t"); Serial.print(clear);
  //Serial.print("\tR:\t"); Serial.print(red);
  //Serial.print("\tG:\t"); Serial.print(green);
  //Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; 
  g *= 256; 
  b *= 256;
  //Serial.print("\t");
  //Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  //Serial.println();

  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

  analogWrite(redpin, gammatable[(int)r]);
  analogWrite(greenpin, gammatable[(int)g]);
  analogWrite(bluepin, gammatable[(int)b]);
  switch(rgb){
    case 0:
      return(r);
      break;
    case 1:
      return(g);
      break;
    case 2:
      return(b);
      break;
  }
}

float IRTemp(boolean k, boolean sur) {
  if(sur) {
    return(measureSurTemp(k));
  }
  else {
    return(measureObjectTemp(k));
  }
}

float binSearch(long x)// this function used for measure the surrounding temperature
{
  int low,mid,high;
  low=0;
  //mid=0;
  high=100;
  while (low<=high)
  {
    mid=(low+high)/2;
    if(x<res[mid])
      low= mid+1;
    else//(x>res[mid])
      high=mid-1;
  }
  return mid;
}

float arraysearch(float x,float y)//x is the surrounding temperature,y is the object temperature
{
  int i=0;
  float tem_coefficient=100;//Magnification of 100 times  
  i=(x/10)+1;//Ambient temperature      
  voltage=(float)y/tem_coefficient;//the original voltage   
  //Serial.print("sensor voltage:\t");    
  //Serial.print(voltage,5);  
  //Serial.print("V");      
  for(temp3=0;temp3<13;temp3++)   
  {     
    if((voltage>obj[temp3][i])&&(voltage<obj[temp3+1][i]))        
    {     
      return temp3;         
    }     
  }
}
float measureSurTemp(boolean k)
{  
  unsigned char i=0;
  float current_temp1=0;    
  int signal=0;   
  tempValue=0;
  switch (k) {
    case false:
      for(i=0;i<10;i++)          
      {     
        tempValue+= analogRead(SUR_TEMP_PIN0);       
        delay(10);    
      }  
      break;
    case true:
      for(i=0;i<10;i++)        
      {     
        tempValue+= analogRead(SUR_TEMP_PIN1);       
        delay(10);    
      }  
      break;
  }
   
  tempValue=tempValue/10;   
  temp = tempValue*1.1/1023;    
  R=2000000*temp/(2.50-temp);   
  signal=binSearch(R);    
  current_temp=signal-1+temp_calibration+(res[signal-1]-R)/(res[signal-1]-res[signal]);
  return current_temp;
}

float measureObjectTemp(boolean k)
{
  unsigned char i=0;  
  float sur_temp=0;  
  unsigned int array_temp=0;  
  float temp1,temp2; 
  float final_temp=0;
  objtValue=0;  
  switch (k) {
    case false:
      for(i=0;i<10;i++)
      {
        objtValue+= analogRead(OBJ_TEMP_PIN0); 
        delay(10); 
      }  
      break;
    case true:
      for(i=0;i<10;i++)
      { 
        objtValue+= analogRead(OBJ_TEMP_PIN1); 
        delay(10); 
      }
      break;      
  }   
  objtValue=objtValue/10;//Averaging processing     
  temp1=objtValue*1.1/1023;//+objt_calibration; 
  sur_temp=temp1-(reference_vol+offset_vol);              
  array_temp=arraysearch(current_temp,sur_temp*1000);        
  temp2=current_temp;        
  temp1=(temperature_range*voltage)/(obj[array_temp+1][(int)(temp2/10)+1]-obj[array_temp][(int)(temp2/10)+1]);        
  final_temp=temp2+temp1;        
  if((final_temp>100)||(final_temp<=-10))
  {
    return(99999);
  }
  else
  {
    return(final_temp); 
  }
}

void Gyro() {
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
  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX);
  Serial.print(" | Y = "); Serial.print(AcY);
  Serial.print(" | Z = "); Serial.println(AcZ); 
  
  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  Serial.println(" ");
  delay(333);
}

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
  analogReference(INTERNAL1V1);//set the refenrence voltage 1.1V,the distinguishability can up to 1mV.
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
    //Serial.println(gammatable[i]);
  }
}


void loop() {
  for(int i=0; i<8; i++) {
    tcaselect(i);
      switch(i) {
        case 0:
          if(!lox0.begin()) {
            Serial.print("ToF0 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof0 "); Serial.print(ToF(0)); Serial.print("\t");
          }
          break;
        case 1:  
          if(!lox1.begin()) {
            Serial.print("ToF1 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof1 "); Serial.print(ToF(1)); Serial.print("\t");
          }
          break;
        case 2:  
          if(!lox2.begin()) {
            Serial.print("ToF2 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof2 "); Serial.print(ToF(2)); Serial.print("\t");
          }
          break;
        case 3:  
          if(!lox3.begin()) {
            Serial.print("ToF3 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof3 "); Serial.print(ToF(3)); Serial.print("\t");
          }
          break;
        case 4:  
          if(!lox4.begin()) {
            Serial.print("ToF4 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof4 "); Serial.print(ToF(4)); Serial.print("\t");;
          }
          break;
        case 5:  
          if(!lox5.begin()) {
            Serial.print("ToF5 0 \t");
            delay(1);
          }
          else {
            Serial.print("tof5 "); Serial.print(ToF(5)); Serial.print("\t");
          }
          break;
        case 6:
            Serial.print("col0 "); 
            Serial.print((int)Color(0, 0), HEX); 
            Serial.print(" "); 
            Serial.print((int)Color(0, 1), HEX); 
            Serial.print(" "); 
            Serial.print((int)Color(0, 2), HEX); 
            Serial.print("\t");
          break;
        case 7:
            Serial.print("col1 "); 
            Serial.print((int)Color(1, 0), HEX); 
            Serial.print(" "); 
            Serial.print((int)Color(1, 1), HEX); 
            Serial.print(" "); 
            Serial.print((int)Color(1, 2), HEX); 
            Serial.print("\t");
          break;
      }
    }
  Serial.print("ObT0 "); Serial.print(IRTemp(false, false)); Serial.print("\t");
  Serial.print("SuT0 "); Serial.print(IRTemp(false, true)); Serial.print("\t");
  Serial.print("ObT1 "); Serial.print(IRTemp(true, false)); Serial.print("\t");
  Serial.print("SuT1 "); Serial.print(IRTemp(true, true)); Serial.println("\t");
  //Gyro();
}
