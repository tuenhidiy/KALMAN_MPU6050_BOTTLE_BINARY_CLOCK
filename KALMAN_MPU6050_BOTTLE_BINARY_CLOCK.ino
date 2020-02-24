//************************************************************************************************************//
#include <SPI.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include "timezone.h"

#define blank_pin       D6  // BLANK PIN - 74HC595
#define latch_pin       D4  // LATCH PIN - 74HC595
#define clock_pin       D5  // CLOCK PIN - 74HC595
#define data_pin        D7  // DATA PIN - 74HC595

#define ROW0            D8
#define ROW1            D0


//MAPPING TO PORT

#define Blank_Pin_Bit     12  // GPIO12
#define Latch_Pin_Bit     2   // GPIO2
#define Clock_Pin_Bit     14  // GPIO14
#define Data_Pin_Bit      13  // GPIO13

const char *WIFI_NETWORK_NAME = "XXXXXXXX"; // Change to your wifi network name
const char *WIFI_PASSWORD     = "XXXXXXXX";   // Change to your wifi password

const char *TIME_SERVER       = "asia.pool.ntp.org";
int myTimeZone = VST;   // change this to your time zone (see in timezone.h)

time_t now;

char H1_Number, H0_Number, M1_Number, M0_Number;

char s0, s1, m0, m1, h0, h1;
char prves0, prves1, prvem0, prvem1, prveh0, prveh1;

unsigned long samplingtime = 0;
unsigned long samplingtimem0 = 0;
unsigned long samplingtimem1 = 0;
unsigned long samplingtimeh0 = 0;
unsigned long samplingtimeh1 = 0;

//************************************************************************************************************//

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D1;
const uint8_t sda = D2;

/////////////////////////////////////////////////////////////////////////////////////
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* MPU Data */

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
uint8_t poses;
//************************************************************************************************************//

byte red[4][2];
byte green[4][2];
byte blue[4][2];

int row=0;
int BAM_Bit, BAM_Counter=0; 

struct Color
{
  unsigned char red, green, blue;

  Color(int r, int g, int b) : red(r), green(g) , blue(b){}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color tealcolor       = Color(0x00, 0x0F, 0x04);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);
const Color purplecolor     = Color(0x0F, 0x00, 0x0F);
const Color whitecolor      = Color(0x0F, 0x0F, 0x0F);
const Color blackcolor      = Color(0x00, 0x00, 0x00);

//************************************************************************************************************//

#define BAM_RESOLUTION 4  
#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473

// upright, lying down position
byte H0[4][2] = {{1, 0},{1, 1},{0, 1},{0, 0}};  // Ten digit
byte H1[4][2] = {{1, 2},{1, 3},{0, 3},{0, 2}};  // Unit digit
byte M0[4][2] = {{1, 4},{1, 5},{0, 5},{0, 4}};
byte M1[4][2] = {{1, 6},{1, 7},{0, 7},{0, 6}};

// upside down position
byte H0_INV[4][2] = {{1, 7},{1, 6},{0, 6},{0, 7}};
byte H1_INV[4][2] = {{1, 5},{1, 4},{0, 4},{0, 5}};
byte M0_INV[4][2] = {{1, 3},{1, 2},{0, 2},{0, 3}};
byte M1_INV[4][2] = {{1, 1},{1, 0},{0, 0},{0, 1}};

void LED(int X, int Y, int R, int G, int B);
void timer1_ISR(void);
void clearfast();
void fill_colour_wheel(void);
void get_colour(int16_t p, uint8_t *R, uint8_t *G);
void get_next_colour(uint8_t *R, uint8_t *G);
void increment_colour_pos(uint8_t i);

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
void ReadMPU();

void DrawDot(byte number, byte coordinates[4][2], Color frontcolor, Color backcolor);
void Effect_M0(Color frontcolor, Color backcolor);
void Effect_M1(Color frontcolor, Color backcolor);
void Effect_H0(Color frontcolor, Color backcolor);
void Effect_H1(Color frontcolor, Color backcolor);
void ReadTime();

void setup()
{
//************************************************************************************************************//
WiFi.begin(WIFI_NETWORK_NAME, WIFI_PASSWORD);

while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

configTime(3600*myTimeZone, 0, TIME_SERVER);

while (now < EPOCH_1_1_2019)
  {
    now = time(nullptr);
    delay(500);
  }


//************************************************************************************************************//

//Serial.begin(115200);
Wire.begin(sda, scl);
//#if ARDUINO >= 157
//  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
//#else
//  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
//#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  
//************************************************************************************************************//



SPI.setDataMode(SPI_MODE0);
SPI.setBitOrder(MSBFIRST);
SPI.setFrequency(4000000);
noInterrupts();
  
pinMode(latch_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(ROW0, OUTPUT);
pinMode(ROW1, OUTPUT);

SPI.begin();

timer1_isr_init();
timer1_attachInterrupt(timer1_ISR);
timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
timer1_write(80);
interrupts();
clearfast();

if (WiFi.status() == WL_CONNECTED)
  {
    for (int x=0; x<2; x++)
    {
      for (int y=0; y<8; y++)
        {
          LED(x, y, random(16), random(16), random(16));
          delay(200);
        }
      }
    for (int x=0; x<2; x++)
      {
        for (int y=0; y<8; y++)
        {
          LED(x, y, 0, 0, 0);
          delay(200);
        }
      }
   }
clearfast();
fill_colour_wheel();
}

void loop()
{     
  Effect_M0(orangecolor, blackcolor);
  Effect_M1(purplecolor, blackcolor);
  Effect_H0(greencolor, blackcolor);
  Effect_H1(redcolor, blackcolor); 

}         

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 1);
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));
    
    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  } 
}

void rowScan(byte row)
{  
  if (row == 0)
  {
    digitalWrite(ROW0,HIGH);
    delayMicroseconds(1);
  }
  else 
  {
    digitalWrite(ROW0,LOW);
    delayMicroseconds(1);
  }
      
  
  if (row == 1)
  {
    digitalWrite(ROW1,HIGH);
    delayMicroseconds(1);
  }
  else
  {
    digitalWrite(ROW1,LOW);
    delayMicroseconds(1);
  }
}

void ICACHE_RAM_ATTR timer1_ISR(void)
{
  
digitalWrite(blank_pin, HIGH);  // Set BLANK PIN high - TPIC6B595   

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      SPI.transfer(blue[0][row]);   
      SPI.transfer(green[0][row]);      
      SPI.transfer(red[0][row]);
      break;
    case 1:
      SPI.transfer(blue[1][row]);    
      SPI.transfer(green[1][row]);  
      SPI.transfer(red[1][row]);        
      break;
    case 2:     
      SPI.transfer(blue[2][row]); 
      SPI.transfer(green[2][row]); 
      SPI.transfer(red[2][row]);     
      break;
    case 3:
      SPI.transfer(blue[3][row]); 
      SPI.transfer(green[3][row]); 
      SPI.transfer(red[3][row]);    
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

digitalWrite(latch_pin, HIGH);    // Set LATCH PIN low - TPIC6B595
delayMicroseconds(3);
digitalWrite(latch_pin, LOW);     // Set LATCH PIN low - TPIC6B595
delayMicroseconds(3);
digitalWrite(blank_pin, LOW);     // Set BLANK PIN low - TPIC6B595
delayMicroseconds(3);
row++;
if(row==2)
row=0;
timer1_write(80);     //Interrupt will be called every 80 x 0.2us = 16us
pinMode(blank_pin, OUTPUT);
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 2);
    memset(green, 0, sizeof(green[0][0]) * 4 * 2);
    memset(blue, 0, sizeof(blue[0][0]) * 4 * 2);
        
}

void fillTable(byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {
      for (byte y=0; y<8; y++)
      {
        LED(x, y, R, G, B);
      }
    }
}

void fillTable_colorwheelRGB(int potentio, byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {      
      for (byte y=0; y<8; y++)
      {
        get_colour(potentio + 36*(y+2*x), &R, &G, &B);
        LED(x, y, R, G, B);      
      }
    }
}  

//************************************************************************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}


void DrawDot(byte number, byte coordinates[4][2], Color frontcolor, Color backcolor)
{ 
    
    for (int i = 0; i < 4; i++)
      {
        if (bitRead(number, i))
        {
        LED(coordinates[i][0], coordinates[i][1], frontcolor.red, frontcolor.green, frontcolor.blue);
        }
        else
        {
        LED(coordinates[i][0], coordinates[i][1], backcolor.red, backcolor.green, backcolor.blue);  
        }
      }
}



void Effect_M1(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimem1) > 155  )
  {     
    ReadTime();
    ReadMPU();
    m1 = M1_Number;
    if (poses)
        {
        DrawDot(H0_Number, H0_INV, backcolor, backcolor);
        DrawDot(M1_Number, M1, frontcolor, backcolor);      
        }
        else
        {
        DrawDot(H0_Number, H0, backcolor, backcolor);
        DrawDot(M1_Number, M1_INV, frontcolor, backcolor); 
        }
    if (m1 != prvem1)  
      {      
        if (poses)
        {
          DrawDot(M1_Number, M1, frontcolor, backcolor);      
        }
        else
        {
        DrawDot(M1_Number, M1_INV, frontcolor, backcolor); 
        }            
        prvem1 = m1;
      }
      samplingtimem1 = micros(); 
    }
}   

void Effect_M0(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimem0) > 255  )
  {
    ReadTime();
    ReadMPU();
    m0 = M0_Number;
    if (poses)
        {
        DrawDot(H1_Number, H1_INV, backcolor, backcolor);
        DrawDot(M0_Number, M0, frontcolor, backcolor);
        }
        else
        {
        DrawDot(H1_Number, H1, backcolor, backcolor);
        DrawDot(M0_Number, M0_INV, frontcolor, backcolor);
        
        }
    if (m0!=prvem0)  
      {         
        if (poses)
        {
        DrawDot(M0_Number, M0, frontcolor, backcolor);
        }
        else
        {
        DrawDot(M0_Number, M0_INV, frontcolor, backcolor);
        
        }
        prvem0 = m0;
      }
      samplingtimem0 = micros(); 
    }
} 

void Effect_H1(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimeh1) > 355  )
  {
    ReadTime();
    ReadMPU();
    h1 = H1_Number;
      if (poses)
        {
        DrawDot(M0_Number, M0_INV, backcolor, backcolor);
        DrawDot(H1_Number, H1, frontcolor, backcolor);
        }
        else
        {
        DrawDot(M0_Number, M0, backcolor, backcolor);
        DrawDot(H1_Number, H1_INV, frontcolor, backcolor);
        }
    
    if (h1 != prveh1)  
      {          
        if (poses)
        {
        DrawDot(H1_Number, H1, frontcolor, backcolor);
        }
        else
        {
        DrawDot(H1_Number, H1_INV, frontcolor, backcolor);
        }
        prveh1 = h1;
      }
      samplingtimeh1 = micros(); 
    }
}       

void Effect_H0(Color frontcolor, Color backcolor)
{ 
  if ( (unsigned long) (micros() - samplingtimeh0) > 455  )
  {    
    ReadTime();
    ReadMPU();
    h0 = H0_Number;    
    if (poses)
        {
        DrawDot(M1_Number, M1_INV, backcolor, backcolor);
        DrawDot(H0_Number, H0, frontcolor, backcolor);
        }
        else
        {
        DrawDot(M1_Number, M1, backcolor, backcolor);
        DrawDot(H0_Number, H0_INV, frontcolor, backcolor);
        }
    
    if (h0 != prveh0)  
      {   
        if (poses)
        {
        DrawDot(H0_Number, H0, frontcolor, backcolor);
        }
        else
        {
        DrawDot(H0_Number, H0_INV, frontcolor, backcolor);
        }
        prveh0 = h0;
      }
      samplingtimeh0 = micros(); 
    }
}

void ReadTime()
{
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    //int year        = timeinfo->tm_year + 1900;
    //int month       = timeinfo->tm_mon + 1;
    //int day         = timeinfo->tm_mday;
    int hour        = timeinfo->tm_hour;
    int mins        = timeinfo->tm_min;
    //int sec         = timeinfo->tm_sec;
    //int day_of_week = timeinfo->tm_wday;

    H0_Number = ((hour/10) %10) + 48;
    H1_Number = (hour%10) + 48;

    M0_Number = ((mins/10) %10) + 48;
    M1_Number = (mins%10) + 48;
}


//*******************************************************MPU*****************************************************//

void ReadMPU()
{
 /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    poses = ((((kalAngleX >-4.99) && (kalAngleX < 4.99)) && ((kalAngleY > -14.99)&& (kalAngleY < 14.99))) ? 0:1);
 /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(kalAngleX); Serial.print("\t");
  Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
#endif
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
