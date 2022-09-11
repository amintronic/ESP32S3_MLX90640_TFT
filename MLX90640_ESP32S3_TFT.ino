#include <Wire.h>
#include "TFT_eSPI.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

float mlx90640To[768];
paramsMLX90640 mlx90640;

TFT_eSPI lcd = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&lcd);

#define gray 0x6B6D
#define blue 0x0967
#define orange 0xC260
#define purple 0x604D
#define green 0x1AE9

float mapValue(float value, float curMin, float curMax, float desMin, float desMax);
uint16_t hsv_to_rgb16(float h, float s, float v);

void setup()
{
//  pinMode(calcStart, OUTPUT);

  Wire.setPins(43, 44);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.begin(115200); //Fast serial as possible

  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

  lcd.init();
  lcd.fillScreen(TFT_BLACK);
  lcd.setRotation(0);

  sprite.createSprite(170, 320);
  sprite.setTextDatum(3);
  sprite.setSwapBytes(true);

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");
  else
    Serial.println("System parameters loaded");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");
  else
    Serial.println("Parameter extraction OK");

  //Once params are extracted, we can release eeMLX90640 array

  //Set refresh rate
  //A rate of 0.5Hz takes 4Sec per reading because we have to read two frames to get complete picture
  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails

  //Once EEPROM has been read at 400kHz we can increase to 1MHz
  Wire.setClock(1000000); //Teensy will now run I2C at 800kHz (because of clock division)
}

void loop()
{
  long startTime = millis();
  for (byte x = 0 ; x < 2 ; x++)
  {
    uint16_t mlx90640Frame[834];
    [[maybe_unused]] int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

    [[maybe_unused]] float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    //Calculation time on a Teensy 3.5 is 71ms
  }
  long stopReadTime = millis();

  float maxHeat = 0, minHeat = 500;
  float minHue = 180.0, maxHue = 360.0;
  for (int x = 0; x < 768; x++)
  {
    maxHeat = (mlx90640To[x] > maxHeat) ? mlx90640To[x] : maxHeat;
    minHeat = (mlx90640To[x] < minHeat) ? mlx90640To[x] : minHeat;
  }
  
  
//  for (int x = 0 ; x < 768 ; x++)
//  {
//    //if(x % 8 == 0) Serial.println();
//    Serial.print(mlx90640To[x], 2);
//    Serial.print(",");
//  }
//  Serial.println("");
//  long stopPrintTime = millis();
//
//  Serial.print("Read rate: ");
//  Serial.print( 1000.0 / (stopReadTime - startTime), 2);
//  Serial.println(" Hz");
//  Serial.print("Read plus print rate: ");
//  Serial.print( 1000.0 / (stopPrintTime - startTime), 2);
//  Serial.println(" Hz");
//  Serial.println();


  sprite.fillSprite(TFT_BLACK);
  for (int i = 0; i < 32; i++)
  {
    for (int j = 0; j < 24; j++)
    {
      sprite.fillRect(5 + i*5,j*5, 5,5, 
        hsv_to_rgb16(mapValue(mlx90640To[i + (j * 32)], minHeat, maxHeat, minHue, maxHue)/360.0, 1.0, 1.0));
    }
  }

  for (int i = 0; i < 170; i++)
  {
    sprite.fillRect(i, 180, 1, 20, hsv_to_rgb16(float(180.0+(i*180.0/170.0))/360.0, 1.0, 1.0));
  }

//  sprite.fillRect(20,40, 10,30, 0x604D);
//  sprite.fillRoundRect(52,5,38,32,4,blue);
//  sprite.setFreeFont();
//  sprite.drawLine(gx+(i*17),gy,gx+(i*17),gy-gh,gray);
  sprite.setTextColor(TFT_WHITE,TFT_BLACK);
  sprite.drawString("MIN:"+String(minHeat),10,150); 
  sprite.drawString("MAX:"+String(maxHeat),10,160); 
  sprite.drawString("FREQ:"+String(1000.0 / (stopReadTime - startTime)),10,170); 
 
  sprite.pushSprite(0,0);
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  for (int i = 0; i < 5; i++)
  {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() == 0)
      return true;
  }
  return false; //Sensor did not ACK
}

float mapValue(float value, float curMin, float curMax, float desMin, float desMax)
{
    float curDistance = value - curMax;
    if (curDistance == 0)
        return desMax;
    float curRange = curMax - curMin;
    float ratio = curRange / curDistance;
    float desRange = desMax - desMin;
    return desMax + (desRange / ratio);
}

uint16_t rgb_to_rgb16(float r, float g, float b)
{
  r *= 31.0;
  g *= 63.0;
  b *= 31.0;
  return uint16_t(((uint16_t(r) & 0x001F) << 11) + ((uint16_t(g) & 0x003F) << 5) + (uint16_t(b) & 0x001F));
}

uint16_t hsv_to_rgb16(float h, float s, float v)
{
  if (s == 0.0)
    return rgb_to_rgb16(v, v, v);
  
  int i = int(h * 6.);
  float f = (h * 6.) - i;
  float p = v * (1. - s);
  float q = v * (1. - s * f);
  float t = v * (1. - s * (1. - f));

  switch(i % 6)
  {
    case 0: return rgb_to_rgb16(v, t, p);
    case 1: return rgb_to_rgb16(q, v, p);
    case 2: return rgb_to_rgb16(p, v, t);
    case 3: return rgb_to_rgb16(p, q, v);
    case 4: return rgb_to_rgb16(t, p, v);
    default: return rgb_to_rgb16(v, p, q);
  }
}
