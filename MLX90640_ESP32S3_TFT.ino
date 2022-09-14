#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"


#define _INTERPOLATE
#define _DISPLAY

#ifdef _DISPLAY
#include "TFT_eSPI.h"
#endif

#define _STATUS_REPORT 10 // seconds

#define _SCALE      2
#define _INTRP      3
#define inputX      32
#define inputY      24
#define outputX     ((inputX-1)*_INTRP+1)
#define outputY     ((inputY-1)*_INTRP+1)
#define TA_SHIFT    8 /* Default shift for MLX90640 in open air */

const uint8_t MLX90640_address = 0x33; /* Default 7-bit unshifted address of the MLX90640 */

float mlx90640To[768];
float mlx90640INTP[outputX*outputY] = {0};
paramsMLX90640 mlx90640;

uint32_t cycleStartTime = millis();
uint32_t lastStatusReportTime = millis();
uint32_t CalcTime = 0, DispTime = 0, ReadTime = 0, IntrpTime = 0, cycleTime = 0;

#ifdef _DISPLAY
TFT_eSPI lcd = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&lcd);

float mapValue(float value, float curMin, float curMax, float desMin, float desMax);
uint16_t hsv_to_rgb16(float h, float s, float v);
uint16_t hsv_lite_to_rgb16(float h);
#endif

void setup()
{
  Wire.setPins(43, 44);  // SDA, SCL
//  Wire.setSDA(16);
//  Wire.setSCL(17);
  
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz

  Serial.begin(115200);  // Fast serial as possible

  while (!Serial);       // Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");

#ifdef _DISPLAY
  lcd.init();
  lcd.fillScreen(TFT_BLACK);
  lcd.setRotation(1);    // 0=Portrait, 1=landscape

  sprite.createSprite(320, 170);
  sprite.setTextDatum(3);
  sprite.setSwapBytes(true);
#endif

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }

  // Get device parameters - We only have to do this once
  uint8_t status;
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

  // Once params are extracted, we can release eeMLX90640 array

  // Set refresh rate
  // A rate of 0.5Hz takes 4Sec per reading because we have to read two frames to get complete picture
  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails

  // Once EEPROM has been read at 400kHz we can increase to 1MHz
  Wire.setClock(1000000);  // Check device clock division
}

void loop()
{
  uint32_t startReadTime = micros();
  for (uint8_t x = 0 ; x < 2 ; x++)
  {
    uint16_t mlx90640Frame[834];
    [[maybe_unused]] int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

//    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT;  // Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    uint32_t startCalcTime = micros();
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    CalcTime = micros() - startCalcTime;
    // Calculation time on a Teensy3.5 is 71 ms!
    // Calculation time on a ESP32S3   is 20080 us, intrp(3) 4033  us
    // Calculation time on a Teensy4.1 is 827   us, intrp(3) 690   us
    // Calculation time on a RPi-Pico  is 17780 us, intrp(3) 27820 us
  }

  float maxHeat = -100.0, minHeat = 500.0;
  float maxHeatReal = 0.0, minHeatReal = 0.0;
  [[maybe_unused]] float minHue = 180.0, maxHue = 360.0;
  float centerPointTemp = float(mlx90640To[15 + (11 * 32)] +
                                mlx90640To[16 + (11 * 32)] +
                                mlx90640To[16 + (12 * 32)] +
                                mlx90640To[15 + (12 * 32)]) / 4.0;
  for (uint16_t x = 0; x < 768; x++) {
    maxHeat = (mlx90640To[x] > maxHeat) ? mlx90640To[x] : maxHeat;
    minHeat = (mlx90640To[x] < minHeat) ? mlx90640To[x] : minHeat;
  }
  maxHeatReal = maxHeat;
  minHeatReal = minHeat;
  if ((maxHeat - minHeat) < 15.0)
    maxHeat = minHeat + 15.0;
  ReadTime = micros() - startReadTime;

  uint32_t startIntrpTime = micros();
#ifdef _INTERPOLATE
//  for (uint8_t i = 0; i < 32; i++) {
//    for (uint8_t j = 0; j < 24; j++) {
//      mlx90640To[i + (j * 32)] = 
//        float((int(mapValue(mlx90640To[i + (j * 32)], minHeat, maxHeat, minHue, maxHue)) + 50) % 360) / 360.0;
//    }
//  }

  for (int j = 0; j < outputY; j++) {
    for (int i = 0; i < outputX; i++) {
      if ((i % _INTRP == 0) && (j % _INTRP == 0)) {
        // printf("i %d, j %d => %0.1f\n", i, j, input[i/_INTRP + j/_INTRP*inputX]);
        mlx90640INTP[i + j*outputX] = mlx90640To[i/_INTRP + j/_INTRP*inputX];

        if (j > 0)
        {
          float c00 = mlx90640INTP[i + (j-_INTRP)*outputX];
          float c11 = mlx90640INTP[i + j*outputX];
          for (int c = 1; c < _INTRP; c++)
          {
            float ty = (j-c) % _INTRP;

          //   printf("-- %d, %0.1f %0.1f %0.1f , %0.1f %0.1f\n",
          // i + (j-c)*outputX, c00, c11, ty, ty / float(_INTRP) * c11, float(_INTRP - ty) / float(_INTRP) * c00);

            mlx90640INTP[i + (j-c)*outputX] = (ty / float(_INTRP) * c11) + (float(_INTRP - ty) / float(_INTRP) * c00);
          }
        }
      }
      else if (j % _INTRP == 0)
      {
        int xCells = i / _INTRP + j / _INTRP * inputX;
        float c00 = mlx90640To[xCells];
        float c11 = mlx90640To[xCells + 1];
        float tx = i % _INTRP;

        // printf("-- %d %0.1f %0.1f %0.1f , %0.1f %0.1f\n",
        //   i + j*outputX, c00, c11, tx, tx / float(_INTRP) * c11, float(_INTRP - tx) / float(_INTRP) * c00);

        mlx90640INTP[i + j*outputX] = (tx / float(_INTRP) * c11) + (float(_INTRP - tx) / float(_INTRP) * c00);

        if (j > 0)
        {
          c00 = mlx90640INTP[i + (j-_INTRP)*outputX];
          c11 = mlx90640INTP[i + j*outputX];
          for (int c = 1; c < _INTRP; c++)
          {
            float ty = (j-c) % _INTRP;
  
          //   printf("-- %d, %0.1f %0.1f %0.1f , %0.1f %0.1f\n",
          // i + (j-c)*outputX, c00, c11, ty, ty / float(_INTRP) * c11, float(_INTRP - ty) / float(_INTRP) * c00);
  
            mlx90640INTP[i + (j-c)*outputX] = (ty / float(_INTRP) * c11) + (float(_INTRP - ty) / float(_INTRP) * c00);
          }
        }
      }
    }
  }
#endif // _INTERPOLATE && _DISPLAY
  IntrpTime = micros() - startIntrpTime;
  
  uint32_t startDispTime = micros();
#ifdef _DISPLAY
  sprite.fillSprite(TFT_BLACK);
  
  sprite.drawCircle(112, 84, 5, rgb_to_rgb16(1., 1., 1.));
//  sprite.fillRect(20,40, 10,30, 0x604D);
//  sprite.fillRoundRect(52,5,38,32,4,blue);
//  sprite.setFreeFont();
//  sprite.drawLine(gx+(i*17),gy,gx+(i*17),gy-gh,gray);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString(String(minHeatReal), 230, 140); 
  sprite.drawString(String(maxHeatReal), 290, 140);
  sprite.drawString("FREQ:" + String(1000.0 / (cycleTime)), 230, 10);
  sprite.drawString("READ:" + String(ReadTime / 1000), 230, 20);
  sprite.drawString("CALC:" + String(CalcTime / 1000), 230, 30);
  sprite.drawString("INTR:" + String(IntrpTime / 1000), 230, 40);
  sprite.drawString("DISP:" + String(DispTime / 1000), 230, 50);

  for (uint8_t i = 0; i < 90; i++)
  {
    float mapped = (180 + (i*2) + 50) % 360;
    sprite.fillRect(230 + i, 150, 1, 20, hsv_to_rgb16(mapped / 360.0, 1.0, 1.0));
  }

#ifdef _INTERPOLATE
  for (uint8_t j = 0; j < outputY; j++)
  {
    for (uint8_t i = 0; i < outputX; i++)
    {
      float mapped = float((int(mapValue(mlx90640INTP[i + j * outputX], minHeat, maxHeat, minHue, maxHue)) + 50) % 360) / 360.0;
//      sprite.drawPixel(i,j, hsv_lite_to_rgb16(mapped));
      sprite.fillRect(18+(outputX-i)*_SCALE, 15+j*_SCALE, _SCALE,_SCALE, hsv_lite_to_rgb16(mapped));
    }
  }
#else
  for (uint8_t j = 0; j < 24; j++)
  {
    for (uint8_t i = 0; i < 32; i++)
    {
      float mapped = float((int(mapValue(mlx90640To[i + j * 32], minHeat, maxHeat, minHue, maxHue)) + 50) % 360) / 360.0;
      sprite.fillRect(i*7, j*7, 7,7, hsv_lite_to_rgb16(mapped));
    }
  }
#endif // _INTERPOLATE
  
  sprite.drawString(String(centerPointTemp), 115, 70, 4); //  + "°C"
  sprite.drawCircle(112, 84, 3, rgb_to_rgb16(1., 1., 1.));

  sprite.pushSprite(0, 0);
#endif // _DISPLAY
  DispTime = micros() - startDispTime;

  cycleTime = millis() - cycleStartTime;
  cycleStartTime = millis();
//  if (millis() - lastStatusReportTime > (_STATUS_REPORT * 1000))
//  {
//    lastStatusReportTime = millis();
//    Serial.println("Min:  " + String(minHeatReal) + "°C, T: " + String(centerPointTemp) + "°C, Max: " + String(maxHeatReal) + "°C");
//    Serial.println("FREQ: " + String(1000.0 / (cycleTime)) + " Hz");
//    Serial.println("READ: " + String(ReadTime) + " us");
//    Serial.println("CALC: " + String(CalcTime) + " us");
//    Serial.println("INTR: " + String(IntrpTime) + " us");
//    Serial.println("DISP: " + String(DispTime) + " us");
//    Serial.println();
//  }
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

#ifdef _DISPLAY
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

uint16_t hsv_lite_to_rgb16(float h)
{ 
  int i = int(h * 6.);
  float f = (h * 6.) - i;
  float q = 1. - f;

  switch(i % 6)
  {
    case 0:  return rgb_to_rgb16(1., f, 0.);
    case 1:  return rgb_to_rgb16(q, 1., 0.);
    case 2:  return rgb_to_rgb16(0., 1., f);
    case 3:  return rgb_to_rgb16(0, q, 1.);
    case 4:  return rgb_to_rgb16(f, 0., 1.);
    default: return rgb_to_rgb16(1., 0., q);
  }
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
#endif // _DISPLAY
