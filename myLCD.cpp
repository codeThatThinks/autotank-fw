
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include "myLCD.h"

Adafruit_FT6206 ts = Adafruit_FT6206();

#define TFT_CS 7
#define TFT_DC 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

boolean DEBUG = false;

boolean runOnce = true;

int whichTab = 0;


#define TABFRAME_X 0
#define TABFRAME_Y 0
#define TABFRAME_W 75
#define TABFRAME_H 50
#define TABOFFSET 50

#define STATUSTAB_X TABFRAME_X
#define STATUSTAB_Y TABFRAME_Y
#define STATUSTAB_W TABFRAME_W
#define STATUSTAB_H TABFRAME_H

#define MOTORSTAB_X (TABFRAME_X + TABFRAME_W + TABOFFSET)
#define MOTORSTAB_Y TABFRAME_Y
#define MOTORSTAB_W TABFRAME_W
#define MOTORSTAB_H TABFRAME_H

#define MODESTAB_X (MOTORSTAB_X + TABFRAME_W + TABOFFSET)
#define MODESTAB_Y TABFRAME_Y
#define MODESTAB_W TABFRAME_W
#define MODESTAB_H TABFRAME_H

void myLCD::statusTabDraw()
{
  tft.fillRect(STATUSTAB_X, STATUSTAB_Y, STATUSTAB_W, STATUSTAB_H, ILI9341_GREEN);
  tft.fillRect(MOTORSTAB_X, MOTORSTAB_Y, MOTORSTAB_W, MOTORSTAB_H, DARKGREY);
  tft.fillRect(MODESTAB_X, MODESTAB_Y, MODESTAB_W, MODESTAB_H, DARKGREY);
  textTabDraw();
  whichTab = 0;
}

void myLCD::motorsTabDraw()
{
  tft.fillRect(STATUSTAB_X, STATUSTAB_Y, STATUSTAB_W, STATUSTAB_H, DARKGREY);
  tft.fillRect(MOTORSTAB_X, MOTORSTAB_Y, MOTORSTAB_W, MOTORSTAB_H, ILI9341_GREEN);
  tft.fillRect(MODESTAB_X, MODESTAB_Y, MODESTAB_W, MODESTAB_H, DARKGREY);
  textTabDraw();
  whichTab = 1;
}

void myLCD::modesTabDraw()
{
  tft.fillRect(STATUSTAB_X, STATUSTAB_Y, STATUSTAB_W, STATUSTAB_H, DARKGREY);
  tft.fillRect(MOTORSTAB_X, MOTORSTAB_Y, MOTORSTAB_W, MOTORSTAB_H, DARKGREY);
  tft.fillRect(MODESTAB_X, MODESTAB_Y, MODESTAB_W, MODESTAB_H, ILI9341_GREEN);
  textTabDraw();
  whichTab = 2;
}

void myLCD::textTabDraw()
{
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  tft.setCursor(MODESTAB_X + 3 , MODESTAB_Y + (MODESTAB_H / 2.8));
  tft.println("MODES");
  tft.setCursor(MOTORSTAB_X + 3 , MOTORSTAB_Y + (MOTORSTAB_H / 2.8));
  tft.println("MOTORS");
  tft.setCursor(STATUSTAB_X + 3 , STATUSTAB_Y + (STATUSTAB_H / 2.8));
  tft.println("STATUS");
}

void myLCD::textDraw(float vescData[])
{
  switch (whichTab) {
    case 0:

      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setTextSize(2);

      tft.setCursor(0 , 75);
      tft.println("Voltage:");
      tft.setCursor(100 , 75);
      tft.print(vescData[2]);
      tft.println("V");

      tft.setCursor(0 , 105);
      tft.println("Current:");
      tft.setCursor(100 , 105);
      tft.print(vescData[3]);
      tft.println("A");

      tft.setCursor(0 , 125);
      tft.println("MotorTempL:");
      tft.setCursor(135 , 125);
      tft.print(vescData[4]);
      tft.println("C");

      tft.setCursor(0 , 145);
      tft.println("MotorTempR:");
      tft.setCursor(135 , 145);
      tft.print(vescData[8]);
      tft.println("C");

      tft.setCursor(0 , 165);
      tft.println("FetTempL:");
      tft.setCursor(135 , 165);
      tft.print(vescData[5]);
      tft.println("C");

      tft.setCursor(0 , 185);
      tft.println("FetTempR:");
      tft.setCursor(135 , 185);
      tft.print(vescData[9]);
      tft.println("C");

      if (vescData[0]) {
        tft.setCursor(215 , 60);
        tft.println("ArmState");
        tft.fillCircle(260, 105, 25, ILI9341_RED);
      }
      else {
        tft.setCursor(215 , 60);
        tft.println("ArmState");
        tft.fillCircle(260, 105, 25, ILI9341_GREEN);
      }

      if (vescData[1]) {
        tft.setCursor(215 , 135);
        tft.println("Failsafe");
        tft.fillCircle(260, 180, 25, ILI9341_RED);
      }
      else {
        tft.setCursor(215 , 135);
        tft.println("Failsafe");
        tft.fillCircle(260, 180, 25, ILI9341_GREEN);
      }
      break;
    case 1:

      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setTextSize(2);
      
      tft.setCursor(0 , 75);
      tft.println("MotorDutyL:");
      tft.setCursor(135 , 75);
      tft.print(vescData[6]);

      tft.setCursor(0 , 105);
      tft.println("MotorDutyR:");
      tft.setCursor(135 , 105);
      tft.print(vescData[10]);

      tft.setCursor(0 , 135);
      tft.println("MotorRpmL:");
      tft.setCursor(135 , 135);
      tft.print(vescData[7]);

      tft.setCursor(0 , 165);
      tft.println("MotorRpmR:");
      tft.setCursor(135 , 165);
      tft.print(vescData[11]);
      break;
    case 2:
      tft.setTextColor(ILI9341_CYAN);
      tft.setTextSize(4);
      tft.setCursor(125 , 110);
      tft.println("WIP");
      break;

    default:
      break;
  }
}

void myLCD::lcdInit(void) {

  tft.begin();
  if (!ts.begin(40)) {
    Serial.println("Unable to start touchscreen.");
  }
  else {
    Serial.println("Touchscreen started.");
  }

  tft.fillScreen(ILI9341_BLACK);
  // origin = left,top landscape (USB left upper)
  tft.setRotation(1);
  statusTabDraw();
}

void myLCD::lcdUpdate(float vescData[]) {

  if (ts.touched() || runOnce)
  {
    runOnce = false;

    // Retrieve a point
    TS_Point p = ts.getPoint();
    // rotate coordinate system
    // flip it around to match the screen.
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);
    int y = tft.height() - p.x;
    int x = p.y;

    if ((x > STATUSTAB_X) && (x < (STATUSTAB_X + STATUSTAB_W))) {
      if ((y > STATUSTAB_Y) && (y <= (STATUSTAB_Y + STATUSTAB_H))) {
        tft.fillScreen(ILI9341_BLACK);
        statusTabDraw();
        whichTab = 0;
      }
    }

    if ((x > MOTORSTAB_X) && (x < (MOTORSTAB_X + MOTORSTAB_W))) {
      if ((y > MOTORSTAB_Y) && (y <= (MOTORSTAB_Y + MOTORSTAB_H))) {
        tft.fillScreen(ILI9341_BLACK);
        motorsTabDraw();
        whichTab = 1;
      }
    }

    if ((x > MODESTAB_X) && (x < (MODESTAB_X + MODESTAB_W))) {
      if ((y > MODESTAB_Y) && (y <= (MODESTAB_Y + MODESTAB_H))) {
        tft.fillScreen(ILI9341_BLACK);
        modesTabDraw();
        whichTab = 2;
      }
    }

  }
  textDraw(vescData);

}
