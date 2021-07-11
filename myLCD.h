#ifndef myLCD_h
#define myLCD_h

#define LIGHTGREY 0xC618   ///< 198, 195, 198
#define DARKGREY 0x7BEF    ///< 123, 125, 123

class myLCD
{
  public:
    void lcdInit(void);
    void lcdUpdate(float vescData[]);
  private:
    void statusTabDraw();
    void motorsTabDraw();
    void modesTabDraw();
    void textTabDraw();
    void textDraw(float vescData[]);
};

#endif
