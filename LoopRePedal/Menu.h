#ifndef Menu_h
#define Menu_h

#include <Arduino.h>
#include <U8x8lib.h>


class Menu {
  
  private:
    int arraySize=0;
    int index=0;
    char** items=0;


  public:
   
    Menu(char** _items, int _arraySize); 
    void draw(U8X8_SSD1306_128X64_NONAME_HW_I2C _screen);
    void updateMenu(U8X8_SSD1306_128X64_NONAME_HW_I2C _screen, int encVal);
    int getIndex();
    void setIndex(int _index);
    void action();

    int getSize();
    void setSize(int s);

    void (*actions[20])(void);
  
};
#endif
