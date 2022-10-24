#line 1 "c:\\Users\\Florian\\Documents\\Arduino\\Arduino\\Remotev2\\Menu.cpp"
#include "Menu.h"
#include <U8x8lib.h>

Menu::Menu(char** _items,int _arraySize){
  
  arraySize = _arraySize;
  items = _items;
  index = 0;
}

void Menu::draw(U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI screen) {
  screen.clear();
  for(int i= 0;i<min(4, arraySize);i++) {

    int sub = max(0, index-2);
    if(index == arraySize-1) {
      sub = max(0, index-3);
    }
    
    screen.drawString(2, i*2,items[i+sub]);

    if(index == i+sub) {
      screen.drawString(0,i*2, "*");
    } else {
      screen.drawString(0,i*2, " ");
    }
  }
}

void Menu::updateMenu(U8X8_SSD1309_128X64_NONAME0_4W_HW_SPI screen,int encVal) {
  
  
  if(encVal<0) {
    encVal = arraySize-1;
  }
  int menuPosNew = (encVal%arraySize);

  if(menuPosNew != index) {
    index = menuPosNew;
    draw(screen);
  }
}

int Menu::getIndex() {
  return index;
}

void Menu::setIndex(int _index) {
  index = _index;
}

int Menu::getSize() {
  return arraySize;
}

void Menu::setSize(int s) {
  arraySize = s;
}

void Menu::action() {
  actions[index]();
}
