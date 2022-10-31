//My first program in Curses
//Create a program, which allows you to move window using arrows
//After each movement window shows coordinates of its 4 corners in itself.

#include "keyboard.h"
#define GEOFF_KEY_UP (int) 119
#define GEOFF_KEY_RIGHT (int) 100
#define GEOFF_KEY_DOWN (int) 115
#define GEOFF_KEY_LEFT (int) 97
void geoff::key::init_key(){
    initscr();
    keypad(stdscr, TRUE);
}

int geoff::key::get_key(){
    int input = getch();
    switch(input) {
        case GEOFF_KEY_UP:
            return 0;
        case GEOFF_KEY_RIGHT:
            return 1;
        case GEOFF_KEY_DOWN:
            return 2;
        case GEOFF_KEY_LEFT:
            return 3;
    }
    return 4;
}