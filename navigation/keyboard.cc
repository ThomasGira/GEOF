//My first program in Curses
//Create a program, which allows you to move window using arrows
//After each movement window shows coordinates of its 4 corners in itself.

#include "keyboard.h"

void geoff::key::init_key(){
    initscr();
    keypad(stdscr, TRUE);
}

int geoff::key::get_key(){
    int input = getch();
    switch(input) {
        case KEY_UP:
            return 0;
        case KEY_RIGHT:
            return 1;
        case KEY_DOWN:
            return 2;
        case KEY_LEFT:
            return 3;
    }
    return 4;
}