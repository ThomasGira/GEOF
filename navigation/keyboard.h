//My first program in Curses
//Create a program, which allows you to move window using arrows
//After each movement window shows coordinates of its 4 corners in itself.

#include <ncurses.h>
#include <iostream>

namespace geoff{
namespace key{

void init_key();
int get_key();

}
}