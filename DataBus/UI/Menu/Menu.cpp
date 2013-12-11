#include "mbed.h"
#include "Menu.h"
#include <string.h>

Menu::Menu():
    _item(0), _itemCount(0)
{
}

void Menu::add(const char *name, FunctionPtr f)
{
    if (_itemCount < _ITEM_MAX) {
        _exec[_itemCount] = f;
        strncpy(_name[_itemCount], name, NAMESIZ-1);
        _itemCount++;
    }
    
    return;
}

void Menu::next()
{
    _item++;
    if (_item >= _itemCount) _item = 0;
    
    return;
}

void Menu::prev()
{
    if (_item == 0) _item = _itemCount;
    _item--;
    
   return;
}

void Menu::select()
{
    (_exec[_item])();
}

char *Menu::getItemName(int i)
{
    return _name[i];
}


char *Menu::getItemName()
{
    return _name[_item];
}

void Menu::printAll()
{
    fprintf(stdout, "Menus:\n");
    for (int i=0; i < _itemCount; i++) {
        fprintf(stdout, "%s\n", _name[i]);
    }
    fprintf(stdout, "\n");
    
    return;   
}
