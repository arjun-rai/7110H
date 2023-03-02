#include <string>
#ifndef AUTONSELECTOR_H
#define AUTONSELECTOR_H
class Button {
   public:
    int x, y,  width, height;
    std::string text;
    std::string buttonColor, textColor;
    bool pressed;
    Button(int x, int y, int width, int height, std::string text, std::string buttonColor, std::string textColor);
    void show();
    bool isPressed();

};
int autonSelector();
#endif