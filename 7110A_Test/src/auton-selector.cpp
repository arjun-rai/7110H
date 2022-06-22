#include "vex.h"
class Button
{
  public:
    int x, y,  width, height;
    std::string text;
    std::string buttonColor, textColor;
    bool pressed;
    Button(int x, int y, int width, int height, std::string text, std::string buttonColor, std::string textColor)
    {
      this->x = x;
      this->y = y;
      this->width = width;
      this->height = height;
      this->text = text;
      this->buttonColor = buttonColor;
      this->textColor = textColor;
      this->pressed=false;
    }
    
  //displays button
  void show()
  {
    //Brain.Screen.clearScreen(vex::white);
    if (!pressed)
    {
      Brain.Screen.setFillColor(buttonColor.c_str());
      Brain.Screen.setPenColor(buttonColor.c_str());
    }
    else 
    {
      Brain.Screen.setFillColor("#2EFF00");
      Brain.Screen.setPenColor("#2EFF00");
    }
    Brain.Screen.drawRectangle(x, y, width, height);
    Brain.Screen.setPenColor(textColor.c_str());
    Brain.Screen.printAt(x+(width/2.0)- Brain.Screen.getStringWidth(text.c_str())/2.0, y+Brain.Screen.getStringHeight(text.c_str())/2.0 +height/2.0, false, text.c_str());
  }
  //checks if the button is pressed;
  bool isPressed()
  {
    if (Brain.Screen.pressing() && Brain.Screen.xPosition()>=x && Brain.Screen.xPosition()<=x+width 
    && Brain.Screen.yPosition() >=y && Brain.Screen.yPosition()<=y+height)
    {
      
        return true;
    }
    
    return false;
  }

  
};
int autonSelector()
{
  Button autonButtons[] = {
  Button(10, 10, 150, 50, "Auton Red 1", "#FF0000", "#FFFFFF"),
  Button(170, 10, 150, 50, "Auton Red 2", "#FF0000", "#FFFFFF"),
  Button(330, 10, 150, 50, "Auton Red 3", "#FF0000", "#FFFFFF"),
  Button(10, 70, 150, 50, "Auton Blue 1", "#0000FF", "#FFFFFF"),
  Button(170, 70, 150, 50, "Auton Blue 2", "#0000FF", "#FFFFFF"),
  Button(330, 70, 150, 50, "Auton Blue 3", "#0000FF", "#FFFFFF"),
  Button(10, 130, 480, 50, "Submit", "#2EFF00", "#FFFFFF")
  };
  bool chosenOn[] = {false, false, false, false, false, false, false};
  bool chosen[] = {false, false, false, false, false, false, false};
  bool submitted = false;
  int index = -1;
  int len = sizeof(autonButtons)/sizeof(autonButtons[0]);
  while (!submitted)
  {
    for (int i =0;i<len; i++)
    {
      Button b = autonButtons[i];
      if (autonButtons[i].isPressed() && (index==i || index==-1) && i!=len-1)
      {
        if (!chosenOn[i])
        {
          chosen[i] = !chosen[i];
          chosenOn[i]=true;
          if (chosen[i])
          {
            autonButtons[len-1] = Button(10, 130, 480, 50, "Submit " + b.text, "#2EFF00", "#FFFFFF");
            autonButtons[i].pressed=true;
            index =i;
          }
          else
          {
            autonButtons[len-1] = Button(10, 130, 480, 50,"Submit", "#2EFF00", "#FFFFFF");
            autonButtons[i].pressed=false;
            index =-1;
          }
        }
      }
      else
      {
        chosenOn[i] = false;
      }
      //once submit is pressed and there is an auton selected, break out of the loop
      if (i==len-1 && autonButtons[i].isPressed() && index!=-1)
      {
        return index;
        submitted = true;
        break;
      }
      b.show();
    }
    vex::wait(20,msec);
  }
  return -1;
}