#include "EnableInterrupt.h"

char key_value = '\0';
char key_flag = '\0';
char key_mode = 0;
char prev_key_mode = 0;
unsigned long key_mode_time = 0;

void keyValue()
{
  if (millis() - key_mode_time > 500)
  {
    key_mode_time = millis();

    key_mode++;

    if (key_mode >= 5)
    {
      key_mode = 0;
    }
  }
}

bool getKeyValue()
{
  if (prev_key_mode != key_mode)
  {
    prev_key_mode = key_mode;
    switch (key_mode)
    {
    case 0:
      key_value = 's';
      function_mode = IDLE;
      break;
    case 1:
      key_value = '1';
      break;
    case 2:
      key_value = '2';
      break;
    default:
      break;
    }
    return true;
  }
  return false;
}

void keyInit()
{
  pinMode(KEY_MODE, INPUT_PULLUP);
  enableInterrupt(KEY_MODE, keyValue, FALLING);
}

