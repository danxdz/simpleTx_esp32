
// menu code based on work from:
// https://www.youtube.com/watch?v=fjsmVzMdqyU
// https://drive.google.com/drive/folders/1w3jhBr031lmTeeE9Du9uZkYuoHx6m21P
// dsnmechanic@gmail.com
// instagram: dsnmechanics

#include <Arduino.h>
#include "menus.h"
#include "oled.h"
#include "uart.h"

Menu menuItems[CRSF_MAX_PARAMS];

int entered = -2; //-2 idle // -1 main menu // 0 options/submenu

int mmOptionSelected = -1;
int selected = 0;
int subSelected = 0;

uint8_t params_loaded = 0;

// parameter and chunk currently being read
uint8_t next_param = 0;
uint8_t next_chunk = 0;

static void parse_bytes(enum data_type type, char **buffer, char *dest)
{
  switch (type)
  {
  case UINT8:
    *(uint8_t *)dest = (uint8_t)(*buffer)[0];
    *buffer += 1;
    break;
  case INT8:
    *(int8_t *)dest = (int8_t)(*buffer)[0];
    *buffer += 1;
    break;
  case UINT16:
    *(uint16_t *)dest = (uint16_t)(((*buffer)[0] << 8) | (*buffer)[1]);
    *buffer += 2;
    break;
  case INT16:
    *(int16_t *)dest = (int16_t)(((*buffer)[0] << 8) | (*buffer)[1]);
    *buffer += 2;
    break;
  case FLOAT:
    *(int32_t *)dest = (int32_t)(((*buffer)[0] << 24) | ((*buffer)[1] << 16) | ((*buffer)[2] << 8) | (*buffer)[3]);
    *buffer += 4;
    break;
  default:
    break;
  }
}

char *Menu::getMainMenuItem_StatusText()
{
  if (optionsMainMenu[status])
    return optionsMainMenu[status];
  return 0;
}

// debug
void Menu::displayInfo()
{
  if (name)
  {
    dbout.printf("%u:%s:%u:%u:%u:%u:%u\n", id, name, parent, p_type, hidden, max_value, status);
    if (max_value)
    {
      for (size_t i = 0; i <= max_value; i++)
      {
        dbout.printf("%s:",
                     optionsMainMenu[i]);
      }
    }
    if (p_type == 9)
      dbout.printf(" :: OPT");
    if (p_type == 11)
      dbout.printf(" :: MainMenuItem");
    if (p_type == 12)
      dbout.printf(" :: INFO");
    if (p_type == 13)
      dbout.printf(" :: CMD");
    dbout.printf("\n");
  }
}

void Menu::divideValueParam(char *values)
{
  char *start = (char *)values;
  uint8_t count = 0;
  for (char *p = (char *)values; *p; p++)
  {
    if (*p == ';')
    {
      int len = (strlen(start) - strlen(p));
      optionsMainMenu[count] = new char[len + 1];
      strlcpy(optionsMainMenu[count], start, len + 1);

      start = p + 1;
      count += 1;
    }
  }
  int len = strlen(start);
  optionsMainMenu[count] = new char[len + 1];
  strlcpy(optionsMainMenu[count], start, len + 1);
  max_value = count;
}

void Menu::getParams(char *buffer, int iid)
{
  // set main menu items
  // dbout.printf("get P: %i\n",iid);
  id = iid;
  parent = *buffer++;
  p_type = *buffer & 0x7f;
  hidden = *buffer++ & 0x80;

  name = new char[strlen(buffer) + 1];
  strlcpy(name, (const char *)buffer, strlen(buffer) + 1);
  buffer += strlen(buffer) + 1;

  switch (p_type)
  {
  case 9: // text
    value = new char[strlen(buffer) + 1];
    strlcpy(value, (const char *)buffer, strlen(buffer) + 1);
    buffer += strlen(buffer) + 1;
    divideValueParam(value);
    // dbout.printf("%s",value);
    parse_bytes(UINT8, &buffer, (char *)&status);
    parse_bytes(UINT8, &buffer, (char *)&min_value);
    parse_bytes(UINT8, &buffer, (char *)&count); // don't use incorrect parameter->max_value
    parse_bytes(UINT8, &buffer, (char *)&default_value);
    break;
  case 11: // folder
    max_value = 0;
    break;
  case 12: // info
    max_value = 0;
    value = new char[strlen(buffer) + 1];
    strlcpy(value, (const char *)buffer, strlen(buffer) + 1);
    buffer += strlen(buffer) + 1;
    // dbout.printf("%s",value);
    break;
  case 13: // command
    parse_bytes(UINT8, &buffer, (char *)&status);
    // parse_bytes(UINT8, &buffer, (char *) &timeout);
    max_value = 0;

    //*(uint8_t *)status = (uint8_t) (*buffer)[0];
    //*buffer += 1;
    timeout = (uint8_t)buffer[0];
    *buffer += 1;
    info = new char[20];
    strlcpy(info, (const char *)buffer, 20);
  default:
    break;
  }
}
