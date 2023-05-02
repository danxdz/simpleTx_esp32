#ifndef WEBUI_H
#define WEBUI_H
#include "menus.h"

void updatePacketRate(int packet_rate);
void updateTelemRatio(int telem_ratio);
void startWebServer(int params,Menu menuItems[]);

#endif
