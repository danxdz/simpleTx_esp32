#include "webui.h"
#include "uart.h"
#include "menus.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include "crsf.h"


static AsyncWebServer server(80);

const char* ssid = "simpleTx";
const char* password = "testingH0m&";



String generateMenu3(const Menu& menu, int params) {
  String html = "<div class='menu-item'>" + String(menu.name);
  if (menu.p_type == 9) { //"OPT"
    html += "<select>";
    for (int i = 0; i < menu.max_value; i++) {
        html += "<option value='" + String(i) + "'>" + (menu.status == i ? "*" : "") + String(menu.optionsMainMenu[i]) + "</option>";
    }
    html += "</select>";
  }else if(menu.p_type == 13){ // "CMD"
    html += "<button onClick='sendCommand(" + menu.id + String(")'>") + menu.optionsMainMenu[0]+ "</button>";
  }else if(menu.p_type == 11){
    html += "<br>";
    for (int j = 0; j < params; j++) {
      if (menuItems[j].parent == menu.id && menuItems[j].p_type != 12 && menuItems[j].p_type != 13) {
        html += "\t" + generateMenu3(menuItems[j], params);
      }
    }
  }
  html += "</div>";
  return html;
}



String generateInfo(int params) {
  String html = "<div class='info-item'>";
  for (int i = 0; i < params; i++) {
    if (menuItems[i].p_type == 12) {
      html += "<div class='info-item'>" + String(menuItems[i].name) + " - " + String(menuItems[i].value) + "</div>";
    }
  }
  html += "</div>";
  return html;
}


void handleDashboardRequest2(AsyncWebServerRequest *request, int params,  Menu menuItems[]){
  String dashboard_html = "<html><head><title>TX Parameters</title>";
  dashboard_html += "<style>";
  dashboard_html += "table {border-collapse: collapse; border-radius: 12px; width: 100%; margin: 0 auto;}";
  dashboard_html += "th, td {border: 1px solid #dddddd; text-align: left; padding: 8px;}";
  dashboard_html += "select {background-color: #4CAF50; border: none; color: white; padding: 16px 32px; text-align: center; font-size: 16px; margin: 4px 2px; transition-duration: 0.4s; cursor: pointer; border-radius: 10px;}";
  dashboard_html += ".container {display: grid; grid-template-columns: repeat(2, 1fr); grid-gap: 20px;}";
  dashboard_html += ".menu-item {display: flex; align-items: center;}";
  dashboard_html += ".info-item {text-align: right;}";
  dashboard_html += "</style>";
  dashboard_html += "</head><body>";
  dashboard_html += generateInfo(params);
  dashboard_html += "<div class='container'>";
  
  for (int i = 0; i < params; i++) {
    if (menuItems[i].name == "" || menuItems[i].p_type == 12 || menuItems[i].p_type == 13) {
      continue;
    }
    dashboard_html += "<td>";
    if(menuItems[i].p_type == 11){
      dashboard_html += "<br>";
      for (int j = 0; j < params; j++) {
        if (menuItems[j].parent == menuItems[i].id && menuItems[j].p_type != 12 && menuItems[j].p_type != 13) {
          dashboard_html += generateMenu3(menuItems[j], params);
        }
      }
    } else {
       dashboard_html += "<tr><td><h2>" + String(menuItems[i].name) + "</h2></td>";
        
    }
  }


      dashboard_html += "</div>";
      dashboard_html += "</body></html>";

      request->send(200, "text/html", dashboard_html);
      }


String generateMenu2(const Menu& menu, int params) {
  String html = "<div class='menu-item'>" + String(menu.name);
  if (menu.p_type == 9) { //"OPT"
    html += "<select>";

    int selected = menu.status;
    html += "<option value='" + String(selected) + "'>" + String(menu.optionsMainMenu[selected]) + "</option>";

    for (int i = 0; i < menu.max_value; i++) {
        if(i != selected){
           html += "<option value='" + String(i) + "'>" + String(menu.optionsMainMenu[i]) + "</option>";
        }
    }
    html += "</select>";
  }
  for (int i = 0; i < params; i++) {
     if (menuItems[i].parent == menu.id && menuItems[i].p_type != 12 && menuItems[i].p_type != 13) {
         html += generateMenu2(menuItems[i], params);
    }
  }
  html += "</div>";
  return html;
}

void handleDashboardRequest(AsyncWebServerRequest *request, int params,  Menu menuItems[]){
  String dashboard_html = "<html><head><title>TX Parameters</title>";
  dashboard_html += "<style>";
  dashboard_html += "table {border-collapse: collapse;}";
  dashboard_html += "th, td {border: 1px solid black; text-align: left;}";
  dashboard_html += "select {background-color: #4CAF50; border: none; color: white; padding: 16px 32px; text-align: center; font-size: 16px; margin: 4px 2px; transition-duration: 0.4s; cursor: pointer; border-radius: 10px;}";
  dashboard_html += "</style>";
  dashboard_html += "</head><body>";
  dashboard_html += generateInfo(params);
  dashboard_html += "<table>";
  // loop through all the menuItems
  for (int i = 0; i < params; i++) {
    if (menuItems[i].name == "" || menuItems[i].p_type == 12 || menuItems[i].p_type == 13) {
      continue;
    }
    dashboard_html += "<tr><td><h2>" + String(menuItems[i].name) + "</h2></td>";
    dashboard_html += "<td>";
    if(menuItems[i].p_type == 11){
      dashboard_html += "<br>";
      for (int j = 0; j < params; j++) {
        if (menuItems[j].parent == menuItems[i].id && menuItems[j].p_type != 12 && menuItems[j].p_type != 13) {
          dashboard_html += generateMenu2(menuItems[j], params);
        }
      }
  }else{
    dashboard_html += generateMenu2(menuItems[i], params);
  }
  dashboard_html += "</td></tr>";
}
dashboard_html += "</table>";
dashboard_html += "</body></html>";

  request->send(200, "text/html", dashboard_html);
}


String generateMenu(const Menu& menu, int params) {
  String html = "<div class='menu-item'>" + String(menu.name);
  if (menu.p_type == 9) { //"OPT"
    html += "<select>";
    for (int i = 0; i < menu.max_value; i++) {
      html += "<option value='" + String(i) + "'>" + String(menu.optionsMainMenu[i]) + "</option>";
    }
    html += "</select>";
  }
  for (int i = 0; i < menuItems->max_value; i++) {
    if (menuItems[i].parent == menu.id) {
      html += generateMenu(menuItems[i], params);
    }
  }
  html += "</div>";
  return html;
}

void handleTable2Request(AsyncWebServerRequest *request, int params) {
  String html = "<html><head><title>TX Parameters</title>";

  html += "<style>";
  html += "table {border-collapse: collapse;}";
  html += "th, td {border: 1px solid black; text-align: left;}";
  html += "select {background-color: #4CAF50; border: none; color: white; padding: 16px 32px; text-align: center; font-size: 16px; margin: 4px 2px; transition-duration: 0.4s; cursor: pointer; border-radius: 10px;}";
  html += "</style>";
  html += "</head><body>";

  html += "<table>";
  
  for (int i = 0; i < params; i++) {
    if (menuItems[i].name == "" || menuItems[i].p_type == 12 || menuItems[i].p_type == 13) {
      continue;
    }
    html += "<tr><td><h2>" + String(menuItems[i].name) + "</h2></td>";
    html += "<td>";
    if(menuItems[i].p_type == 11){ // if MainMenuItem
      html += "<br>";
      for (int j = 0; j < params; j++) {
        if (menuItems[j].parent == menuItems[i].id && menuItems[j].p_type != 12 && menuItems[j].p_type != 13) {
          html += generateMenu(menuItems[j], params);
        }
      }
    }else{ // if OPT
      html += generateMenu(menuItems[i], params);
    }
    html += "</td></tr>";
  }
  html += "</table>";
  html += "</body></html>";
  request->send(200, "text/html", html);
}



int serverStart =0;

void startWebServer(int params,Menu menuItems[]) {
  dbout.printf("serverStart with %i \n", params);
 for (int i = 0; i < params; i++)
{
   if (menuItems[i].name == "") {
            continue;
        }
  menuItems[i].displayInfo();
}


  if (serverStart == 0)  {
    dbout.println("server starting");
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    server.begin();
    serverStart = 1;
    // Print the IP address of the AP
    dbout.println(WiFi.softAPIP());
    
    server.on("/ip", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<html><head><title>Local IP</title></head><body><h1>Local IP</h1>";
      html += "<p>Your local IP address is: " + WiFi.softAPIP().toString() + "</p>";
      html += "</body></html>";
      request->send(200, "text/html", html);
    });
      
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<html><head><title>WElcome</title></head><body><h1>Local IP</h1>";
      html += "<p>Your local IP address is: " + WiFi.softAPIP().toString() + "</p>";
      html += "</body></html>";
      request->send(200, "text/html", html);
    });
    
    // Add the HTML form to reboot the ESP32
    server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<html><head><title>Reboot</title></head><body><h1>Reboot</h1>";
      html += "<form method='post'><input type='submit' value='Reboot'></form>";
      html += "</body></html>";
      request->send(200, "text/html", html);
    });
    
    server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200, "text/html", "Rebooting...");
      ESP.restart();
    });

    server.on("/table2", HTTP_GET, [menuItems, params](AsyncWebServerRequest *request){
    // String html = "<html><head><script src='https://code.jquery.com/jquery-3.6.3.min.js' integrity='sha256-pvPw+upLPUjgMXY0G+8O0xUf+/Im1MZjXxxgOcBQBXU=' crossorigin='anonymous'></script><title>TX Parameters</title></head><body>";
      handleTable2Request(request, params);
    });
    server.on("/dash", HTTP_GET, [menuItems, params](AsyncWebServerRequest *request){
       handleDashboardRequest2(request, params, menuItems);
    });
  }
}

