// Library for web page definition for OTA code update
// Scorgan - Ulrich Meine

#include "otaupdate.h"

AsyncWebServer myWebServer(WEBSERVER_PORT);
AsyncWebSocket myLogWebSocket("/log");

int otaStartWifi() // Setup WiFi access point with SSID and password
{
    int errorCode = 0;

    debugOutput("Setting WiFi access point ...", 6, true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IPADDRESS, AP_GATEWAY, AP_SUBNET);
    if (!WiFi.softAP(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD)) {
        errorCode = 1; // error access point setup
    }

    debugOutput("Access point: " + String(SECRET_WIFI_SSID), 4, true);
    IPAddress IP = WiFi.softAPIP();
    debugOutput("AP IP address: " + IP.toString(), 4, true);

    if (!MDNS.begin(WIFI_HOST)) { // use mdns for host name resolution
        errorCode = 2; // error mDNS responder setup
    }

    return errorCode; // 0 = success; 1 = access point not established; 2 = mDNS responder not established
}


// Web socket event handler
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if(type == WS_EVT_CONNECT) {
        debugOutput("Web client connected.", 6);
        client->text("Welcome to batteryESP32 Web log!");
    } else if(type == WS_EVT_DISCONNECT) {
        debugOutput("Web client disconnected.", 6);
    }
}


void otaDefineOTAWebSite() // Define OTA web pages with code update function
{
    String wbPgLogin;
    String wbPgUpdateIndex;

    otaDefineOTAWebPages(wbPgLogin, wbPgUpdateIndex);

    myWebServer.on("/", HTTP_GET, [wbPgLogin](AsyncWebServerRequest* request) {
        request->send(200, "text/html", wbPgLogin);
    });

    myWebServer.on("/update", HTTP_GET, [wbPgUpdateIndex](AsyncWebServerRequest* request) {
        request->send(200, "text/html", wbPgUpdateIndex);
    });

    myWebServer.on("/update", HTTP_POST, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    response->addHeader("Connection", "close");
    request->send(response);
    ESP.restart(); }, [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    if (!index){
      debugOutput("Update start: " + filename, 4);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        debugOutput("Update error.", 4);
        Update.printError(Serial);
      }
    }
    if (Update.write(data, len) != len) {
      debugOutput("Update error.", 4);
      Update.printError(Serial);
    }
    if (final) {
      if (Update.end(true)) {
        debugOutput("Update success: " + String(index+len), 4);
      } else {
        debugOutput("Update error.", 4);
        Update.printError(Serial);
      }
    } });

    myWebServer.on("/log", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", 
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
            "<title>ESP32 Log</title>"
            "<script>"
            "var socket = new WebSocket('ws://' + window.location.hostname + '/log');"
            "socket.onmessage = function(event) {"
            "  var log = document.getElementById('log');"
            "  log.innerHTML += event.data + '<br>';"
            "};"
            "</script>"
            "</head>"
            "<body>"
            "<h1>ESP32 Log</h1>"
            "<div id='log' style='height:1200px;font-size:10px;overflow:auto;border:1px solid #ccc;'></div>"
            "</body>"
            "</html>"
        );
    });
}

void otaDefineOTAWebPages(String& pg_login, String& pg_update) // Define OTA login page
{
    String style = "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
                   "input{width:75%;background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
                   "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
                   "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
                   "form{background:#fff;max-width:400px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
                   ".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

    pg_login = "<form name=loginForm>"
               "<h1>BatteryESP32 Login</h1>"
               "<input name=userid placeholder='User ID'>"
               "<input name=pwd placeholder=Password type=Password>"
               "<input type=submit onclick=check(this.form) class=btn value=Login></form>"
               "<script>"
               "function check(form) {"
               "if(form.userid.value=='admin' && form.pwd.value=='esp32')"
               "{window.open('/update')}"
               "else"
               "{alert('Error Password or Username')}"
               "}"
               "</script>"
        + style;

    pg_update = "<!DOCTYPE html>"
                "<html>"
                "<head>"
                "<title>BatteryESP32 OTA Update</title>"
                "</head>"
                "<body>"
                "<form method='POST' action='#' enctype='multipart/form-data' id='upload-form'>"
                "<h1>BatteryESP32 OTA Update</h1>"
                "<input type='file' name='update'>"
                "<input type='submit' class='btn' value='Update'>" // added quotes around 'btn'
                "<div style='width:100%;background-color:#e0e0e0;border-radius:8px;'>"
                "<div id='prg' style='width:0%;background-color:#2196F3;padding:2px;border-radius:8px;color:white;text-align:center;'>0%</div>"
                "</div>"
                "</form>"
                "<script>"
                "var prg = document.getElementById('prg');"
                "var form = document.getElementById('upload-form');"
                "form.addEventListener('submit', function(e){"
                "e.preventDefault();"
                "var data = new FormData(form);"
                "var req = new XMLHttpRequest();"
                "req.open('POST', '/update');"
                "req.upload.addEventListener('progress', function(p){"
                "if(p.lengthComputable){"
                "let w = Math.round((p.loaded / p.total) * 100) + '%';"
                "prg.innerHTML = w;"
                "prg.style.width = w;"
                "if(w === '100%') prg.style.backgroundColor = '#04AA6D';"
                "}"
                "});"
                "req.send(data);"
                "});"
                "</script>"
        + style + "</body>"
                  "</html>";
}
