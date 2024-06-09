// Library for web page definition for OTA code update
// Scorgan - Ulrich Meine

#include "otaupdate.h"

//    int OTAUpdate::startWifi() // Setup WiFi access point with SSID and password
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
    debugOutput("AP IP address: " + String(IP), 4, true);
    Serial.println(IP);

    if (!MDNS.begin(WIFI_HOST)) { // use mdns for host name resolution -> http://batteryesp32.local
        errorCode = 2; // error mDNS responder setup
    }

    return errorCode; // 0 = success; 1 = access point not established; 2 = mDNS responder not established
}

int otaDefineOTAWebServer(WebServer* server) // Define OTA web server with code update function
{
    int errorCode = 0;

    String wbPgLogin;
    String wbPgUpdateIndex;

    otaDefineOTAWebPages(wbPgLogin, wbPgUpdateIndex);

    // return index page which is stored in wbPgLogin
    server->on("/", HTTP_GET, [server, wbPgLogin]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/html", wbPgLogin);
    });
    server->on("/updateIndex", HTTP_GET, [server, wbPgUpdateIndex]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/html", wbPgUpdateIndex);
    });

    // handling uploading firmware file
    server->on("/update", HTTP_POST, [server]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
            ESP.restart(); }, [server, &errorCode]() {
        HTTPUpload& upload = server->upload();
        if (upload.status == UPLOAD_FILE_START) {
            debugOutput("Update: " + upload.filename, 4, true);
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // start with max available size
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            // flashing firmware to ESP
            debugOutput("File flashing", 6);
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { // true to set the size to the current progress
                debugOutput("Update success. Rebooting ...", 4, true);
                Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
                errorCode = 1; // error updating firmware
            }
        } });

    return errorCode; // 0 = success; 1 = error updating firmware
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
               "{window.open('/updateIndex')}"
               "else"
               "{alert('Error Password or Username')}"
               "}"
               "</script>"
        + style;

    pg_update = "<body>"
                "<form method='POST' action='#' enctype='multipart/form-data' id='upload-form'>"
                "<h1>BatteryESP32 OTA</h1>"
                "<input type='file' name='update'>"
                "<input type='submit' class=btn value='Update'>"
                "<div style='width:100%;background-color:#e0e0e0;border-radius:8px;'>"
                "<div id='prg' style='width:0%;background-color:#2196F3;padding:2px;border-radius:8px;color:white;text-align:center;'>0%</div>"
                "</div>"
                "</form>"
                "</body>"
                "<script>"
                "var prg = document.getElementById('prg');"
                "var form = document.getElementById('upload-form');"
                "form.addEventListener('submit', e=>{"
                "e.preventDefault();"
                "var data = new FormData(form);"
                "var req = new XMLHttpRequest();"
                "req.open('POST', '/update');"
                "req.upload.addEventListener('progress', p=>{"
                "let w = Math.round((p.loaded / p.total)*100) + '%';"
                "if(p.lengthComputable){"
                "prg.innerHTML = w;"
                "prg.style.width = w;"
                "}"
                "if(w == '100%') prg.style.backgroundColor = '#04AA6D';"
                "});"
                "req.send(data);"
                "});"
                "</script>"
        + style;
}