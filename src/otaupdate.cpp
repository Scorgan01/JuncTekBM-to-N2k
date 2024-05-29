// Library for web page definition for OTA code update
// Scorgan - Ulrich Meine

#include "otaupdate.h"

// const char* WIFI_HOST = "batteryhost";

//    int OTAUpdate::startWifi() // Setup WiFi access point with SSID and password
int otaStartWifi() // Setup WiFi access point with SSID and password
{
    int errorCode = 0;

    debugOutput("Setting WiFi access point ...", 6, true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IPADDRESS, AP_GATEWAY, AP_SUBNET);
    if(!WiFi.softAP(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD)) {
        errorCode = 1; // error access point setup
    }

    debugOutput("Access point: " + String(SECRET_WIFI_SSID), 4, true);
    IPAddress IP = WiFi.softAPIP();
    debugOutput("AP IP address: " + String(IP), 4, true);
    Serial.println(IP);

    if (!MDNS.begin(WIFI_HOST)) {     // use mdns for host name resolution -> http://batteryesp32.local
        errorCode = 2; // error mDNS responder setup
    }

    return errorCode; // 0 = success; 1 = access point not established; 2 = mDNS responder not established
}

int otaDefineOTAWebServer(WebServer *server) // Define OTA web server with code update function
{
    int errorCode = 0;

    String wbPgLogin;
    String wbPgUpdateIndex;

    otaDefineOTAWebPages(wbPgLogin, wbPgUpdateIndex);

    /*return index page which is stored in serverIndex */
    server->on("/", HTTP_GET, [&server, &wbPgLogin]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/html", wbPgLogin);
    });
    server->on("/updateIndex", HTTP_GET, [&server, &wbPgUpdateIndex]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/html", wbPgUpdateIndex);
    });

    /*handling uploading firmware file */
    server->on("/update", HTTP_POST, [&server]() {
        server->sendHeader("Connection", "close");
        server->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
            ESP.restart(); }, [&server, &errorCode]() {
        HTTPUpload& upload = server->upload();
        if (upload.status == UPLOAD_FILE_START) {
            debugOutput("Update: " + upload.filename, 4, true);
            Serial.printf("Update: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            /* flashing firmware to ESP*/
            debugOutput("Start flashing", 6, true);
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) { //true to set the size to the current progress
                debugOutput("Update success./nRebooting ...", 4, true);
                Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            } else {
                Update.printError(Serial);
                errorCode = 1; // error updating firmware
            }
        } });

    return errorCode; // 0 = success; 1 = error updating firmware
}

void otaDefineOTAWebPages(String pg_login, String pg_update) // Define OTA login page
{
    pg_login = "<form name='loginForm'>"
               "<table width='20%' bgcolor='A09F9F' align='center'>"
               "<tr>"
               "<td colspan=2>"
               "<center><font size=4><b>ESP32 Login Page</b></font></center>"
               "<br>"
               "</td>"
               "<br>"
               "<br>"
               "</tr>"
               "<td>Username:</td>"
               "<td><input type='text' size=25 name='userid'><br></td>"
               "</tr>"
               "<br>"
               "<br>"
               "<tr>"
               "<td>Password:</td>"
               "<td><input type='Password' size=25 name='pwd'><br></td>"
               "<br>"
               "<br>"
               "</tr>"
               "<tr>"
               "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
               "</tr>"
               "</table>"
               "</form>"
               "<script>"
               "function check(form)"
               "{"
               "if(form.userid.value=='admin' && form.pwd.value=='admin')"
               "{"
               "window.open('/updateIndex')"
               "}"
               "else"
               "{"
               " alert('Error Password or Username')/*displays error message*/"
               "}"
               "}"
               "</script>";

    pg_update = "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
                "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
                "<input type='file' name='update'>"
                "<input type='submit' value='Update'>"
                "</form>"
                "<div id='prg'>progress: 0%</div>"
                "<script>"
                "$('form').submit(function(e){"
                "e.preventDefault();"
                "var form = $('#upload_form')[0];"
                "var data = new FormData(form);"
                " $.ajax({"
                "url: '/update',"
                "type: 'POST',"
                "data: data,"
                "contentType: false,"
                "processData:false,"
                "xhr: function() {"
                "var xhr = new window.XMLHttpRequest();"
                "xhr.upload.addEventListener('progress', function(evt) {"
                "if (evt.lengthComputable) {"
                "var per = evt.loaded / evt.total;"
                "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
                "}"
                "}, false);"
                "return xhr;"
                "},"
                "success:function(d, s) {"
                "console.log('success!')"
                "},"
                "error: function (a, b, c) {"
                "}"
                "});"
                "});"
                "</script>";
}