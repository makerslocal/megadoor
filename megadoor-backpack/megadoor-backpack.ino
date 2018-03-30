#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <FS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <ESP8266httpUpdate.h>

#define VERSION 4

HTTPClient http;
ESP8266WebServer httpd(80);

const char * header = R"(<!DOCTYPE html>
<html>
<head>
<title>megadoor-backpack</title>
<link rel="stylesheet" href="/style.css">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
</head>
<body>
<div id="top">
        <span id="title">megadoor-backpack</span>
        <a href="/">Configuration</a>
        <a href="/debug">Debug</a>
</div>
)";

void spiffsWrite(String path, String contents) {
    File f = SPIFFS.open(path, "w");
    f.println(contents);
    f.close();
}
String spiffsRead(String path) {
    File f = SPIFFS.open(path, "r");
    String x = f.readStringUntil('\n');
    f.close();
    return x;
}

bool requestIsAuthorized() {
    String ip = httpd.client().remoteIP().toString();
    if ( ip.startsWith("10.56.5.") ) {
        return true; //It's the remote Web server.
    }
    return ( ip == "10.56.0.11" );
}
        

void setup()
{
    /*
    Serial.begin(115200);
    Serial.println();
    delay(200);
    Serial.print("MEGADOOR BACKPACK v"); Serial.println(VERSION);
    */

    //Serial.print("Mounting disk... ");
    FSInfo fs_info;
    SPIFFS.begin();
    if ( ! SPIFFS.info(fs_info) ) {
        //the FS info was not retrieved correctly. it's probably not formatted
        //Serial.print("failed. Formatting disk... ");
        SPIFFS.format();
        //Serial.print("done. Mounting disk... ");
        SPIFFS.begin();
        SPIFFS.info(fs_info);
    }
    //Serial.print("done. ");
    //Serial.print(fs_info.usedBytes); Serial.print("/"); Serial.print(fs_info.totalBytes); Serial.println(" bytes used");
    
    //Serial.println("Checking version");
    String lastVerString = spiffsRead("/version");
    if ( lastVerString.toInt() != VERSION ) {
        //we just got upgrayedded or downgrayedded
        //Serial.print("We just moved from v"); Serial.println(lastVerString);
        spiffsWrite("/version", String(VERSION));
        //Serial.print("Welcome to v"); Serial.println(VERSION);
    }

    //Serial.println("Setting up pins");
    pinMode(D1, OUTPUT);

    //Serial.println("Starting wireless.");
    WiFi.hostname("megadoor-backpack");
    WiFiManager wifiManager; //Load the Wi-Fi Manager library.
    wifiManager.setTimeout(300); //Give up with the AP if no users gives us configuration in this many secs.
    if(!wifiManager.autoConnect("megadoor-backpack Setup")) {
        //Serial.println("failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
    }
    //Serial.print("WiFi connected: ");
    //Serial.println(WiFi.localIP());

    //Serial.println("Starting Web server.");
    httpd.on("/update", HTTP_POST, [&](){
        httpd.sendHeader("Connection", "close");
        httpd.sendHeader("Access-Control-Allow-Origin", "*");
        httpd.send(200, "text/plain", (Update.hasError())?"FAIL":"OK");
    },[&](){
        // handler for the file upload, get's the sketch bytes, and writes
        // them through the Update object
        HTTPUpload& upload = httpd.upload();
        if(upload.status == UPLOAD_FILE_START){
            if ( !requestIsAuthorized ) {
                return;
            }
            uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
            if(!Update.begin(maxSketchSpace)){//start with max available size
                //it failed because it was too big
            }
        } else if(upload.status == UPLOAD_FILE_WRITE){
            if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
                //it failed to write
            }
        } else if(upload.status == UPLOAD_FILE_END){
            if(Update.end(true)){ //true to set the size to the current progress
                ESP.restart();
            } else {
                //it failed for some reason
            }
        } else if(upload.status == UPLOAD_FILE_ABORTED){
            Update.end();
        }
        delay(0);
    });
    httpd.on("/style.css", [&](){
        httpd.send(200, "text/css",R"(
            html {
                font-family:sans-serif;
                background-color:black;
                color: #e0e0e0;
            }
            div {
                background-color: #202020;
            }
            h1,h2,h3,h4,h5 {
                color: #2020e0;
            }
            a {
                color: #50f050;
            }
            form * {
                display:block;
                border: 1px solid #000;
                font-size: 14px;
                color: #fff;
                background: #004;
                padding: 5px;
            }
        )");
        httpd.client().stop();
    });
    httpd.on("/", [&](){
        httpd.setContentLength(CONTENT_LENGTH_UNKNOWN);
        httpd.send(200, "text/html", header);
        httpd.client().stop();
    });
    httpd.on("/key", HTTP_GET, [&](){
        if ( ! requestIsAuthorized() ) { return; }

        httpd.setContentLength(CONTENT_LENGTH_UNKNOWN);
        String index = httpd.arg("index");
        httpd.send(200, "text/html", "");
        //httpd.sendContent(index + ":");
        
        Serial.flush();
        String cmd = String("[gk") + index + "]";

        Serial.print(cmd);
        if ( ! Serial.available() ) {
            httpd.sendContent("\n\n\n\n\n\n\n\n\n\n\n\n\n");
            Serial.flush();
            Serial.print(cmd);
        }

        String result;
        while ( Serial.available() > 0 ) {
            String temp = Serial.readStringUntil('\n');
            //httpd.sendContent(String("LINE") + temp.length() + ":");
            //httpd.sendContent(temp);
            httpd.sendContent("\n");
            if ( temp.indexOf("0x") >= 0 ) {
                result = String(" ") + temp;
            }
        }
        if ( result.length() == 102 ) {
            //string surgery
            while ( result.indexOf(" 0x") != -1 ) {
                result.remove(result.indexOf(" 0x"), 3);
            }
            result.toLowerCase();
            httpd.sendContent(result);
        }
        httpd.client().stop();
    });
    httpd.on("/key", HTTP_POST, [&](){
        if ( ! requestIsAuthorized() ) { return; }

        httpd.setContentLength(CONTENT_LENGTH_UNKNOWN);
        String key = httpd.arg("key");
        if ( key.length() != 40 ) {
            httpd.send(400, "text/html", "Wrong length");
        } else {
            httpd.send(200, "text/html", "");
            Serial.flush();
            String cmd = String("[ka" + key + "]");
            Serial.print(cmd);
            if ( ! Serial.available() ) {
                httpd.sendContent("\n\n\n\n\n\n\n\n\n\n\n\n\n");
                Serial.flush();
                Serial.print(cmd);
            }
            
            String result;
            while ( Serial.available() > 0 ) {
                result += Serial.readString();
            }
            httpd.sendContent(result);
        }
        httpd.client().stop();
    });
    httpd.on("/key", HTTP_DELETE, [&](){
        if ( ! requestIsAuthorized() ) { return; }

        httpd.setContentLength(CONTENT_LENGTH_UNKNOWN);
        String index = httpd.arg("index");
        httpd.send(200, "text/html", "");
        Serial.flush();
        String cmd = String("[kd" + index + "]");
        Serial.print(cmd);
        if ( ! Serial.available() ) {
            httpd.sendContent("\n\n\n\n\n\n\n\n\n\n\n\n\n");
            Serial.flush();
            Serial.print(cmd);
        }
        
        String result;
        while ( Serial.available() > 0 ) {
            result += Serial.readString();
        }
        httpd.sendContent(result);
        httpd.client().stop();
    });
    httpd.on("/unlock", HTTP_POST, [&](){
        if ( ! requestIsAuthorized() ) { return; }

        digitalWrite(D1, HIGH);

        httpd.send(200, "text/html", "ok");
        Serial.write("[du]\n");
        httpd.client().stop();

        delay(200);
        digitalWrite(D1, LOW);
    });
    httpd.begin();

    //Serial.println("Starting interface to atmega.");
    Serial.begin(38400);
    Serial.swap();

    //Serial.println("Startup complete.");
}

void loop()
{
    
    httpd.handleClient();

}



