#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <FS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>

#define VERSION 3

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

	/*
	Serial.println("Starting OTA.");
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	*/
	ArduinoOTA.begin();

	//Serial.println("Starting Web server.");
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
	httpd.on("/key", [&](){
		httpd.setContentLength(CONTENT_LENGTH_UNKNOWN);
		String index = httpd.arg("index");
		httpd.send(200, "text/html", ""); //"get ") + index + ", result:");
		Serial.flush();
		String cmd = String("[gk") + index + "]";
		Serial.print(cmd);
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
		if ( result.length() == 101 ) {
			//string surgery
			while ( result.indexOf(" 0x") != -1 ) {
				result.remove(result.indexOf(" 0x"), 3);
			}
			result.toLowerCase();
			httpd.sendContent(result);
		} else {
			httpd.sendContent(String("") + result.length());
		}
		httpd.client().stop();
	});
	httpd.on("/unlock", [&](){
		httpd.send(200, "text/html", "ok");
		Serial.write("[du]\n");
		httpd.client().stop();
	});
	httpd.begin();

	//Serial.println("Starting interface to atmega.");
	Serial.begin(38400);
	Serial.swap();

	//Serial.println("Startup complete.");
}

void loop()
{
	
	ArduinoOTA.handle();
	httpd.handleClient();

}



