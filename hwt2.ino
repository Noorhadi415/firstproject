#include <Wire.h>
#include "HWT9053.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// ==== WiFi Credentials ====
const char* ssid = "TECNO CAMON 17";
const char* password = "huzaifa03";

// ==== Flask Server IP ====
const char* serverIP = "http://192.168.73.192:5000/sensor/upload";

// ==== SD card CS pin ====
#define SD_CS D8

RTC_DS3231 rtc;
SoftwareSerial mySerial(D4, D3); // RX, TX for HWT9053
File dataFile;

unsigned char cmd[8] = {0X50, 0X03, 0X00, 0X34, 0X00, 0X10, 0X08, 0X49};
bool wasConnected = false;
bool rtcSynced = false;

// NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 5 * 3600, 60000); // GMT+5

void connectToWiFiAndSyncRTC() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    timeClient.begin();
    delay(1000);
    timeClient.update();

    rtc.adjust(DateTime(timeClient.getEpochTime()));
    rtcSynced = true;
    Serial.println("🕒 RTC synced from NTP.");
  } else {
    Serial.println("\n❌ WiFi connection failed.");
  }
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin(D2, D1);
  connectToWiFiAndSyncRTC();

  if (!rtc.begin()) {
    Serial.println("RTC not found!");
    while (1) delay(10);
  }

  // No isrunning() for DS3231, so we skip this check.
// You can optionally set compile time if needed
  if (rtc.lostPower()) {
  Serial.println("⚠️ RTC lost power. Setting compile time.");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD Card init failed!");
  } else {
    Serial.println("✅ SD Card ready.");
    if (!SD.exists("/HWT9053_log.csv")) {
      dataFile = SD.open("/HWT9053_log.csv", FILE_WRITE);
      dataFile.println("\"Time\",angleX,angleY,angleZ,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,temp");
      dataFile.close();
    }
    if (!SD.exists("/unsent.csv")) {
      dataFile = SD.open("/unsent.csv", FILE_WRITE);
      dataFile.println("\"Time\",angleX,angleY,angleZ,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,temp");
      dataFile.close();
    }
  }
}

void uploadUnsentData() {
  File unsent = SD.open("/unsent.csv", FILE_READ);
  if (unsent) {
    Serial.println("📤 Uploading unsent data...");
    String line;
    bool header = true;
    while (unsent.available()) {
      line = unsent.readStringUntil('\n');
      if (header) {
        header = false;
        continue;
      }
      line.trim();
      if (line.length() > 0) {
        String json = csvToJson(line);
        WiFiClient client;
        HTTPClient http;
        http.begin(client, serverIP);
        http.addHeader("Content-Type", "application/json");
        int response = http.POST(json);
        http.end();
        delay(50);
      }
    }
    unsent.close();
    SD.remove("/unsent.csv");
    Serial.println("✅ Unsent data uploaded.");
  }
}

String csvToJson(String csvLine) {
  csvLine.replace("\"", "");
  csvLine.trim();
  String parts[14];
  int idx = 0;
  while (csvLine.length() && idx < 14) {
    int comma = csvLine.indexOf(',');
    if (comma == -1) {
      parts[idx++] = csvLine;
      break;
    }
    parts[idx++] = csvLine.substring(0, comma);
    csvLine = csvLine.substring(comma + 1);
  }

  String json = "{";
  json += "\"timestamp\":\"" + parts[0] + "\",";
  json += "\"angle\":[" + parts[1] + "," + parts[2] + "," + parts[3] + "],";
  json += "\"acc\":[" + parts[4] + "," + parts[5] + "," + parts[6] + "],";
  json += "\"gyro\":[" + parts[7] + "," + parts[8] + "," + parts[9] + "],";
  json += "\"mag\":[" + parts[10] + "," + parts[11] + "," + parts[12] + "],";
  json += "\"temp\":" + parts[13];
  json += "}";
  return json;
}

void loop() {
  // Resync RTC if WiFi is reconnected
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    if (!rtcSynced) {
      rtc.adjust(DateTime(timeClient.getEpochTime()));
      rtcSynced = true;
      Serial.println("🔁 RTC re-synced with NTP.");
    }
  } else {
    rtcSynced = false;
  }

  while (mySerial.available()) {
    HWT9053.CopeSerialData(mySerial.read());
  }

  static unsigned short OutputCnt = 0, ReadCnt = 0;
  OutputCnt++; ReadCnt++;

  if (OutputCnt > 500) {
    OutputCnt = 0;
    DateTime now = rtc.now();

    float angleX = HWT9053.stcAngle.Angle[0] / 32768.0 * 180;
    float angleY = HWT9053.stcAngle.Angle[1] / 32768.0 * 180;
    float angleZ = HWT9053.stcAngle.Angle[2] / 32768.0 * 180;

    float accX = HWT9053.stcAcc.a[0] / 32768.0 * 16;
    float accY = HWT9053.stcAcc.a[1] / 32768.0 * 16;
    float accZ = HWT9053.stcAcc.a[2] / 32768.0 * 16;

    float gyroX = HWT9053.stcGyro.w[0] / 32768.0 * 2000;
    float gyroY = HWT9053.stcGyro.w[1] / 32768.0 * 2000;
    float gyroZ = HWT9053.stcGyro.w[2] / 32768.0 * 2000;

    float magX = HWT9053.stcMag.h[0] / 74.92;
    float magY = HWT9053.stcMag.h[1] / 74.92;
    float magZ = HWT9053.stcMag.h[2] / 74.92;

    float temp = HWT9053.stcTemp.t[0] / 100.0;

    char timeStr[20];
    sprintf(timeStr, "\"%04d-%02d-%02d %02d:%02d:%02d\"",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());

    String csvLine = String(timeStr) + "," +
                     String(angleX, 2) + "," + String(angleY, 2) + "," + String(angleZ, 2) + "," +
                     String(accX, 2) + "," + String(accY, 2) + "," + String(accZ, 2) + "," +
                     String(gyroX, 2) + "," + String(gyroY, 2) + "," + String(gyroZ, 2) + "," +
                     String(magX, 2) + "," + String(magY, 2) + "," + String(magZ, 2) + "," +
                     String(temp, 2);

    dataFile = SD.open("/HWT9053_log.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println(csvLine);
      dataFile.close();
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!wasConnected) {
        uploadUnsentData();
      }
      wasConnected = true;

      String json = csvToJson(csvLine);
      WiFiClient client;
      HTTPClient http;
      http.begin(client, serverIP);
      http.addHeader("Content-Type", "application/json");
      http.POST(json);
      http.end();
    } else {
      wasConnected = false;
      dataFile = SD.open("/unsent.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println(csvLine);
        dataFile.close();
      }
    }
  }

  if (ReadCnt > 500) {
    ReadCnt = 0;
    mySerial.write(cmd, 8);
    delayMicroseconds(8000);
  }

  delay(1);
}
