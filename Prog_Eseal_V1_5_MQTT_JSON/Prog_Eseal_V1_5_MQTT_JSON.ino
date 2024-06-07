/*
- Buat Program
  .program get location (GPS) (ombrometer esp32) (ok)
  .Send Data Sim800l (ombrometer esp32)
  .Motor L298N (cari referensi) (ok)
  .Buzzer (ok)
  .Fungsi Buka, Tutup, Kabel diputus (syarat) (ok)
  .RFID (cari referensi) (ok)
  .LED indicator (berhasil buka menyala blink 2x, jika gagal led merah menyala blink 2x) (ok)
  .BMS info bat (ombrometer esp32) (ok)
  .Trigger pin (ok)
  .Wifi, MQTT
  .PinMode (ok)
  .SleepMode , Wakeup Trigger (ok)
  .Setup (baru catatan fungsi apa saja yang dilakukan)
  .Loop (baru catatan fungsi apa saja yang dilakukan)
*/
//========= Library ============//
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>

//========= Define Pin ==========//
#define SIM800L_TRIG 12
#define RFID_TRIG 14
#define L298N_TRIG 27
#define GPS_TRIG 4
#define BUZZER 25
#define MOTOR 34
#define INTERRUP_RFID 26 // di pullup
#define INPUT_ESEAL 39 
#define RST_RFID 13
#define SS_RFID 5
#define LED_HIJAU 12
#define LED_MERAH 33
#define IN1 22
#define IN2 21
#define BMS 35
#define SPARE_BT 36
#define PIN_IN1 22
#define PIN_IN2 21
#define PIN_TRIG_L298N 27
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//========== DEKLARASI TASK =========//
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;
TaskHandle_t Task6;
TaskHandle_t Task7;
TaskHandle_t Task8;
TaskHandle_t Task9;
TaskHandle_t Task10;

//========== EEPROM =========//
// char sizeOfByteEachListEepromMemory[60] = {
//   sizeof(xAddress),               //0
//   sizeof(calibration_battery_m),  //1
//   sizeof(calibration_battery_c),  //2
//   sizeof(log_interval),           //3
// };
// char getEepromMemory(char addressNum) {
//   char memoryPoint = 0;
//   for (int i = 0; i < addressNum; i++) {
//     memoryPoint += sizeOfByteEachListEepromMemory[i];
//   }
//   return memoryPoint;
// }
// void vEEPROMRead() {
//   EEPROM.begin(EEPROM_SIZE);
//   EEPROM.get(getEepromMemory(0), xAddress);
//   if (xAddress != 125) {
//     xAddress = 125;
//     version = "1.0.0"
//     calibration_battery_m = 0;
//     calibration_battery_c = 0;
//     log_interval = 5;
//   } else {
//     EEPROM.get(getEepromMemory(0), xAddress);
//     EEPROM.get(getEepromMemory(1), calibration_battery_m);
//     EEPROM.get(getEepromMemory(2), iTotalNode);
//     EEPROM.get(getEepromMemory(3), fCalBatPusat);
//     EEPROM.get(getEepromMemory(4), iTimeReportPeriodic);
//     EEPROM.get(getEepromMemory(5), iTimePeriodicSensor);
//     EEPROM.get(getEepromMemory(6), CRC32Web);
//     for (int i=0; i<10; i++){
//       EEPROM.get(getEepromMemory(7+i), fCalBatS[i+1]);
//       EEPROM.get(getEepromMemory(17+i), fCalTempM[i+1]);
//       EEPROM.get(getEepromMemory(27+i), fCalTempB[i+1]);
//       EEPROM.get(getEepromMemory(37+i), fCalHumM[i+1]);
//       EEPROM.get(getEepromMemory(47+i), fCalHumB[i+1]);
//     }
//     EEPROM.get(getEepromMemory(57), iJumlahSensor);
//   }
//   //Update oldData
//   oldLastVersion = iLastVersion;
//   oldTotalNode = iTotalNode;
//   oldTimeReportPeriodic = iTimeReportPeriodic;
//   oldTimePeriodicSensor = iTimePeriodicSensor;
//   oldJumlahSensor = iJumlahSensor;
//   oldCalBatPusat = fCalBatPusat;
//   for (int i = 1; i <= iJumlahSensor; i++) {
//     oldCalBatS[i] = fCalBatS[i];
//     oldCalTempM[i] = fCalTempM[i];
//     oldCalTempB[i] = fCalTempB[i];
//     oldCalHumM[i] = fCalHumM[i];
//     oldCalHumB[i] = fCalHumB[i];
//   }
//   strcpy(oldCRC32Web, CRC32Web);
  
//   vSerialEEPROM("vEEPROMRead");
// }
//========== DEKLARASI VARIABLE =========//
// Wifi
const char* ssid = "Marnov_plus";
const char* password = "jujurdanamanah";
// OTA
const char* firmwareURL = "https://firebasestorage.googleapis.com/v0/b/otastorage-503b9.appspot.com/o/ESP32UV.bin?alt=media";
// MQTT
const char* MQTT_SERVER = "103.167.112.188";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "/eshield:eshield";
const char* MQTT_PASS = "eshield";

//GPS
TinyGPSPlus gps;
//BMS
float VoltageBattery;
float CalBatScale = 1;
int BatteryPercentage;
//I298N PWM
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 4;
const int resolution = 8;
// Variable to save epoch time
unsigned long epochTime;
// Version
const int versionNumber = 1;
// Mac Address
char MergedMacAddress[13];
//RFID
MFRC522 rfid(SS_RFID, RST_RFID); // Instance of the class
MFRC522::MIFARE_Key key; 
byte nuidPICC[4]; // Init array that will store new NUID 
char hexNUID[9];
//Sleep 
int TIME_SLEEP = 10 * 60; //menit
// Send Config Interval
int sendConfigInterval = 5;

//========= RTC MEMORY ESP3 ===========//
RTC_DATA_ATTR double LatitudeRTC;
RTC_DATA_ATTR double LongitudeRTC;
RTC_DATA_ATTR bool DataGPSUploadRTC;
RTC_DATA_ATTR bool bWaktuCekGPSRTC;
RTC_DATA_ATTR bool bNyalakanGPSRTC;
RTC_DATA_ATTR bool bSudahDapatLokasiRTC;
RTC_DATA_ATTR bool bStatusKabelPutus; //Kabel Putus 1, Kabel Menyambung 0
RTC_DATA_ATTR bool bStatusLock; //Terkunci 1, Kunsi Terbuka 0
RTC_DATA_ATTR bool bInputStatusSeal; //terbuka 1, tertutup 0
RTC_DATA_ATTR bool bOtaOnProcess; //onOta 1, offOta 0
RTC_DATA_ATTR bool bScanNuid; //on scan 1, off scan 0
RTC_DATA_ATTR bool bSendGPS;
RTC_DATA_ATTR bool bStartOpen;
RTC_DATA_ATTR bool bBukaGembok;
RTC_DATA_ATTR bool bMotorOpen;
RTC_DATA_ATTR bool bMotorWait;
RTC_DATA_ATTR bool bMotorClose;
RTC_DATA_ATTR bool bNormalOpen;
RTC_DATA_ATTR bool bSendStatus;
bool bDoneSendStatus;
bool bSendConfig;
bool bDoneSendConfig;
bool bSendLog;

//========== FUNGSI Setup =========//
WiFiClient espClient;
PubSubClient client(espClient);
void vSetupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void vSetupEpocTime(){
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone (GMT +7)
  timeClient.setTimeOffset(25200);

  vGetEpochTime();
}
//===================== PrepareAddres =====================//
void vPrepareID(){
  unsigned char mac[6];
  WiFi.macAddress(mac);
  snprintf(MergedMacAddress,sizeof(MergedMacAddress), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
//===================== Subs & Send Config MQTT =====================//
void vSubsConfig(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[50];
  sprintf(topic, "devices/%s/config", MergedMacAddress);

  client.subscribe(topic);
  Serial.print("Subscribe to: ");
  Serial.println(topic);
}
void vSendConfig(const char* version, const char* calibration_battery_m, const char* calibration_battery_c, const char* log_interval){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char status[10];
  if (digitalRead(INPUT_ESEAL) == 0) {
    strcpy(status, "LOCKED");
  } else {
    strcpy(status, "UNLOCKED");
  }

  char topic[50];
  sprintf(topic, "devices/%s/config", MergedMacAddress);
  // Membuat data JSON
  DynamicJsonDocument doc(200); // Ukuran buffer disesuaikan dengan kebutuhan
  doc["version"] = version;
  doc["calibration_battery_m"] = calibration_battery_m;
  doc["calibration_battery_c"] = calibration_battery_c;
  doc["log_interval"] = log_interval;

  // Serialisasi JSON ke string
  char jsonStringBuffer[200]; // Ukuran buffer disesuaikan dengan kebutuhan
  serializeJson(doc, jsonStringBuffer, sizeof(jsonStringBuffer));

  Serial.println("Data JSON:");
  Serial.println(jsonStringBuffer);
  
  // Publish a message
  Serial.print("SEND CONFIG: ");
  Serial.println(topic);
  client.publish(topic, jsonStringBuffer); // Mengirimkan pesan menggunakan buffer karakter
  bSendConfig = 1;
  bDoneSendConfig = 1;
}
//===================== Subs & Send STATUS MQTT =====================//
void vSubsStatus(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[50];
  sprintf(topic, "devices/%s/status", MergedMacAddress);

  client.subscribe(topic);
  Serial.print("Subscribe to: ");
  Serial.println(topic);
}
void vSendStatus(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char status[10];
  if (digitalRead(INPUT_ESEAL) == 0) {
    strcpy(status, "LOCKED");
  } else {
    strcpy(status, "UNLOCKED");
  }

  char topic[50];
  sprintf(topic, "devices/%s/status", MergedMacAddress);
  // Membuat data JSON
  DynamicJsonDocument doc(200); // Ukuran buffer disesuaikan dengan kebutuhan
  doc["user_id"] = "1234";
  doc["datetime"] = epochTime;
  doc["status"] = status;
  doc["latitude"] = "-6.905977";
  doc["longitude"] = "-6.905977";
  doc["network"] = "WIFI"; // Perbaikan pada nama field "network"
  doc["battery"] = BatteryPercentage;

  // Serialisasi JSON ke string
  char jsonStringBuffer[260]; // Ukuran buffer disesuaikan dengan kebutuhan
  serializeJson(doc, jsonStringBuffer, sizeof(jsonStringBuffer));

  Serial.println("Data JSON:");
  Serial.println(jsonStringBuffer);
  
  // Publish a message
  Serial.print("SEND STATUS: ");
  Serial.println(topic);
  client.publish(topic, jsonStringBuffer); // Mengirimkan pesan menggunakan buffer karakter
  bSendStatus = 1;
  bDoneSendStatus = 1;
}
//===================== Subs & Send RFID MQTT =====================//
void vSubsRFID(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[50];
  sprintf(topic, "devices/%s/rfid", MergedMacAddress);

  client.subscribe(topic);
  Serial.print("Subscribe to: ");
  Serial.println(topic);
}
void vSendRFID(String rfid){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[50];
  sprintf(topic, "devices/%s/rfid", MergedMacAddress);
  // Membuat data JSON
  DynamicJsonDocument doc(100); // Ukuran buffer disesuaikan dengan kebutuhan
  doc["device_id"] = MergedMacAddress;
  doc["rfid"] = rfid;

  // Serialisasi JSON ke string
  char jsonStringBuffer[100]; // Ukuran buffer disesuaikan dengan kebutuhan
  serializeJson(doc, jsonStringBuffer, sizeof(jsonStringBuffer));

  Serial.println("Data JSON:");
  Serial.println(jsonStringBuffer);

  // Publish a message
  Serial.print("SEND RFID: ");
  Serial.println(topic);
  client.publish(topic, jsonStringBuffer); // Mengirimkan pesan menggunakan buffer karakter
}
//===================== Subs & Send UNLOCK MQTT =====================//
void vSubsUnlock(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[50];
  sprintf(topic, "devices/%s/unlock", MergedMacAddress);

  client.subscribe(topic);
  Serial.print("Subscribe to: ");
  Serial.println(topic);
}
void vSendUnlock(const char* method){
  Serial.println(method);
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char topic[80];
  sprintf(topic, "devices/%s/unlock", MergedMacAddress);
  // Membuat data JSON
  DynamicJsonDocument doc(200); // Ukuran buffer disesuaikan dengan kebutuhan
  doc["device_id"] = MergedMacAddress;
  doc["user_id"] = "1234";
  doc["datetime"] = epochTime;
  doc["method"] = method;
  doc["status"] = "UNLOCKED";
  doc["latitude"] = "-6.905977";
  doc["longitude"] = "-6.905977";
  doc["network"] = "WIFI";
  doc["battery"] = BatteryPercentage;

  // Serialisasi JSON ke string
  char jsonStringBuffer[260]; // Ukuran buffer disesuaikan dengan kebutuhan
  serializeJson(doc, jsonStringBuffer, sizeof(jsonStringBuffer));

  Serial.println("Data JSON:");
  Serial.println(jsonStringBuffer);

  // Publish a message
  Serial.print("SEND UNLOCK: ");
  Serial.println(topic);
  client.publish(topic, jsonStringBuffer); // Mengirimkan pesan menggunakan buffer karakter
}
//===================== Subs LOG MQTT =====================//
void vSendLogs(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }

  char status[10];
  if (digitalRead(INPUT_ESEAL) == 0) {
    strcpy(status, "LOCKED");
  } else {
    strcpy(status, "UNLOCKED");
  }

  char topic[80];
  sprintf(topic, "devices/%s/logs", MergedMacAddress);
  // Membuat data JSON
  DynamicJsonDocument doc(200); // Ukuran buffer disesuaikan dengan kebutuhan
  doc["device_id"] = MergedMacAddress;
  doc["user_id"] = "null";
  doc["datetime"] = epochTime;
  doc["method"] = "LOG";
  doc["status"] = status;
  doc["latitude"] = "-6.905977";
  doc["longitude"] = "-6.905977";
  doc["network"] = "WIFI";
  doc["battery"] = BatteryPercentage;

  // Serialisasi JSON ke string
  char jsonStringBuffer[260]; // Ukuran buffer disesuaikan dengan kebutuhan
  serializeJson(doc, jsonStringBuffer, sizeof(jsonStringBuffer));

  Serial.println("Data JSON:");
  Serial.println(jsonStringBuffer);

  // Publish a message
  Serial.print("SEND UNLOCK: ");
  Serial.println(topic);
  client.publish(topic, jsonStringBuffer); // Mengirimkan pesan menggunakan buffer karakter
  bSendLog = 1;
}
//===================== Send & Subs Message MQTT =====================//
void vSendNUID(String rfid) {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      // delay(2000);
    }
  }

  char rfidData[70];
  sprintf(rfidData, "%s,%d,%d,%d", rfid.c_str(), epochTime, BatteryPercentage, versionNumber);

  // Publish a message
  Serial.print("SEND RFID: ");
  Serial.println(rfidData);
  client.publish("B399401C", rfidData, true);
  // client.publish("RFID", rfidData, true);
  // client.publish("RFID", rfidData);
}
void vSendFeedback(const char* message) {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      // delay(2000);
    }
  }

  // Publish a message
  Serial.print("SEND FEEDBACK: ");
  Serial.println(message);
  client.publish("B399401C", message, true);
  // client.publish("FEEDBACK", message, true);
  // client.publish("FEEDBACK", message);
}
void vSendGPS() {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress , MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      // delay(2000);
    }
  }

  char gpsData[50];
  sprintf(gpsData, "%f,%f,%d,%d", -6.905977, -6.905977, epochTime, BatteryPercentage);

  bSendGPS = 1;
  
  // Publish a message
  Serial.print("SEND GPS: ");
  Serial.println(gpsData);
  client.publish("B399401C", gpsData, true);
  // client.publish("GPS", gpsData, true);
  // client.publish("GPS", gpsData);
}
void vCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  String callbackMessage;
  for (int i = 0; i < length; i++) {
    callbackMessage += (char)message[i];
  }
  // Dapatkan nilai topic STATUS
  char statusTopic[50];
  sprintf(statusTopic, "devices/%s/status", MergedMacAddress);
  if (strcmp(topic, statusTopic) == 0) {
    if (bSendStatus == 0){
      // bSendStatus = 1;
      Serial.print("Message received: ");
      Serial.println(callbackMessage);
      // Membuat objek DynamicJsonDocument dengan kapasitas yang cukup
      const size_t capacity = JSON_OBJECT_SIZE(1) + 30; // menyesuaikan kapasitas
      DynamicJsonDocument doc(capacity);
      // Mengurai (parse) pesan JSON
      DeserializationError error = deserializeJson(doc, callbackMessage);

      // Mengecek apakah ada kesalahan saat parsing
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      // Mengecek dan mendapatkan nilai user_id jika ada
      if (doc.containsKey("user_id")) {
        // Mendapatkan nilai user_id
        const char* user_id = doc["user_id"];
        // Menampilkan nilai user_id
        Serial.print("user_id: ");
        Serial.println(user_id);

        // kirimkan status alat saat ini
        vSendStatus();
      } else {
        Serial.println("user_id key not found in JSON.");
      }
    }
  }
  // Dapatkan nilai topic CONFIG
  char configTopic[50];
  sprintf(configTopic, "devices/%s/config", MergedMacAddress);
  if (strcmp(topic, configTopic) == 0) {
    if (bSendConfig == 0){
      Serial.print("Message received: ");
      Serial.println(callbackMessage);
      // Membuat objek DynamicJsonDocument dengan kapasitas yang cukup
      const size_t capacity = JSON_OBJECT_SIZE(1) + 30; // menyesuaikan kapasitas
      DynamicJsonDocument doc(capacity);
      // Mengurai (parse) pesan JSON
      DeserializationError error = deserializeJson(doc, callbackMessage);

      // Mengecek apakah ada kesalahan saat parsing
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      // Mengecek dan mendapatkan nilai key jika ada
      if (doc.containsKey("version") && doc.containsKey("calibration_battery_m") && doc.containsKey("calibration_battery_c")&& doc.containsKey("log_interval")) {
        // Mendapatkan nilai sesuai key
        const char* version = doc["version"];
        const char* calibration_battery_m = doc["calibration_battery_m"];
        const char* calibration_battery_c = doc["calibration_battery_c"];
        const char* log_interval = doc["log_interval"];
        // Menampilkan nilai key
        Serial.print("version: ");
        Serial.println(version);
        Serial.print("calibration_battery_m: ");
        Serial.println(calibration_battery_m);
        Serial.print("calibration_battery_c: ");
        Serial.println(calibration_battery_c);
        Serial.print("log_interval: ");
        Serial.println(log_interval);

        // Simpan di EEPROM

        // Jika nilai sesuai, kirimkan config alat
        vSendConfig(version, calibration_battery_m, calibration_battery_c, log_interval);
      } else {
        Serial.println("key not found in JSON.");
      }
    }
  }
  // Dapatkan nilai topic RFID
  char rfidTopic[50];
  sprintf(rfidTopic, "devices/%s/rfid", MergedMacAddress);
  if (strcmp(topic, rfidTopic) == 0) {
    Serial.print("Message received: ");
    Serial.println(callbackMessage);
  }
  // Dapatkan nilai topic UNLOCK
  char unlockTopic[50];
  sprintf(unlockTopic, "devices/%s/unlock", MergedMacAddress);
  if (strcmp(topic, unlockTopic) == 0) {
    Serial.print("Message received: ");
    Serial.println(callbackMessage);
    // Membuat objek DynamicJsonDocument dengan kapasitas yang cukup
    const size_t capacity = JSON_OBJECT_SIZE(4) + 100; // menyesuaikan kapasitas
    DynamicJsonDocument doc(capacity);
    // Mengurai (parse) pesan JSON
    DeserializationError error = deserializeJson(doc, callbackMessage);

    // Mengecek apakah ada kesalahan saat parsing
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    // Mengecek dan mendapatkan nilai permissions jika ada
    if (doc.containsKey("permissions")) {
      // Mendapatkan nilai permissions
      const char* permissions = doc["permissions"];
      const char* method = doc["method"];
      // Menampilkan nilai permissions
      Serial.print("permissions: ");
      Serial.println(permissions);
      // Menampilkan nilai method
      Serial.print("method: ");
      Serial.println(method);

      // Jika diizinkan, nyalakan buzzer 2 kali dan buka gembok
      if (strcmp(permissions, "PERMITTED") == 0) {
        bStatusKabelPutus = 0;
        Serial.print("permissions: ");
        Serial.println(permissions);
        vTwoTimeBuzzer();
        // Kirimkan UNLOCK dengan method RFID
        vSendUnlock(method);
        bStartOpen = 1;
        bMotorOpen = 1;
        bInputStatusSeal = 1;
      } else {
        vFiveTimeBuzzer();
      }
    } else {
      Serial.println("Permissions key not found in JSON.");
    }
  }
}
// void vCallback(char* topic, byte* message, unsigned int length) {
//   Serial.print("Message arrived on topic: ");
//   Serial.println(topic);
//   String callbackMessage;
//   for (int i = 0; i < length; i++) {
//     callbackMessage += (char)message[i];
//   }
//   // Dapatkan nilai topic STATUS
//   char statusTopic[50];
//   sprintf(statusTopic, "devices/%s/status", MergedMacAddress);
//   if (strcmp(topic, statusTopic) == 0) {
//     Serial.print("Message received: ");
//     Serial.println(callbackMessage);
//   }
//   // Dapatkan nilai topic CONFIG
//   char configTopic[50];
//   sprintf(configTopic, "devices/%s/config", MergedMacAddress);
//   if (strcmp(topic, configTopic) == 0) {
//     Serial.print("Message received: ");
//     Serial.println(callbackMessage);
//   }
//   // Dapatkan nilai topic RFID
//   char rfidTopic[50];
//   sprintf(rfidTopic, "devices/%s/rfid", MergedMacAddress);
//   if (strcmp(topic, rfidTopic) == 0) {
//     Serial.print("Message received: ");
//     Serial.println(callbackMessage);
//   }
//   // Dapatkan nilai topic UNLOCK
//   char unlockTopic[50];
//   sprintf(unlockTopic, "devices/%s/unlock", MergedMacAddress);
//   if (strcmp(topic, unlockTopic) == 0) {
//     Serial.print("Message received: ");
//     Serial.println(callbackMessage);
//     // Membuat objek DynamicJsonDocument dengan kapasitas yang cukup
//     const size_t capacity = JSON_OBJECT_SIZE(2) + 40; // menyesuaikan kapasitas
//     DynamicJsonDocument doc(capacity);
//     // Mendapatkan nilai permissions
//     const char* permissions = doc["permissions"];

//     // Menampilkan nilai permissions
//     Serial.print("permissions: ");
//     Serial.println(permissions);

//     // Mengurai (parse) pesan JSON
//     DeserializationError error = deserializeJson(doc, callbackMessage);
//   }
// }
void vSubsCommand(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(MergedMacAddress, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      // delay(2000);
    }
  }

  client.subscribe("COMMAND");
  Serial.print("Subscribe to: ");
  Serial.println("COMMAND");
}
void vGetEpochTime(){
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // Get epoch time
  epochTime = timeClient.getEpochTime();
  Serial.print("epochTime: ");
  Serial.println(epochTime);
}
void vIniWakeupTrigger(){
  //pin ketika di putus kabel ==> INPUT_ESEAL 39
  //pin ketika ada yang tap RFID ==> INTERRUP_RFID 26
  //pin ketika ada yang ingin mengunlock (apakah pakai button untuk menyalakan alat) ==> SPARE_BT 36
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_26,0);
}
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    Serial.print("AlatON=");
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1: 
      Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program");
      break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}
void vPinSetup(){
  pinMode(SIM800L_TRIG,OUTPUT);
  pinMode(RFID_TRIG,OUTPUT);
  pinMode(L298N_TRIG,OUTPUT);
  pinMode(GPS_TRIG,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(LED_HIJAU,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(LED_MERAH,OUTPUT);
  pinMode(INPUT_ESEAL,INPUT);
  pinMode(MOTOR, INPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_TRIG_L298N, OUTPUT);
}
void vStateSIM800l(bool Kondisi){
  if (Kondisi){
    digitalWrite(SIM800L_TRIG,HIGH);
  }
  else {
    digitalWrite(SIM800L_TRIG,LOW);
  }
}
void vStateRFID(bool Kondisi){
  if (Kondisi){
    digitalWrite(RFID_TRIG,HIGH);
  }
  else {
    digitalWrite(RFID_TRIG,LOW);
  }
}
void vStateL298N(bool Kondisi){
  if (Kondisi){
    digitalWrite(L298N_TRIG,HIGH);
  }
  else {
    digitalWrite(L298N_TRIG,LOW);
  }
}
void vOffAllTrigger(){
  digitalWrite(SIM800L_TRIG,LOW);
  digitalWrite(RFID_TRIG,LOW);
  digitalWrite(L298N_TRIG,LOW);
  digitalWrite(GPS_TRIG,LOW);
}
void vStateBUZZER(bool Kondisi){
  if (Kondisi){
    digitalWrite(BUZZER,HIGH);
  }
  else {
    digitalWrite(BUZZER,LOW);
  }  
}
void vTwoTimeBuzzer(){
  Serial.println("vTwoTimeBuzzer()");
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  delay(100);
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
}
void vFiveTimeBuzzer(){
  Serial.println("vFiveTimeBuzzer()");
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  delay(100);
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  delay(100);
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  delay(100);
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  delay(100);
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
}
void vLedUnlockSucces(){
  for (int i=0; i<3; i++){
    digitalWrite(LED_HIJAU,!digitalRead(LED_HIJAU));
    delay(1000);
  }
  digitalWrite(LED_HIJAU,LOW);
}
void vLedUnlockFailed(){
  for (int i=0; i<3; i++){
    digitalWrite(LED_MERAH,!digitalRead(LED_MERAH));
    delay(1000);
  }
  digitalWrite(LED_MERAH,LOW);
}
//----------- PWM ------------//
void vSetupMotorPWM(){
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(IN1, pwmChannel1);
  ledcAttachPin(IN2, pwmChannel2);
}
void vMotorForward(int Speed, int Delay){
  ledcWrite(pwmChannel1, Speed);
  delay(Delay);
  ledcWrite(pwmChannel1, Speed);
  digitalWrite(IN1, LOW); // stop going backward
}
void vMotorBackward(int Speed, int Delay){
  ledcWrite(pwmChannel2, Speed);
  delay(Delay);
  ledcWrite(pwmChannel2, Speed);
  digitalWrite(IN2, LOW); // stop going backward
}
//----------- MOTOR ------------//
void vSetupMotor(){
  digitalWrite(PIN_TRIG_L298N, HIGH);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}
void vMotor(){
  Serial.println("vMotor()");
  // if (MOTOR == 0) {
  //   Serial.println("Proses pembukaan gembok dimulai");
  //   bStartOpen = 1;
  // }
  // if (bStartOpen == 1){
  //   delay(500);
  //   Serial.println("Motor berhenti setelah membuka ke kanan");
  //   digitalWrite(PIN_IN1, HIGH);
  //   digitalWrite(PIN_IN2, LOW);
  //   delay(300);
  //   digitalWrite(PIN_IN1, LOW);
  //   digitalWrite(PIN_IN2, LOW);
  //   delay(3000);
  //   digitalWrite(PIN_IN1, LOW);
  //   digitalWrite(PIN_IN2, HIGH);
  //   delay(100);
  //   digitalWrite(PIN_IN1, LOW);
  //   digitalWrite(PIN_IN2, LOW);
  //   bStartOpen = 0;
  // }
  delay(500);
  Serial.println("Motor berhenti setelah membuka ke kanan");
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  delay(300);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  delay(3000);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  delay(100);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}
//----------- RFID ------------//
void vSetupRFID(){
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  Serial.println(F("This code scan the MIFARE Classsic NUID."));
  Serial.print(F("Using the following key:"));
  printHex(key.keyByte, MFRC522::MF_KEY_SIZE);
}
void getHex(byte *buffer, byte bufferSize, char *result) {
  for (byte i = 0; i < bufferSize; i++) {
    sprintf(result + (i * 2), "%02X", buffer[i]);
  }
}
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(' ');
    Serial.print(buffer[i], DEC);
  }
}
void vReadNUIDCard(){
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));
  // Check if the PICC is of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&
      piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
      piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }
  // Store the NUID
  for (byte i = 0; i < 4; i++) {
    nuidPICC[i] = rfid.uid.uidByte[i];
  }
  // Convert the NUID to a hex string
  for (byte i = 0; i < 4; i++) {
    sprintf(&hexNUID[i * 2], "%02X", nuidPICC[i]);
  }
  Serial.println(F("The NUID tag is:"));
  Serial.print(F("In hex: "));
  Serial.println(hexNUID);
  Serial.print(F("In dec: "));
  printDec(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();
  bScanNuid = 1;
  
  // Send to MQTT
  vSendRFID(String(hexNUID));
  // Halt PICC
  rfid.PICC_HaltA();
  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
}
//----------- BMS ------------//
float voltageToPercentage(float voltage) {
  if (voltage >= 3.85) {
    return 100.0;
  } else if (voltage <= 3.55) {
    return 0.0;
  } else {
    // Linearly interpolate between 3.55V and 3.85V
    return (voltage - 3.55) / (3.85 - 3.55)*100.0;
}
}
void vReadVoltageBMS() {
  unsigned long BMSADC = 0;
  unsigned long BMSOne = 0;
  float BMSADCSampling;
  float VoltageBMS;
  // Serial.print("CalBatScale : ");
  // Serial.println(CalBatScale);
  for (int i = 0; i < 10; i++) {
    BMSOne = analogRead(BMS);
    BMSADC += BMSOne;
    delay(10);
  }
  BMSADCSampling = BMSADC / 10;
  VoltageBMS = BMSADCSampling / 4095;
  VoltageBattery = 4.2 * VoltageBMS * CalBatScale;// * 0.98;//* 0.9487;* 0.9755;
  BatteryPercentage = (int)voltageToPercentage(VoltageBattery);
  // Serial.print("VoltageBattery=");
  // Serial.println(VoltageBattery);
}
//----------- GPS ------------//
void vSetupGPS(){
  Serial2.begin(9600);
}
void vGetCordinat() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()){
    LatitudeRTC = gps.location.lat();
    LongitudeRTC = gps.location.lng();
    Serial.print(F("LatitudeRTC: "));
    Serial.print(LatitudeRTC, 6);
    Serial.print(F(", LongitudeRTC: "));
    Serial.println(LongitudeRTC, 6);
    bNyalakanGPSRTC = 0;
    bWaktuCekGPSRTC = 0;
    bSudahDapatLokasiRTC = 1;
  }  
  else {
    Serial.println(F("INVALID"));
  }
}
void vStateGPS(bool Kondisi){
  if (Kondisi){
    digitalWrite(GPS_TRIG,HIGH);
    delay(1000);
  }
  else {
    digitalWrite(GPS_TRIG,LOW);
    delay(1000);
  }
} 
void vKondisiEseal(){
  Serial.println("Kondisi saat ini: ");
  Serial.print("Status = ");
  if (bInputStatusSeal) {
    Serial.println("OPEN");
  } else {
    Serial.println("CLOSE");
  }
  Serial.print("Timestamp = ");
  Serial.println(epochTime);
  Serial.print("BatPercent = ");
  Serial.println(VoltageBattery);
  Serial.print("VersionNumber = ");
  Serial.println(versionNumber);
  Serial.print("StatusLock = ");
  Serial.println(bStatusLock);
  Serial.print("StatusKabelPutus = ");
  Serial.println(bStatusKabelPutus);
}
void vCekBukaGembok() {
  if (digitalRead(INPUT_ESEAL) != bBukaGembok) {
    Serial.println("Ada perubahan!");
    bStatusLock = 1;
    bBukaGembok = digitalRead(INPUT_ESEAL);
  } else {
    Serial.println("Checking...");
    Serial.print("Status Kunci: ");
    Serial.println(digitalRead(INPUT_ESEAL));
    Serial.print("Status Force: ");
    Serial.println(bStatusKabelPutus);
    
    bStatusLock = 0;
  }
}
void vSendMessage(){
  if (bStatusLock == 1) {
    if (bInputStatusSeal == 1) {
      if (digitalRead(INPUT_ESEAL) == 1) {
        Serial.println("KUNCI TERBUKA");
        bStatusKabelPutus = 0;
        // Kirim FEEDBACK OPEN ke server
      }
    } else {
      if (digitalRead(INPUT_ESEAL) == 1) {
        Serial.println("KUNCI DIBUKA PAKSA");
        bStatusKabelPutus = 1;
        // Kirim FEEDBACK FORCED ke server
        vSendUnlock("FORCED");
      }
    }
    if (digitalRead(INPUT_ESEAL) == 0) {
      Serial.println("KUNCI TERTUTUP");
      bStatusKabelPutus = 0;
      // Kirim FEEDBACK CLOSE ke server
    }
  }
}
void vCekInfoBukaGembok(){
  if (bBukaGembok) {
    vStateL298N(1);
    vMotorForward(125,2000);
    vMotorBackward(125,2000);
    bStatusLock = 0;
    vStateL298N(0);
  }
}
void vCekKondisiBuzzer(){
  if (bStatusKabelPutus){
    vStateBUZZER(1);
  }
  else {
    vStateBUZZER(0);
  }
}
void vSleepTimer(){
  Serial.print("Sleeping for = ");
  Serial.println(TIME_SLEEP);
  esp_sleep_enable_timer_wakeup(TIME_SLEEP * 1000000);
}
void vOtaOnProcess(){
  if (bOtaOnProcess == 1){
    Serial.println("SEDANG PROSES OTA");
  } else {
    Serial.println("OTA OFF");
    // Function send message bisa dikirimkan
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("ALATON");
  vSetupWiFi();
  // print_wakeup_reason();
  vPinSetup();
  // vSetupMotorPWM();
  vSetupMotor();
  vStateSIM800l(1);
  client.setCallback(vCallback);
  vPrepareID();
  vSetupGPS();
  vSetupEpocTime();
  vKondisiEseal();
  bSendGPS = 1;
  bSendLog = 1;
  // Serial.println("MergedMacAddress: ");
  // Serial.println(MergedMacAddress);

  //========== RTOS PinnedToCore=========//
  xTaskCreatePinnedToCore(
                    Subs,   /* Task function. */
                    "Subs",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    NUIDCard,   /* Task function. */
                    "NUIDCard",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    CekKondisiBuzzer,   /* Task function. */
                    "CekKondisiBuzzer",     /* name of task. */
                    2000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task3,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    ReadVoltageBMS,   /* Task function. */
                    "ReadVoltageBMS",     /* name of task. */
                    2000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task4,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    CekBukaGembok,   /* Task function. */
                    "CekBukaGembok",     /* name of task. */
                    2000,       /* Stack size of task */
                    // 10000,       /* SEMENTARA */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &Task5,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    SendMessage,   /* Task function. */
                    "SendMessage",     /* name of task. */
                    2000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task6,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    SendLog,   /* Task function. */
                    "SendLog",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &Task7,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    SendFunctionTester,   /* Task function. */
                    "SendFunctionTester",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    3,           /* priority of the task */
                    &Task8,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    Motor,   /* Task function. */
                    "Motor",     /* name of task. */
                    2000,        /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task9,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
  xTaskCreatePinnedToCore(
                    ResetSend,   /* Task function. */
                    "ResetSend",     /* name of task. */
                    1000,        /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task10,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  
}

//========== RTOS Function=========//
void Subs( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    if (!client.connected()) {
      vSubsConfig();
      vSubsStatus();
      vSubsRFID();
      vSubsUnlock();
    }
    client.loop();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void NUIDCard( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  vStateRFID(1);
  vSetupRFID();
  int Counter5s;
  for(;;){
    // Baca RFID
    if (bScanNuid == 0) {
      vReadNUIDCard();
    } else {
      Counter5s++;
      Serial.println(Counter5s);
      if (Counter5s >= 100){
        bScanNuid = 0;
        Serial.print("bScanNuid = ");
        Serial.println(bScanNuid);
        Counter5s = 0;
      }
    }
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void CekKondisiBuzzer( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    vCekKondisiBuzzer();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void ReadVoltageBMS( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    // Baca RFID
    vReadVoltageBMS();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void CekBukaGembok( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    // Serial.print("Task1 Stack Subs = ");Serial.println(uxTaskGetStackHighWaterMark(Task1));
    // Serial.print("Task2 Stack NUIDCard = ");Serial.println(uxTaskGetStackHighWaterMark(Task2));
    // Serial.print("Task3 Stack CekKondisiBuzzer = ");Serial.println(uxTaskGetStackHighWaterMark(Task3));
    // Serial.print("Task4 Stack ReadVoltageBMS = ");Serial.println(uxTaskGetStackHighWaterMark(Task4));
    // Serial.print("Task5 Stack CekBukaGembok = ");Serial.println(uxTaskGetStackHighWaterMark(Task5));
    // Serial.print("Task6 Stack SendMessage = ");Serial.println(uxTaskGetStackHighWaterMark(Task6));
    // Serial.print("Task7 Stack SendLog = ");Serial.println(uxTaskGetStackHighWaterMark(Task7));
    vCekBukaGembok();
    int Counter10s;
    if (bInputStatusSeal == 1){
      Counter10s++;
      Serial.println(Counter10s);
      if (Counter10s >= 10){
        bInputStatusSeal = 0;
        Serial.print("bInputStatusSealNol = ");
        Serial.println(bInputStatusSeal);
        Counter10s = 0;
      }
    }

    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void SendMessage( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    vSendMessage();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void SendLog( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  int Counter;
  for(;;){
    if (bSendLog == 1){
      Counter++;
      Serial.println(Counter);
      if (Counter >= 10){
        bSendLog = 0;
        Serial.print("bSendLog = ");
        vSendLogs();
        Counter = 0;
      }
    }
    // if (bSendGPS == 1){
    //   Counter5min++;
    //   Serial.println(Counter5min);
    //   if (Counter5min >= 600){
    //     bSendGPS = 0;
    //     bSendLog = 0;
    //     Serial.print("bSendGPS = ");
    //     Serial.println(bSendGPS);
    //     vSendGPS();
    //     vSendLogs();
    //     Counter5min = 0;
    //   }
    // }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void SendFunctionTester( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  int Counter5s;
  for(;;){
      Counter5s++;
      // Serial.println(Counter5s);
      if (Counter5s >= 5){
        // vSendStatus();s
        // vSendRFID();
        // vSendUnlock();
        // vTwoTimeBuzzer();
        // vFiveTimeBuzzer();
        // bStartOpen = 1;
        // vSendUnlock("RFID");
        Counter5s = 0;
      }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void Motor( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100/ portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount ();
  int Counter;
  for(;;){
    if (bStartOpen == 1) {
      bStatusKabelPutus = 0;
      // Lampu nyala/mati untuk indikator proses motor
      // ubah jadi counter
      if (bMotorOpen == 1) {
        Counter++;
        if (Counter <= 3){
          Serial.println("Motor berhenti setelah membuka ke kanan");
          digitalWrite(PIN_IN1, HIGH);
          digitalWrite(PIN_IN2, LOW);
        } else {
          bMotorOpen = 0;
          bMotorWait = 1;
          Counter = 0;
        }
      } else if (bMotorWait == 1) {
        Counter++;
        if (Counter <= 30) {
          digitalWrite(PIN_IN1, LOW);
          digitalWrite(PIN_IN2, LOW);
        } else {
          bMotorWait = 0;
          bMotorClose = 1;
          Counter = 0;
        }
      } else if (bMotorClose) {
        Counter++;
        if (Counter <= 1) {
          digitalWrite(PIN_IN1, LOW);
          digitalWrite(PIN_IN2, HIGH);
        } else {
          bMotorClose = 0;
          Counter = 0;
          digitalWrite(PIN_IN1, LOW);
          digitalWrite(PIN_IN2, LOW);
          bStartOpen = 0;
        }
      }
      // Serial.println("Motor berhenti setelah membuka ke kanan");
      // digitalWrite(PIN_IN1, HIGH);
      // digitalWrite(PIN_IN2, LOW);
      // vTaskDelay(300);
      // digitalWrite(PIN_IN1, LOW);
      // digitalWrite(PIN_IN2, LOW);
      // vTaskDelay(3000);
      // digitalWrite(PIN_IN1, LOW);
      // digitalWrite(PIN_IN2, HIGH);
      // vTaskDelay(100);
      // digitalWrite(PIN_IN1, LOW);
      // digitalWrite(PIN_IN2, LOW);
      // bStartOpen = 0;
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
void ResetSend( void * pvParameters ){
  (void) pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000/ portTICK_PERIOD_MS;
  int WaitCounter;
  int WaitCounterConfig;
  xLastWakeTime = xTaskGetTickCount ();
  for(;;){
    if (bDoneSendStatus == 1){
      WaitCounter++;
      if (WaitCounter >= 3){
        if (bSendStatus == 1) {
          bSendStatus = 0;
          bDoneSendStatus = 0;
        }
      }
    }
    if (bDoneSendConfig == 1){
      WaitCounterConfig++;
      if (WaitCounterConfig >= 1) {
        if (bSendConfig == 1) {
          bSendConfig = 0;
          bDoneSendConfig = 0;
        }
      }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}



// int Counter1s;
// int Counter10s;
// int Counter1min;
// // unsigned long startOneMin = millis(); 
// void loop(){
//   /*
//     1. Cek kondisi Eseal
//     2. Cek apakah ada yang ingin membuka gembok
//     3. Baca RFID
//     4. jika sudah lebih dari 30s maka kembali deepsleep
//     5. sebelum deepsleep ambil lokasi tapi dengan mengaktifkankan trigger gps ketika awal menyala agar gps sudah dalam kondisi siap
//     6. dan jika lokasi berubah kirim data melalui internet
//   */

//   static uint32_t LastTime1s = 0;
//   static uint32_t LastTime1min = 0;
//   if (millis() - LastTime1s >= 1000){
//     LastTime1s = millis();
//     Counter1s++;
//     if (Counter1s >= 1){
//       vCekBukaGembok();
//       Counter1s = 0;
//     }
//     if (Counter1s >= 60){
//       Serial.println("SATUMENIT");
//       Counter1s = 0;
//     }
//     if (bInputStatusSeal == 1){
//       Counter10s++;
//       Serial.println(Counter10s);
//       if (Counter10s >= 10){
//         bInputStatusSeal = 0;
//         Serial.print("bInputStatusSealNol = ");
//         Serial.println(bInputStatusSeal);
//         Counter10s = 0;
//       }
//     }
//   }

//   // if(startOneMin - LastTime1min > 10000){
//   //   LastTime1min = startOneMin;
//   //   Counter1min++;
//   //   if (Counter1min >= 1){
//   //     Serial.println("SATUMENIT");
//   //     Counter1min = 0;
//   //   }
//   // }
  
//   // Hidupkan driver motor
//   // vCekInfoBukaGembok();

//   vCekKondisiBuzzer();
//   vReadNUIDCard();
// } 

void loop() {
  
}