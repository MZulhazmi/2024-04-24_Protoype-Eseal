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
#include <PubSubClient.h>

//========= Define Pin ==========//
#define SIM800L_TRIG 12
#define RFID_TRIG 14
#define L298N_TRIG 27
#define GPS_TRIG 4
#define BUZZER 25
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

//========== DEKLARASI VARIABLE =========//
// Wifi
const char* ssid = "Marnov_plus";
const char* password = "jujurdanamanah";
// MQTT
const char* MQTT_SERVER = "103.167.112.188";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "/eshield:eshield";
const char* MQTT_PASS = "eshield";
const char* PAYLOAD = "eshield";
//GPS
TinyGPSPlus gps;
//BMS
float VoltageBattery;
float CalBatScale = 1;
//I298N PWM
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 4;
const int resolution = 8;
// Mac Address
char MergedMacAddress[13];
//RFID
MFRC522 rfid(SS_RFID, RST_RFID); // Instance of the class
MFRC522::MIFARE_Key key; 
byte nuidPICC[4]; // Init array that will store new NUID 
//Sleep 
int TIME_SLEEP = 10 * 60; //menit

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
RTC_DATA_ATTR bool bBukaGembok;

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
//===================== PrepareAddres =====================//
void vPrepareID(){
  unsigned char mac[6];
  WiFi.macAddress(mac);
  snprintf(MergedMacAddress,sizeof(MergedMacAddress), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
void vSendMessage(const char* message) {
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("MARNOV", MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  // Publish a message
  Serial.print("SEND DATA: ");
  Serial.println(message);
  client.publish(PAYLOAD, message, true);
  Serial.print("Statecode: ");
  Serial.println(client.state());
}
void vCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];

    if (messageTemp == "OPEN") {
      Serial.println("Open Eshild");
      // Kirim response balikan
      char response[20];
      sprintf(response, "%s OPENED", MergedMacAddress);
      vSendMessage(response);
    }
  }
  Serial.println();
}
void vSubsOpenCondition(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("MARNOV", MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  // Subscribe to a message
  client.subscribe(PAYLOAD);
  Serial.print("Subscribe to: ");
  Serial.println(PAYLOAD);
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
  if ( !rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
  return;
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));
  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }
  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));
    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }
    Serial.println(F("The NUID tag is:"));
    Serial.print(F("In hex: "));
    printHex(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    Serial.print(F("In dec: "));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();

    // Send to MQTT
    char hexResult[30];
    memset(hexResult, 0, sizeof(hexResult));
    getHex(rfid.uid.uidByte, rfid.uid.size, hexResult);
    vSendMessage(hexResult);
  }
  else Serial.println(F("Card read previously."));
  // Halt PICC
  rfid.PICC_HaltA();
  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
}
//----------- BMS ------------//
void vReadVoltageBMS() {
  unsigned long BMSADC = 0;
  unsigned long BMSOne = 0;
  float BMSADCSampling;
  float VoltageBMS;
  Serial.print("CalBatScale : ");
  Serial.println(CalBatScale);
  for (int i = 0; i < 10; i++) {
    BMSOne = analogRead(BMS);
    BMSADC += BMSOne;
    delay(10);
  }
  BMSADCSampling = BMSADC / 10;
  VoltageBMS = BMSADCSampling / 4095;
  VoltageBattery = 4.2 * VoltageBMS * CalBatScale;// * 0.98;//* 0.9487;* 0.9755;
  Serial.print("VoltageBattery=");
  Serial.println(VoltageBattery);
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
//----------- Input Syarat ------------//
void vCekKondisiESeal(){
  if (bInputStatusSeal){
    //Open
    if(bStatusLock){ //harusnya sedang terkunci tetapi malah terbuka
      //Kabel Ada yang memutus
      bStatusKabelPutus = 1;
    }
  }
  else {
    //Lock
    delay(2000);
    bStatusLock = 1;
    bStatusKabelPutus = 0;
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
//----------- MQTT Wifi ------------//

void vSleepTimer(){
  Serial.print("Sleeping for = ");
  Serial.println(TIME_SLEEP);
  esp_sleep_enable_timer_wakeup(TIME_SLEEP * 1000000);
}
void setup() {
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println("ALATON");
  vSetupWiFi();
  print_wakeup_reason();
  vPinSetup();
  vSetupRFID();
  vSetupMotorPWM();
  vStateSIM800l(1);
  vStateRFID(1);
  client.setCallback(vCallback);
  vPrepareID();
}
void loop(){
  /*
    1. Cek kondisi Eseal
    2. Cek apakah ada yang ingin membuka gembok
    3. Baca RFID
    4. jika sudah lebih dari 30s maka kembali deepsleep
    5. sebelum deepsleep ambil lokasi tapi dengan mengaktifkankan trigger gps ketika awal menyala agar gps sudah dalam kondisi siap
    6. dan jika lokasi berubah kirim data melalui internet
  */
  vCekKondisiESeal();
  vCekInfoBukaGembok();
  vCekKondisiBuzzer();
  vReadNUIDCard();

  if (!client.connected()) {
    vSubsOpenCondition();
  }
  client.loop();
} 