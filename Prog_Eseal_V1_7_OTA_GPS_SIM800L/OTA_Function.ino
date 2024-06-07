//----------- OTA ---------------//
void updateFirmware() {
  HTTPClient http;
  http.begin(firmwareURL);
  Serial.println("TRY DOWNLOAD");
  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.printf("HTTP failed, error: %s\n", http.errorToString(httpCode).c_str());
    Serial.println("DOWNLOAD FAIL");
    return;
  }
  Serial.println("DOWNLOAD PROGRAM");
  int contentLen = http.getSize();
  Serial.printf("Content-Length: %d\n", contentLen);
  if (!Update.begin(contentLen)) {
    Serial.println("Not enough space to begin OTA");
    Serial.println("NOT SPACE");
    return;
  }
  Serial.println("UPDATING PROGRAM");
  WiFiClient* client = http.getStreamPtr();
  size_t written = Update.writeStream(*client);
  Serial.printf("OTA: %d/%d bytes written.\n", written, contentLen);
  if (written != contentLen) {
    Serial.println("Wrote partial binary. Giving up.");
    Serial.println("WROTE FAIL");
    return;
  }
  if (!Update.end()) {
    Serial.println("FAILED Error from Update.end(): " + String(Update.getError()));
    return;
  }
  if (Update.isFinished()) {
    Serial.println("RESTART 3s");
    Serial.println("Update successfully completed. Rebooting.");
    vTaskDelay(3000);
    ESP.restart();
  } 
  else {
    Serial.println(" FAILED Error from Update.isFinished(): " + String(Update.getError()));
    return;
  }
}