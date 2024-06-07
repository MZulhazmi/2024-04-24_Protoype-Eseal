//========== EEPROM =========//
char sizeOfByteEachListEepromMemory[6] = {
  sizeof(xAddress),                  //0
  sizeof(eepVersion),                //1
  sizeof(eepCalibration_battery_m),  //2
  sizeof(eepCalibration_battery_c),  //3
  sizeof(eepLog_interval)           //4
};
char getEepromMemory(char addressNum) {
  char memoryPoint = 0;
  for (int i = 0; i < addressNum; i++) {
    memoryPoint += sizeOfByteEachListEepromMemory[i];
  }
  return memoryPoint;
}
void vEEPROMRead() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(getEepromMemory(0), xAddress);
  if (xAddress != 125) {
    xAddress = 125;
    eepVersion = 1;
    eepCalibration_battery_m = 0;
    eepCalibration_battery_c = 0;
    eepLog_interval = 5;
    vUpxAddressEEPROM();
  } else {
    EEPROM.get(getEepromMemory(0), xAddress);
    EEPROM.get(getEepromMemory(1), eepVersion);
    EEPROM.get(getEepromMemory(2), eepCalibration_battery_m);
    EEPROM.get(getEepromMemory(3), eepCalibration_battery_c);
    EEPROM.get(getEepromMemory(4), eepLog_interval);
  }
  //Update oldData
  // Tiap data dibuat old data nya untuk membandingkan apakah ada perubahan atau tidak
  oldVersion = eepVersion;
  oldCalibration_battery_m = eepCalibration_battery_m;
  oldCalibration_battery_c = eepCalibration_battery_c;
  oldLog_interval = eepLog_interval;
  vSerialEEPROM("vEEPROMRead");
}
void vUpxAddressEEPROM() {
  EEPROM.put(getEepromMemory(0), xAddress);
  EEPROM.commit();
}
void vUpVersionEPROM() {
  EEPROM.put(getEepromMemory(1), eepVersion);
  EEPROM.commit();
}
void vUpCalibrationBatteryMEEPROM() {
  EEPROM.put(getEepromMemory(2), eepCalibration_battery_m);
  EEPROM.commit();
}
void vUpCalibrationBatteryCEEPROM() {
  EEPROM.put(getEepromMemory(3), eepCalibration_battery_c);
  EEPROM.commit();
}
void vUpLogIntervalEEPROM() {
  EEPROM.put(getEepromMemory(4), eepLog_interval);
  EEPROM.commit();
}
void vSerialEEPROM(const char *Posisi){
  Serial.println(Posisi);
  Serial.print("xAddress : ");
  Serial.print(xAddress);
  Serial.print(", eepVersion : ");
  Serial.print(eepVersion);
  Serial.print(", eepCalibration_battery_m : ");
  Serial.println(eepCalibration_battery_m);
  Serial.print(", eepCalibration_battery_c : ");
  Serial.println(eepCalibration_battery_c);
  Serial.print(", eepLog_interval : ");
  Serial.println(eepLog_interval);
}