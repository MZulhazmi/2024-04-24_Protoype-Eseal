const int analogPin = A0;  // Pin analog untuk membaca tegangan
unsigned long previousMillis = 0;  // Menyimpan waktu terakhir pembacaan
const long interval = 50;  // Interval pembacaan dalam milidetik

void setup() {
  Serial.begin(9600);      // Inisialisasi komunikasi serial
}

void loop() {
  // Dapatkan waktu saat ini
  unsigned long currentMillis = millis();
  // Periksa apakah interval waktu telah berlalu
  if (currentMillis - previousMillis >= interval) {
    // Simpan waktu terakhir pembacaan
    previousMillis = currentMillis;
    // Baca nilai tegangan dari pin analog
    int sensorValue = analogRead(analogPin);
    // Konversi nilai sensor (0-1023) ke tegangan (0-5V)
    float voltage = sensorValue * (5.0 / 1023.0);
    // Tampilkan nilai tegangan di Serial Plotter
    Serial.println(voltage);
  }
}
