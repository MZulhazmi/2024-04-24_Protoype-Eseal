// Definisikan pin untuk IN1 dan IN2 pada L298N
const int motorIn1 = 22;
const int motorIn2 = 21;

// Waktu interval untuk berganti arah (5 detik)
const unsigned long interval = 5000; 
unsigned long previousMillis = 0;
bool motorDirection = false; // false untuk satu arah, true untuk arah sebaliknya

void setup() {
  Serial.begin(115200);
  // Atur pin motor sebagai output
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Mulai dengan motor dalam keadaan mati
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  // Periksa apakah sudah waktunya untuk mengganti arah
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Ganti arah motor
    if (motorDirection) {
      Serial.println("KANAN");
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
    } else {
      Serial.println("KIRI");
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, HIGH);
    }
    // Ubah arah motor untuk iterasi berikutnya
    motorDirection = !motorDirection;
  }
}
