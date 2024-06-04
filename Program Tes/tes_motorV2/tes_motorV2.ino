// Definisikan pin untuk IN1 dan IN2 pada L298N
const int motorIn1 = 22;
const int motorIn2 = 21;

// Waktu untuk menjalankan motor dan interval menunggu
const unsigned long motorRunTime = 300; // Motor berputar selama 300 ms
const unsigned long interval = 5000;    // Interval 5 detik
unsigned long previousMillis = 0;
int StartOpen = 0;  // Variabel untuk menandakan proses pembukaan
bool motorRunning = false;
bool waitingPeriod = false;

void setup() {
  Serial.begin(115200);
  // Atur pin motor sebagai output
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(0, INPUT);
  // Mulai dengan motor dalam keadaan mati
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    // delay(200);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  if (digitalRead(0) == 0) {
      Serial.println("Proses pembukaan gembok dimulai");
      StartOpen = 1;
      
  }
  if (StartOpen == 1){
    Serial.println("Motor berhenti setelah membuka ke kanan");
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    delay(1000);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    delay(5000);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);
    delay(200);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    StartOpen = 0;
  }
}
