#define PIN_TRIG_L298N 27
#define PIN_IN1 22
#define PIN_IN2 21

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
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_TRIG_L298N, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);
  pinMode(34, INPUT);
  pinMode(39, INPUT);
  // Mulai dengan motor dalam keadaan mati
  Serial.println("Alat ON");
  digitalWrite(PIN_TRIG_L298N, HIGH);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  // Serial.println("PUTAR KE KANAN");
  // digitalWrite(PIN_IN1, HIGH);
  // digitalWrite(PIN_IN2, LOW);
  // delay(2000);
  // Serial.println("PUTAR KE KIRI");
  // digitalWrite(PIN_IN1, LOW);
  // digitalWrite(PIN_IN2, HIGH);
  // delay(2000);
  // Serial.println("STOP");
  // digitalWrite(PIN_IN1, LOW);
  // digitalWrite(PIN_IN2, LOW);
}

void loop() {
  // unsigned long currentMillis = millis();
  static uint32_t LastTime = 0;
  if (millis() - LastTime >= 1000){
    LastTime = millis();
    Serial.println(digitalRead(39));
  }
  if (digitalRead(34) == 0) {
    Serial.println("Proses pembukaan gembok dimulai");
    StartOpen = 1;
  }
  if (StartOpen == 1){
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
    StartOpen = 0;
  }
}
