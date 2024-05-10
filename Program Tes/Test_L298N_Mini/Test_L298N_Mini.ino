#define IN1 22
#define IN2 21

int motor1Pin1 = 22; 
int motor1Pin2 = 21; 
int enable1Pin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int pwmChannel2 = 4;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  pinMode(motor1Pin1, OUTPUT); //set pins to output
  pinMode(motor1Pin2, OUTPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(motor1Pin1, pwmChannel);
  ledcAttachPin(motor1Pin2, pwmChannel2);
}
void loop() {
  digitalWrite(IN1, HIGH); // go forward at max speed
  delay(3000); // for 3 seconds
  digitalWrite(IN1, LOW); // stop going forward

  digitalWrite(IN2, HIGH); // go backward at max speed
  delay(3000); // for 3 seconds
  digitalWrite(IN2, LOW); // stop going backward

  ledcWrite(pwmChannel, dutyCycle);  
  // analogWrite(IN1, 128); // go forward at half speed, 0 to 2IN1IN
}