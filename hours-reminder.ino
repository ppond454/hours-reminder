#include <AccelStepper.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define CONNECTION_TIMEOUT 10
#define LED 2
#define PIN_OUT 14

#define A 16
#define B 17
#define C 18
#define D 19
#define motorInterfaceType 4


// Configuration
const float stepsPerRevolution = 360 / (5.625 / 64);                   //
const float totalRevolutions = 30;                                     // Target: 30 revolutions
const float totalSteps = stepsPerRevolution * totalRevolutions * 0.5;  // Total steps
const float speedMotor = 700.00;

const char* ssid = "Weed_1g";
const char* password = "0800569812";
int tempMin = -1;
bool isPlaying = false;

AccelStepper motor(motorInterfaceType, A, B, C, D);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  Serial.begin(115200);
  setupPin();
  motor.setMaxSpeed(speedMotor);
  motor.setAcceleration(speedMotor);
  delay(1000);
  connectWifi();
}

void setupPin() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(LED, OUTPUT);

  // Setup pin motor
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
}

void connectWifi() {
  WiFi.mode(WIFI_STA);  //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");
  int timeout_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED, HIGH);
    Serial.print(".");
    delay(500);
    timeout_counter++;
    if (timeout_counter >= CONNECTION_TIMEOUT * 5) {
      ESP.restart();
    }
  }
  digitalWrite(LED, LOW);
  Serial.println("\nConnected to the WiFi network ");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
  timeClient.setTimeOffset(3600 * 7);  //GMT+07
}

void loop() {
  timeClient.update();
  int t = timeClient.getHours();

  if (t != tempMin && !isPlaying) {
    tempMin = t;
    Serial.print("time: ");
    Serial.println(tempMin);
    motor.moveTo(motor.currentPosition() + totalSteps);
  }

  if (motor.distanceToGo() != 0) {
    Serial.print("distanceToGo: ");
    Serial.println(motor.distanceToGo());
    isPlaying = true;
    motor.run();
  } else {
    stop();
    isPlaying = false;
    Serial.println("stop");
  }
}

void stop() {
  motor.stop();  // Smoothly decelerate the motor
  // Optionally, power off the motor
  digitalWrite(A, LOW);
  digitalWrite(B, LOW);
  digitalWrite(C, LOW);
  digitalWrite(D, LOW);
}
