#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Stepper.h>

#define CONNECTION_TIMEOUT 10
#define LED 2
#define PIN_OUT 14

#define A 16
#define B 17
#define C 18
#define D 19

Stepper motor(32, A, B, C, D);

// Configuration
const int stepsPerRevolution = 360 / 5.625 / 64;               // 360 degrees / (5.625 / 64)
const int totalRevolutions = 30;                               // Target: 30 revolutions
const int totalSteps = stepsPerRevolution * totalRevolutions;  // Total steps

const char* ssid = "Weed_1g";
const char* password = "0800569812";
const int ROUND = 30;
const int motorSpeed = 100;
int tempMin = -1;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  Serial.begin(115200);
  setupPin();
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
  int t = timeClient.getMinutes();
  motor.setSpeed(700);
  motor.step(totalSteps);
}
