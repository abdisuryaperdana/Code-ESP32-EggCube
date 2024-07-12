#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PID_v1.h>
#include "esp32-hal-ledc.h" // Library PWM ESP32
#include <MQTT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "wifiFix.h"
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

const char* ssid = "1st Floor";
const char* password = "shellacat";

#define DHTPIN 19  //kiri
#define DHTPIN2 15 //kanan
#define DHTPIN3 32 //tengah
#define DHTPIN4 18 //belakang
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);
DHT dht4(DHTPIN4, DHTTYPE);

#define PWM_PIN 13      // Pin PWM untuk mengontrol AC light dimmer
#define PWM_FREQ 500   // Frekuensi PWM (dalam Hertz)
#define LEDC_CHANNEL 0

// Motor Driver
int ENA = 12;
int IN1 = 5;
int IN2 = 4;

int ENB = 25;
int IN3 = 27;
int IN4 = 26;

// Relay pin
#define RELAY_PIN 16

double kp = 18.585;
double ki = 4;
double kd = 1.37;

double setpoint = 37.1;  // Setpoint suhu dalam derajat Celsius

int pwmAC = 0; 
int pwmMotor = 255;

// Inisialisasi PID
double input, outputPID;
PID pid(&input, &outputPID, &setpoint, kp, ki, kd, DIRECT);

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// Task handle untuk kontrol relay
TaskHandle_t relayTaskHandle = NULL;

const unsigned long SECOND = 1000;
const unsigned long HOUR = 3600 * SECOND;

// Length of Keycode + '\0' char
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "AbdiSuryaPerdana28"
#define AIO_KEY "aio_aEta55ViANfsTWnmw7GzN6xCzpyl"

WiFiClient* wifi = new WiFiClientFixed();
MQTTClient mqtt;

unsigned long prev_milis = millis();
bool machine_power = true; // Variabel untuk menyimpan status relay

String host = "http://eggcube.online";
String URL = host + "/index.php";

bool relay_status = false; // Variabel untuk menyimpan status relay (default: LOW)

void process();
void messageReceived(String &topic, String &payload);
void offmachine();

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nConnecting MQTT Server...");
  while (!(mqtt.connect("deviceincubator", AIO_USERNAME, AIO_KEY, false))) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  mqtt.subscribe(AIO_USERNAME "/feeds/incubator-setting");
}

// Fungsi untuk mengontrol relay
void relayControl(void *parameter) {
  for (;;) { // loop tak terbatas

    digitalWrite(RELAY_PIN, HIGH); // Set relay ke LOW
    relay_status = false;  // Update status relay menjadi LOW

    vTaskDelay(60 * SECOND / portTICK_PERIOD_MS); // Tunggu 15 menit

    digitalWrite(RELAY_PIN, LOW); // Set relay ke HIGH
    relay_status = true;  // Update status relay menjadi HIGH

    vTaskDelay(8 * SECOND / portTICK_PERIOD_MS); // Tunggu 8 detik
  }
}

void setup() {
  Serial.begin(115200);

  dht.begin();
  dht2.begin();
  dht3.begin();
  dht4.begin();

  // Setup PWM untuk mengontrol AC light dimmer
  ledcSetup(LEDC_CHANNEL, PWM_FREQ, 10);
  ledcAttachPin(PWM_PIN, LEDC_CHANNEL); 

  // Setup pin untuk Motor Driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup pin untuk Relay
  pinMode(RELAY_PIN, OUTPUT);

  // Membuat task untuk kontrol relay
  xTaskCreatePinnedToCore(
    relayControl,     /* Task function */
    "Relay Control",  /* Name of task */
    10000,            /* Stack size of task */
    NULL,             /* Parameter of the task */
    1,                /* Priority of the task */
    &relayTaskHandle, /* Task handle */
    0);               /* Core number (0 or 1) where the task should run */
  
  // Setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 1023); // Batasan output PWM

  lcd.begin(16, 2);

  WiFi.begin(ssid, password);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  mqtt.begin(AIO_SERVER, AIO_SERVERPORT, *wifi);
  mqtt.onMessage(messageReceived);

  connect();

  prev_milis = millis();
}

void loop() {
  mqtt.loop();

  if (!mqtt.connected()) {
    connect();
  }

  if (millis() - prev_milis > 20000)
  {
    if (machine_power == false)
    {
      offmachine();
    } else {
      process(); 
    }
    prev_milis = millis();
  }
}

void messageReceived(String &topic, String &payload){
  Serial.print(F("Data setting didapatkan: "));
  // Parsing JSON
  StaticJsonDocument<255> doc;

  DeserializationError error = deserializeJson(doc, payload);
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  const bool power = doc["power"].as<bool>();
  const int pwm_lampu = doc["pwm_lampu"].as<int>();
  const int pwm_kipas = doc["pwm_kipas"].as<int>();

  machine_power = (bool) power;
  pwmAC = (int) pwm_lampu;
  pwmMotor = (int) pwm_kipas;

  if (power)
  {
    pwmMotor = 255;
  }
}

void process(){ // memproses semua data
  // Baca nilai suhu dan kelembaban dari DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  float temperature2 = dht2.readTemperature();
  float humidity2 = dht2.readHumidity();
  
  float temperature3 = dht3.readTemperature();
  float humidity3 = dht3.readHumidity();
  
  float temperature4 = dht4.readTemperature();
  float humidity4 = dht4.readHumidity();

  if (isnan(temperature) || isnan(temperature2) || isnan(temperature3) || isnan(temperature4) || isnan(outputPID)) {
    // Jika ada nilai NaN, cetak pesan kesalahan dan lakukan inisialisasi ulang variabel
    Serial.println("Error: Nilai suhu atau PWM lampu NaN. Memulai ulang proses...");
    Serial.print("temperatur: ");
    Serial.println(isnan(temperature));
    Serial.print("temperatur2: ");
    Serial.println(isnan(temperature2));
    Serial.print("temperatur3: ");
    Serial.println(isnan(temperature3));
    Serial.print("temperatur4: ");
    Serial.println(isnan(temperature4));
    delay(1000);
    return; // Keluar dari loop saat ini dan mulai loop baru
  }
  // Hitung rata-rata suhu dan kelembaban
  float averageTemperature = (temperature + temperature2 + temperature3 + temperature4) / 4.0;
  float averageHumidity = (humidity + humidity2 + humidity3 + humidity4) / 4.0;

  // Update nilai input PID
  input = averageTemperature;

  // Hitung PID
  pid.Compute();

  pwmAC = outputPID;
  ledcWrite(LEDC_CHANNEL, pwmAC);

  // Set PWM untuk motor driver ke kecepatan 255
  analogWrite(ENA, pwmMotor);  // Kecepatan motor A
  analogWrite(ENB, pwmMotor);  // Kecepatan motor B
  digitalWrite(IN1, LOW);  // Motor A hidup
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Motor B hidup
  digitalWrite(IN4, HIGH);

  // Tampilkan nilai suhu dan kelembaban
  Serial.print(temperature);
  Serial.print(" ");
  Serial.print(humidity);
  Serial.print("    ");

  Serial.print(temperature2);
  Serial.print(" ");
  Serial.print(humidity2);
  Serial.print("   ");

  Serial.print(temperature3);
  Serial.print(" ");
  Serial.print(humidity3);
  Serial.print("     ");

  Serial.print(temperature4);
  Serial.print(" ");
  Serial.print(humidity4);
  Serial.print("        ");

  Serial.print(averageTemperature);
  Serial.print(" ");
  Serial.print(averageHumidity);
  Serial.print("        ");

  Serial.println(outputPID);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Suhu:");
  lcd.print(averageTemperature);


  lcd.setCursor(0, 1);
  lcd.print("Kelembapan:");
  lcd.print(averageHumidity);

  String data = "suhu="+String(averageTemperature)+"&kelembapan="+String(averageHumidity) + "&pwm_lampu="+String(outputPID) + "&pwm_kipas=" + String(pwmMotor) + "&relay_status=" + (relay_status ? "HIGH" : "LOW");
  Serial.println(data);
  HTTPClient http;
  http.begin(*wifi, URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpResponseCode = http.POST(data);
  // Mengecek response code
  if (httpResponseCode > 0) {
    // Mengambil response dari server
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  
  http.end();
}

void offmachine(){
  // Baca nilai suhu dan kelembaban dari DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  float temperature2 = dht2.readTemperature();
  float humidity2 = dht2.readHumidity();
  
  float temperature3 = dht3.readTemperature();
  float humidity3 = dht3.readHumidity();
  
  float temperature4 = dht4.readTemperature();
  float humidity4 = dht4.readHumidity();

  if (isnan(temperature) || isnan(temperature2) || isnan(temperature3) || isnan(temperature4) || isnan(outputPID)) {
    // Jika ada nilai NaN, cetak pesan kesalahan dan lakukan inisialisasi ulang variabel
    Serial.println("Error: Nilai suhu atau PWM lampu NaN. Memulai ulang proses...");
    Serial.print("temperatur: ");
    Serial.println(isnan(temperature));
    Serial.print("temperatur2: ");
    Serial.println(isnan(temperature2));
    Serial.print("temperatur3: ");
    Serial.println(isnan(temperature3));
    Serial.print("temperatur4: ");
    Serial.println(isnan(temperature4));
    return; // Keluar dari loop saat ini dan mulai loop baru
  }
  // Hitung rata-rata suhu dan kelembaban
  float averageTemperature = (temperature + temperature2 + temperature3 + temperature4) / 4.0;
  float averageHumidity = (humidity + humidity2 + humidity3 + humidity4) / 4.0;

  ledcWrite(LEDC_CHANNEL, pwmAC);

  analogWrite(ENA, pwmMotor);  // Kecepatan motor A
  analogWrite(ENB, pwmMotor);  // Kecepatan motor B
  digitalWrite(IN1, LOW);     // Motor A hidup
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);     // Motor B hidup
  digitalWrite(IN4, HIGH);

  Serial.print("pwm lampu: ");
  Serial.println(pwmAC);
  Serial.print("pwm kipas:");
  Serial.println(pwmMotor);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Suhu :");
  lcd.print(averageTemperature);
  
  lcd.setCursor(0, 1);
  lcd.print("Kelembapan :");
  lcd.print(averageHumidity);

  String data = "suhu="+String(averageTemperature)+"&kelembapan="+String(averageHumidity) + "&pwm_lampu="+String(pwmAC) + "&pwm_kipas=" + String(pwmMotor) + "&relay_status=" + (relay_status ? "HIGH" : "LOW");
  Serial.println(data);
  HTTPClient http;
  http.begin(*wifi, URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int httpResponseCode = http.POST(data);
  // Mengecek response code
  if (httpResponseCode > 0) {
    // Mengambil response dari server
    String response = http.getString();
    Serial.println("HTTP Response code: " + String(httpResponseCode));
    Serial.println("Response: " + response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  http.end();
}