// Exmaple of using the MQTT library for ESP32
// Library by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt
// Modified by Arnan Sipitakiat

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL6SDv0bNVK"
#define BLYNK_TEMPLATE_NAME "ESP32Boat"
#define BLYNK_AUTH_TOKEN "iRNdF6Bvfmh28bg8a4menzf2SOeHtR3F"

#include <ESP32Servo.h>
#include <MPU6050_tockn.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include <MQTT.h>
#include <math.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <esp_log.h>  // Include the ESP-IDF logging library
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#define ANSWERSIZE 1

BlynkTimer timer;

int pinValue0;
int pinValue1;
int pinValue2;
int pinValue3;
int pinValuePump;
int pumpcount;

int angle;

#define bldc1 4
#define bldc2 5
#define in1 12
#define in2 14

// Define a structure to hold point coordinates
struct Point {
  float x = 0;
  float y = 0;
  float x2 = 0;
  float y2 = 0;
};

//define for Navigation
int AngletoPoint1 = 0;
int AngletoPoint2 = 0;
int AngletoPoint3 = 0;
int Angletoback = 0;

int distance1 = 0;
int distance2 = 0;
int distance3 = 0;
int distancetoback = 0;

//define Pump
const int pump_pin1 = 15;
const int pump_pin2 = 16;
const int pump_pin3 = 17;

long timer_bldc = 0;
int throttleValue1 = 180;
int throttleValue2 = 180;
int reverse1 = LOW;
int reverse2 = LOW;
Servo ESC1;
Servo ESC2;

//Wifi Form Phone
const char ssid[] = "OPPOReno2";
const char pass[] = "1234567890";

//MQTT Server Data
const char mqtt_broker[] = "test.mosquitto.org";
const char mqtt_topic[] = "angkaewone/1";
const char mqtt_topicAngle[] = "angkaewone/angle";
const char mqtt_topicStatus[] = "angkaewone/status";
const char mqtt_topicRSSI_X[] = "angkaewone/rssi/x";
const char mqtt_topicRSSI_Y[] = "angkaewone/rssi/y";
const char mqtt_client_id[] = "arduino_group_x";  // must change this string to a unique value
int MQTT_PORT = 1883;

int counter = 0;

WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

//messageReceived Variable coordinates
// Define the points with unique variable names
float x_start = 0, y_start = 0;
float x_point1 = 0, y_point1 = 0;
float x_point2 = 0, y_point2 = 0;
float x_point3 = 0, y_point3 = 0;

int x_point = 0, y_point = 0;

// Function to calculate distance
float calculateDistance(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Function to calculate angle in degrees
float calculateAngle(float x1, float y1, float x2, float y2) {
  return atan2(y2 - y1, x2 - x1) * (180.0 / PI);
}

//define for direction
int firstPoint;
int secondPoint;
int thirdPoint;
int backPoint;

//boolean Start
bool startEngine = false;
bool point1topoint2 = false;
bool point2topoint3 = false;
bool point3tostartEngine = false;
int timerCounting = 0;
bool startCalculate = false;

char buf1[100], buf2[100];
char result1[10], result2[10];
Point coordinates[3];
int payloadCount;
int currentIntIndex = 0;

//Gyro Variable
MPU6050 mpu6050(Wire);

int constant_angle = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2);
long timers = 0;

//define all variable rssi
const int node_count = 3;
const int SLAVE_ADDR[node_count] = {4,5,6};
float distance[node_count] = {};
Point center_circle[3] = {};
Point point[3] = {};
Point point_avg;
//end of rssi

float extractNumber(String &str);

void connect() {
  lcd.setCursor(0, 0);
  lcd.print("checking wifi...");
  Serial.print("checking wifi...");
  lcd.setCursor(15, 0);
  while (WiFi.status() != WL_CONNECTED) {
    lcd.print(".");
    delay(1000);
  }

  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("connecting...");
  Serial.print("\nconnecting...");
  lcd.setCursor(15, 0);
  while (!client.connect(mqtt_client_id)) {
    lcd.print(".");
    delay(1000);
  }

  lcd.clear();
  lcd.print("connected!");
  Serial.println("\nconnected!");

  delay(500);

  client.subscribe(mqtt_topic);
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  //store from payload to coordinates array
  currentIntIndex = currentIntIndex % 3;
  if (payload) {
    payload.toCharArray(buf1, 100);
    payload.toCharArray(buf2, 100);
    StrContains(buf1, "x", result1);
    StrContains(buf2, "y", result2);

    Serial.print("messageReceivedCounting : ");
    Serial.println(currentIntIndex);

    if (strcmp(result1, "Found") == 0) {
      x_point = extractNumber(payload);
      coordinates[currentIntIndex].x = x_point;
      Serial.print("Stored X: ");
      Serial.println(coordinates[currentIntIndex].x);
    }

    if (strcmp(result2, "Found") == 0) {
      y_point = extractNumber(payload);
      coordinates[currentIntIndex].y = y_point;
      Serial.print("Stored Y: ");
      Serial.println(coordinates[currentIntIndex].y);
    }
    currentIntIndex++;
  }
  if (currentIntIndex == 3) {
    Serial.println("Array is full. Cannot store more strings.");
    client.publish(mqtt_topicStatus, "FULL.");
    for (int i = 0; i < currentIntIndex; i++) {
      Serial.print("point ");
      Serial.print(i);
      Serial.print(" >> ");
      Serial.print(coordinates[i].x);
      Serial.print(", ");
      Serial.println(coordinates[i].y);
    }
    startCalculate = true;
  }
}

float extractNumber(String &str) {
  String numberStr = "";     // Store the extracted number as a string
  int number = 0;            // Store the extracted number as a float
  bool foundNumber = false;  // Flag to track if a number has been found
  bool hasDecimal = false;   // Flag to track if a decimal point has been found
  int charCount = 0;

  // Iterate through each character of the string
  for (int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    charCount++;

    // Check if the character is a digit or a decimal point
    if (isdigit(c) || (c == '.' && !hasDecimal)) {
      numberStr += c;      // Append digit or decimal point to the number string
      foundNumber = true;  // Set the flag indicating a number has been found
      if (c == '.') {
        hasDecimal = true;  // Set the flag indicating a decimal point has been found
      }
    } else if (foundNumber) {
      // If a number has been found and we encounter a non-digit/non-decimal character,
      // break out of the loop
      break;
    }
  }

  // Convert the number string to a Int
  number = numberStr.toInt();

  // Remove the extracted number from the original string
  if (foundNumber) {
    str.remove(0, charCount);
  }
  return number;  // Return the extracted number
}

void StrContains(char *str, const char *sfind, char *result) {
  int found = 0;
  int index = 0;
  int len = strlen(str);
  int sfindLen = strlen(sfind);

  if (sfindLen > len) {
    strcpy(result, "NotFound");
    return;
  }

  while (index < len) {
    if (str[index] == sfind[found]) {
      found++;
      if (sfindLen == found) {
        strcpy(result, "Found");
        return;
      }
    } else {
      found = 0;
    }
    index++;
  }
  strcpy(result, "NotFound");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();

  pinMode(pump_pin1, OUTPUT);
  pinMode(pump_pin2, OUTPUT);
  pinMode(pump_pin3, OUTPUT);
  digitalWrite(pump_pin1, LOW);
  digitalWrite(pump_pin2, LOW);
  digitalWrite(pump_pin3, LOW);

  //Brushless setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  startEngine = false;
  point1topoint2 = false;
  point2topoint3 = false;
  point3tostartEngine = false;
  startCalculate = false;

  lcd.setCursor(2, 0);
  lcd.print("Setup BLDC ...");
  ESC1.attach(bldc1, 1000, 2000);
  ESC2.attach(bldc2, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  delay(5000);
  throttleValue1 = 0;
  throttleValue2 = 0;

  lcd.setCursor(2, 1);
  lcd.print("Complete.");
  delay(500);
  lcd.clear();

  Wire.begin();

  mpu6050.begin();
  lcd.setCursor(0, 0);
  lcd.print("Calculating gyro");
  lcd.setCursor(0, 1);
  lcd.print("DO NOT MOVE MPU.");
  mpu6050.calcGyroOffsets(true);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Program Will  ");
  lcd.setCursor(0, 1);
  lcd.print("Start in 3 Sec..");
  delay(3000);
  lcd.clear();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  constant_angle = mpu6050.getAngleZ();
  timers = millis();
  lcd.clear();

  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);

  center_circle[0].x = 0.0;
  center_circle[1].x = 4.0;
  center_circle[2].x = 0.0;

  center_circle[0].y = 0.0;
  center_circle[1].y = 0.0;
  center_circle[2].y = 4.0;

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  connect();
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  // publish a message roughly every second.
  // not that we don't use delay() because we need to keep calling the client.loop()
  // to keep the connection alive
  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
  }

  /*lcd.clear();
  lcd.print("Waiting Until   ");
  lcd.setCursor(0, 1);
  lcd.print("For Navigating  ");*/

  ManualControl();
  //AutoControl();
  rssi_mqtt();

  realTimeAngle();
}

/*
void AutoControl(){
}
*/

void rssi_mqtt() {
  Point rahyah = check_pos_rssi();
  client.publish(mqtt_topicRSSI_X, String(rahyah.x));
  client.publish(mqtt_topicRSSI_Y, String(rahyah.y));
}

void ManualControl() {
  Blynk.run();
  if (pinValue0 == 1) {
    ESC1.write(8);
    ESC2.write(8);
  } else if (pinValue1 == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    ESC1.write(8);
    ESC2.write(8);
  } else if (pinValue2 == 1) {
    ESC1.write(0);
    ESC2.write(8);
  } else if (pinValue3 == 1) {
    ESC1.write(8);
    ESC2.write(0);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ESC1.write(0);
    ESC2.write(0);
  }

  if (pinValuePump == 1) {
    delay(300);
    if (pinValuePump == 1) {
      pumpcount = pumpcount + 1;
      Serial.print("PumpCount :");
      Serial.println(pumpcount);
      if (pumpcount == 1) {
        digitalWrite(pump_pin1, HIGH);
        Serial.println("i got 1");
        client.publish(mqtt_topicStatus, "Tube 1 is Pumping...");
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Pump 1 is workin");
        delay(5000);
        client.publish(mqtt_topicStatus, "Complete...");
      } else if (pumpcount == 2) {
        digitalWrite(pump_pin2, HIGH);
        Serial.println("change to 2");
        client.publish(mqtt_topicStatus, "Tube 2 is Pumping...");
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Pump 2 is workin");
        delay(5000);
        client.publish(mqtt_topicStatus, "Complete...");
      } else if (pumpcount == 3) {
        digitalWrite(pump_pin3, HIGH);
        Serial.println("change to 3");
        client.publish(mqtt_topicStatus, "Tube 3 is Pumping...");
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Pump 3 is workin");
        delay(5000);
        client.publish(mqtt_topicStatus, "Complete...");
      }
      lcd.clear();
    }
  }

  digitalWrite(pump_pin1, LOW);
  digitalWrite(pump_pin2, LOW);
  digitalWrite(pump_pin3, LOW);

  if (pumpcount >= 3) {
    delay(300);
    if (pumpcount >= 3) {
      Serial.print("PumpCount :");
      Serial.println(pumpcount);
      pumpcount = 0;
    }
    client.publish(mqtt_topicStatus, "Now 3 Tube is full go back to collect tube.");
  }
}

void realTimeAngle() {

  mpu6050.update();
  // Serial.print("angleX : ");
  // Serial.print(mpu6050.getAngleX());
  // Serial.print("\tangleY : ");
  // Serial.print(mpu6050.getAngleY());
  if (millis() - timers > 1000) {
    timers = millis();
    lcd.clear();
  }

  //angle position
  lcd.setCursor(0, 0);
  lcd.print("angle : ");

  angle = (int)(mpu6050.getAngleZ() - constant_angle);
  angle = angle % 360;
  if (angle < 0) {
    angle = 360 + angle;
  }
  lcd.print(angle);
  client.publish(mqtt_topicAngle, String(angle));
}

Point findCircleIntersection(Point center1, double radius1, Point center2, double radius2) {
    double d = sqrt(sq(center2.x - center1.x) + sq(center2.y - center1.y));
    Point intersection;
    if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
        ESP_LOGI("INTERSECTION", "No intersection points.");
        return intersection;;
    }
    double a = (sq(radius1) - sq(radius2) + sq(d)) / (2 * d);
    double h = sqrt(sq(radius1) - sq(a));
    intersection.x = center1.x + a * (center2.x - center1.x) / d;
    intersection.y = center1.y + a * (center2.y - center1.y) / d;
    intersection.x2 = intersection.x + h * (center2.y - center1.y) / d;
    intersection.y2 = intersection.y - h * (center2.x - center1.x) / d;
    return intersection;
}

int receive_i2c(int node_addr) {
  ESP_LOGI("I2C", "Receive data node_addr: %d", node_addr);
  // Read response from Slave---------
  Wire.requestFrom(node_addr, ANSWERSIZE);
  // Add characters to string
  String text_response = "";
  byte response[ANSWERSIZE];
  while (Wire.available()) {
    for (byte i = 0; i < ANSWERSIZE; i++) {
      response[i] = (byte)Wire.read();
    }
  }
  for (byte i = 0; i < ANSWERSIZE; i++) {
    ESP_LOGI("I2C", "%c", char(response[i]));
    text_response += (char)response[i];
  }
  ESP_LOGI("I2C", " m");
  return text_response.toFloat();
}

Point check_pos_rssi() {
    for (int i = 0; i < node_count; i++) {
        distance[i] = receive_i2c(SLAVE_ADDR[i]);
        distance[i] += 0.0;
        ESP_LOGI("RSSI", "distance %d : %f", i, distance[i]);
    }
    ESP_LOGI("RSSI", "----------------------------------");
    //lcd.clear();
    /*lcd.setCursor(0, 0);
    lcd.print(distance[0]);
    lcd.print(", ");
    lcd.print(distance[1]);
    lcd.print(", ");
    lcd.print(distance[2]);*/
    // Call the function to find intersection points
    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3; // Correcting index wrap-around
        Point intersection = findCircleIntersection(center_circle[j], distance[j], center_circle[i], distance[i]);
        point[i] = intersection;
        ESP_LOGI("INTERSECTION", "%d and %d >> %f, %f || %f, %f", i, j, point[i].x, point[i].y, point[i].x2, point[i].y2);
    }
    point_avg.x = (point[0].x2 + point[2].x2) / 2;
    point_avg.y = (point[0].y2 + point[2].y2) / 2;
    // Output the intersection point
    ESP_LOGI("INTERSECTION", "Intersection Point (x, y): %f, %f", point_avg.x, point_avg.y);
    lcd.setCursor(1, 1);
    lcd.print("(");
    lcd.print(point_avg.x);
    lcd.print(",");
    lcd.print(point_avg.y);
    lcd.print(")");
    return point_avg;
}

BLYNK_WRITE(V0) {
  pinValue0 = param.asInt();  // รับค่าจาก Blynk และเก็บในตัวแปร pinValue
  Serial.print("Forward : ");
  Serial.println(pinValue0);  // แสดงค่าที่อ่านจาก Blynk ใน Serial Monitor
}

BLYNK_WRITE(V1) {
  pinValue1 = param.asInt();  // รับค่าจาก Blynk และเก็บในตัวแปร pinValue
  Serial.print("Back : ");
  Serial.println(pinValue1);  // แสดงค่าที่อ่านจาก Blynk ใน Serial Monitor
}

BLYNK_WRITE(V2) {
  pinValue2 = param.asInt();  // รับค่าจาก Blynk และเก็บในตัวแปร pinValue
  Serial.print("Left : ");
  Serial.println(pinValue2);  // แสดงค่าที่อ่านจาก Blynk ใน Serial Monitor
}

BLYNK_WRITE(V3) {
  pinValue3 = param.asInt();  // รับค่าจาก Blynk และเก็บในตัวแปร pinValue
  Serial.print("Right : ");
  Serial.println(pinValue3);  // แสดงค่าที่อ่านจาก Blynk ใน Serial Monitor
}

BLYNK_WRITE(V4) {
  pinValuePump = param.asInt();  // รับค่าจาก Blynk และเก็บในตัวแปร pinValue
  Serial.print("Pump : ");
  Serial.println(pinValuePump);  // แสดงค่าที่อ่านจาก Blynk ใน Serial Monitor
}