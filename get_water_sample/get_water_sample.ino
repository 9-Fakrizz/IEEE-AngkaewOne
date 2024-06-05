#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <MQTT.h>
#include <esp_log.h>  // Include the ESP-IDF logging library
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define bldcLeft 5
#define bldcRight 4
#define Ldir 12
#define Rdir 14

#define ANSWERSIZE 1

Servo ESC1;
Servo ESC2;

MPU6050 mpu6050(Wire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define a structure to hold point coordinates
struct Point {
  float x = 0;
  float y = 0;
  float x2 = 0;
  float y2 = 0;
};

struct Data {
  float angle = 0;
  float distance = 0;
};

const char ssid[] = "Ok";
const char pass[] = "q12345678";
//MQTT Server Data
const char mqtt_broker[] = "test.mosquitto.org";
const char mqtt_topic[] = "angkaewone/1";
const char mqtt_topictwo[] = "angkaewtwo/1";
const char mqtt_client_id[] = "arduino_group_x";  // must change this string to a unique value
int MQTT_PORT = 1883;

WiFiClient net;
MQTTClient client;

const int node_count = 3;
const int SLAVE_ADDR[node_count] = {4,5,6};
const int pump_pin[3] = {15,16,17};
float distance[node_count] = {};
const float k_angle = 9.0;
float angle_z = 0;
float angle_z_setup = 0;
long timer0 = 0;
int count = 0;
int count1 = 0;
int estimate_distance = 0;

// Initialize variables for circle 
Point center_circle[3] = {};
Point point[3] = {};
Point point_avg; 

char buf1[100], buf2[100], buf3[100], buf4[100], buf5[100], buf6[100];
char result1[10], result2[10], result3[10], result4[10], result5[10], result6[10];
Point coordinates[3];
int payloadCount;
int currentIntIndex = 0;
int x_point = 0, y_point = 0;

bool input = false;

void messageReceived(String &topic, String &payload) {
    // Store from payload to coordinates array
    currentIntIndex = currentIntIndex % 3;
    if (payload) {
        payload.toCharArray(buf1, 100);
        payload.toCharArray(buf2, 100);
        StrContains(buf1, "x", result1);
        StrContains(buf2, "y", result2);

        ESP_LOGI("MESSAGE", "messageReceivedCounting : %d", currentIntIndex);

        if (strcmp(result1, "Found") == 0) {
            x_point = extractNumber(payload);
            coordinates[currentIntIndex].x = x_point / 100.0;
            ESP_LOGI("MESSAGE", "Stored X: %f", coordinates[currentIntIndex].x);
        }

        if (strcmp(result2, "Found") == 0) {
            y_point = extractNumber(payload);
            coordinates[currentIntIndex].y = y_point / 100.0;
            ESP_LOGI("MESSAGE", "Stored Y: %f", coordinates[currentIntIndex].y);
        }
        currentIntIndex++;
    }
    if (currentIntIndex == 1) {
        ESP_LOGI("MESSAGE", "Array is full. Cannot store more strings.");
        client.publish(mqtt_topictwo, "FULL.");
        for (int i = 0; i < currentIntIndex; i++) {
            ESP_LOGI("MESSAGE", "point %d >> %f, %f", i, coordinates[i].x, coordinates[i].y);
        }
        // startEngine = true;
        // startCalculate = true;
        input = true;
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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable detector
  Serial.begin(9600);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Setup System");
  //Brushless setup
  pinMode(Ldir, OUTPUT);
  pinMode(Rdir, OUTPUT);
  digitalWrite(Ldir, LOW);
  digitalWrite(Rdir, LOW);
  ESC1.attach(bldcLeft, 1000, 2000);
  ESC2.attach(bldcRight, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  delay(1000);
  pinMode(pump_pin[0], OUTPUT);
  pinMode(pump_pin[1], OUTPUT);
  pinMode(pump_pin[2], OUTPUT);
  digitalWrite(pump_pin[0], LOW);
  digitalWrite(pump_pin[1], LOW);
  digitalWrite(pump_pin[2], LOW);
  WiFi.begin(ssid, pass);
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);
  connect();

  center_circle[0].x = 0.0;
  center_circle[1].x = 4.0;
  center_circle[2].x = 0.0;

  center_circle[0].y = 0.0;
  center_circle[1].y = 0.0;
  center_circle[2].y = 4.0;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setting up Angle");
  lcd.setCursor(0,1);
  lcd.print("DO NOT MOVE");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  angle_z_setup = mpu6050.getAngleZ();
  ESP_LOGI("SETUP", "angleZ Setup >> %f", angle_z_setup);
  timer0 = millis();
}

void applyThrottle(String side,int throttleValue){
  if(side == "left"){
    for(int i = 0; i < throttleValue; i++){
      ESC1.write(i);
    }
  }
  else if(side == "right"){
    for(int i = 0; i < throttleValue; i++){
      ESC2.write(i);
    }
  }
}

void default_bldc(){
  digitalWrite(Ldir,LOW);
  digitalWrite(Rdir, LOW);
  //delay(100);
}

void clockewiseOn(){
  //default_bldc();
  digitalWrite(Ldir,HIGH);
  digitalWrite(Rdir, LOW);
}
void counterclockewiseOn(){
  //default_bldc();
  digitalWrite(Ldir,LOW);
  digitalWrite(Rdir, HIGH);  
}
void loop(){
  Data data_input = get_angle_and_distance();
  Point home = check_pos_rssi();
  float targetAngle = data_input.angle;
  float targetDistance = data_input.distance;
  driving_and_check_pos(targetAngle, targetDistance, coordinates[0]);
  input = false;
  ESP_LOGI("MAIN", "Reach Target !!!!!");
  //collecting 5 sec per round
  for(int i = 0 ; i < 4 ; i++){
    collectinOn(15);
  }
  ESP_LOGI("MAIN", " Got Water Sample");
  driving_and_check_pos(180.0, targetDistance, home);
  delay(2000);
}

void driving_and_check_pos(float targetAngle, float targetDistance, Point targetPos) {
  bool startEngine = true;
  int estimate_count_1m = 10;
  mpu6050.update();
  angle_z_setup = mpu6050.getAngleZ();
  ESP_LOGI("SETUP", "angleZ Setup >> %f", angle_z_setup);
  timer0 = millis();
  while (startEngine) {
    if (millis() - timer0 >= 1000) {
      timer0 = millis();
      lcd.clear();
    }
    mpu6050.update();
    angle_z = mpu6050.getAngleZ();
    angle_z = (angle_z - angle_z_setup) * k_angle;
    ESP_LOGI("DRIVING", "angleZ : %f", angle_z);
    lcd.setCursor(0, 0);
    lcd.print("angle: ");
    lcd.print(angle_z);
    lcd.setCursor(0, 1);
    lcd.print("set angle: ");
    lcd.print(targetAngle);

    if (angle_z >= targetAngle - 2 && angle_z <= targetAngle + 2) {
      count += 1;
      ESP_LOGI("DRIVING", "IN DIRECTION %d", count);
      ESC1.write(7);
      ESC2.write(7);
      delay(100);
      ESC1.write(0);
      ESC2.write(0);
      if (count % 10 == 0) {
        ESC1.write(15);
        ESC2.write(15);
        delay(150);
        ESC1.write(0);
        ESC2.write(0);
      }
      if (count >= estimate_count_1m) { // around 1 metters
        estimate_distance += 1;
        count = 0;
        ESC1.write(0);
        ESC2.write(0);
        delay(5000);
        Point currentPoint = check_pos_rssi();
        if(abs(currentPoint.x - targetPos.x) <= 1 && abs(currentPoint.y - targetPos.y) <= 1){
          estimate_count_1m = 0;
          startEngine = false;
        }else if(abs(targetDistance - estimate_distance) < 1){
          estimate_count_1m = 0;
          startEngine = false;
        }else{
          startEngine = true;
        }
      }

    } else {
      count1 += 1;
      if (angle_z > targetAngle) { // need counterclockwise move
        applyThrottle("left", 8);
      }
      if (angle_z < targetAngle) {
        applyThrottle("right", 8);
      }
      if (count1 % 5 == 0) {
        count1 = 0;
        ESC1.write(7);
        ESC2.write(7);
        delay(100);
        ESC1.write(0);
        ESC2.write(0);
      }
    }
  } // end while loop
}

// Function to find the intersection points of two circles
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
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(distance[0]);
    lcd.print(", ");
    lcd.print(distance[1]);
    lcd.print(", ");
    lcd.print(distance[2]);
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
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(point_avg.x);
    lcd.print(",");
    lcd.print(point_avg.y);
    lcd.print(")");
    return point_avg;
}

// Function to calculate distance
float calculateDistance(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Function to calculate angle in degrees
float calculateAngle(float x1, float y1, float x2, float y2) {
  if(x2 - x1 < 0 || y2 - y1 < 0){
    float result = atan2(y2 - y1, x2 - x1) * (180.0 / PI);
    result = result/(-1.0);
    return result ;
  }
  else{
    return atan2(y2 - y1, x2 - x1) * (180.0 / PI);
  }
}

Data get_angle_and_distance(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wait for Input");
  while (!input) {
    ESP_LOGI("DATA", "waiting for input..");
    client.loop();
    delay(10);  // <- fixes some issues with WiFi stability
    if (!client.connected()) {
      connect();
    }
  }
  Data data_result;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calculating");
  Point start_point = check_pos_rssi();
  //Point start_point;
  // start_point.x = 0;
  // start_point.y = 2;
  ESP_LOGI("CALCULATION", "Start Point from RSSI : %f, %f", start_point.x, start_point.y);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println("Start Point ");
  lcd.print(start_point.x);
  lcd.print(",");
  lcd.print(start_point.y);
  float distance1 = calculateDistance(start_point.x, start_point.y, coordinates[0].x, coordinates[0].y);
  float AngletoPoint1 = calculateAngle(start_point.x, start_point.y, coordinates[0].x, coordinates[0].y);
  if(coordinates[0].y - start_point.y > 0) {
      AngletoPoint1 = (-1) * AngletoPoint1;
  }
  ESP_LOGI("CALCULATION", "angle Point 1 : %s", String(AngletoPoint1).c_str());
  ESP_LOGI("CALCULATION", "distance Point 1 : %f", distance1);
  data_result.angle = AngletoPoint1;
  data_result.distance = distance1;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calculation Success");
  return data_result;
}

void connect() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("checking wifi...");
    ESP_LOGI("CONNECT", "checking wifi...");
    lcd.setCursor(15, 0);
    while (WiFi.status() != WL_CONNECTED) {
        lcd.print(".");
        delay(1000);
    }
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("connecting...");
    ESP_LOGI("CONNECT", "connecting...");
    lcd.setCursor(15, 0);
    while (!client.connect(mqtt_client_id)) {
        lcd.print(".");
        delay(1000);
    }
    lcd.clear();
    lcd.print("connected!");
    ESP_LOGI("CONNECT", "connected!");
    delay(500);
    client.subscribe(mqtt_topic);
    // client.unsubscribe("/hello");
}

void collectinOn(int num){
  digitalWrite(num, HIGH);
  ESP_LOGI("COLLECTING", "PUMP{%d} Working",num);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Collecting.");
  delay(10000);
  digitalWrite(num, LOW);
}




