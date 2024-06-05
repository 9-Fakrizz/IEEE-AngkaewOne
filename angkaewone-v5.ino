
#include <ESP32Servo.h>
#include <MPU6050_tockn.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include <MQTT.h>
#include <math.h>

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
const char ssid[] = "Ok";
const char pass[] = "q12345678";

//MQTT Server Data
const char mqtt_broker[] = "test.mosquitto.org";
const char mqtt_topic[] = "angkaewone/1";
const char mqtt_topictwo[] = "angkaewtwo/1";
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

//start
Point start_point;
float backhome = 180;

//boolean Start
bool startEngine = false;
bool point1topoint2 = false;
bool point2topoint3 = false;
bool point3tostartEngine = false;
int timerCounting = 0;
bool startCalculate = false;
bool home = false;

char buf1[100], buf2[100], buf3[100], buf4[100], buf5[100], buf6[100];
char result1[10], result2[10], result3[10], result4[10], result5[10], result6[10];
Point coordinates[3];
int payloadCount;
int currentIntIndex = 0;

//Gyro Variable
MPU6050 mpu6050(Wire);

int constant_angle = 0;
LiquidCrystal_I2C lcd(0x27, 16, 2);
long timer = 0;

//Define for PID Using Function
int angle;
int rightPIDPosition;
int leftPIDPosition;

float extractNumber(String &str);

// Define the throttle range
const float MIN_THROTTLE = 6.0;
const float MAX_THROTTLE = 12.0;

// Function to calculate motor throttle based on angle
void calculateThrottle(float angle, float &rightThrottle, float &leftThrottle) {
  // Ensure the angle is within the range -360 to 360
  if (angle < -360) angle = -360;
  if (angle > 360) angle = 360;

  // Map the angle to throttle values
  if (angle >= 0) {
    // Positive angle: increase right motor throttle, decrease left motor throttle
    rightThrottle = map(angle, 0, 360, MIN_THROTTLE, MAX_THROTTLE);
    leftThrottle = map(angle, 0, 360, MAX_THROTTLE, MIN_THROTTLE);
  } else {
    // Negative angle: decrease right motor throttle, increase left motor throttle
    rightThrottle = map(angle, -360, 0, MAX_THROTTLE, MIN_THROTTLE);
    leftThrottle = map(angle, -360, 0, MIN_THROTTLE, MAX_THROTTLE);
  }
}

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
      coordinates[currentIntIndex].x = x_point/100.0;
      Serial.print("Stored X: ");
      Serial.println(coordinates[currentIntIndex].x);
    }

    if (strcmp(result2, "Found") == 0) {
      y_point = extractNumber(payload);
      coordinates[currentIntIndex].y = y_point/100.0;
      Serial.print("Stored Y: ");
      Serial.println(coordinates[currentIntIndex].y);
    }
    currentIntIndex++;
  }
  if (currentIntIndex == 1) {
    Serial.println("Array is full. Cannot store more strings.");
    client.publish(mqtt_topictwo, "FULL.");
    for (int i = 0; i < currentIntIndex; i++) {
      Serial.print("point ");
      Serial.print(i);
      Serial.print(" >> ");
      Serial.print(coordinates[i].x);
      Serial.print(", ");
      Serial.println(coordinates[i].y);
    }
    startEngine = true;
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

// Function to calculate distance
float calculateDistance(float x1, float y1, float x2, float y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Function to calculate angle in degrees
float calculateAngle(float x1, float y1, float x2, float y2) {
  if(x2 - x1 < 0 || y2 - y1 < 0){
    Serial.println("negativvvvvvvvvv");
    float result = atan2(y2 - y1, x2 - x1) * (180.0 / PI);
    result = result/(-1.0);
    return result ;
  }
  else{
    return atan2(y2 - y1, x2 - x1) * (180.0 / PI);
  }
}

// ------------------------------------- MAIN SET UP --------------------------------------------------//

void setup() {
  Serial.begin(15200);
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
  timer = millis();
  lcd.clear();

  start_point.x = 0.0;
  start_point.y = 0.0;

  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqtt_broker, MQTT_PORT, net);
  client.onMessage(messageReceived);

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

  lcd.clear();
  lcd.print("Waiting");
  lcd.setCursor(0, 1);
  lcd.print("For Input ");
  client.publish(mqtt_topictwo, "Ready for Input");

  while (!input) {
    Serial.println("waiting for input..");
    client.loop();
    delay(10);  // <- fixes some issues with WiFi stability

    if (!client.connected()) {
      connect();
  }
  }

  lcd.setCursor(0, 1);
  lcd.print("Calculating");
  //client.publish(mqtt_topictwo, "Calculating Angle and Distance");

  home = false;
  float distance1 = calculateDistance(start_point.x, start_point.y, coordinates[0].x, coordinates[0].y);
  float AngletoPoint1 = calculateAngle(start_point.x, start_point.y, coordinates[0].x, coordinates[0].y);
  if(coordinates[0].x - start_point.x < 0 ||coordinates[0].y - start_point.y < 0 ){
    AngletoPoint1 = (-1)*AngletoPoint1;
  }
  Serial.print("angle Point 1 : ");
  Serial.println(String(AngletoPoint1));
  Serial.print("distance Point 1 : ");
  Serial.println(distance1);

  delay(1000);
  lcd.clear();
  lcd.print("Calculatie Succes");
  //client.publish(mqtt_topictwo, "Calculating Complete");


  while (startEngine == true) {
    //PID Function Setting
    lcd.setCursor(0, 1);
    lcd.print("Point 1 : ");
    lcd.setCursor(10, 1);
    lcd.print(AngletoPoint1);
    //client.publish(mqtt_topictwo, "Navigating from start to point 1");

    rightPIDPosition = map(angle, 0, AngletoPoint1, 12, 6);
    leftPIDPosition = map(angle, AngletoPoint1, 0, 6, 12);

    if (angle < AngletoPoint1 ) {
      Serial.print("Brushless Right on at :");
      Serial.println(rightPIDPosition);
      ESC2.write(rightPIDPosition);
    }

    else if (angle >  AngletoPoint1) {
      Serial.print("Brushless Left on at :");
      Serial.println(leftPIDPosition);
      ESC1.write(leftPIDPosition);
    }

    if (angle >= AngletoPoint1 -5 && angle <= AngletoPoint1 + 5) {
      startEngine = false;
      if (angle < AngletoPoint1 ) {
        ESC1.write(8);
        delay(1000);
        ESC1.write(0);
      }

      else if (angle >  AngletoPoint1) {
        ESC2.write(8);
        delay(1000);
        ESC2.write(0);
      }
      ESC1.write(6);
      ESC2.write(6);
      delay(5000);
      ESC1.write(0);
      ESC2.write(0);
      breaked();

      // collecting water
      lcd.setCursor(0, 0);
      lcd.print("Collecting Water..");
      for(int k = 0; k < 10; k++){
        digitalWrite(pump_pin1,HIGH);
        delay(2000);
        digitalWrite(pump_pin1,LOW);
        delay(500);
      }
      lcd.clear();
      delay(1000);

      // HOME COMING 
      lcd.setCursor(0, 0);
      lcd.print("HOMING!");
      homing();
      delay(1000);
      lcd.clear();
    }
    realTimeAngle();
  }

  delay(1);
}




// ------------------------------------------------ FUNCTION ------------------------------------------------------------------- //

void homing(){
  
  while(!home){
    angle = 0;
    //PID Function Setting
    rightPIDPosition = map(angle, 0, backhome, 12, 6);
    leftPIDPosition = map(angle, backhome, 0, 6, 12);

    if (angle < backhome ) {
      Serial.print("Brushless Right on at :");
      Serial.println(rightPIDPosition);
      ESC2.write(rightPIDPosition);
    } else {
      ESC2.write(0);
    }
    
    if (angle > backhome) {
      Serial.print("Brushless Left on at :");
      Serial.println(leftPIDPosition);
      ESC1.write(leftPIDPosition);
    } else {
      ESC1.write(0);
    }

    if (angle >= backhome -5 && angle <= backhome + 5) {
      ESC1.write(6);
      ESC2.write(6);
      delay(5000);
      ESC1.write(0);
      ESC2.write(0);
      breaked();
      home = true;
    }
    realTimeAngle();
  }
}

void breaked(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  delay(20);
  ESC1.write(8);
  ESC1.write(8);
  delay(2000);
  ESC1.write(0);
  ESC1.write(0);
  delay(20);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(500);

}
void realTimeAngle() {

  mpu6050.update();
  // Serial.print("angleX : ");
  // Serial.print(mpu6050.getAngleX());
  // Serial.print("\tangleY : ");
  // Serial.print(mpu6050.getAngleY());
  if (millis() - timer > 1000) {
    timer = millis();
    lcd.clear();
  }

  //angle position
  lcd.setCursor(0, 0);
  lcd.print("angle : ");

  angle = (int)(mpu6050.getAngleZ() - constant_angle);
  angle = angle % 360;
  // if (angle < 0) {
  //   angle = 360 + angle;
  // }
  lcd.print(angle);
}
