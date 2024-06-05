#include <MPU6050_tockn.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

#define bldcLeft 5
#define bldcRight 4
#define Ldir 12
#define Rdir 14

#define maxThrottle 10
#define minThrottle 5

Servo ESC1;
Servo ESC2;

MPU6050 mpu6050(Wire);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const float k_angle = 9.0;
float angle_z = 0;
float angle_z_setup = 0;
float tagetAngle = 50;

long timer0 = 0;
int count = 0;
int count1 = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
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
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  angle_z_setup = mpu6050.getAngleZ();
  Serial.print("\tangleZ Setup >> ");
  Serial.println(angle_z_setup);
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

void loop() {
  if(millis() - timer0 >= 1000){
    timer0 = millis();
    lcd.clear();
  }
  mpu6050.update();
  angle_z = mpu6050.getAngleZ();
  angle_z = (angle_z - angle_z_setup) * k_angle;
  Serial.print("\tangleZ : ");
  Serial.println(angle_z);
  lcd.setCursor(0,0);
  lcd.print("angle: ");
  lcd.print(angle_z);
  lcd.setCursor(0,1);
  lcd.print("set angle: ");
  lcd.print(tagetAngle);

  if(angle_z >= tagetAngle-6 && angle_z <= tagetAngle +6 ){
    count += 1;
    Serial.println("IN DIRECTION " + String(count));
    ESC1.write(7);
    ESC2.write(7);
    delay(100);
    ESC1.write(0);
    ESC2.write(0);
    if(count % 10 == 0){
      ESC1.write(15);
      ESC2.write(15);
      delay(150);
      ESC1.write(0);
      ESC2.write(0);
    }
    if(count >= 200){
      count = 0;
      ESC1.write(0);
      ESC2.write(0);
      delay(5000);
    }
  }
  else{
    count1 += 1;
    if(angle_z > tagetAngle){ // need counterclockwise move
      applyThrottle("left",8);
    }
    if(angle_z < tagetAngle){
      applyThrottle("right",8);
    }
    if(count1 % 5 == 0){
      count1 = 0;
      ESC1.write(7);
      ESC2.write(7);
      delay(100);
      ESC1.write(0);
      ESC2.write(0);
    }
  }
}