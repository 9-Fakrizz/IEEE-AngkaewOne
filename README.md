# 🤖 IEEE AngkaewOne – Water Sampling Robot

**Author:** Supakrit Kongkham

**Student ID:** 650610858

**Faculty of Engineering, Chiang Mai University**


---

## 📦 Project Overview

**AngkaewOne** is a simple **line following robot** prototype built for the IEEE Chiang Mai University competition. It uses infrared (IR) sensors to detect line deviation and adjusts motor speeds to maintain its path.

This repository includes:

* Arduino code for motor control and sensor reading
* Basic PID logic implementation
* Wiring diagram for hardware setup

---

## ✨ Features

* ✅ Follows a black line on white surface using IR sensors
* ✅ Adjustable motor speed via PWM
* ✅ Smooth turns and real-time correction
* ✅ Compatible with most low-cost chassis kits

---

## ⚙️ Hardware Requirements

* Arduino Uno / Nano
* 5 IR Reflective Sensors (QRE1113 or equivalent)
* L298N Motor Driver
* 2 DC Motors
* Power supply (Battery Pack 6V–12V)
* Chassis, wheels, and caster

---

## 🧠 Code Logic

The robot uses five IR sensors positioned like this:

```
[ S1 | S2 | S3 | S4 | S5 ]
```

Each sensor returns digital values (0 for white, 1 for black).
Based on which sensors detect the line, the code adjusts the left and right motor speeds.

### Example Logic (simplified):

```cpp
if (S3 == 1) {
    // Centered — go forward
}
else if (S2 == 1 || S1 == 1) {
    // Line is on the left — turn left
}
else if (S4 == 1 || S5 == 1) {
    // Line is on the right — turn right
}
else {
    // Line lost — stop or search
}
```

You can further improve this with PID control to smoothen turns and improve accuracy.

---

## 🛠️ Getting Started

### 1. Clone this Repository

```bash
git clone https://github.com/9-Fakrizz/IEEE-AngkaewOne.git
```

### 2. Open the Code

Use the Arduino IDE to open and upload `line_follower.ino` to your Arduino board.

### 3. Wiring

> *Refer to the included wiring diagram or image (if available).*

Make sure motor driver, sensors, and power lines are properly connected.

---


Also, if you want to expand the robot with features like Bluetooth control or obstacle avoidance, I can help with that too.
