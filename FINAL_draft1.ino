#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>  // Include ESP32Servo library instead of standard Servo library

// Motor 1 (First Motor)
#define MOTOR1_FORWARD_PIN 19   // Forward pin for motor 1 control
#define MOTOR1_BACKWARD_PIN 17  // Backward pin for motor 1 control
#define MOTOR1_PWM_PIN 4        // PWM pin for motor 1 speed control (not used now)

// Motor 2 (Second Motor)
#define MOTOR2_FORWARD_PIN 7    // Forward pin for motor 2 control
#define MOTOR2_BACKWARD_PIN 10  // Backward pin for motor 2 control
#define MOTOR2_PWM_PIN 5        // PWM pin for motor 2 speed control (not used now)

// Wi-Fi credentials for the Access Point (ESP32 network)
const char *ssid = "meam8";  // Set the SSID for your ESP32 AP network
const char *pwd = "meepmoop023";  // Set the password for your ESP32 AP network

// Wi-Fi configurations
IPAddress local_IP(192, 168, 1, 122);
IPAddress gateway_IP(192, 168, 1, 1);
IPAddress subnet_IP(255, 255, 255, 0);

// Web server on port 80
WebServer server(80);

// VL53L0X sensor (ToF sensor)
VL53L0X sensor;

// Set custom I2C pins for ESP32
int sda_pin = 8;  // GPIO8 as I2C SDA
int scl_pin = 9;  // GPIO9 as I2C SCL

// Servo motor at pin 6

#define SERVO_PIN 14  // Pin to control the servo
Servo myServo;  // Create a Servo object

// Set the delay interval for the servo movement
unsigned long lastServoMoveTime = 0;
unsigned long servoMoveInterval = 500;  // 2 seconds

// Speed limits (just for reference, no PWM used here)
#define MAX_SPEED 255  // Max motor speed (PWM value)
#define MIN_SPEED 0    // Min motor speed (PWM value)

// Desired distance thresholds in cm
#define MIN_DISTANCE 5
#define RIGHT_DISTANCE 13
#define MAX_DISTANCE 40

// Flag to control auto-follow mode
bool autoFollowEnabled = false;
bool autoservo = false;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);

  // Initialize motor control pins
  pinMode(MOTOR1_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR1_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);  // Not used now

  pinMode(MOTOR2_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR2_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);  // Not used now
  // Stop both motors initially
  stopMotor1();
  stopMotor2();

  // Set custom I2C pins
  Wire.setPins(sda_pin, scl_pin); // Set the I2C SDA and SCL pins
  Wire.begin();

  // Initialize the VL53L0X sensor
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1); // Halt if sensor not found
  }

  // Set measurement timing budget (optional, default is 200ms)
  sensor.setMeasurementTimingBudget(200000);
  Serial.println("VL53L0X sensor initialized!");

  // Start Wi-Fi Access Point (AP)
  WiFi.softAP(ssid, pwd); // Set the SSID and password for your ESP32 AP network
  WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP); // Set static IP for the AP network
  IPAddress softAP_IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(softAP_IP); // Print the AP's IP address

  // Define routes for web control
  server.on("/", HTTP_GET, []() {
    String html = "<html><body>";
    html += "<h1>Robot Control</h1>";
    html += "<p><a href='/forward'><button>Forward</button></a></p>";
    html += "<p><a href='/slowlongforward'><button>slowlongforward</button></a></p>";
    html += "<p><a href='/backward'><button>Backward</button></a></p>";
    html += "<p><a href='/left'><button>Turn Left</button></a></p>";
    html += "<p><a href='/right'><button>Turn Right</button></a></p>";
    html += "<p><a href='/stop'><button>Stop</button></a></p>";
    html += "<p><a href='/startAutoFollow'><button>Start Auto Follow</button></a></p>";
    html += "<p><a href='/stopAutoFollow'><button>Stop Auto Follow</button></a></p>";
    html += "<p><a href='/servoON'><button>ServoON</button></a></p>";
    html += "<p><a href='/servoOFF'><button>ServoOFF</button></a></p>";
    html += "<p><a href='/auto1'><button>auto1</button></a></p>";
    html += "<p><a href='/auto2'><button>auto2</button></a></p>";
    html += "<p><a href='/goup'><button>goup</button></a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  // Handle motor control requests
  server.on("/forward", HTTP_GET, []() {
    Serial.println("Forward button clicked");
    moveForward();

    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });
  server.on("/slowlongforward", HTTP_GET, []() {
    Serial.println("slowlongforward button clicked");
    moveMotor1Forward(80);
    moveMotor2Forward(65);
    delay(2000);
    stopMotors();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/backward", HTTP_GET, []() { 
    Serial.println("Backward button clicked");
    moveBackward();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/left", HTTP_GET, []() {
    Serial.println("Left button clicked");
    moveLeft();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/right", HTTP_GET, []() {
    Serial.println("Right button clicked");
    moveRight();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/stop", HTTP_GET, []() {
    Serial.println("Stop button clicked");
    stopMotors();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/startAutoFollow", HTTP_GET, []() {
    autoFollowEnabled = true;
    Serial.println("Auto-follow started");
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  server.on("/stopAutoFollow", HTTP_GET, []() {
    autoFollowEnabled = false;
    Serial.println("Auto-follow stopped");
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });

  // Servo control routes
  server.on("/servoON", HTTP_GET, []() {
    myServo.attach(SERVO_PIN);  // Attach the servo on pin 6
    Serial.println("Moving servo to 0 degrees");
    autoservo = true;
    // myServo.write(0);
    // delay(300);  
    // myServo.write(110); 
    // delay(300); 
    // myServo.write(0);
    // delay(300);  
    // myServo.write(110); 
    // delay(300); 
    Serial.println("On Moving Servo");
    server.sendHeader("Location", "/", true);
    server.send(303);
  });

  server.on("/servoOFF", HTTP_GET, []() {
    autoservo = false;
    Serial.println("Off Moving Servo");
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });
  server.on("/auto1", HTTP_GET, []() {
    Serial.println("Auto1 started");
    moveForward();
    moveForward();
    delay(200);
    stopMotors();
    moveMotor1Forward(80);
    moveMotor2Backward(65);
    delay(120);
    stopMotors();
    delay(200);
    moveForward();
    moveForward();
    moveMotor1Forward(250);
    moveMotor2Forward(170);
    delay(100);
    stopMotors();
    wallFollow();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });
  server.on("/auto2", HTTP_GET, []() {
    Serial.println("Auto2 started");
    moveForward();
    moveForward();
    delay(200);
    moveRight();
    delay(200);
    moveForward();
    moveForward();
    moveMotor1Forward(250);
    moveMotor2Forward(170);
    delay(200);
    stopMotors();
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });
  server.on("/goup", HTTP_GET, []() {
    Serial.println("Auto2 started");
    moveMotor1Forward(250);
    moveMotor2Forward(250);
    delay(700);
    moveRight();
    delay(50);
    moveMotor1Forward(250);
    moveMotor2Forward(250);
    delay(700);
    moveLeft();
    delay(50);
    moveMotor1Forward(250);
    moveMotor2Forward(250);
    delay(500);
    // moveRight();
    // delay(50);
    // moveMotor1Forward(250);
    // moveMotor2Forward(170);
    // delay(500);
    // moveLeft();
    // delay(50);
    // moveMotor1Forward(250);
    // moveMotor2Forward(170);
    // delay(500);
    stopMotors();

    //  moveRight();
    // delay(100);
    // moveMotor1Forward(250);
    // moveMotor2Forward(170);
    // delay(700);
    server.sendHeader("Location", "/", true);  // Redirect to home after action
    server.send(303);  // 303 Redirect
  });
  // Start the web server
  server.begin();
  Serial.println("Web server started");


}

void loop() {
  server.handleClient();  // Handle web server requests
  // If in auto-follow mode, follow the wall
  if (autoFollowEnabled) {
    wallFollow();
  }
  if(autoservo){
    myServo.write(0);
    delay(300);  
    myServo.write(110); 
    delay(300); 
  }

}
void servomove(){
  myServo.write(0);
  delay(300);  
  myServo.write(110); 
  delay(300); 
}
// Motor movement functions
void moveForward() {
  moveMotor1Forward(250);
  moveMotor2Forward(170);
  delay(350);
  stopMotor1();
  stopMotor2();
}

void moveBackward() {
  moveMotor1Backward(80);
  moveMotor2Backward(70);
  delay(300);
  stopMotor1();
  stopMotor2();
}

void moveLeft() {
  moveMotor1Backward(80);
  moveMotor2Forward(65);
  delay(300);
  stopMotor1();
  stopMotor2();
}

void moveRight() {
  moveMotor1Forward(80);
  moveMotor2Backward(65);
  delay(300);
  stopMotor1();
  stopMotor2();
}

void stopMotors() {
  stopMotor1();
  stopMotor2();
}

void stopMotor1() {
  digitalWrite(MOTOR1_FORWARD_PIN, LOW);
  digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
}

void stopMotor2() {
  digitalWrite(MOTOR2_FORWARD_PIN, LOW);
  digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
}

void wallFollow() {
  uint16_t distance = sensor.readRangeSingleMillimeters() / 10;

  if (sensor.timeoutOccurred()) {
    Serial.println("Sensor timeout");
    return;
  }

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= MIN_DISTANCE) {  // Too close
    moveBackward();
  } 
  else if (distance <= RIGHT_DISTANCE) {  // Close to the right wall
    moveRight();
  } 
  else if(distance>=MAX_DISTANCE){
    moveLeft();
  }
  else {  // Move forward
    moveMotor1Forward(150);
    moveMotor2Forward(110);
    delay(450);
    stopMotor1();
    stopMotor2();
  }

  delay(100);
}

// Motor 1 movement functions
void moveMotor1Forward(int speed) {
    digitalWrite(MOTOR1_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
    analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
    // Serial.print("Motor 1 moving forward at speed: ");
    Serial.println(speed);
}

void moveMotor1Backward(int speed) {
    digitalWrite(MOTOR1_FORWARD_PIN, LOW);
    digitalWrite(MOTOR1_BACKWARD_PIN, HIGH);
    analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
    // Serial.print("Motor 1 moving backward at speed: ");
    Serial.println(speed);
}

// Motor 2 movement functions
void moveMotor2Forward(int speed) {
    digitalWrite(MOTOR2_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
    analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
    // Serial.print("Motor 2 moving forward at speed: ");
    Serial.println(speed);
}

void moveMotor2Backward(int speed) {
    digitalWrite(MOTOR2_FORWARD_PIN, LOW);
    digitalWrite(MOTOR2_BACKWARD_PIN, HIGH);
    analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
    // Serial.print("Motor 2 moving backward at speed: ");
    Serial.println(speed);
}


//////////BEST WORKING 1 ////////////////
// #include <Wire.h>
// #include <VL53L0X.h>
// #include <WiFi.h>
// #include <WebServer.h>

// // Motor 1 (First Motor)
// #define MOTOR1_FORWARD_PIN 19   // Forward pin for motor 1 control
// #define MOTOR1_BACKWARD_PIN 17  // Backward pin for motor 1 control
// #define MOTOR1_PWM_PIN 4        // PWM pin for motor 1 speed control (not used now)

// // Motor 2 (Second Motor)
// #define MOTOR2_FORWARD_PIN 7    // Forward pin for motor 2 control
// #define MOTOR2_BACKWARD_PIN 10  // Backward pin for motor 2 control
// #define MOTOR2_PWM_PIN 5        // PWM pin for motor 2 speed control (not used now)

// // Wi-Fi credentials for the Access Point (ESP32 network)
// const char *ssid = "meam29";  // Set the SSID for your ESP32 AP network
// const char *pwd = "meepmoop023";  // Set the password for your ESP32 AP network

// // Wi-Fi configurations
// IPAddress local_IP(192, 168, 1, 122);
// IPAddress gateway_IP(192, 168, 1, 1);
// IPAddress subnet_IP(255, 255, 255, 0);

// // Web server on port 80
// WebServer server(80);

// // VL53L0X sensor (ToF sensor)
// VL53L0X sensor;

// // Set custom I2C pins for ESP32
// int sda_pin = 8;  // GPIO8 as I2C SDA
// int scl_pin = 9;  // GPIO9 as I2C SCL

// // Speed limits (just for reference, no PWM used here)
// #define MAX_SPEED 150  // Max motor speed (PWM value)
// #define MIN_SPEED 70   // Min motor speed (PWM value)
// // Desired distance thresholds in cm
// #define MIN_DISTANCE 5
// #define RIGHT_DISTANCE 13
// #define MAX_DISTANCE 40

// // Flag to control auto-follow mode
// bool autoFollowEnabled = false;

// void setup() {
//   // Initialize Serial for debugging
//   Serial.begin(115200);

//   // Initialize motor control pins
//   pinMode(MOTOR1_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_PWM_PIN, OUTPUT);  // Not used now

//   pinMode(MOTOR2_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_PWM_PIN, OUTPUT);  // Not used now

//   // Stop both motors initially
//   stopMotor1();
//   stopMotor2();

//   // Set custom I2C pins
//   Wire.setPins(sda_pin, scl_pin); // Set the I2C SDA and SCL pins
//   Wire.begin();

//   // Initialize the VL53L0X sensor
//   if (!sensor.init()) {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1); // Halt if sensor not found
//   }

//   // Set measurement timing budget (optional, default is 200ms)
//   sensor.setMeasurementTimingBudget(200000);
//   Serial.println("VL53L0X sensor initialized!");

//   // Start Wi-Fi Access Point (AP)
//   WiFi.softAP(ssid, pwd); // Set the SSID and password for your ESP32 AP network
//   WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP); // Set static IP for the AP network
//   IPAddress softAP_IP = WiFi.softAPIP();
//   Serial.print("AP IP Address: ");
//   Serial.println(softAP_IP); // Print the AP's IP address

//   // Define routes for web control
//   server.on("/", HTTP_GET, []() {
//     String html = "<html><body>";
//     html += "<h1>Robot Control</h1>";
//     html += "<p><a href='/forward'><button>Forward</button></a></p>";
//     html += "<p><a href='/backward'><button>Backward</button></a></p>";
//     html += "<p><a href='/left'><button>Turn Left</button></a></p>";
//     html += "<p><a href='/right'><button>Turn Right</button></a></p>";
//     html += "<p><a href='/stop'><button>Stop</button></a></p>";
//     html += "<p><a href='/startAutoFollow'><button>Start Auto Follow</button></a></p>";
//     html += "<p><a href='/stopAutoFollow'><button>Stop Auto Follow</button></a></p>";
//     html += "</body></html>";
//     server.send(200, "text/html", html);
//   });

//   // Handle motor control requests
//   server.on("/forward", HTTP_GET, []() {
//     Serial.println("Forward button clicked");
//     moveForward();
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/backward", HTTP_GET, []() {
//     Serial.println("Backward button clicked");
//     moveBackward();
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/left", HTTP_GET, []() {
//     Serial.println("Left button clicked");
//     moveLeft();
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/right", HTTP_GET, []() {
//     Serial.println("Right button clicked");
//     moveRight();
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/stop", HTTP_GET, []() {
//     Serial.println("Stop button clicked");
//     stopMotors();
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/startAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = true;
//     Serial.println("Auto-follow started");
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   server.on("/stopAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = false;
//     Serial.println("Auto-follow stopped");
//     server.sendHeader("Location", "/", true);  // Redirect to home after action
//     server.send(303);  // 303 Redirect
//   });

//   // Start the web server
//   server.begin();
//   Serial.println("Web server started");
// }

// void loop() {
//   server.handleClient();  // Handle web server requests

//   // If in auto-follow mode, follow the wall
//   if (autoFollowEnabled) {
//     wallFollow();
//   }
// }

// // Motor movement functions
// void moveForward() {
//   moveMotor1Forward(85);
//   moveMotor2Forward(65);
//   delay(350);
//   stopMotor1();
//   stopMotor2();
// }

// void moveBackward() {
//   moveMotor1Backward(85);
//   moveMotor2Backward(65);
//   delay(200);
//   stopMotor1();
//   stopMotor2();
// }

// void moveLeft() {
//   moveMotor1Backward(85);
//   moveMotor2Forward(65);
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void moveRight() {
//   moveMotor1Forward(85);
//   moveMotor2Backward(65);
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotors() {
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotor1() {
//   digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
// }

// void stopMotor2() {
//   digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
// }

// // Auto-follow logic
// void wallFollow() {
//   uint16_t distance = sensor.readRangeSingleMillimeters() / 10;

//   if (sensor.timeoutOccurred()) {
//     Serial.println("Sensor timeout");
//     return;
//   }

//   Serial.print("Distance: ");
//   Serial.println(distance);

//   if (distance <= MIN_DISTANCE) {  // Too close
//     moveBackward();
//   } 
//   else if (distance <= RIGHT_DISTANCE) {  // Close to the right wall
//     moveRight();
//   } 
//   else {  // Move forward
//     moveForward();
//   }

//   delay(100);
// }

// // Motor 1 movement functions
// void moveMotor1Forward(int speed) {
//     digitalWrite(MOTOR1_FORWARD_PIN, HIGH);
//     digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
//     analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 1 moving forward at speed: ");
//     Serial.println(speed);
// }

// void moveMotor1Backward(int speed) {
//     digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//     digitalWrite(MOTOR1_BACKWARD_PIN, HIGH);
//     analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 1 moving backward at speed: ");
//     Serial.println(speed);
// }

// // Motor 2 movement functions
// void moveMotor2Forward(int speed) {
//     digitalWrite(MOTOR2_FORWARD_PIN, HIGH);
//     digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
//     analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 2 moving forward at speed: ");
//     Serial.println(speed);
// }

// void moveMotor2Backward(int speed) {
//     digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//     digitalWrite(MOTOR2_BACKWARD_PIN, HIGH);
//     analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 2 moving backward at speed: ");
//     Serial.println(speed);
// }

////////////////////test4///////////
// #include <Wire.h>
// #include <VL53L0X.h>
// #include <WiFi.h>
// #include <WebServer.h>

// // Motor 1 (First Motor)
// #define MOTOR1_FORWARD_PIN 19   // Forward pin for motor 1 control
// #define MOTOR1_BACKWARD_PIN 17   // Backward pin for motor 1 control
// #define MOTOR1_PWM_PIN 4         // PWM pin for motor 1 speed control (not used now)

// // Motor 2 (Second Motor)
// #define MOTOR2_FORWARD_PIN 7     // Forward pin for motor 2 control
// #define MOTOR2_BACKWARD_PIN 10    // Backward pin for motor 2 control
// #define MOTOR2_PWM_PIN 5        // PWM pin for motor 2 speed control (not used now)

// // Wi-Fi credentials for the Access Point (ESP32 network)
// const char *ssid = "meam29";   // Set the SSID for your ESP32 AP network
// const char *pwd = "meepmoop023";  // Set the password for your ESP32 AP network

// // Wi-Fi configurations
// IPAddress local_IP(192, 168, 1, 122);
// IPAddress gateway_IP(192, 168, 1, 1);
// IPAddress subnet_IP(255, 255, 255, 0);

// // Web server on port 80
// WebServer server(80);

// // VL53L0X sensor
// VL53L0X sensor;

// // Set custom I2C pins for ESP32
// int sda_pin = 8; // GPIO8 as I2C SDA
// int scl_pin = 9; // GPIO9 as I2C SCL

// // Speed limits (just for reference since no PWM is being used)
// #define MAX_SPEED 255  // Max motor speed (PWM value)
// #define MIN_SPEED 0   // Min motor speed (PWM value)

// // Desired distance thresholds in cm
// #define MIN_DISTANCE 5
// #define RIGHT_DISTANCE 13
// #define MAX_DISTANCE 40

// // Flag to control auto-follow mode
// bool autoFollowEnabled = false;

// void setup() {
//   // Initialize Serial for debugging
//   Serial.begin(115200);

//   // Initialize motor control pins
//   pinMode(MOTOR1_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_PWM_PIN, OUTPUT);  // Not used now

//   pinMode(MOTOR2_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_PWM_PIN, OUTPUT);  // Not used now

//   // Stop both motors initially
//   stopMotor1();
//   stopMotor2();

//   // Set custom I2C pins
//   Wire.setPins(sda_pin, scl_pin); // Set the I2C SDA and SCL pins
//   Wire.begin();

//   // Initialize the VL53L0X sensor
//   if (!sensor.init()) {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1); // Halt if sensor not found
//   }

//   // Set measurement timing budget (optional, default is 200ms)
//   sensor.setMeasurementTimingBudget(200000);
//   Serial.println("VL53L0X sensor initialized!");

//   // Start Wi-Fi Access Point (AP)
//   WiFi.softAP(ssid, pwd); // Set the SSID and password for your ESP32 AP network
//   WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP); // Set static IP for the AP network
//   IPAddress softAP_IP = WiFi.softAPIP();
//   Serial.print("AP IP Address: ");
//   Serial.println(softAP_IP); // Print the AP's IP address

//   // Define routes for web control
//   server.on("/", HTTP_GET, []() {
//     String html = "<html><body>";
//     html += "<h1>Robot Control</h1>";
//     html += "<p><a href='/forward'><button>Forward</button></a></p>";
//     html += "<p><a href='/backward'><button>Backward</button></a></p>";
//     html += "<p><a href='/left'><button>Turn Left</button></a></p>";
//     html += "<p><a href='/right'><button>Turn Right</button></a></p>";
//     html += "<p><a href='/stop'><button>Stop</button></a></p>";
//     html += "<p><a href='/startAutoFollow'><button>Start Auto Follow</button></a></p>";
//     html += "<p><a href='/stopAutoFollow'><button>Stop Auto Follow</button></a></p>";
//     html += "</body></html>";
//     server.send(200, "text/html", html);
//   });

//   // Handle motor control requests
//   server.on("/forward", HTTP_GET, []() {
//     moveForward();
//     // server.send(200, "text/html", "Moving Forward! <a href='/'>Back</a>");
//   });

//   server.on("/backward", HTTP_GET, []() {
//     moveBackward();
//     // server.send(200, "text/html", "Moving Backward! <a href='/'>Back</a>");
//   });

//   server.on("/left", HTTP_GET, []() {
//     moveLeft();
//     // server.send(200, "text/html", "Turning Left! <a href='/'>Back</a>");
//   });

//   server.on("/right", HTTP_GET, []() {
//     moveRight();
//     // server.send(200, "text/html", "Turning Right! <a href='/'>Back</a>");
//   });

//   server.on("/stop", HTTP_GET, []() {
//     stopMotors();
//     // server.send(200, "text/html", "Motors Stopped! <a href='/'>Back</a>");
//   });

//   server.on("/startAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = true;
//     // server.send(200, "text/html", "Auto Wall Follow Started! <a href='/'>Back</a>");
//   });

//   server.on("/stopAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = false;
//     // server.send(200, "text/html", "Auto Wall Follow Stopped! <a href='/'>Back</a>");
//   });

//   // Start the web server
//   server.begin();
//   Serial.println("Web server started");
// }

// void loop() {
//   server.handleClient();  // Handle web server requests

//   // If in auto-follow mode, follow the wall
//   if (autoFollowEnabled) {
//     wallFollow();
//   }
// }

// // Motor movement functions
// void moveForward() {
//   moveMotor1Forward();
//   moveMotor2Forward();
//   delay(350);
//   stopMotor1();
//   stopMotor2();
// }

// void moveBackward() {
//   moveMotor1Backward();
//   moveMotor2Backward();
//   delay(200);
//   stopMotor1();
//   stopMotor2();
// }

// void moveLeft() {
//   moveMotor1Backward();
//   moveMotor2Forward();
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void moveRight() {
//   moveMotor1Forward();
//   moveMotor2Backward();
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotors() {
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotor1() {
//   digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
// }

// void stopMotor2() {
//   digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
// }

// // Auto-follow logic
// void wallFollow() {
//   uint16_t distance = sensor.readRangeSingleMillimeters() / 10;

//   if (sensor.timeoutOccurred()) {
//     Serial.println("Sensor timeout");
//     return;
//   }

//   Serial.print("Distance: ");
//   Serial.println(distance);

//   if (distance <= MIN_DISTANCE) {  // Too close
//     moveBackward();
//   } 
//   else if (distance <= RIGHT_DISTANCE) {  // Close to the right wall
//     moveRight();
//   } 
//   else {  // Move forward
//     moveForward();
//   }

//   delay(100);
// }

// // Motor 1 movement functions
// void moveMotor1Forward() {
//   digitalWrite(MOTOR1_FORWARD_PIN, HIGH);
//   digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
//   Serial.println("Motor 1 moving forward");
// }

// void moveMotor1Backward() {
//   digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR1_BACKWARD_PIN, HIGH);
//   Serial.println("Motor 1 moving backward");
// }

// // Motor 2 movement functions
// void moveMotor2Forward() {
//   digitalWrite(MOTOR2_FORWARD_PIN, HIGH);
//   digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
//   Serial.println("Motor 2 moving forward");
// }

// void moveMotor2Backward() {
//   digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR2_BACKWARD_PIN, HIGH);
//   Serial.println("Motor 2 moving backward");
// }

////////////////////test2///////////////
// #include <WiFi.h>
// #include "html510.h"

// // Wi-Fi credentials
// const char *ssid = "meam29";
// const char *pwd = "meepmoop023";

// // Static IP configuration
// IPAddress local_IP(192, 168, 1, 122);
// IPAddress gateway_IP(192, 168, 1, 1);
// IPAddress subnet_IP(255, 255, 255, 0);

// // Web server configuration
// HTML510Server h(80);  // Start the HTML510Server on port 80

// // Handler functions for different routes
// void handleRoot() {
//   String html = "<html><body><h1>Welcome to the Web Server</h1></body></html>";
//   h.sendhtml(html);
// }

// void handleForward() {
//   // Perform the movement logic for moving forward
//   h.sendhtml("<html><body><h1>Moving Forward</h1></body></html>");
// }

// void handleBackward() {
//   // Perform the movement logic for moving backward
//   h.sendhtml("<html><body><h1>Moving Backward</h1></body></html>");
// }

// void setup() {
//   Serial.begin(115200);

//   // Start the Wi-Fi in AP mode
//   WiFi.softAP(ssid, pwd);
//   WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP);
  
//   // Start the HTML510Server
//   h.begin(80);
  
//   // Attach handlers for the routes
//   h.attachHandler("/", handleRoot);      // Home page
//   h.attachHandler("/forward", handleForward);  // Forward movement
//   h.attachHandler("/backward", handleBackward);  // Backward movement

//   Serial.println("Server is ready!");
// }

// void loop() {
//   // Handle incoming HTTP requests
//   h.serve();
// }

///////////////////////////////test1///////////////////////
// #include <Wire.h>
// #include <VL53L0X.h>
// #include <WiFi.h>
// #include <WebServer.h>

// // Motor 1 (First Motor)
// #define MOTOR1_FORWARD_PIN 19   // Forward pin for motor 1 control
// #define MOTOR1_BACKWARD_PIN 17   // Backward pin for motor 1 control
// #define MOTOR1_PWM_PIN 4         // PWM pin for motor 1 speed control

// // Motor 2 (Second Motor)
// #define MOTOR2_FORWARD_PIN 7     // Forward pin for motor 2 control
// #define MOTOR2_BACKWARD_PIN 10    // Backward pin for motor 2 control
// #define MOTOR2_PWM_PIN 5        // PWM pin for motor 2 speed control

// //WiFiUDP UDPTestServer;
// IPAddress local_IP(192, 168, 1, 122);
// IPAddress gateway_IP(192, 168, 1, 1);
// IPAddress subnet_IP(255, 255, 255, 0);
// WiFiServer wifi_server(8080);

// // VL53L0X (ToF) sensor
// VL53L0X sensor;

// // Set custom I2C pins for ESP32-S2 (GPIO 8 for SDA, GPIO 9 for SCL)
// int sda_pin = 8; // GPIO8 as I2C SDA
// int scl_pin = 9; // GPIO9 as I2C SCL

// // Speed limits
// #define MAX_SPEED 150  // Max motor speed (PWM value)
// #define MIN_SPEED 70   // Min motor speed (PWM value)

// // Desired distance thresholds in cm
// #define MIN_DISTANCE 5
// #define RIGHT_DISTANCE 13
// #define MAX_DISTANCE 40

// // Web server on port 80
// WebServer server(80);

// // Flag to control auto-follow mode
// bool autoFollowEnabled = false;

// void setup() {
//   // Initialize Serial for debugging
//   Serial.begin(115200);
  
//   // Initialize motor control pins
//   pinMode(MOTOR1_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR1_PWM_PIN, OUTPUT);

//   pinMode(MOTOR2_FORWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_BACKWARD_PIN, OUTPUT);
//   pinMode(MOTOR2_PWM_PIN, OUTPUT);

//   // Stop both motors initially
//   stopMotor1();
//   stopMotor2();

//   // Set custom I2C pins
//   Wire.setPins(sda_pin, scl_pin); // Set the I2C SDA and SCL pins
//   Wire.begin();

//   // Initialize the VL53L0X sensor
//   if (!sensor.init()) {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1); // Infinite loop to halt if the sensor is not found
//   }

//   // Set measurement timing budget (optional, default is 200ms)
//   sensor.setMeasurementTimingBudget(200000);
//   Serial.println("VL53L0X sensor initialized!");

//   // Start Wi-Fi Access Point (AP)
//   WiFi.softAP("ESP32_Robot", "123456789"); // Set your SSID and password
//   WiFi.softAPConfig(local_IP, gateway_IP, subnet_IP);
//   Serial.print("Connecting to ");
//   //WiFi.begin();
//   IPAddress softAP_IP = WiFi.softAPIP();
//   Serial.println("WiFi connected");

//   // Define routes for web control
//   server.on("/", HTTP_GET, []() {
//     String html = "<html><body>";
//     html += "<h1>Robot Control</h1>";
//     html += "<p><a href='/forward'><button>Forward</button></a></p>";
//     html += "<p><a href='/backward'><button>Backward</button></a></p>";
//     html += "<p><a href='/left'><button>Turn Left</button></a></p>";
//     html += "<p><a href='/right'><button>Turn Right</button></a></p>";
//     html += "<p><a href='/stop'><button>Stop</button></a></p>";
//     html += "<p><a href='/startAutoFollow'><button>Start Auto Follow</button></a></p>";
//     html += "<p><a href='/stopAutoFollow'><button>Stop Auto Follow</button></a></p>";
//     html += "</body></html>";
//     server.send(200, "text/html", html);
//   });

//   // Handle motor control requests
//   server.on("/forward", HTTP_GET, []() {
//     moveForward();
//     server.send(200, "text/html", "Moving Forward! <a href='/'>Back</a>");
//   });

//   server.on("/backward", HTTP_GET, []() {
//     moveBackward();
//     server.send(200, "text/html", "Moving Backward! <a href='/'>Back</a>");
//   });

//   server.on("/left", HTTP_GET, []() {
//     moveLeft();
//     server.send(200, "text/html", "Turning Left! <a href='/'>Back</a>");
//   });

//   server.on("/right", HTTP_GET, []() {
//     moveRight();
//     server.send(200, "text/html", "Turning Right! <a href='/'>Back</a>");
//   });

//   server.on("/stop", HTTP_GET, []() {
//     stopMotors();
//     server.send(200, "text/html", "Motors Stopped! <a href='/'>Back</a>");
//   });

//   server.on("/startAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = true;
//     server.send(200, "text/html", "Auto Wall Follow Started! <a href='/'>Back</a>");
//   });

//   server.on("/stopAutoFollow", HTTP_GET, []() {
//     autoFollowEnabled = false;
//     server.send(200, "text/html", "Auto Wall Follow Stopped! <a href='/'>Back</a>");
//   });

//   // Start the web server
//   server.begin();
//   Serial.println("Web server started");
// }

// void loop() {
//   server.handleClient();  // Handle web server requests

//   // If in auto-follow mode, follow the wall
//   if (autoFollowEnabled) {
//     wallFollow();
//   }
// }

// void moveForward(  ) {
//   moveMotor1Forward(85);
//   moveMotor2Forward(65);
//   delay(350);
//   stopMotor1();
//   stopMotor2();
// }

// void moveBackward(  ) {
//   moveMotor1Backward(85);
//   moveMotor2Backward(65);
//   delay(200);
//   stopMotor1();
//   stopMotor2();
// }

// void moveLeft(  ) {
//   moveMotor1Backward(85);
//   moveMotor2Forward(65);
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void moveRight(  ) {
//   moveMotor1Forward(85);
//   moveMotor2Backward(65);
//   delay(300);
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotors() {
//   stopMotor1();
//   stopMotor2();
// }

// void stopMotor1() {
//   digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
//   analogWrite(MOTOR1_PWM_PIN, 0);
// }

// void stopMotor2() {
//   digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//   digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
//   analogWrite(MOTOR2_PWM_PIN, 0);
// }

// // Auto-follow logic
// void wallFollow() {
//   uint16_t distance = sensor.readRangeSingleMillimeters() / 10;

//   if (sensor.timeoutOccurred()) {
//     Serial.println("Sensor timeout");
//     return;
//   }

//   Serial.print("Distance: ");
//   Serial.println(distance);

//   if (distance <= MIN_DISTANCE) {  // Too close
//     moveBackward();
//   } 
//   else if (distance <= RIGHT_DISTANCE) {  // Close to the right wall
//     moveRight();
//   } 
//   else {  // Move forward
//     moveForward();
//   }

//   delay(100);
// }


// // Motor 1 movement functions
// void moveMotor1Forward(int speed) {
//     digitalWrite(MOTOR1_FORWARD_PIN, HIGH);
//     digitalWrite(MOTOR1_BACKWARD_PIN, LOW);
//     analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 1 moving forward at speed: ");
//     Serial.println(speed);
// }

// void moveMotor1Backward(int speed) {
//     digitalWrite(MOTOR1_FORWARD_PIN, LOW);
//     digitalWrite(MOTOR1_BACKWARD_PIN, HIGH);
//     analogWrite(MOTOR1_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 1 moving backward at speed: ");
//     Serial.println(speed);
// }

// // Motor 2 movement functions
// void moveMotor2Forward(int speed) {
//     digitalWrite(MOTOR2_FORWARD_PIN, HIGH);
//     digitalWrite(MOTOR2_BACKWARD_PIN, LOW);
//     analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 2 moving forward at speed: ");
//     Serial.println(speed);
// }

// void moveMotor2Backward(int speed) {
//     digitalWrite(MOTOR2_FORWARD_PIN, LOW);
//     digitalWrite(MOTOR2_BACKWARD_PIN, HIGH);
//     analogWrite(MOTOR2_PWM_PIN, constrain(speed, MIN_SPEED, MAX_SPEED));
//     Serial.print("Motor 2 moving backward at speed: ");
//     Serial.println(speed);
// }