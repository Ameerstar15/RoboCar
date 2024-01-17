// Arduino IR Sensor Code
/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-rfid-nfc
 */

#include <SPI.h>
#include <MFRC522.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <iostream>

#define R_IRSensor  2  
#define L_IRSensor 15
#define motor1EN  25
#define motor1pin1  33  // green forward
#define motor1pin2  32  //yellow backward
#define motor2EN 26
#define motor2pin1  27  //green forward
#define motor2pin2  14  // yellow backward



unsigned long startTime = 0;
const unsigned long waitTime = 5000; // 5 seconds in milliseconds
bool cardDetected = false;
// change SSID to the network you want to use
const char* ssid = "";
// Passord of your network 
const char* password = "";
const char* mqttServer = "hairdresser.cloudmqtt.com";
const char* topic = "RoboCar";
const int mqttPort = 15727;
const char* mqttUser = "iqobbxec";
const char* mqttPassword = "2J36PpURz5Zv";
WiFiClient espClient;
PubSubClient client(espClient);

char msg[20];
char uidString[18]; // Array to store the UID as a string (16 characters + '\0')

bool waitingForNFC = false; // Flag to track if waiting for NFC card
unsigned long nfcStartTime = 0; // Time when waiting for NFC card started
unsigned long end = millis() - nfcStartTime;
int SS_PIN = 5;  // ESP32 pin GPIO5
int RST_PIN = 0 ;// ESP32 pin GPIO27 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle0 = 160; 
int dutyCycle1 = 160; 


MFRC522 rfid(SS_PIN, RST_PIN);

void forward() {
  Serial.println("Forward");

  digitalWrite(motor1pin1, HIGH); // Right Motor forward Pin
  digitalWrite(motor1pin2, LOW);  // Right Motor backward Pin
  digitalWrite(motor2pin1, HIGH); // Left Motor backward Pin
  digitalWrite(motor2pin2, LOW);  // Left Motor forward Pin
  analogWrite(motor1EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(motor2EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
  delay(50);
  analogWrite(motor1EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(motor2EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed

  int targetDutyCycle0 = 200; // Set the target duty cycle for motor 1 (starting from 0)
  int targetDutyCycle1 = 200; // Set the target duty cycle for motor 2 (starting from 0) 
  // Gradually increase the duty cycle to the target value
  //if (dutyCycle0 <= 200 || dutyCycle1 <= 200) {
    
      //ledcWrite(pwmChannel0, 170); // Update motor 1 duty cycle
      //dutyCycle0 += 10;
      //ledcWrite(pwmChannel1, 180); // Update motor 2 duty cycle
      //dutyCycle1 += 10;
    

    Serial.print("Duty cycle 0: ");
    Serial.println(dutyCycle0);
    Serial.print("Duty cycle 1: ");
    Serial.println(dutyCycle1);
    //delay(500);
 // }
}



void turnRight(){ //turnRight
printf("Right \n");

digitalWrite(motor1pin1, LOW);  //Right Motor forword Pin 
digitalWrite(motor1pin2, HIGH); //Right Motor backword Pin 
digitalWrite(motor2pin1, HIGH);  //Left Motor forward Pin 
digitalWrite(motor2pin2, LOW); //Left Motor backward Pin 
analogWrite(motor1EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(motor2EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
delay(100);
analogWrite(motor1EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(motor2EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed

}

void turnLeft(){ //turnLeft
printf("Left \n");
/*while (160 < dutyCycle1 <= 200){
    ledcWrite(pwmChannel0, dutyCycle1);   
    Serial.print("Forward with duty cycle: ");
    Serial.println(dutyCycle1);
    dutyCycle1 = dutyCycle1 - 5;
    delay(500);
} */
digitalWrite(motor1pin1, HIGH); //Right Motor forword Pin 
digitalWrite(motor1pin2, LOW);  //Right Motor backword Pin 
digitalWrite(motor2pin1, LOW); //Left Motor backword Pin 
digitalWrite(motor2pin2, HIGH);  //Left Motor forword Pin <aw2xsd
 analogWrite(motor1EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(motor2EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
  delay(100);
  analogWrite(motor1EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(motor2EN, 60); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed

}

void Stop() {
  Serial.println("Stop");
  /*int targetDutyCycle0 = 170; // Set the target duty cycle for motor 1 (starting from 0)
  int targetDutyCycle1 = 170; // Set the target duty cycle for motor 2 (starting from 0)

  // Gradually reduce the duty cycle to the target value
  // Gradually increase the duty cycle to the target value
  if (dutyCycle0 > 160 || dutyCycle1 > 160) {
    
      ledcWrite(pwmChannel0, dutyCycle0); // Update motor 1 duty cycle
      dutyCycle0 -= 10;
      ledcWrite(pwmChannel1, dutyCycle1); // Update motor 2 duty cycle
      dutyCycle1 -= 10;
    

    Serial.print("Duty cycle 0: ");
    Serial.println(dutyCycle0);
    Serial.print("Duty cycle 1: ");
    Serial.println(dutyCycle1);
    delay(500);
  }*/

  // Turn off both motors
  //if (dutyCycle0 <= 170 && dutyCycle1 <= 170)
  {
  digitalWrite(motor1pin1, LOW); // Right Motor forward Pin
  digitalWrite(motor1pin2, LOW); // Right Motor backward Pin
  digitalWrite(motor2pin1, LOW); // Left Motor backward Pin
  digitalWrite(motor2pin2, LOW); // Left Motor forward Pin
  }
}



/*
void Forward()
{


  analogWrite(motor1EN, 0);
  analogWrite(motor2EN, 0);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void Stop()
{
  /*digitalWrite(motor1EN, 0);
  digitalWrite(motor2EN, 0);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);

}

void TurnAround()
{
  analogWrite(motor1EN, -255);
  analogWrite(motor2EN, -255);

  digitalWrite(motor1pin1, HIGH);
  delay(300);
  digitalWrite(motor1pin1, LOW);
}


void Acc()
{
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  
  for (int i = 0; i <= 255; i++) {
    ledcWrite(pwmChannel0, i); // Increase duty cycle for motor 1
    delay(20);
  }
  
  for (int i = 0; i <= 255; i++) {
    ledcWrite(pwmChannel1, i); // Increase duty cycle for motor 2
    delay(20);
  }
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}*/


void callback(char* topic, byte* payload, unsigned int length) {
int analogValue = analogRead(32);
Serial.print("Analog value: ");
Serial.println(analogValue);
Serial.print("Message arrived in topic: ");
Serial.println(topic);
Serial.print("Message:");
String message;
for (int i = 0; i < length; i++) {
message = message + (char) payload[i]; // convert *byte to string
}
Serial.print(message);

Serial.println();
Serial.println("-----------------------");
}


void setup() {
  
  pinMode(R_IRSensor, INPUT);
  pinMode(L_IRSensor, INPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1,   OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1EN,   OUTPUT); 
  pinMode(motor2EN, OUTPUT);

  // configure LED PWM functionalitites
  /*ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motor1EN, pwmChannel0);
  ledcAttachPin(motor2EN, pwmChannel1);*/
  analogWrite(motor1EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(motor2EN, 100); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed

  Serial.begin(115200); // Init Serila at 115200 Baud

  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
  delay(500);
  Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  //connecting to a mqtt broker
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) 
  {
    Serial.println("Connecting to mqtt broker.....");
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) 
    {
      Serial.println("mqtt broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  // publish and subscribe
  client.publish(topic, "RoboCar online!");
  client.subscribe(topic);

  SPI.begin(); // init SPI bus
  rfid.PCD_Init(); // init MFRC522

  Serial.println("Tap an RFID/NFC tag on the RFID-RC522 reader");
}



void loop() {

  int R_sensorStatus = digitalRead(R_IRSensor); // Set the GPIO as Input
  int L_sensorStatus =digitalRead(L_IRSensor);
  printf("R : %d    L : %d\n",R_sensorStatus,L_sensorStatus);

   //if Right Sensor and Left Sensor are at White color then it will call forword function
  if((digitalRead(R_IRSensor) == 0)&&(digitalRead(L_IRSensor) == 0)){forward();}  

  //if Right Sensor is Black and Left Sensor is White then it will call turn Right function
  if((digitalRead(R_IRSensor) == 1)&&(digitalRead(L_IRSensor) == 0)){turnLeft();}   

  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
  if((digitalRead(R_IRSensor) == 0)&&(digitalRead(L_IRSensor) == 1)){turnRight();}  

  //if Right Sensor and Left Sensor are at Black color then it will call Stop function
  if((digitalRead(R_IRSensor) == 1)&&(digitalRead(L_IRSensor) == 1))
  {

    Stop();
    

    if (rfid.PICC_IsNewCardPresent())
    { // new tag is available
      if (rfid.PICC_ReadCardSerial())
      { // NUID has been readed

        MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
        Serial.print("RFID/NFC Tag Type: ");
        Serial.println(rfid.PICC_GetTypeName(piccType));

        // print UID in Serial Monitor in the hex format
        Serial.print("UID:");
        for (int i = 0; i < rfid.uid.size; i++)
        {
          Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
          Serial.print(rfid.uid.uidByte[i], HEX);
          // Format and store the HEX values in the uidString array
          sprintf(uidString + i * 2, "%02X", rfid.uid.uidByte[i]);
        }
        
        Serial.println();

        rfid.PICC_HaltA();      // halt PICC
        rfid.PCD_StopCrypto1(); // stop encryption on PCD
        strcpy(msg, uidString);// Set the message to the UID
        client.publish(topic, msg);// Publish the UID as the message to the MQTT topic

        forward();
        delay(300);
      }
    }
  } 

/*
  if (sensorStatus == 1) // Check if the pin high or not
  {
 
  // if the pin is high turn off the onboard Led
  // digitalWrite(LED, LOW); // LED LOW
  Serial.println("Motion Ended!"); // print Motion Detected! on the serial monitor window
  
  }

  else
  {
    //Acc();
    // delay(1000);
    // else turn on the onboard LED
    Serial.println("Motion Detected!"); // print Motion Ended! on the serial monitor window
    
  }

  if (sensorStatus == 0)
  {
    
    //diff = start - end;
    printf("start %d\n", start);
    while (start <= 100)
    {
    if (rfid.PICC_IsNewCardPresent())
    { // new tag is available
      if (rfid.PICC_ReadCardSerial())
      { // NUID has been readed

        MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
        Serial.print("RFID/NFC Tag Type: ");
        Serial.println(rfid.PICC_GetTypeName(piccType));

        // print UID in Serial Monitor in the hex format
        Serial.print("UID:");
        for (int i = 0; i < rfid.uid.size; i++)
        {
          Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
          Serial.print(rfid.uid.uidByte[i], HEX);
          // Format and store the HEX values in the uidString array
          sprintf(uidString + i * 2, "%02X", rfid.uid.uidByte[i]);
        }
        
        Serial.println();

        rfid.PICC_HaltA();      // halt PICC
        rfid.PCD_StopCrypto1(); // stop encryption on PCD
        strcpy(msg, uidString);// Set the message to the UID
        client.publish(topic, msg);// Publish the UID as the message to the MQTT topic
        
        
      }
      break;
    }
      start += 1;
    }
    start = 0;
  }*/

  // test DC motor
  /*
  //Controlling speed (0   = off and 255 = max speed):     
  //(Optional)
  printf("enable \n");
  
  //analogWrite(10, 200); //ENB pin
  //(Optional)
  
  printf("spin \n");
  digitalWrite(motor1pin1,   HIGH);
  digitalWrite(motor1pin2, LOW);

  delay(1000); // rotate at maximum speed 2 seconds in clockwise direction

  // change direction
  digitalWrite(motor1pin1, LOW);   // control the motor's direction in anti-clockwise
  digitalWrite(motor1pin2, HIGH);  // control the motor's direction in anti-clockwise

  delay(1000); // rotate at maximum speed for 2 seconds in anti-clockwise direction

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
  */


}




