#include <DFRobot_DHT11.h>
#include <HX711_ADC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>
#include <SoftwareSerial.h>

DFRobot_DHT11 DHT;
#define DHT11_PIN 4
#define MQ_PIN A0
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define SERVO_PIN 9
#define GREEN_LED_1_PIN 6
#define RED_LED_1_PIN 7
#define GREEN_LED_2_PIN 8
#define RED_LED_2_PIN 5
#define LCD_COLUMNS 16
#define LCD_ROWS 2

Servo doorServo;
LiquidCrystal_I2C lcd(0x27, LCD_COLUMNS, LCD_ROWS);
HX711_ADC loadCell_1(10,11);
HX711_ADC loadCell_2(A2,A3);
SoftwareSerial mySerial(12,13);

#define TRAY_CAPACITY 100.000 // grams
#define ULTRASONIC_THRESHOLD 50 // cm
#define ULTRASONIC_DURATION 2000 // milliseconds

bool isGasDetected = false;
bool isDoorOpen = false;

void setup() {
  pinMode(MQ_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GREEN_LED_1_PIN, OUTPUT);
  pinMode(RED_LED_1_PIN, OUTPUT);
  pinMode(GREEN_LED_2_PIN, OUTPUT);
  pinMode(RED_LED_2_PIN, OUTPUT);
  mySerial.begin(9600);
  Serial.begin(9600);
  
  doorServo.attach(SERVO_PIN);
  doorServo.write(0); // Close the door initially
  lcd.init();
  lcd.backlight();
  lcd.print(" FOOD GUARD PRO");
  delay(3000);
  lcd.clear();
  
  loadCell_1.begin();
  loadCell_2.begin();
  loadCell_1.start(2000); // load cells gets 2000ms of time to stabilize
  loadCell_1.setCalFactor(200.0);
  loadCell_2.start(2000); // load cells gets 2000ms of time to stabilize
  loadCell_2.setCalFactor(200.0);
  
}

void loop() {
  float temperature = getTemperature();
  Serial.print("temp");
  Serial.println(temperature);
  lcd.setCursor(0, 0);
  lcd.print("TEMPERATURE:");
  lcd.print(temperature);
  
  if (isGasDetected) {
    SendMessageGas();
    digitalWrite(GREEN_LED_1_PIN, LOW);
    digitalWrite(RED_LED_1_PIN, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("FOOD SPOILED");
    
  } else {
    digitalWrite(GREEN_LED_1_PIN, HIGH);
    digitalWrite(RED_LED_1_PIN, LOW);
    lcd.setCursor(0, 1);
    lcd.print("            ");
  }

  
  loadCell_1.update(); // retrieves data from the load cell
  float fruitWeight = loadCell_1.getData(); // get output value
  fruitWeight = fruitWeight;
  loadCell_2.update(); // retrieves data from the load cell
  float vegetableWeight = loadCell_2.getData();
  //vegetableWeight = -(vegetableWeight);
  Serial.print("fw");
  Serial.println(fruitWeight);
  Serial.print("vw");
  Serial.println(vegetableWeight);
  Serial.println("");

  if (fruitWeight > TRAY_CAPACITY && vegetableWeight > TRAY_CAPACITY) {
    digitalWrite(GREEN_LED_2_PIN, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("         ");
  } else {
    digitalWrite(GREEN_LED_2_PIN, LOW);
    if (fruitWeight < TRAY_CAPACITY) {
      SendMessageLoadF();
      digitalWrite(RED_LED_2_PIN, HIGH);
      digitalWrite(GREEN_LED_2_PIN, LOW);
      lcd.setCursor(0, 1);
      lcd.print("FL");
      
    } else {
      //digitalWrite(RED_LED_2_PIN, LOW);
    }

    if (vegetableWeight < TRAY_CAPACITY) {
      SendMessageLoadV();
      //digitalWrite(RED_LED_2_PIN, HIGH);
      //digitalWrite(GREEN_LED_2_PIN, LOW);
      lcd.setCursor(3, 1);
      lcd.print("VL");
      
    } else {
      digitalWrite(RED_LED_2_PIN, LOW);
    }
  }

  long duration, distance;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  delay(1000);


  if (distance < ULTRASONIC_THRESHOLD && !isDoorOpen) {
    doorServo.write(180);
    isDoorOpen = true;
    delay(5000);
    isDoorOpen = false;
  }

  if (!isDoorOpen) {
    doorServo.write(0);
  }

  int gasValue = analogRead(MQ_PIN);
  Serial.print("gasValue");
  Serial.println(gasValue);
  isGasDetected = (gasValue > 300); // Adjust threshold as needed*/
}
  
float getTemperature() {
  DHT.read(DHT11_PIN);
  return DHT.temperature;
}

void SendMessageGas()
{
mySerial.println("AT+CMGF=1"); //Sets the GSM Module in Text Mode
delay(1000); // Delay of 1000 milli seconds or 1 second
mySerial.println("AT+CMGS=\"+918156964720\"\r"); // Replace x with mobile number
delay(1000);
mySerial.println("FOOD SPOILED");// The SMS text you want to send
delay(100);
mySerial.println((char)26);// ASCII code of CTRL+Z
delay(1000);
}

void SendMessageLoadF()
{
mySerial.println("AT+CMGF=1"); //Sets the GSM Module in Text Mode
delay(1000); // Delay of 1000 milli seconds or 1 second
mySerial.println("AT+CMGS=\"+918156964720\"\r"); // Replace x with mobile number
delay(1000);
mySerial.println("FRUIT TRAY LOW");// The SMS text you want to send
delay(100);
mySerial.println((char)26);// ASCII code of CTRL+Z
delay(1000);
}


void SendMessageLoadV()
{
mySerial.println("AT+CMGF=1"); //Sets the GSM Module in Text Mode
delay(1000); // Delay of 1000 milli seconds or 1 second
mySerial.println("AT+CMGS=\"+918156964720\"\r"); // Replace x with mobile number
delay(1000);
mySerial.println("VEGETABLES TRAY LOW");// The SMS text you want to send
delay(100);
mySerial.println((char)26);// ASCII code of CTRL+Z
delay(1000);
}
