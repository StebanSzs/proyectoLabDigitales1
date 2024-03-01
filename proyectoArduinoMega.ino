#include <LiquidCrystal.h>
#include <Keypad.h>
#include <AsyncTaskLib.h>
#include <DHT.h>
#include "StateMachineLib.h"

const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

DHT dht(A8, DHT11);

#define greenLed  6
#define blueLed  A4
#define redLed A5
#define potPin 22
#define photoresistorPin A10

const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 4;

byte rowPins[KEYPAD_ROWS] = {5, 4, 3, 2};
byte colPins[KEYPAD_COLS] = {A3, A2, A1, A0};
char keys[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', '+'},
  {'4', '5', '6', '-'},
  {'7', '8', '9', '*'},
  {'.', '0', '=', '/'}
};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

char password[5] = "1234";
unsigned char attemptCount = 0;
unsigned char idx = 0;
unsigned char numKeys = 0;


enum State
{
  begin = 0,
  monitoring = 1,
  alarm = 2,
  blockade = 3
};

enum Input
{
  signB = 0,
  signP = 1,
  signT = 2,
  signH = 3,
  Unknown = 4,
};

StateMachine stateMachine(4, 5);

//Time para blink red
unsigned long previousMillis = 0;
const long interval = 800;

//Time para estado bloqueado
unsigned long blockadeStartTime = 0;
const unsigned long blockadeDuration = 5000;
 unsigned long startTime = millis();
Input input;

void setupStatemachine() {
  stateMachine.AddTransition(begin, monitoring, []() { return input == signP; });
  stateMachine.AddTransition(begin, blockade, []() { return input == signB; });

  stateMachine.AddTransition(monitoring, alarm, []() { return input == signP; });
  stateMachine.AddTransition(monitoring, blockade, []() { return input == signH; });

  stateMachine.AddTransition(blockade, monitoring, []() { return input == signT; });
  stateMachine.AddTransition(blockade, begin, []() { return input == signH; });

  stateMachine.AddTransition(alarm, monitoring, []() { return input == signT; });

  stateMachine.SetOnEntering(begin, beginEntering);
  stateMachine.SetOnEntering(monitoring, monitoringEntering);
  stateMachine.SetOnEntering(blockade, blockadeEntering);
  stateMachine.SetOnEntering(alarm, alarmEntering);

  stateMachine.SetOnLeaving(begin, []() {Serial.println("Leaving beging"); });
  stateMachine.SetOnLeaving(monitoring, []() {Serial.println("Leaving monitoring"); });
  stateMachine.SetOnLeaving(blockade, []() {Serial.println("Leaving blockade"); });
  stateMachine.SetOnLeaving(alarm, []() {Serial.println("Leaving alarm"); });
  stateMachine.SetState(blockade, true, true);
  stateMachine.SetState(begin, true, true); 
}

void start();
void correctPass();
void incorrectPass();
void reset();
void bloq();
void dht11();
void poten();
void blinkLedRed();

AsyncTask ini(100, false, start);
AsyncTask acces(100, false, correctPass);
AsyncTask noPass(100, false, incorrectPass);
AsyncTask attempts(1000, false, reset);
AsyncTask bloqq(100, false, bloq);
AsyncTask dht11Start(2000, true, dht11);
AsyncTask potenciometro(5000, true, poten);
AsyncTask photoresistor(3000, true, photore);
AsyncTask blinkRed(800, true, blinkLedRed);


void setup() {
	Serial.begin(9600);

  lcd.begin(16, 2);
  pinMode(greenLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(photoresistorPin, INPUT);

  Serial.println("Starting State Machine...");
	setupStatemachine();
	Serial.println("Start Machine Started");
  stateMachine.SetState(begin, true, true);

}

void loop() {

  ini.Update();
  acces.Update();
  noPass.Update();
  attempts.Update();
  bloqq.Update();
  dht11Start.Update();
  stateMachine.Update();
  potenciometro.Update();
  photoresistor.Update();
  blinkRed.Update();

  char key = keypad.getKey();
  if (key) {
    lcd.print('*');
    if (idx < 4) {
      checkPassword(key);
    }
  }
}

void checkPassword(char key) {
  if (idx < 4) {
    if (key == password[idx]) {
      idx++;
      if (idx == 4) {
        acces.Start();
      }
    } else if (numKeys < 3) {
      numKeys++;
    } else {
      noPass.Start();
      numKeys = 0;
    }
  }
}

void reset() {
  digitalWrite(blueLed, LOW);
  attemptCount++;

  if (attemptCount == 3) {
    lcd.clear();
    bloqq.Start();
    attemptCount = 0;
  } else if (attemptCount < 3) {
    start();
  }
  idx = 0;
}

void start() {
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, LOW);
  lcd.setCursor(0, 0);
  lcd.print("Ingrese clave");
  lcd.setCursor(5, 1);
}

void correctPass() {
  lcd.clear();
  lcd.print("Correcto");
  digitalWrite(greenLed, HIGH);
  stateMachine.SetState(monitoring, true, true);
  idx = 0;
}

void incorrectPass() {
  digitalWrite(blueLed, HIGH);
  lcd.clear();
  lcd.print("Incorrecta");
  attempts.Start();
}

void bloq() {
  lcd.clear();
  lcd.print("Bloqueado");
   digitalWrite(redLed, HIGH); // Enciende el LED rojo
    delay(500); // Espera 500 ms
  digitalWrite(redLed, LOW); 
  lcd.clear();
   attempts.Start();
  AsyncTask blockadeTask(5000, false, []() {
    while (millis() - startTime <= 500) { // Ejecuta durante 5 segundos
      digitalWrite(redLed, HIGH); // Enciende el LED rojo
      delay(500); // Espera 500 ms
    }
 
  });
}

void dht11() {
  digitalWrite(greenLed, LOW);
  lcd.clear();
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  lcd.print("Temp: 30° ");
  lcd.print(temperature);
  digitalWrite(redLed, HIGH);
  if (temperature > 30){
     lcd.print("Temp: ¡Atención! Mayor a 30°"); // Muestra el mensaje de alerta
  }
  lcd.setCursor(0, 1);
  lcd.print("C Hum: ");
  lcd.print(humidity);
}

void poten() {
  lcd.clear();
  int potValue = analogRead(potPin);
  int mappedValue = map(potValue, 0, 1023, 0, 255);
  lcd.setCursor(0, 0);
  lcd.print("Pot: ");
  lcd.print(mappedValue);
  digitalWrite(greenLed, LOW);
}

void photore() {
  lcd.clear();
  int photoresistorValue = analogRead(photoresistorPin);
  lcd.setCursor(0, 0);
  lcd.print("LDR: ");
  lcd.print(photoresistorValue);
}

void beginEntering() {
  ini.Start();
}

void monitoringEntering() {
  dht11Start.Start();
  potenciometro.Start();
  photoresistor.Start();
}

void blockadeEntering() {
  blinkRed.Start();
  blockadeStartTime = millis();
  if (blockadeStartTime >= blockadeDuration) {
    stateMachine.SetState(begin, true, true);
  }
}

void alarmEntering() {

}

void blinkLedRed() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (digitalRead(redLed) == HIGH) {
      digitalWrite(redLed, HIGH);
    } else {
      digitalWrite(redLed, LOW);
    }
  }
}