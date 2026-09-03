#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define MQ3_PIN A0
#define GREEN_LED 12
#define RED_LED 13
#define BUZZER 8

LiquidCrystal_I2C lcd(0x27, 16, 2);

int alcoholThreshold = 100;

bool brakeActivated = false;

void setup() {

  Serial.begin(9600);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER, LOW);

  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ALCOHOL SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("READY");

  delay(2000);
}

void loop() {

  // =================================
  // BRAKE ACTIVATED
  // =================================

  if (brakeActivated) {

    // 7 BRAKE ITERATIONS
    for (int i = 1; i <= 7; i++) {

      Serial.print("BRAKE ");
      Serial.println(i);

      // -----------------------------
      // BRAKE ON - 1 SECOND
      // -----------------------------

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BRAKE ");
      lcd.print(i);

      lcd.setCursor(0, 1);
      lcd.print("ACTIVATED");

      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BUZZER, HIGH);

      delay(1000);

      // -----------------------------
      // BRAKE OFF - 0.5 SECOND
      // -----------------------------

      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);

      lcd.clear();

      delay(500);
    }

    // =================================
    // CAR STOPPED
    // =================================

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CAR IS");
    lcd.setCursor(0, 1);
    lcd.print("STOPPED");

    Serial.println(">>> CAR IS STOPPED <<<");

    // Continuous beep for 5 seconds
    digitalWrite(BUZZER, HIGH);

    delay(5000);

    digitalWrite(BUZZER, LOW);

    // =================================
    // SYSTEM RESTARTING
    // =================================

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SYSTEM");
    lcd.setCursor(0, 1);
    lcd.print("RESTARTING");

    Serial.println(">>> SYSTEM RESTARTING <<<");

    delay(2000);

    // Reset system
    brakeActivated = false;

    return;
  }

  // =================================
  // READ MQ-3
  // =================================

  int sensorValue = analogRead(MQ3_PIN);

  Serial.print("MQ-3 Value: ");
  Serial.println(sensorValue);

  // =================================
  // ALCOHOL DETECTED
  // =================================

  if (sensorValue >= alcoholThreshold) {

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(BUZZER, LOW);

    // ---------------------------------
    // ALCOHOL DETECTED - 1 SECOND
    // ---------------------------------

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ALCOHOL");
    lcd.setCursor(0, 1);
    lcd.print("DETECTED");

    Serial.println("ALCOHOL DETECTED");

    delay(1000);

    // ---------------------------------
    // ABNORMALITY DETECTION ENGAGED
    // ---------------------------------

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ABNORMALITY");
    lcd.setCursor(0, 1);
    lcd.print("DETECTION ENGAGED");

    Serial.println("ABNORMALITY DETECTION ENGAGED");
    Serial.println("Waiting for TRUE...");

    // =================================
    // LOCKED ALCOHOL STATE
    // =================================
    // DO NOT CHECK MQ-3 AGAIN
    // FALSE DOES NOTHING
    // ONLY TRUE ACTIVATES BRAKE
    // =================================

    while (true) {

      if (Serial.available() > 0) {

        String input = Serial.readStringUntil('\n');

        input.trim();
        input.toUpperCase();

        // ---------------------------------
        // TRUE → BRAKE
        // ---------------------------------

        if (input == "TRUE") {

          brakeActivated = true;

          Serial.println("ABNORMAL DRIVING: TRUE");
          Serial.println(">>> BRAKE ACTIVATED <<<");

          break;
        }

        // ---------------------------------
        // FALSE → DO ABSOLUTELY NOTHING
        // ---------------------------------

        else if (input == "FALSE") {

          Serial.println("ABNORMAL DRIVING: FALSE");
          Serial.println("FALSE IGNORED");

          // LCD remains exactly the same
          // System remains locked
        }
      }

      delay(100);
    }
  }

  // =================================
  // NO ALCOHOL
  // =================================

  else {

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ALCOHOL NOT");
    lcd.setCursor(0, 1);
    lcd.print("DETECTED");

    Serial.println("ALCOHOL NOT DETECTED");
  }

  Serial.println("--------------------");

  delay(1000);
}
