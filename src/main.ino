#include <LiquidCrystal.h>

const byte VinfeedPin = A5;
const byte VxfeedPin = A4;
const int REFERENCE_R_NUM = 8;
const int Rpins[REFERENCE_R_NUM] = { 3, 4, 8, 9, 10, 11, 12, 13};
const double Rvalues[REFERENCE_R_NUM] = { 1.0, 9.8, 99.9, 1000.2, 10.04E+03, 98.1E+03, 0.982E+06, 10.15E+06};

const double VswitchUP = 4;
const double VswitchDOWN = 5-VswitchUP;

int selectedR = 0;

bool isEnabled = false;
const int IS_NOT_ENABLED_CODE = -1;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
const int buttonPin = 2;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

const int relaySettlingTime = 40;

unsigned long sampleTimer = 0;
const unsigned long sampleFreqHz = 20;
unsigned long sampleInterval = 0;
bool isFirstMeasure = true;
unsigned long initialTimeOffset;
unsigned long lastSampleTime;

const int LCD_rsPin = 7, LCD_enPin = 5, LCD_contrastPin = 6;
LiquidCrystal screen(LCD_rsPin, LCD_enPin, A0, A1, A2, A3);
const int CONTRAST = 125;

void setup() {
  sampleInterval = 1000 / sampleFreqHz;
  Serial.begin(115200);
  Serial.print(sampleFreqHz);
  Serial.println(" Hz");
  
  for (int i = 0; i < REFERENCE_R_NUM; i++) {
    pinMode(Rpins[i], OUTPUT);
    digitalWrite(Rpins[i], HIGH);
  }
  pinMode(VinfeedPin, INPUT);
  pinMode(VxfeedPin, INPUT);

  pinMode(buttonPin, INPUT);

  analogWrite(LCD_contrastPin, CONTRAST);
  screen.begin(16, 2);
  writeReady();

  if (isEnabled) digitalWrite(Rpins[selectedR], LOW);
  delay(relaySettlingTime);
}

void loop() {
  
  handleIsEnabled();
  
  if (isEnabled) { 
    unsigned long currMillis = millis();   
    if (currMillis - sampleTimer >= sampleInterval) { // is it time for a sample?
      selectRelayChannel();
      double Rx = measureResistance();

      writeResistance(Rx);
      Serial.print(Rx);
      Serial.print(";");
      Serial.println(getSampleTime(currMillis)); // So we have t0 = 0 s
    }
  }
}

unsigned long getSampleTime(unsigned long currMillis) {
  sampleTimer = currMillis;
  unsigned long sampleTime;

  // Gestisce inizializzazione timer da 0 s
  if (isFirstMeasure) initialTimeOffset = sampleTimer - sampleInterval;

  sampleTime = sampleTimer - sampleInterval - initialTimeOffset;

  // Gestisce eventuali disallineamenti di qualche millisecondo
  if ((sampleTime - lastSampleTime != sampleInterval) && (!isFirstMeasure)) {
    initialTimeOffset += sampleTime - lastSampleTime - sampleInterval;
    sampleTime = sampleTimer - sampleInterval - initialTimeOffset;
  }

  if (isFirstMeasure) isFirstMeasure = false;
  lastSampleTime = sampleTime;
  return sampleTime;
}

void resetSampleTime() {
  initialTimeOffset = 0;
  isFirstMeasure = true;
  lastSampleTime = 0;
  sampleTimer = 0;
}

void selectRelayChannel() {
  double Vx = analogRead(VxfeedPin);
  Vx = Vx / 1023 * 5;
    
  if (Vx > VswitchUP) {
    if (selectedR + 1 < REFERENCE_R_NUM) {
      digitalWrite(Rpins[selectedR], HIGH);
      selectedR++;
      digitalWrite(Rpins[selectedR], LOW);
      delay(relaySettlingTime);
    }
  }
  else if (Vx < VswitchDOWN) {
    if (selectedR > 0) {
      digitalWrite(Rpins[selectedR], HIGH);
      selectedR--;
      digitalWrite(Rpins[selectedR], LOW);
      delay(relaySettlingTime);
    }
  }
}

double measureResistance() {  
    double Vx = analogRead(VxfeedPin);
    Vx = Vx / 1023 * 5;
    double Vin = analogRead(VinfeedPin);
    Vin = Vin / 1023 * 5;
    double Ix = (Vin - Vx) / Rvalues[selectedR];
    double Rx = Vx / Ix;

    if (Vx > VswitchUP && selectedR + 1 == REFERENCE_R_NUM) Rx = 1.0 / 0.0;

    return Rx;
}

void handleIsEnabled() {
  int reading = digitalRead(buttonPin);
  
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        isEnabled = !isEnabled;
        if (isEnabled) {
          digitalWrite(Rpins[selectedR], LOW);
          screen.clear();
        }
        else {
          digitalWrite(Rpins[selectedR], HIGH);
          writeReady();
          Serial.println(IS_NOT_ENABLED_CODE);
          resetSampleTime();
        }
        delay(relaySettlingTime);
      }
    }
  }

  lastButtonState = reading;
}

unsigned long LCD_lastDebounceTime = 0;
unsigned long LCD_debounceDelay = 300;
float lastRx;
void writeResistance(float Rx) {
  if (lastRx != Rx) {
    if ((millis() - LCD_lastDebounceTime) > LCD_debounceDelay) {
      screen.clear();
      screen.print("Measuring...");
      screen.setCursor(0, 1);

      int unitOhm;
      
      if (Rx < 1000) {
        Rx = Rx;
        unitOhm = 0;
      }
      else if (Rx >= 1000 && Rx < 1000000) {
        Rx = Rx / 1000;
        unitOhm = 1;
      }
      else if (Rx >= 1000000 && Rx != 1.0/0.0) {
        Rx = Rx / 1000000;
        unitOhm = 2;
      }
      else {
        Rx = Rx;
        unitOhm = -1;
      }

      screen.print(Rx);
      if (unitOhm >= 0) {
        int spaces = 4;
        if (unitOhm == 0) spaces = 3;
        screen.setCursor(16 - spaces, 1);
        if (unitOhm == 1) screen.print("k");
        else if (unitOhm == 2) screen.print("M");
        screen.print("Ohm");
      }
      lastRx = Rx;
      LCD_lastDebounceTime = millis();
    }
  }
}

void writeReady() {
  screen.clear();
  screen.print("Ready");
  lastRx = -1;
}
