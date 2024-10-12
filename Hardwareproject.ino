#include <HX711.h>
#include <Servo.h>
#include <Stepper.h>

#define calibration_factor 110

const int relayPin = 6;
const int switchPinA = 23;
const int switchPinB = 25;
const int switchPinFront = 27;
const int switchPinBack = 29;
const int NEMA23dirPin = 31;
const int NEMA23stepPin = 39;
const int NEMA17dirPin = 35;
const int NEMA17stepPin = 37;
const int irSensorPin = 33;
const int servoPin = 41;
const int LOADCELL_DOUT_PIN = 43;
const int LOADCELL_SCK_PIN = 45;
const int IN1_28BYJ48 = 47;
const int IN2_28BYJ48 = 49;
const int IN3_28BYJ48 = 51;
const int IN4_28BYJ48 = 53;
const int ThermistorPin = A0;

const float weightThresholdGrams = 26.0;
const int stepsPerRevolution28BYJ48 = 2048;

bool currentDirectionNEMA23 = LOW;
bool lastSwitchStateA = LOW;
bool lastSwitchStateB = LOW;
bool currentDirectionNEMA17 = LOW;
bool lastSwitchStateFront = LOW;
bool lastSwitchStateBack = LOW;
bool motorActivatedNEMA23 = false;
bool motorActivatedNEMA17 = false;
bool frytimereached = false;
bool delayActive = false;
bool trapdoorActivated = false;
bool motorActivated28BYJ48 = false;
bool initialrotation28BYJ48 = true;
bool operationStarted = false;

unsigned long fryingStartTime = 0;
unsigned long delayStartTime = 0;

HX711 scale;
Servo trapDoorServo;
Stepper stepper28BYJ48(stepsPerRevolution28BYJ48, IN1_28BYJ48, IN3_28BYJ48, IN2_28BYJ48, IN4_28BYJ48);

enum NEMA23State {
  NEMA23_IDLE,
  NEMA23_NORMAL_ACTION,
  NEMA23_TO_INITIAL_POSITION
};

NEMA23State nema23State = NEMA23_IDLE;

enum NEMA17State {
  NEMA17_IDLE,
  MOVING_FORWARD,
  MOVING_BACKWARD,
};

NEMA17State nema17State = NEMA17_IDLE;

float R1 = 5000;
float logR2, R2, T, Tc;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

void setup() {
  Serial.begin(9600);
  pinMode(NEMA23dirPin, OUTPUT);
  pinMode(NEMA23stepPin, OUTPUT);
  pinMode(switchPinA, INPUT_PULLUP);
  pinMode(switchPinB, INPUT_PULLUP);
  pinMode(NEMA17dirPin, OUTPUT);
  pinMode(NEMA17stepPin, OUTPUT);
  pinMode(switchPinFront, INPUT_PULLUP);
  pinMode(switchPinBack, INPUT_PULLUP);
  pinMode(irSensorPin, INPUT);
  pinMode(relayPin, OUTPUT);

  digitalWrite(NEMA23dirPin, currentDirectionNEMA23);
  digitalWrite(NEMA17dirPin, currentDirectionNEMA17);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();

  trapDoorServo.attach(servoPin);
  trapDoorServo.write(120);

  stepper28BYJ48.setSpeed(5);
  digitalWrite(relayPin, HIGH);
}

void loop() {
  bool currentSwitchStateA = digitalRead(switchPinA);
  bool currentSwitchStateB = digitalRead(switchPinB);
  bool currentSwitchStateFront = digitalRead(switchPinFront);
  bool currentSwitchStateBack = digitalRead(switchPinBack);
  bool irDetected = digitalRead(irSensorPin) == LOW;

  const unsigned long interval = 1000;
  static unsigned long previousMillisThermistor = 0;
  unsigned long currentMillisThermistor = millis();

  if (currentMillisThermistor - previousMillisThermistor >= interval) {
    previousMillisThermistor = currentMillisThermistor;
    int Vo = analogRead(ThermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;
    Serial.print("Temperature: ");
    Serial.print(Tc);
    Serial.println(" C");

    if (Tc <= 200) {
      digitalWrite(relayPin, LOW);
    } else if (Tc >= 230) {
      digitalWrite(relayPin, HIGH);
    }

    if (Tc >= 140 && !operationStarted) {
      operationStarted = true;
    }
  }

  if (!operationStarted) {
    return;
  }

  if (initialrotation28BYJ48) {
    stepper28BYJ48.step(-stepsPerRevolution28BYJ48);
    motorActivatedNEMA23 = true;
    nema23State = NEMA23_NORMAL_ACTION;
    initialrotation28BYJ48 = false;
  }

  if (irDetected) {
    motorActivatedNEMA23 = true;
    nema23State = NEMA23_NORMAL_ACTION;
  }

  static unsigned long previousMillisLOADCELL = 0;
  unsigned long currentMillisLOADCELL = millis();

  if (currentMillisLOADCELL - previousMillisLOADCELL >= interval) {
    float weight = scale.get_units();
    Serial.print("Weight: ");
    Serial.println(weight);
    previousMillisLOADCELL = currentMillisLOADCELL;

    if (weight > weightThresholdGrams) {
      trapDoorServo.write(0);
      trapdoorActivated = true;
      fryingStartTime = millis();
      Serial.println("trapdoor opened");
      Serial.print("trapdoorActivated status: ");
      Serial.println(trapdoorActivated);
      nema23State = NEMA23_TO_INITIAL_POSITION;
    } else {
      trapDoorServo.write(120);
    }
  }

  if (trapdoorActivated) {
    Serial.println("Trapdoor activated");

    unsigned long currentMillisFRY = millis();
    const unsigned long fryingDuration = 180000;

    Serial.print("currentMillisFRY: ");
    Serial.println(currentMillisFRY);
    Serial.print("fryingStartTime: ");
    Serial.println(fryingStartTime);
    Serial.print("Time difference: ");
    Serial.println(currentMillisFRY - fryingStartTime);

    if (currentMillisFRY - fryingStartTime >= fryingDuration) {
      motorActivatedNEMA17 = true;
      trapdoorActivated = false;
      Serial.println("Frying time completed, activating NEMA17");
      nema17State = MOVING_FORWARD;
    }
  }

  switch (nema23State) {
    case NEMA23_IDLE:
      break;

    case NEMA23_NORMAL_ACTION:

      if (motorActivatedNEMA23) {
        digitalWrite(NEMA23dirPin, currentDirectionNEMA23);
        digitalWrite(NEMA23stepPin, HIGH);
        digitalWrite(NEMA23stepPin, LOW);
        delayMicroseconds(60);

        if (currentSwitchStateA == LOW && lastSwitchStateA == HIGH) {
          currentDirectionNEMA23 = !currentDirectionNEMA23;
          lastSwitchStateA = currentSwitchStateA;
        } else if (currentSwitchStateB == LOW && lastSwitchStateB == HIGH) {
          currentDirectionNEMA23 = !currentDirectionNEMA23;
          lastSwitchStateB = currentSwitchStateB;
          stepper28BYJ48.step(-stepsPerRevolution28BYJ48);
        }

        if (currentSwitchStateA == HIGH) {
          lastSwitchStateA = HIGH;
        }

        if (currentSwitchStateB == HIGH) {
          lastSwitchStateB = HIGH;
        }
      }
      break;

    case NEMA23_TO_INITIAL_POSITION:
      Serial.println("Moving NEMA23 to Initial Position");

      if (motorActivatedNEMA23) {
        currentDirectionNEMA23 = HIGH;
        digitalWrite(NEMA23dirPin, currentDirectionNEMA23);
        motorActivatedNEMA23 = true;

        while (digitalRead(switchPinB) == HIGH) {
          digitalWrite(NEMA23stepPin, HIGH);
          delayMicroseconds(60);
          digitalWrite(NEMA23stepPin, LOW);
          delayMicroseconds(60);
        }

        Serial.println("Reached Initial Position");
        motorActivatedNEMA23 = false;
        nema23State = NEMA23_IDLE;
      }
      break;
  }

  switch (nema17State) {
    case NEMA17_IDLE:

      break;

    case MOVING_FORWARD:

      if (motorActivatedNEMA17) {
        Serial.println("NEMA17 Motor Moving Forward");
        digitalWrite(NEMA17dirPin, currentDirectionNEMA17);

        digitalWrite(NEMA17stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(NEMA17stepPin, LOW);
        delayMicroseconds(1000);

        Serial.print("Before state transition, back switch state: ");
        Serial.println(currentSwitchStateBack);

        if (currentSwitchStateBack == LOW) {
          Serial.println("Back limit switch pressed");
          currentDirectionNEMA17 = !currentDirectionNEMA17;
          delayStartTime = millis();
          delayActive = true;
          nema17State = MOVING_BACKWARD;
          Serial.println("Transition to NEMA17_MOVING_BACKWARD state");
        }
      }
      break;

    case MOVING_BACKWARD:
    
      if (motorActivatedNEMA17) {
        Serial.println("NEMA17 Motor Moving Backward");
        digitalWrite(NEMA17dirPin, currentDirectionNEMA17);

        digitalWrite(NEMA17stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(NEMA17stepPin, LOW);
        delayMicroseconds(1000);

        Serial.print("Before state transition, front switch state: ");
        Serial.println(currentSwitchStateFront);

        if (currentSwitchStateFront == LOW) {
          Serial.println("Front limit switch pressed");
          currentDirectionNEMA17 = !currentDirectionNEMA17;
          motorActivatedNEMA17 = false;
          motorActivated28BYJ48 = true;
          nema17State = NEMA17_IDLE;  
          Serial.println("Transition to NEMA17_IDLE state");
        }
      }
      break;
  }

  if (delayActive && (millis() - delayStartTime >= 10)) {
    delayActive = false;
  }

  if (motorActivated28BYJ48) {
    Serial.println("28BYJ-48 Motor Performing Full Rotation");
    stepper28BYJ48.step(-stepsPerRevolution28BYJ48);  
    motorActivated28BYJ48 = false;                    
  }
}
