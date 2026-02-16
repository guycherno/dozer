#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
HardwareSerial MyUART(1);

#define LT1 15
#define LT2 5
#define LT3 14

#define RX0 3
#define TX0 1

#define ripperServoPin 23

Servo ripperServo;

#define leftMotor0 4  // \ Used for controlling rear drive motor movement
#define leftMotor1 2 // /
#define rightMotor0 12   // \ Used for controlling second rear drive motor movement.
#define rightMotor1 13   // /

#define leftBladeTilt0 16   // \ "Aux1" on PCB. Used for controlling auxillary motor or lights.  Keep in mind this will always breifly turn on when the model is powered on.
#define leftBladeTilt1 17   // /
#define rightBladeTilt0 27  // \ "AUX2" on PCB. Used for controlling auxillary motors or lights.
#define rightBladeTilt1 18   // /
#define bladeTilt0 25       // \ "Aux3" on PCB. Used for controlling auxillary motors or lights.
#define bladeTilt1 26       // /
#define ripperMotor0 33     // \ Used for controlling front drive motor movement
#define ripperMotor1 32     // /

unsigned long lastInputTime = 0;
const unsigned long INPUT_TIMEOUT = 40;  // ms — adjust if needed

int lightSwitchButtonTime = 0;
int lightSwitchTime = 0;
int ripperServoValue = 90;
bool lightMode = false;
int servoDelay = 2;
bool lightsOn = false;
bool moveRipperServoUp = false;
bool moveRipperServoDown = false;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

// ----- UART command input (Serial2) -----
#define UART_BAUD 115200
#define UART_RX_PIN 19   // choose free pins
#define UART_TX_PIN 22

static char uartLine[96];
static size_t uartPos = 0;

// Convert 0..100 (%) -> 0..255 PWM
int pctToPwm(int pct) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (pct * 255) / 100;
}


void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void handleUartCommand(const char* line) {
  // Copy to a mutable buffer for strtok
  char buf[96];
  int xx;
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  // Tokenize: command [arg]
  char* cmd = strtok(buf, " ");
  char* arg = strtok(nullptr, " ");
  if (!line || line[0] == '\0') return;

  // Require a prefix to avoid accidental parsing of debug text
  if (line[0] != '!') return;
  line++; // skip '!'
  if (!cmd) return;

  // Update failsafe timer on any command line we accept/handle
  // (we'll also update it again on success)
  // lastInputTime = millis();

  // Helper lambda-ish pattern (C-style):
  auto readPct = [&](int defaultPct = 0) -> int {
    if (!arg) return defaultPct;
    return atoi(arg);
  };
  
  // ---- Drive motors ----
  xx=strcmp(cmd, "!left-motor-forward");
  Serial.println(xx);
  Serial.println(cmd);
  if  (strcmp(cmd, "!left-motor-reverse")  == 0) {
    lastInputTime = millis();
     
    Serial.println("-----");  
    moveMotor(leftMotor0, leftMotor1, pctToPwm(readPct()));
  } else if (strcmp(cmd, "!left-motor-forward") == 0) {
    lastInputTime = millis();
    moveMotor(leftMotor0, leftMotor1, -pctToPwm(readPct()));
  } else if (strcmp(cmd, "!left-motor-stop") == 0) {
    lastInputTime = millis();
    moveMotor(leftMotor0, leftMotor1, 0);
  } else if (strcmp(cmd, "!right-motor-reverse") == 0) {
    lastInputTime = millis();
    moveMotor(rightMotor0, rightMotor1, pctToPwm(readPct()));
  } else if (strcmp(cmd, "!right-motor-forward") == 0) {
    lastInputTime = millis();
    moveMotor(rightMotor0, rightMotor1, -pctToPwm(readPct()));
  } else if (strcmp(cmd, "!right-motor-stop") == 0) {
    lastInputTime = millis();
    moveMotor(rightMotor0, rightMotor1, 0);

  // ---- Blade tilt motor (Aux3) ----
  } else if (strcmp(cmd, "blade-tilt-up") == 0) {
    lastInputTime = millis();
    moveMotor(bladeTilt0, bladeTilt1, pctToPwm(readPct()));
  } else if (strcmp(cmd, "blade-tilt-down") == 0) {
    lastInputTime = millis();
    moveMotor(bladeTilt0, bladeTilt1, -pctToPwm(readPct()));
  } else if (strcmp(cmd, "blade-tilt-stop") == 0) {
    lastInputTime = millis();
    moveMotor(bladeTilt0, bladeTilt1, 0);

  // ---- Ripper motor ----
  } else if (strcmp(cmd, "ripper-motor-forward") == 0) {
    lastInputTime = millis();
    moveMotor(ripperMotor0, ripperMotor1, pctToPwm(readPct()));
  } else if (strcmp(cmd, "ripper-motor-reverse") == 0) {
    lastInputTime = millis();
    moveMotor(ripperMotor0, ripperMotor1, -pctToPwm(readPct()));
  } else if (strcmp(cmd, "ripper-motor-stop") == 0) {
    lastInputTime = millis();
    moveMotor(ripperMotor0, ripperMotor1, 0);

  // ---- Left blade tilt (Aux1) direct direction pins ----
  } else if (strcmp(cmd, "left-blade-tilt-forward") == 0) {
    lastInputTime = millis();
    digitalWrite(leftBladeTilt0, HIGH);
    digitalWrite(leftBladeTilt1, LOW);
  } else if (strcmp(cmd, "left-blade-tilt-reverse") == 0) {
    lastInputTime = millis();
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, HIGH);
  } else if (strcmp(cmd, "left-blade-tilt-stop") == 0) {
    lastInputTime = millis();
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);

  // ---- Right blade tilt (Aux2) direct direction pins ----
  } else if (strcmp(cmd, "right-blade-tilt-forward") == 0) {
    lastInputTime = millis();
    digitalWrite(rightBladeTilt0, HIGH);
    digitalWrite(rightBladeTilt1, LOW);
  } else if (strcmp(cmd, "right-blade-tilt-reverse") == 0) {
    lastInputTime = millis();
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, HIGH);
  } else if (strcmp(cmd, "right-blade-tilt-stop") == 0) {
    lastInputTime = millis();
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);

  // ---- Lights ----
  } else if (strcmp(cmd, "lights") == 0) {
    // expects: "lights on|off|toggle"
    if (!arg) return;
    lastInputTime = millis();
    if (strcmp(arg, "on") == 0) {
      lightMode = true;
      digitalWrite(LT1, HIGH);
    } else if (strcmp(arg, "off") == 0) {
      lightMode = false;
      digitalWrite(LT1, LOW);
    } else if (strcmp(arg, "toggle") == 0) {
      lightMode = !lightMode;
      digitalWrite(LT1, lightMode ? HIGH : LOW);
    }

  // ---- Servo ----
  } else if (strcmp(cmd, "ripper-servo-angle") == 0) {
    // expects degrees, e.g. 10..170
    if (!arg) return;
    int deg = atoi(arg);
    if (deg < 10) deg = 10;
    if (deg > 170) deg = 170;
    lastInputTime = millis();
    ripperServoValue = deg;
    ripperServo.write(ripperServoValue);

  } else if (strcmp(cmd, "stop-all") == 0) {
    lastInputTime = millis();
    // Same “stop” behavior as failsafe, but on-demand
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
    digitalWrite(bladeTilt0, LOW);
    digitalWrite(bladeTilt1, LOW);
    digitalWrite(leftMotor0, LOW);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(rightMotor0, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(ripperMotor0, LOW);
    digitalWrite(ripperMotor1, LOW);

  } else if (strcmp(cmd, "help") == 0) {
    Serial.println("UART commands examples:");
    Serial.println(" left-motor-forward 50 | left-motor-reverse 50 | left-motor-stop");
    Serial.println(" right-motor-forward 50 | right-motor-reverse 50 | right-motor-stop");
    Serial.println(" blade-tilt-up 80 | blade-tilt-down 80 | blade-tilt-stop");
    Serial.println(" ripper-motor-forward 60 | ripper-motor-reverse 60 | ripper-motor-stop");
    Serial.println(" left-blade-tilt-forward | left-blade-tilt-reverse | left-blade-tilt-stop");
    Serial.println(" right-blade-tilt-forward | right-blade-tilt-reverse | right-blade-tilt-stop");
    Serial.println(" lights on|off|toggle");
    Serial.println(" ripper-servo-angle 120");
    Serial.println(" stop-all");
  }
}

void pollUartCommands() {
  
  while (MyUART.available() > 0) {
    char c = (char)MyUART.read();
   

    if (c == '\r') continue;

    if (c == '\n') {
      uartLine[uartPos] = '\0';
      if (uartPos > 0) {
        handleUartCommand(uartLine);
        
      }
      uartPos = 0;
      continue;
    }

    // normal char
    if (uartPos < sizeof(uartLine) - 1) {
      uartLine[uartPos++] = c;
    } else {
      // overflow: reset line
      uartPos = 0;
    }
     Serial.println(uartLine);  
     Serial.println("Here");  
  }
}




void processGamepad(ControllerPtr ctl) {

  // Record that we received fresh input
  lastInputTime = millis();

  //Throttle
  processLeftThrottle(ctl->axisY());
  processRightThrottle(ctl->axisRY());

  //Blade Tilt and Ripper
  processBladeTiltAndRipper(ctl->dpad());

  //Lights
  processLights(ctl->thumbR());

  if (ctl->r1() == 1) {
    digitalWrite(rightBladeTilt0, HIGH);
    digitalWrite(rightBladeTilt1, LOW);
  } else if (ctl->r2() == 1) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, HIGH);
  } else if (ctl->r1() == 0 || ctl->r2() == 0) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
  }
  if (ctl->l1() == 1) {
    digitalWrite(leftBladeTilt0, HIGH);
    digitalWrite(leftBladeTilt1, LOW);
  } else if (ctl->l2() == 1) {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, HIGH);
  } else if (ctl->l1() == 0 || ctl->r2() == 0) {
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
  }

  if (ctl->x() == 1) {
    moveRipperServoDown = true;
  } else if (ctl->b() == 1) {
    moveRipperServoUp = true;
  } else {
    moveRipperServoDown = false;
    moveRipperServoUp = false;
  }
  if (moveRipperServoUp) {
    if (servoDelay == 2) {
      if (ripperServoValue >= 10 && ripperServoValue < 170) {
        //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        ripperServoValue = ripperServoValue + 2;
        ripperServo.write(ripperServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveRipperServoDown) {
    if (servoDelay == 2) {
      if (ripperServoValue <= 170 && ripperServoValue > 10) {
        //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        ripperServoValue = ripperServoValue - 2;
        ripperServo.write(ripperServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}
void processLeftThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
}
void processRightThrottle(int axisRYValue) {
  int adjustedThrottleValue = axisRYValue / 2;
  moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
}

void processBladeTiltAndRipper(int dpadValue) {
  if (dpadValue == 1) {
    moveMotor(bladeTilt0, bladeTilt1, 255);
  } else if (dpadValue == 2) {
    moveMotor(bladeTilt0, bladeTilt1, -255);
  } else {
    moveMotor(bladeTilt0, bladeTilt1, 0);
  }
  if (dpadValue == 8) {
    moveMotor(ripperMotor0, ripperMotor1, 255);
  } else if (dpadValue == 4) {
    moveMotor(ripperMotor0, ripperMotor1, -255);
  } else {
    moveMotor(ripperMotor0, ripperMotor1, 0);
  }
}

void processLights(bool buttonValue) {
  Serial.println(buttonValue);
  if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
    if (!lightMode) {
      digitalWrite(LT1, HIGH);
      delay(10);
      lightMode = true;
    } else {
      digitalWrite(LT1, LOW);
      delay(10);
      lightMode = false;
    }
    lightSwitchButtonTime = millis();
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 15) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -15) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(rightBladeTilt0, OUTPUT);
  pinMode(rightBladeTilt1, OUTPUT);
  digitalWrite(rightBladeTilt0, LOW);
  digitalWrite(rightBladeTilt1, LOW);
  Serial.begin(115200);
  MyUART.begin(115200, SERIAL_8N1, 3,4);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);
  
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(ripperMotor0, OUTPUT);
  pinMode(ripperMotor1, OUTPUT);
  pinMode(leftBladeTilt0, OUTPUT);
  pinMode(leftBladeTilt1, OUTPUT);
  pinMode(bladeTilt0, OUTPUT);
  pinMode(bladeTilt1, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  digitalWrite(leftMotor0, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(rightMotor0, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(ripperMotor0, LOW);
  digitalWrite(ripperMotor1, LOW);
  digitalWrite(leftBladeTilt0, LOW);
  digitalWrite(leftBladeTilt1, LOW);
  digitalWrite(bladeTilt0, LOW);
  digitalWrite(bladeTilt1, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  ripperServo.attach(ripperServoPin);
  ripperServo.write(ripperServoValue);

   lastInputTime = millis();  // initialize failsafe timer
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  pollUartCommands();

  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
  // Failsafe check: if no input for too long, stop motors
  if (millis() - lastInputTime > INPUT_TIMEOUT) {
    digitalWrite(rightBladeTilt0, LOW);
    digitalWrite(rightBladeTilt1, LOW);
    digitalWrite(leftBladeTilt0, LOW);
    digitalWrite(leftBladeTilt1, LOW);
    digitalWrite(bladeTilt0, LOW);
    digitalWrite(bladeTilt1, LOW);
    digitalWrite(leftMotor0, LOW);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(rightMotor0, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(ripperMotor0, LOW);
    digitalWrite(ripperMotor1, LOW);
  }
}
