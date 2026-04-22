#include <Arduino.h>

/*
  Inkstroke - Safe Direction Test for 3x 28BYJ-48 motors (ULN2003 drivers)

  Pin mapping requested:
    X -> 2,3,4,5
    Y -> 6,7,8,9
    Z -> 10,11,12,13

  This sketch does NOT move anything on boot.
  You jog each axis manually from Serial Monitor at 115200 baud.
*/

struct Motor {
  const char* name;
  uint8_t pins[4];
  uint8_t phase;
};

Motor motorX = {"X", {2, 3, 4, 5}, 0};
Motor motorY = {"Y", {6, 7, 8, 9}, 0};
Motor motorZ = {"Z", {10, 11, 12, 13}, 0};

// 8-step half-step sequence for 28BYJ-48 via ULN2003.
const uint8_t HALFSTEP[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// Safety defaults: very small jog, far below one full output shaft turn.
// 28BYJ-48 is typically ~4096 half-steps per output revolution.
int jogSteps = 128;        // ~1/32 rev
int stepDelayMs = 2;       // gentle speed

void setupMotorPins(const Motor& m) {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(m.pins[i], OUTPUT);
    digitalWrite(m.pins[i], LOW);
  }
}

void releaseMotor(const Motor& m) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(m.pins[i], LOW);
  }
}

void applyPhase(const Motor& m, uint8_t phase) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(m.pins[i], HALFSTEP[phase][i]);
  }
}

void stepMotor(Motor& m, int direction, int steps) {
  if (steps <= 0) {
    return;
  }

  int delta = (direction >= 0) ? 1 : -1;

  for (int i = 0; i < steps; i++) {
    m.phase = (uint8_t)((m.phase + delta + 8) % 8);
    applyPhase(m, m.phase);
    delay(stepDelayMs);
  }

  releaseMotor(m);
}

void jog(Motor& m, int direction) {
  Serial.print(F("Jog "));
  Serial.print(m.name);
  Serial.print(direction >= 0 ? F("+ ") : F("- "));
  Serial.print(jogSteps);
  Serial.println(F(" steps"));

  stepMotor(m, direction, jogSteps);
  Serial.println(F("Done."));
}

void printHelp() {
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  x+  x-  y+  y-  z+  z-"));
  Serial.println(F("  all           -> x+, x-, y+, y-, z+, z-"));
  Serial.println(F("  steps N       -> set jog steps (1..1024)"));
  Serial.println(F("  speed N       -> set step delay ms (1..15)"));
  Serial.println(F("  help"));
  Serial.println();
}

void runAllTest() {
  jog(motorX, +1);
  delay(250);
  jog(motorX, -1);
  delay(250);

  jog(motorY, +1);
  delay(250);
  jog(motorY, -1);
  delay(250);

  jog(motorZ, +1);
  delay(250);
  jog(motorZ, -1);
}

void setup() {
  setupMotorPins(motorX);
  setupMotorPins(motorY);
  setupMotorPins(motorZ);

  Serial.begin(115200);
  delay(300);

  Serial.println(F("Inkstroke motor direction test ready."));
  Serial.println(F("No auto-move on startup. Use serial commands."));
  Serial.println(F("Default jog = 128 steps (< 1 full rotation)."));
  printHelp();
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "x+") {
    jog(motorX, +1);
  } else if (cmd == "x-") {
    jog(motorX, -1);
  } else if (cmd == "y+") {
    jog(motorY, +1);
  } else if (cmd == "y-") {
    jog(motorY, -1);
  } else if (cmd == "z+") {
    jog(motorZ, +1);
  } else if (cmd == "z-") {
    jog(motorZ, -1);
  } else if (cmd == "all") {
    runAllTest();
  } else if (cmd.startsWith("steps ")) {
    int v = cmd.substring(6).toInt();
    if (v >= 1 && v <= 1024) {
      jogSteps = v;
      Serial.print(F("jogSteps set to "));
      Serial.println(jogSteps);
    } else {
      Serial.println(F("Invalid steps. Use 1..1024"));
    }
  } else if (cmd.startsWith("speed ")) {
    int v = cmd.substring(6).toInt();
    if (v >= 1 && v <= 15) {
      stepDelayMs = v;
      Serial.print(F("stepDelayMs set to "));
      Serial.println(stepDelayMs);
    } else {
      Serial.println(F("Invalid speed. Use 1..15 ms"));
    }
  } else if (cmd == "help") {
    printHelp();
  } else {
    Serial.println(F("Unknown command."));
    printHelp();
  }
}
