#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

#define DIR_1 3
#define DIR_2 4
#define DIR_3 5
#define STEP_1 6
#define STEP_2 7
#define STEP_3 8
#define EN 9
#define arduinoLED 13   // Arduino LED on board

AccelStepper stepper_1(1, STEP_1, DIR_1);
AccelStepper stepper_2(1, STEP_2, DIR_2);
AccelStepper stepper_3(1, STEP_3, DIR_3);

void setup() {
  pinMode(arduinoLED, OUTPUT);      // Configure the onboard LED for output
  digitalWrite(arduinoLED, LOW);    // default to LED off

  Serial.begin(115200);

  stepper_1.setMaxSpeed(6000); // Set maximum speed for the motor 1 (steps/s)
  stepper_1.setAcceleration(6000); // Set acceleration for the motor 1 (steps/s^2)
  stepper_1.setCurrentPosition(0);

  stepper_2.setMaxSpeed(6000); // Set maximum speed for the motor 1 (steps/s)
  stepper_2.setAcceleration(6000); // Set acceleration for the motor 1 (steps/s^2)
  stepper_2.setCurrentPosition(0);

  stepper_3.setMaxSpeed(6000); // Set maximum speed for the motor 3 (steps/s)
  stepper_3.setAcceleration(6000); // Set acceleration for the motor 3 (steps/s^2)
  stepper_3.setCurrentPosition(0);
  
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

}

void loop() {
  
  while (Serial.available() > 0) {
    char command[700]; // Assuming a maximum command length of 700 characters
    clearCharArray(command,sizeof(command));
    Serial.readBytesUntil('\n', command, sizeof(command));
    //Serial.println(command);
    MoveToPosition(command);
    MoveTrajectory(command);
  }  
}

void MoveToPosition(const char* command) {
  long steps1 = 0;
  long steps2 = 0;
  long steps3 = 0;

  if (command[0] == 'M') {
    steps1 = ExtractValue(command, 'A');
    steps2 = ExtractValue(command, 'B');
    steps3 = ExtractValue(command, 'C');
    MoveMotors(steps1, steps2, steps3);
  }
}

void MoveTrajectory(const char* command) {
  if (command[0] == 'I') {
    char tempCommand[700]; // Assuming a maximum command length of 700 characters
    strcpy(tempCommand, command);
    char* token = strtok(tempCommand, ",");
    while (token != NULL) {
      MoveToPosition(token);
      delay(5);
      token = strtok(NULL, ",");
    }
  }
}

void MoveMotors(float steps_1, float steps_2, float steps_3) {
  stepper_1.moveTo(steps_1);
  stepper_2.moveTo(steps_2);
  stepper_3.moveTo(steps_3);
  while (stepper_1.currentPosition() != steps_1 || stepper_2.currentPosition() != steps_2 || stepper_3.currentPosition() != steps_3) {
    stepper_1.run();
    stepper_2.run();
    stepper_3.run();
  }
}

float ExtractValue(const char* linea, char eje) {
  const char* index = strchr(linea, eje);
  if (index == NULL) {
    return 0;  
  }
  return atof(index + 1);
}

void clearCharArray(char* charArray, size_t size) {
  for (size_t i = 0; i < size; i++) {
    charArray[i] = '\0';  // Fill each element with null character
  }
}

