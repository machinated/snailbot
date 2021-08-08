/*******************************************************************************
*  Sn@il robot firmware
*
*  Copyright (C) 2021 Michał Cieśnik
*
*  Commands:
*     MOVE <DISTANCE>   -- Move the robot in a straigt line by the specified
*                          number of centimeters. Move forward if the argument
*                          is positive, backward if it is negative.
*
*     TURN <DEGREES>    -- Turn in place by the specified angle given in degrees.
*                          Turn left if the argument is positive, right if it
*                          is negative.
*
*     GOTO <DISTANCE>   -- Move forward or backward to reach the specified distance
*                          in centimiters from the nearest object in front.
*
*     LEFT <STEPS>      -- Move only the left motor by the given number of steps.
*
*     RIGHT <STEPS>     -- Move only the right motor by the given number of steps.
*
*     STOP              -- Abort the current movement command.
*
*     SPEED <STEPS/S>   -- Set the maximum speed of the motors.
                           The default is 1000 steps per second or 2.3 cm/s.
*
*     ACCEL <STEPS/^2>  -- Set the acceleration of the motors.
*                          The default is 2000 steps per second^2.
*
*     BATT              -- Print the measuered battery voltage.
*
*     DIST              -- Measure the disctance to the nearest object in front
*                          using the ultrasonic sensor and print the result.
*
*     OFF               -- Power off.
*/

#include <string.h>
#include <stdlib.h>

#include <AccelStepper.h>

#define POWER_ENABLE_PIN 18
#define POWER_BUTTON_PIN 19

#define TRIG_PIN 9
#define ECHO_PIN 3         /* can be attached to INT1, currently isn't */

#define BATTERY_PIN A0

#define LED_PIN 8
#define BUTTON_PIN 2

#define STEPS_PER_CM 439.41
#define STEPS_PER_DEGREE 45.98

#define MIN_BATTERY_VOLTAGE 461     /* 2.95V */

#define GOTO_UPDATE_INTERVAL 2000
#define DEFAULT_SPEED 1000
#define DEFAULT_ACCELERATION 2000

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

typedef enum Command {
   COMMAND_NONE,
   COMMAND_MOVE,
   COMMAND_TURN,
   COMMAND_GOTO,
   COMMAND_STOP,
   COMMAND_LEFT,
   COMMAND_RIGHT,
   COMMAND_OFF
} Command;

Command currentCommand = COMMAND_NONE;

AccelStepper motorLeft(AccelStepper::HALF4WIRE, 10, 12, 13, 11);
AccelStepper motorRight(AccelStepper::HALF4WIRE, 7, 5, 4, 6);

char command[5];
char argument[5];
bool isArg;
byte inputLen;

/* Called on incoming serial data */
void serialEvent()
{
   while(Serial.available())
   {
      char inChar = (char)Serial.read();
      if (inChar == '\n')
      {
         if (isArg)
         {
            argument[inputLen] = '\0';
         }
         else
         {
            command[inputLen] = '\0';
         }
         isArg = false;
         inputLen = 0;
         parseInput();
      }
      else if (inChar == ' ' && !isArg)
      {
         command[inputLen] = '\0';
         isArg = true;
         inputLen = 0;
      }
      else if (inputLen < 5)
      {
         if (isArg)
         {
            argument[inputLen] = inChar;
         }
         else
         {
            command[inputLen] = inChar;
         }
         inputLen++;
      }
   }
}

void parseInput()
{
   long parsedArg = strtol(argument, NULL, 10);

   if (strncmp(command, "STOP", 5) == 0)
   {
      currentCommand = COMMAND_STOP;
      cmdStop();
   }
   /* Commands other than STOP cannot interrupt current command */
   else if (currentCommand != COMMAND_NONE) {
      Serial.println("Still executing previous command");
   }
   else if (strncmp(command, "LEFT", 5) == 0)
   {
      currentCommand = COMMAND_LEFT;
      cmdRunMotor(MOTOR_LEFT, parsedArg);
   }
   else if (strncmp(command, "RIGHT", 5) == 0)
   {
      currentCommand = COMMAND_RIGHT;
      cmdRunMotor(MOTOR_RIGHT, parsedArg);
   }
   else if (strncmp(command, "SPEED", 5) == 0)
   {
      cmdSetSpeed(parsedArg);
   }
   else if (strncmp(command, "ACCEL", 5) == 0)
   {
      cmdSetAcceleration(parsedArg);
   }
   else if (strncmp(command, "MOVE", 5) == 0)
   {
      currentCommand = COMMAND_MOVE;
      cmdMove(parsedArg);
   }
   else if (strncmp(command, "TURN", 5) == 0)
   {
      currentCommand = COMMAND_TURN;
      cmdTurn(parsedArg);
   }
   else if (strncmp(command, "GOTO", 5) == 0)
   {
      currentCommand = COMMAND_GOTO;
      cmdGoto(parsedArg);
   }
   else if (strncmp(command, "DIST", 5) == 0)
   {
      cmdDistance();
   }
   else if (strncmp(command, "BATT", 5) == 0)
   {
      cmdBatt();
   }
   else if (strncmp(command, "OFF", 5) == 0)
   {
      currentCommand = COMMAND_OFF;
      cmdPowerOff();
   }
   else
   {
      Serial.print("Unknown command: ");
      Serial.println(command);
   }
}

void cmdRunMotor(byte motor, long steps)
{
   if (motor == MOTOR_LEFT)
   {
      Serial.print("Advancing left motor by ");
      Serial.print(steps);
      Serial.println(" steps");
      motorLeft.move(steps);
   }
   else if (motor == MOTOR_RIGHT)
   {
      Serial.print("Advancing right motor by ");
      Serial.print(steps);
      Serial.println(" steps");
      motorRight.move(steps);
   }
   else
   {
      return;
   }
}

void cmdMove(int distance)
{
   long steps = (long) distance * STEPS_PER_CM;
   Serial.print("Moving by ");
   Serial.print(distance);
   Serial.println(" cm");
   motorLeft.move(steps);
   motorRight.move(steps);
}

void cmdStop()
{
   if (currentCommand == COMMAND_NONE)
   {
      Serial.println("Already stopped");
      return;
   }
   Serial.println("Stopping");
   motorLeft.stop();
   motorRight.stop();
}

long gotoLastUpdate;
int gotoTarget;

void cmdGoto(int targetDistance)
{
   Serial.print("Moving to distance ");
   Serial.print(targetDistance);
   Serial.println(" cm");
   gotoTarget = targetDistance;
   gotoUpdate();
}

void gotoUpdate()
{
   int moveBy = measureDistance() - gotoTarget;
   long steps = (long) moveBy * STEPS_PER_CM;
   motorLeft.move(steps);
   motorRight.move(steps);
}

void cmdTurn(int degrees)
{
   long steps = (long) degrees * STEPS_PER_DEGREE;
   Serial.print("Turning by ");
   Serial.print(degrees);
   Serial.println(" degrees.");
   motorLeft.move(-steps);
   motorRight.move(steps);
}

void cmdSetSpeed(float speed)
{
   Serial.print("Speed set to ");
   Serial.println(speed);
   motorLeft.setMaxSpeed(speed);
   motorRight.setMaxSpeed(speed);
}

void cmdSetAcceleration(float acceleration)
{
   Serial.print("Acceleration set to ");
   Serial.println(acceleration);
   motorLeft.setAcceleration(acceleration);
   motorRight.setAcceleration(acceleration);
}

void cmdDistance()
{
   Serial.print("Distance: ");
   Serial.print(measureDistance());
   Serial.println(" cm");
}

void cmdBatt()
{
   int reading = analogRead(BATTERY_PIN);
   float voltage = (reading * 5.0) / 1024 + 0.7;
   Serial.print("Battery voltage: ");
   Serial.print(voltage, 2);
   Serial.println("V");
}

void cmdPowerOff()
{
   Serial.println("Powering off now. Bye bye...");
   powerOff();
}

int measureDistance()
{
   digitalWrite(TRIG_PIN, LOW);
   delayMicroseconds(5);
   digitalWrite(TRIG_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG_PIN, LOW);
   long duration = pulseIn(ECHO_PIN, HIGH);
   return (int) duration / 58.2;
}

void powerOff()
{
   for(int i=0; i<4 || digitalRead(POWER_BUTTON_PIN); i++)
   {
      /* flash LED */
      digitalWrite(LED_PIN, LOW);
      delay(125);
      digitalWrite(LED_PIN, HIGH);
      delay(125);
   }
   digitalWrite(LED_PIN, LOW);
   delay(500);
   digitalWrite(POWER_ENABLE_PIN, LOW);
   pinMode(POWER_ENABLE_PIN, INPUT);
   while(true)
   {
      ;           /* wait for regulator shutdown */
   }
}

long lastBlink = 0;
bool ledOn = false;
byte cycle = 0;

/* Arduino library will execute function named 'setup' once at the start */
void setup()
{
   pinMode(POWER_ENABLE_PIN, OUTPUT);
   pinMode(TRIG_PIN, OUTPUT);
   pinMode(LED_PIN, OUTPUT);

   /* keep power regulator on after power button is released */
   digitalWrite(POWER_ENABLE_PIN, HIGH);
   digitalWrite(LED_PIN, HIGH);

   if (!digitalRead(POWER_BUTTON_PIN))
   {
      powerOff();
   }

   while(digitalRead(POWER_BUTTON_PIN))  /* wait for power button release */
   {
      delay(1);
   }
   digitalWrite(LED_PIN, LOW);

   motorLeft.setMaxSpeed(DEFAULT_SPEED);
   motorRight.setMaxSpeed(DEFAULT_SPEED);
   motorLeft.setAcceleration(DEFAULT_ACCELERATION);
   motorRight.setAcceleration(DEFAULT_ACCELERATION);

   Serial.begin(9600);
   /* ignore the starting "ATAT" sent by bluetooth adapter */
   Serial.readBytes((char*) NULL, 10);
   Serial.println("Sn@il firmware version 0.4.0");
   Serial.println("Accepting commands.");

   lastBlink = millis();
}

/* Arduion library will execute this function continuously after setup() returns */
void loop()
{
   /* check battery voltage once every 256 loops */
   if (cycle++ == 0 && analogRead(BATTERY_PIN) < MIN_BATTERY_VOLTAGE)
   {
      Serial.println("Battery critical. Powering off.");
      powerOff();
   }
   if (digitalRead(POWER_BUTTON_PIN))
   {
      Serial.println("Manual power off.");
      powerOff();
   }
   /* blink the status LED every 1s */
   if (millis() - lastBlink >= 1000)
   {
      lastBlink = millis();
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn);
   }
   if (currentCommand == COMMAND_GOTO)
   {
      if (abs(motorLeft.currentPosition() - gotoLastUpdate) >= GOTO_UPDATE_INTERVAL)
      {
         gotoLastUpdate = motorLeft.currentPosition();
         gotoUpdate();
      }
   }
   if (currentCommand != COMMAND_NONE && !motorLeft.isRunning() && !motorRight.isRunning())
   {
      currentCommand = COMMAND_NONE;
      Serial.println("Done");
      motorLeft.disableOutputs();
      motorRight.disableOutputs();
   }
   motorLeft.run();
   motorRight.run();
}
