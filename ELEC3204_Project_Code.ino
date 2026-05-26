// ======================================================
// FINAL WORKING CONVEYOR CONTROL CODE - UPDATED PINS
// ======================================================

const bool CLOSED_LOOP_MODE = true;

float targetRPM = 1000.0;
float openLoopDuty = 0.70;

float accelTime = 1.5;
float decelTime = 2;

float objectStopDistanceCM = 10.0;

float Kp = 0.03;
float Ki = 0.08;

const float pulsesPerRev = 48.0;

// ======================================================
// UPDATED PIN MAP
// ======================================================

const int ENA = 3; // PWM speed control
const int IN1 = 4; // H-bridge direction input 1
const int IN2 = 5; // H-bridge direction input 2

const int encA = 2; // Encoder A yellow wire, interrupt pin

const int startButtonPin = 6;
const int eStopButtonPin = 7;
const int directionButtonPin = 13;

const int trigPin = 10;
const int echoPin = 11;

// ======================================================
// GLOBAL VARIABLES
// ======================================================

volatile unsigned long pulseCount = 0;

float measuredRPM = 0.0;
float targetNowRPM = 0.0;
float pwmCommand = 0.0;
float integralTerm = 0.0;
float distanceCM = 999.0;

int motorDirection = 1; // 1 = forward, -1 = reverse

enum Mode {
STOPPED,
ACCELERATING,
RUNNING,
DECELERATING,
ESTOPPED
};

Mode mode = STOPPED;

bool lastStartState = HIGH;
bool lastEStopState = HIGH;
bool lastDirState = HIGH;

unsigned long lastControlTime = 0;
unsigned long lastDistanceTime = 0;

const unsigned long controlPeriodMs = 50;
const unsigned long distancePeriodMs = 100;

// ======================================================
// SETUP
// ======================================================

void setup() {
Serial.begin(115200);

pinMode(ENA, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);

pinMode(encA, INPUT_PULLUP);

pinMode(startButtonPin, INPUT_PULLUP);
pinMode(eStopButtonPin, INPUT_PULLUP);
pinMode(directionButtonPin, INPUT_PULLUP);

pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);

attachInterrupt(digitalPinToInterrupt(encA), encoderPulse, RISING);

stopMotor();

lastControlTime = millis();
lastDistanceTime = millis();

Serial.println("TargetRPM MeasuredRPM ErrorRPM Duty DistanceCM Direction Mode");
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
readButtons();

unsigned long now = millis();

if (now - lastDistanceTime >= distancePeriodMs) {
lastDistanceTime = now;

distanceCM = readDistanceCM();

if ((mode == ACCELERATING || mode == RUNNING) &&
distanceCM <= objectStopDistanceCM) {
mode = DECELERATING;
integralTerm = 0.0;
}
}

if (now - lastControlTime >= controlPeriodMs) {
float dt = (now - lastControlTime) / 1000.0;
lastControlTime = now;

measureRPM(dt);

if (mode == STOPPED) {
targetNowRPM = 0.0;
pwmCommand = 0.0;
integralTerm = 0.0;
analogWrite(ENA, 0);
}

else if (mode == ACCELERATING) {
if (CLOSED_LOOP_MODE) {
closedLoopAccelerate(dt);
} else {
openLoopAccelerate(dt);
}
}

else if (mode == RUNNING) {
if (CLOSED_LOOP_MODE) {
targetNowRPM = targetRPM;
runClosedLoop(dt);
} else {
pwmCommand = openLoopDuty * 255.0;
applyMotorPWM();
}
}

else if (mode == DECELERATING) {
if (CLOSED_LOOP_MODE) {
closedLoopDecelerate(dt);
} else {
openLoopDecelerate(dt);
}
}

else if (mode == ESTOPPED) {
stopMotor();
}

printData();
}
}

// ======================================================
// BUTTONS
// ======================================================

void readButtons() {
bool startState = digitalRead(startButtonPin);
bool eStopState = digitalRead(eStopButtonPin);
bool dirState = digitalRead(directionButtonPin);

if (lastStartState == HIGH && startState == LOW) {
if (mode == STOPPED || mode == ESTOPPED) {
startMotor();
}
}

if (lastEStopState == HIGH && eStopState == LOW) {
mode = ESTOPPED;
stopMotor();
}

if (lastDirState == HIGH && dirState == LOW) {
if (mode == STOPPED || mode == ESTOPPED) {
motorDirection = -motorDirection;
}
}

lastStartState = startState;
lastEStopState = eStopState;
lastDirState = dirState;
}

void startMotor() {
targetNowRPM = 0.0;
pwmCommand = 0.0;
integralTerm = 0.0;

setDirection();

mode = ACCELERATING;
}

// ======================================================
// OPEN LOOP CONTROL
// ======================================================

void openLoopAccelerate(float dt) {
float targetPWM = openLoopDuty * 255.0;
float rampRate = targetPWM / accelTime;

pwmCommand += rampRate * dt;

if (pwmCommand >= targetPWM) {
pwmCommand = targetPWM;
mode = RUNNING;
}

applyMotorPWM();
}

void openLoopDecelerate(float dt) {
float targetPWM = openLoopDuty * 255.0;
float rampRate = targetPWM / decelTime;

pwmCommand -= rampRate * dt;

if (pwmCommand <= 0.0) {
stopMotor();
mode = STOPPED;
} else {
applyMotorPWM();
}
}

// ======================================================
// CLOSED LOOP CONTROL
// ======================================================

void closedLoopAccelerate(float dt) {
float rampRate = targetRPM / accelTime;

targetNowRPM += rampRate * dt;

if (targetNowRPM >= targetRPM) {
targetNowRPM = targetRPM;
mode = RUNNING;
}

runClosedLoop(dt);
}

void closedLoopDecelerate(float dt) {
float rampRate = targetRPM / decelTime;

targetNowRPM -= rampRate * dt;

if (targetNowRPM <= 0.0) {
targetNowRPM = 0.0;

pwmCommand -= (255.0 / decelTime) * dt;

if (pwmCommand <= 0.0) {
stopMotor();
mode = STOPPED;
} else {
applyMotorPWM();
}
} else {
runClosedLoop(dt);
}
}

void runClosedLoop(float dt) {
float error = targetNowRPM - measuredRPM;

integralTerm += error * dt;

if (integralTerm > 1000.0) integralTerm = 1000.0;
if (integralTerm < -1000.0) integralTerm = -1000.0;

pwmCommand += Kp * error + Ki * integralTerm * dt;

if (pwmCommand > 255.0) pwmCommand = 255.0;
if (pwmCommand < 0.0) pwmCommand = 0.0;

applyMotorPWM();
}

// ======================================================
// MOTOR OUTPUT
// ======================================================

void setDirection() {
if (motorDirection == 1) {
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
} else {
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
}
}

void applyMotorPWM() {
setDirection();
analogWrite(ENA, (int)pwmCommand);
}

void stopMotor() {
analogWrite(ENA, 0);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);

pwmCommand = 0.0;
targetNowRPM = 0.0;
integralTerm = 0.0;
}

// ======================================================
// RPM MEASUREMENT
// ======================================================

void encoderPulse() {
pulseCount++;
}

void measureRPM(float dt) {
noInterrupts();
unsigned long count = pulseCount;
pulseCount = 0;
interrupts();

measuredRPM = (count / pulsesPerRev) * (60.0 / dt);
}

// ======================================================
// HC-SR04 SENSOR
// ======================================================

float readDistanceCM() {
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

unsigned long duration = pulseIn(echoPin, HIGH, 25000);

if (duration == 0) {
return 999.0;
}

return duration * 0.0343 / 2.0;
}

// ======================================================
// SERIAL OUTPUT
// ======================================================

void printData() {
float errorRPM = abs(targetNowRPM - measuredRPM);
float duty = pwmCommand / 255.0;

Serial.print("TargetRPM:");
Serial.print(targetNowRPM);

Serial.print(" MeasuredRPM:");
Serial.print(measuredRPM);

Serial.print(" ErrorRPM:");
Serial.print(errorRPM);

Serial.print(" Duty:");
Serial.print(duty);

Serial.print(" DistanceCM:");
Serial.print(distanceCM);

Serial.print(" Direction:");
Serial.print(motorDirection);

Serial.print(" Mode:");
Serial.println(mode);
}