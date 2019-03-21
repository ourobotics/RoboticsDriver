// Included Libraries
#include <Servo.h>

//////////////////////////////////////
// Config Variables
// Distance to turn
int turnLeftDistance = 280;
int turnRightDistance = 300;
// How long to wait after looking to get data
int getSensorDistanceOneDelay = 1000;
// Wheel speed
int rightWheelSpeed = 255;
int leftWheelSpeed = 255;
// Distance forward to check left and right
float forwardDistanceCheck = 6.0;
float downwardDistanceCheck = 5.0;
//////////////////////////////////////


//////////////////////////////////////
// Global Variables
Servo servo;

// potentiometer Pin
int potentiometerPinSpeed = 14;
int potentiometerPinTurnLeft = 15;
int potentiometerPinTurnRight = 16;

// Used pause our loop()
int finishedState = 1;
float finishedStateDistance = 6.0;

// Right Wheel H-Bridge Pins
int rightPWM = 3;
int rightDigitalOne = 2;
int rightDigitalTwo = 4;

// Left Wheel H-Bridge Pins
int leftPWM = 6;
int leftDigitalOne = 5;
int leftDigitalTwo = 7;

// Ultrasonic Sensor Pins
int triggerPinOne = 12;
int echoPinOne = 11;

int triggerPinTwo = 13;
int echoPinTwo = 10;

// Servo Pin
int servoPin = 9;
//////////////////////////////////////

void setupServo() {
    servo.attach(servoPin);
}

void setupUltrasonicSensorOne() {
    pinMode(triggerPinOne, OUTPUT);
    pinMode(echoPinOne, INPUT);
}

void setupUltrasonicSensorTwo() {
    pinMode(triggerPinTwo, OUTPUT);
    pinMode(echoPinTwo, INPUT);
}

void setupRightWheel() {
    // Digital Output
    pinMode(rightDigitalOne, OUTPUT);
    // PWM Output
    pinMode(rightPWM, OUTPUT);
    // Digital Output
    pinMode(rightDigitalTwo, OUTPUT);
    
    // Set Right Motor Direction
    digitalWrite(rightDigitalOne, LOW);
    digitalWrite(rightDigitalTwo, HIGH);
}

void setupLeftWheel() {
    // Digital Output
    pinMode(leftDigitalOne, OUTPUT);
    // PWM Output
    pinMode(leftPWM, OUTPUT);
    // Digital Output
    pinMode(leftDigitalTwo, OUTPUT);
    
    // Set left Motor Direction
    digitalWrite(leftDigitalOne, LOW);
    digitalWrite(leftDigitalTwo, HIGH);
}

void setup() {
    Serial.begin(115200);
    setupRightWheel();
    setupLeftWheel();
    setupUltrasonicSensorOne();
    setupUltrasonicSensorTwo();
    setupServo();
}

void turnLeft() {
    digitalWrite(leftDigitalOne, HIGH);
    digitalWrite(leftDigitalTwo, LOW);
    digitalWrite(rightDigitalOne, LOW);
    digitalWrite(rightDigitalTwo, HIGH);

    analogWrite(rightPWM, rightWheelSpeed);
    analogWrite(leftPWM, leftWheelSpeed);

    delay(turnLeftDistance);

    analogWrite(rightPWM, 0);
    analogWrite(leftPWM, 0);

    digitalWrite(leftDigitalOne, LOW);
    digitalWrite(leftDigitalTwo, HIGH);
    digitalWrite(rightDigitalOne, LOW);
    digitalWrite(rightDigitalTwo, HIGH);
}

void turnRight() {
    digitalWrite(leftDigitalOne, LOW);
    digitalWrite(leftDigitalTwo, HIGH);
    digitalWrite(rightDigitalOne, HIGH);
    digitalWrite(rightDigitalTwo, LOW);

    analogWrite(rightPWM, rightWheelSpeed);
    analogWrite(leftPWM, leftWheelSpeed);

    delay(turnRightDistance);

    analogWrite(rightPWM, 0);
    analogWrite(leftPWM, 0);

    digitalWrite(leftDigitalOne, LOW);
    digitalWrite(leftDigitalTwo, HIGH);
    digitalWrite(rightDigitalOne, LOW);
    digitalWrite(rightDigitalTwo, HIGH);
}

float getSensorDistanceOne() {
    long duration;
    float distance;

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPinOne, HIGH);
    delay(10);
    digitalWrite(triggerPinOne, LOW);
    // Reads the echoPinOne, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPinOne, HIGH);
    float microseconds = duration;
    // Calculating the distance
    distance = microsecondsToInches(microseconds);

    return distance;
}

int getpotentiometerPositionSpeed() {
    int position = analogRead(potentiometerPinSpeed);

    if (position > 512) {
        position = position - 512;
    }
    else if (position  < 512) {
        position = position - 512;
    }
    position = position / 8;
    return position;
}

void updateWheelSpeed(int position) {
    rightWheelSpeed = 255;
    leftWheelSpeed = 255;
    if (position > 0) {
        rightWheelSpeed -= position;
    }
    if (position < 0) {
        leftWheelSpeed -= position * -1;
    }
}

int getpotentiometerPositionTurnLeft() {
    int position = analogRead(potentiometerPinTurnLeft);
    // position = position / 8;
    return position;
}

void updateTurnLeftDistance(int position) {
    turnLeftDistance = position;
}

int getpotentiometerPositionTurnRight() {
    int position = analogRead(potentiometerPinTurnRight);
    // position = position / 8;
    return position;
}

void updateTurnRightDistance(int position) {
    turnRightDistance = position;
}

float getSensorDistanceTwo() {
    long duration;
    float distance;

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPinTwo, HIGH);
    delay(10);
    digitalWrite(triggerPinTwo, LOW);
    // Reads the echoPinOne, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPinTwo, HIGH);
    float microseconds = duration;
    // Calculating the distance
    distance = microsecondsToInches(microseconds);

    return distance;
}

float microsecondsToInches(float microseconds) {
    // This is tuned pretty well
    float conversionVariable = 69.0;
    return (float)microseconds / conversionVariable / 2.0;
}

float microsecondsToCentimeters(float microseconds) {
    // This is not tuned at all
    float conversionVariable = 29.0;
    return (float)microseconds / conversionVariable / 2.0;
}

void moveForward() {
    digitalWrite(rightDigitalOne, LOW);
    digitalWrite(rightDigitalTwo, HIGH);
    digitalWrite(leftDigitalOne, LOW);
    digitalWrite(leftDigitalTwo, HIGH);

    analogWrite(rightPWM, rightWheelSpeed);
    analogWrite(leftPWM, leftWheelSpeed);
}

void moveBackwards() {
    digitalWrite(rightDigitalOne, HIGH);
    digitalWrite(rightDigitalTwo, LOW);
    digitalWrite(leftDigitalOne, HIGH);
    digitalWrite(leftDigitalTwo, LOW);

    analogWrite(rightPWM, rightWheelSpeed);
    analogWrite(leftPWM, leftWheelSpeed);
}

void stopMoving() {
  
    analogWrite(rightPWM, 0);
    analogWrite(leftPWM, 0);
}

void lookLeft() {
    servo.write(180);
    Serial.println("180 degrees");
}

void lookRight() {
    servo.write(0);
    Serial.println("0 degrees");
}

void lookForward() {
    servo.write(90);
    Serial.println("90 degrees");
}

// void loop() {
//     updateWheelSpeed(getpotentiometerPositionSpeed());
//     Serial.print("Left: ");
//     Serial.println(leftWheelSpeed);
//     Serial.print("Right: ");
//     Serial.println(rightWheelSpeed);
//     delay(500);
// }

// void loop() {
//     Serial.print("Turn Left Pot: ");
//     Serial.println(getpotentiometerPositionTurnLeft());
//     Serial.print("Speed Pot: ");
//     Serial.println(getpotentiometerPositionSpeed());
//     Serial.print("Turn Rigth Pot: ");
//     Serial.println(getpotentiometerPositionTurnRight());
//     Serial.println("");
//     delay(1000);
// }

void loop() {
    if (finishedState == 1) {
        /////////////////////////////////////////////////////
        // Wheel Pot
        updateWheelSpeed(getpotentiometerPositionSpeed());
        // Serial.print("Left: ");
        // Serial.println(leftWheelSpeed);
        // Serial.print("Right: ");
        // Serial.println(rightWheelSpeed);

        /////////////////////////////////////////////////////
        // Turn Left Pot
        updateTurnLeftDistance(getpotentiometerPositionTurnLeft());
        // Serial.print("Turn Left Pot: ");
        // Serial.println(turnLeftDistance);

        /////////////////////////////////////////////////////
        // Turn Right Pot
        updateTurnRightDistance(getpotentiometerPositionTurnRight());
        // Serial.print("Turn Right Pot: ");
        // Serial.println(turnRightDistance);

        /////////////////////////////////////////////////////


        moveForward();
        float distanceDownward = getSensorDistanceTwo();
        //Serial.println(distanceDownward);

        if (distanceDownward > downwardDistanceCheck) {
            stopMoving();
            moveBackwards();
            delay(500);
            stopMoving();
            turnLeft();
        }

        float distanceForward = getSensorDistanceOne();
        // Serial.println(distanceForward);
        if (distanceForward < forwardDistanceCheck) {
            stopMoving();
            lookLeft();
            // This needs Tuned
            delay(getSensorDistanceOneDelay);
            float distanceLeft = getSensorDistanceOne();
            Serial.println(distanceLeft);
            lookRight();
            // This needs Tuned
            delay(getSensorDistanceOneDelay);
            float distanceRight = getSensorDistanceOne();
            Serial.println(distanceRight);
            lookForward();
            if ((distanceForward < finishedStateDistance) and (distanceLeft < finishedStateDistance) and (distanceRight < finishedStateDistance)) {
                finishedState = 0;
            }
            else if (distanceLeft > distanceRight) {
                Serial.println("Turn Left");
                turnLeft();
            }
            else {
                Serial.println("Turn Right");
                turnRight();
            }
        }
    }
}