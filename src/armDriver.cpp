#include <armDriver.hpp>

void ArmManager::setServoAngle(uint8_t servoNum, float angle) {
    if (servoNum < this->numServos) {
        pwm.writeMicroseconds(
            servoNum,
            map(
                angle, 0, 180,
                SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH));
    }
}

ArmManager::ArmManager(
    const uint8_t numServos, const uint8_t servoMinAngles[],
    const uint8_t servoMaxAngles[], const uint8_t servoInitAngles[]) {
    // Initialize the Adafruit_PWMServoDriver object
    this->numServos = numServos;

    this->pwm.begin();
    this->pwm.setOscillatorFrequency(27000000);
    this->pwm.setPWMFreq(50);  // Set the PWM frequency to 50Hz
    this->servoTargetAngles = new uint8_t[numServos];
    this->servoCurrentAngles = new float[numServos];
    this->servoMinAngles = new uint8_t[numServos];
    this->servoMaxAngles = new uint8_t[numServos];
    // Set servoMinAngles and servoMaxAngles
    for (uint8_t i = 0; i < numServos; ++i) {
        this->servoMinAngles[i] = servoMinAngles[i];
        this->servoMaxAngles[i] = servoMaxAngles[i];
        this->servoTargetAngles[i] = servoInitAngles[i];
        // this->servoTargetAngles[i] = 90;
        /*************************************************************************
        You cannot set the current angles by reading the initial angles directly.
        You can set the current angles to be "near" the initial angles
            to ensure that the robot arm doesn't perform redundant actions.
        If you set the current angles to be the same as the initial angles,
            then the robot arm will not move when starting up.
        **************************************************************************/
        this->servoCurrentAngles[i] = (float)servoInitAngles[i] + 1;
    }
}

void ArmManager::setServoTargetAngle(uint8_t servoNum, uint8_t targetAngle) {
    if (servoNum < this->numServos) {
        this->servoTargetAngles[servoNum] = constrain(targetAngle, servoMinAngles[servoNum],
                                                      servoMaxAngles[servoNum]);
    }
}

void ArmManager::changeServoTargetAngle(uint8_t servoNum, int8_t biasAngle) {
    if (servoNum < this->numServos) {
        this->servoTargetAngles[servoNum] = constrain(servoTargetAngles[servoNum] + biasAngle,
                                                      servoMinAngles[servoNum], servoMaxAngles[servoNum]);
    }
}

// Get the current angles of all servos, update currentAngles into passed array
void ArmManager::getCurrentAngles(float currentAngles[]) {
    for (uint8_t i = 0; i < this->numServos; ++i) {
        currentAngles[i] = servoCurrentAngles[i];
    }
}

void ArmManager::moveArm() {
    for (uint8_t i = 0; i < this->numServos; ++i) {
        if (abs(this->servoCurrentAngles[i] - this->servoTargetAngles[i]) >= ARM_MOVEMENT_STEP) {
            float step = (this->servoTargetAngles[i] > this->servoCurrentAngles[i]) ? ARM_MOVEMENT_STEP : -ARM_MOVEMENT_STEP;
            this->servoCurrentAngles[i] += step;
            // this line will occur some delay to let device work not properly
            // please make sure you had connect to PCA9685 pwm driver.
            setServoAngle(i, this->servoCurrentAngles[i]);
        }
    }
}
