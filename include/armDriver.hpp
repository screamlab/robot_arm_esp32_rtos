#pragma once
#include <Adafruit_PWMServoDriver.h>

#include <params.hpp>

/*
 * Use Adafruit_PWMServoDriver to control multiple servos.
 * Pin : use I2C , SDA 21 SCL 22
 *
 */

/**
 * @class ArmManager
 * @brief Manages the servos of a robotic arm using the Adafruit PWM Servo Driver.
 *
 * This class provides methods to initialize and control multiple servos connected to a
 * robotic arm. It allows setting target angles for each servo and moves the servos
 * incrementally towards their target angles.
 *
 * @details
 * The ArmManager class handles the initialization of the Adafruit PWM Servo Driver,
 * sets the PWM frequency, and manages the angles of the servos. It ensures that the
 * servos move smoothly by incrementing their angles in small steps.
 *
 * @note
 * The servo angles are constrained within the specified minimum and maximum angles
 * to prevent damage to the servos or the robotic arm.
 *
 * @param numServos The number of servos to manage.
 * @param servoMinAngles An array of minimum angles for each servo.
 * @param servoMaxAngles An array of maximum angles for each servo.
 * @param servoInitAngles An array of initial angles for each servo.
 */
class ArmManager {
   private:
    static const uint16_t SERVO_MIN_PULSE_WIDTH = 500;
    static const uint16_t SERVO_MAX_PULSE_WIDTH = 2500;

    Adafruit_PWMServoDriver pwm;
    uint8_t numServos;

    uint8_t *servoTargetAngles;
    float *servoCurrentAngles;
    uint8_t *servoMinAngles;
    uint8_t *servoMaxAngles;

    /**
     * @brief Sets the angle of a specified servo.
     *
     * @param servoNum The index of the servo to set the angle for.
     * @param angle The target angle to set for the servo.
     */
    void setServoAngle(uint8_t servoNum, float angle);

   public:
    /**
     * @brief Constructs an ArmManager object.
     *
     * @param numServos The number of servos to manage.
     * @param servoMinAngles An array of minimum angles for each servo.
     * @param servoMaxAngles An array of maximum angles for each servo.
     * @param servoInitAngles An array of initial angles for each servo.
     */
    ArmManager(
        const uint8_t numServos, const uint8_t servoMinAngles[],
        const uint8_t servoMaxAngles[], const uint8_t servoInitAngles[]);

    /**
     * @brief Sets the target angle for a specified servo.
     *
     * @param servoNum The index of the servo to set the target angle for.
     * @param targetAngle The target angle to set for the servo.
     */
    void setServoTargetAngle(uint8_t servoNum, uint8_t targetAngle);

    /**
     * @brief Changes the target angle of a specified servo by a bias angle.
     *
     * @param servoNum The index of the servo to change the target angle for.
     * @param biasAngle The bias angle to add to the current target angle.
     */
    void changeServoTargetAngle(uint8_t servoNum, int8_t biasAngle);

    /**
     * @brief Gets the current angles of all servos.
     *
     * @param currentAngles An array to store the current angles of all servos.
     */
    void getCurrentAngles(float currentAngles[]);

    /**
     * @brief Moves the arm by incrementing the angles of the servos towards their target angles.
     */
    void moveArm();
};
