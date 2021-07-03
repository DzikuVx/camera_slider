// #include <Wire.h>
// #include <Adafruit_VL53L0X.h>
#include "QmuTactile.h"
// #include "filter.h"
// #include "QmuPid.h"

#define PID_UPDATE_TASK_MS 50

#define PIN_STEP1_DIRECTION 13
#define PIN_STEP1_STEP 12

#define PIN_STEP2_DIRECTION 18
#define PIN_STEP2_STEP 5

#define PIN_END_STOP 17

#define PIN_STEP_ENABLE 14

#define PIN_BUTTON_1 21
#define PIN_BUTTON_2 22
#define PIN_BUTTON_3 19
#define PIN_BUTTON_4 23

#define I2C1_SDA_PIN 4
#define I2C1_SCL_PIN 15

#define I2C2_SDA_PIN 27
#define I2C2_SCL_PIN 26

// TwoWire I2C1 = TwoWire(0); //OLED bus
// TwoWire I2C2 = TwoWire(1); //VL53L0X bus

QmuTactile button1(PIN_BUTTON_1);
QmuTactile button2(PIN_BUTTON_2);
QmuTactile button3(PIN_BUTTON_3);
QmuTactile button4(PIN_BUTTON_4);
QmuTactile buttonEnd(PIN_END_STOP);

TaskHandle_t driverTask;
// TaskHandle_t driverPid;

// QmuPid distanceToVelocity(1.0f, 0.0f, 0.0f, 0.0f);
// QmuPid velocityToStep(10, 1.0f, 0.5f, 0);

// QmuPid distanceToStep(0.1, 0, 50, 0);

// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

enum systemFlags_e
{
  SYSTEM_FLAG_NONE = 0,                  //  0
  SYSTEM_FLAG_NEEDS_CALIBRATION = 1 << 0,  //  1
  SYSTEM_FLAG_CALIBRATING = 1 << 1,     //  2
  SYSTEM_FLAG_OPERATIONAL = 1 << 2, // 4 
  SYSTEM_FLAG_FORCE_FORWARD = 1 << 3, // 8 
  SYSTEM_FLAG_MOVE_TO_POSITION = 1 << 4, // 8 
  SYSTEM_FLAG_EMERGENCY_STOP = 1 << 5, // 8 
};


uint8_t _systemFlags;

void setup()
{

    Serial.begin(115200);

    // pinMode(16, OUTPUT);
    // digitalWrite(16, LOW); // set GPIO16 low to reset OLED
    // delay(50);
    // digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high

    // I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 50000);
    // I2C2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 400000);

    // if (!lox.begin(0x29, false, &I2C2, Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE))
    // {
    //     Serial.println(F("Failed to boot VL53L0X"));
    //     while(1);
    // }

    pinMode(PIN_STEP1_DIRECTION, OUTPUT);
    pinMode(PIN_STEP1_STEP, OUTPUT);
    pinMode(PIN_STEP_ENABLE, OUTPUT);

    digitalWrite(PIN_STEP1_DIRECTION, LOW);
    digitalWrite(PIN_STEP1_STEP, LOW);

    digitalWrite(PIN_STEP_ENABLE, LOW);

    button1.start();
    button2.start();
    button3.start();
    button4.start();
    buttonEnd.start();

    _systemFlags |= SYSTEM_FLAG_NEEDS_CALIBRATION;

    // distanceToVelocity.setProperties(-100.0f, 100.0f);
    // distanceToVelocity.setItermProperties(-10.0f, 10.0f);
    // distanceToVelocity.setSetpoint(150.0f);

    // velocityToStep.setProperties(-800.0f, 800.0f);
    // velocityToStep.setItermProperties(-50.0f, 50.0f);


    // distanceToStep.setProperties(-500.0f, 500.0f);
    // distanceToStep.setItermProperties(-100.0f, 100.0f);
    // distanceToStep.setSetpoint(150.0f);

    // xTaskCreatePinnedToCore(
    //     driverTaskHandler, /* Function to implement the task */
    //     "driverTask",              /* Name of the task */
    //     10000,                  /* Stack size in words */
    //     NULL,                   /* Task input parameter */
    //     0,                      /* Priority of the task */
    //     &driverTask,       /* Task handle. */
    //     0);

    // xTaskCreatePinnedToCore(
    //     pidTaskHandler, /* Function to implement the task */
    //     "driverPid",              /* Name of the task */
    //     10000,                  /* Stack size in words */
    //     NULL,                   /* Task input parameter */
    //     0,                      /* Priority of the task */
    //     &driverPid,       /* Task handle. */
    // 0);

}

volatile int stepsPerSecond[2] = {0};

int32_t currentPosition[2] = {0};
int32_t targetPosition[2] = {0};

void loop()
{
	button1.loop();
	button2.loop();
	button3.loop();
	button4.loop();
    buttonEnd.loop();

    if (button1.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetPosition[0] = 1000;
    }

    if (button2.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetPosition[0] = 5000;
    }

    if (button3.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetPosition[0] = 0;
    }

    if (button4.getState() == TACTILE_STATE_SHORT_PRESS) {
        targetPosition[0] = 81000;
    }

    if ((_systemFlags & SYSTEM_FLAG_NEEDS_CALIBRATION) && !buttonEnd.checkFlag(TACTILE_FLAG_PRESSED)) {
        stepsPerSecond[0] = 0;
        _systemFlags &= ~SYSTEM_FLAG_NEEDS_CALIBRATION;
        _systemFlags |= SYSTEM_FLAG_CALIBRATING;
    }

    if ((_systemFlags & SYSTEM_FLAG_NEEDS_CALIBRATION) && buttonEnd.checkFlag(TACTILE_FLAG_PRESSED)) {
        _systemFlags |= SYSTEM_FLAG_FORCE_FORWARD;
        stepsPerSecond[0] = 100;
    }

    if (_systemFlags & SYSTEM_FLAG_CALIBRATING) {
        if (!buttonEnd.checkFlag(TACTILE_FLAG_PRESSED)) {
            stepsPerSecond[0] = -5000;
        } else {
            stepsPerSecond[0] = 0;
            _systemFlags &= ~SYSTEM_FLAG_CALIBRATING;
            currentPosition[0] = -700;
            targetPosition[0] = 0;
            _systemFlags |= SYSTEM_FLAG_MOVE_TO_POSITION;
        }
    }

    if (_systemFlags & SYSTEM_FLAG_OPERATIONAL && buttonEnd.checkFlag(TACTILE_FLAG_PRESSED)) {
        _systemFlags |= SYSTEM_FLAG_EMERGENCY_STOP;
    }

#define MIN_SPEED 200
#define MAX_SPEED 3000
#define SPEED_CONTROLLER_P 8.0f

    /*
     * Position to speed
     */
    if (_systemFlags & SYSTEM_FLAG_MOVE_TO_POSITION) {

        targetPosition[0] = constrain(targetPosition[0], 0, 81000);

        int32_t error = targetPosition[0] - currentPosition[0];

        int32_t targetSpeed = error * SPEED_CONTROLLER_P;

        if (targetSpeed > MAX_SPEED) {
            targetSpeed = MAX_SPEED;
        } else if (targetSpeed < -MAX_SPEED) {
            targetSpeed = -MAX_SPEED;
        } else if (targetSpeed < MIN_SPEED && targetSpeed > 0) {
            targetSpeed = MIN_SPEED;
        } else if (targetSpeed > -MIN_SPEED && targetSpeed < 0) {
            targetSpeed = -MIN_SPEED;
        }
        
        if (error != 0) {
            stepsPerSecond[0] = targetSpeed;
        } else {
            _systemFlags |= SYSTEM_FLAG_OPERATIONAL;
            stepsPerSecond[0] = 0;
        }
    }

    static unsigned long nextChange = 0;
    static uint8_t currentState = LOW;

    if (_systemFlags & SYSTEM_FLAG_EMERGENCY_STOP) {
        digitalWrite(PIN_STEP1_DIRECTION, LOW);
        digitalWrite(PIN_STEP1_STEP, LOW);
    } else {
        
        if (stepsPerSecond[0] == 0) {
            currentState = LOW;
            digitalWrite(PIN_STEP1_STEP, LOW);
        } else {
            if (micros() > nextChange) {

                if (currentState == LOW) {
                    currentState = HIGH;
                    nextChange = micros() + 20;

                    if (stepsPerSecond[0] > 0) {
                        currentPosition[0]++;
                    } else if (stepsPerSecond[0] < 0) {
                        currentPosition[0]--;
                    }
                } else {
                    currentState = LOW;
                    nextChange = micros() + (1000 * abs(1000.0f / stepsPerSecond[0])) - 20;
                }

                if (stepsPerSecond[0] > 0) {
                    digitalWrite(PIN_STEP1_DIRECTION, LOW);
                } else {
                    digitalWrite(PIN_STEP1_DIRECTION, HIGH);
                }
                digitalWrite(PIN_STEP1_STEP, currentState);
            }
        }
    
    }
}
