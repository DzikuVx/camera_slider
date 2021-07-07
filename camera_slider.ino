// #include <Wire.h>
#include "QmuTactile.h"
#include "filter.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

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

// TaskHandle_t driverPid;


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
  SYSTEM_FLAG_REQUESTING_POSITION = 1 << 6, // 16
};

uint8_t _systemFlags;
pt1Filter_t targetPositionFilter;

volatile int stepsPerSecond[2] = {0};

#define SPEED_SLOW 750
#define SPEED_MID 1500
#define SPEED_FAST 3000

int32_t maxSpeed = SPEED_MID;

int32_t currentPosition[2] = {0};   
int32_t targetPosition[2] = {0};    //Hardware operated Position
int32_t requestedPosition[2] = {0}; //User requested Position

AsyncWebServer server(80);

const char* ssid = "DiyCameraSlider";
const char* password = "123456789";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

String pageContent() {
    String pageContent;
    pageContent = "<!DOCTYPE html><html>";
    pageContent += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    pageContent += "<link rel=\"icon\" href=\"data:,\">";
    pageContent += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}";
    pageContent += ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}";
    pageContent += ".button-small { background-color: #0000ff; border: none; color: white; padding: 8px 20px; text-decoration: none; font-size: 15px; margin: 2px; cursor: pointer;}";
    pageContent += ".button2 {background-color: #ff0000;}</style></head>";
    
    pageContent += "<body><h1>DIY Camera Slider</h1>";
    
    pageContent += "<p><a href=\"/forward\"><button class=\"button\">Forward</button></a></p>";
    pageContent += "<p><a href=\"/stop\"><button class=\"button button2\">Stop</button></a></p>";
    pageContent += "<p><a href=\"/backward\"><button class=\"button\">Backward</button></a></p>";
    
    pageContent += "<p>";
    pageContent += "<a href=\"/slow\"><button class=\"button-small\">Slow</button></a>";
    pageContent += "<a href=\"/mid\"><button class=\"button-small\">Mid</button></a>";
    pageContent += "<a href=\"/fast\"><button class=\"button-small\">Fast</button></a>";
    pageContent += "</p>";

    pageContent += "</body></html>";

    return pageContent;
}

void setup()
{
    Serial.begin(115200);

    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", pageContent());
    });

    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request){
        requestedPosition[0] = 81000;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
        request->send(200, "text/html", pageContent());
    });

    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request){
        requestedPosition[0] = 0;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
        request->send(200, "text/html", pageContent());
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
        requestedPosition[0] = currentPosition[0];
        targetPosition[0] = currentPosition[0];
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
        request->send(200, "text/html", pageContent());
    });

    server.on("/slow", HTTP_GET, [](AsyncWebServerRequest *request){
        maxSpeed = SPEED_SLOW;
        request->send(200, "text/html", pageContent());
    });

    server.on("/mid", HTTP_GET, [](AsyncWebServerRequest *request){
        maxSpeed = SPEED_MID;
        request->send(200, "text/html", pageContent());
    });

    server.on("/fast", HTTP_GET, [](AsyncWebServerRequest *request){
        maxSpeed = SPEED_FAST;
        request->send(200, "text/html", pageContent());
    });

    server.onNotFound(notFound);
    server.begin();

    // pinMode(16, OUTPUT);
    // digitalWrite(16, LOW); // set GPIO16 low to reset OLED
    // delay(50);
    // digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high

    // I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 50000);
    // I2C2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 400000);

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

uint32_t nextSlewRate = 0;

void loop()
{
	button1.loop();
	button2.loop();
	button3.loop();
	button4.loop();
    buttonEnd.loop();

    if (button1.getState() == TACTILE_STATE_SHORT_PRESS) {
        requestedPosition[0] = 1000;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
    }

    if (button2.getState() == TACTILE_STATE_SHORT_PRESS) {
        requestedPosition[0] = 5000;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
    }

    if (button3.getState() == TACTILE_STATE_SHORT_PRESS) {
        requestedPosition[0] = 0;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
    }

    if (button4.getState() == TACTILE_STATE_SHORT_PRESS) {
        requestedPosition[0] = 81000;
        _systemFlags |= SYSTEM_FLAG_REQUESTING_POSITION;
    }

    // When requested position settled, settle down
    if ((_systemFlags & SYSTEM_FLAG_REQUESTING_POSITION) && targetPosition[0] == requestedPosition[0]) {
        _systemFlags &= ~SYSTEM_FLAG_REQUESTING_POSITION;
    }

    if (millis() > nextSlewRate) {
        if (_systemFlags & SYSTEM_FLAG_REQUESTING_POSITION) {

            int32_t error = requestedPosition[0] - targetPosition[0];
            int32_t acceleration = error * 100; // In steps per second
            //Constain acceleration
            acceleration = constrain(acceleration, -81000, 81000);
            targetPosition[0] = targetPosition[0] + (acceleration / 100);
        }

        nextSlewRate = millis() + 10;
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
            currentPosition[0] = -800;
            targetPosition[0] = 0;
            _systemFlags |= SYSTEM_FLAG_MOVE_TO_POSITION;
        }
    }

    if (_systemFlags & SYSTEM_FLAG_OPERATIONAL && buttonEnd.checkFlag(TACTILE_FLAG_PRESSED)) {
        _systemFlags |= SYSTEM_FLAG_EMERGENCY_STOP;
    }

#define MIN_SPEED 400
#define SPEED_CONTROLLER_P 8.0f

    /*
     * Position to speed
     */
    if (_systemFlags & SYSTEM_FLAG_MOVE_TO_POSITION) {

        targetPosition[0] = constrain(targetPosition[0], 0, 81000);

        int32_t error = targetPosition[0] - currentPosition[0];

        int32_t targetSpeed = error * SPEED_CONTROLLER_P;

        if (targetSpeed > maxSpeed) {
            targetSpeed = maxSpeed;
        } else if (targetSpeed < -maxSpeed) {
            targetSpeed = -maxSpeed;
        } else if (targetSpeed < maxSpeed && targetSpeed > 0) {
            targetSpeed = maxSpeed;
        } else if (targetSpeed > -maxSpeed && targetSpeed < 0) {
            targetSpeed = -maxSpeed;
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
                    nextChange = micros() + 30;

                    if (stepsPerSecond[0] > 0) {
                        currentPosition[0]++;
                    } else if (stepsPerSecond[0] < 0) {
                        currentPosition[0]--;
                    }
                } else {
                    currentState = LOW;
                    nextChange = micros() + (1000 * abs(1000.0f / stepsPerSecond[0])) - 30;
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
