#include <SparkFun_VL53L1X.h>
#include <ICM_20948.h>
#include "math.h"
#include <Wire.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "d427e7cc-c400-4597-b417-d564e20d6600"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

BLEDevice central;
//////////// BLE UUIDs ////////////

//////////// ICM Sensor ////////////
#define SERIAL_PORT Serial
#define SPI_PORT SPI   // Your desired SPI port.
#define CS_PIN 2       // Which pin you connect CS to.
#define WIRE_PORT Wire // Your desired Wire port.

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;
//////////// ICM Sensor ////////////

//////////// TOF Sensor ////////////
#define XSHUT_PIN A3 //// XSHUT pin for the second sensor
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
//////////// TOF Sensor ////////////

//////////// Motors ////////////
#define LEFT_MOTOR_IN1 4
#define LEFT_MOTOR_IN2 5
#define RIGHT_MOTOR_IN1 7
#define RIGHT_MOTOR_IN2 6

// --- Calibration Constants ---
const int FWD_LEFT_MIN = 40;
const int FWD_LEFT_MAX = 212;
const int FWD_RIGHT_MIN = 50;
const int FWD_RIGHT_MAX = 255;
const int REV_LEFT_MIN = 48;
const int REV_LEFT_MAX = 210;
const int REV_RIGHT_MIN = 200;
const int REV_RIGHT_MAX = 255;
//////////// Motors ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");
// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

// IMU
unsigned long last_imu_time = 0; // in microseconds
// float last_acc_roll, last_acc_pitch;
// float last_gyr_roll, last_gyr_pitch;
// float last_comp_roll, last_comp_pitch;
// float comp_roll, comp_pitch;
float gyr_yaw, gyr_bias_z;
bool imu_init = true;
// float filt_alpha = 0.15;
// float comp_alpha = 0.9;

// TOF
float tof1_dist, tof2_dist;

// Motor
float left_motor_pct = 0.0f;
float right_motor_pct = 0.0f;

// Controller
bool active = false;
float kp = 1.0f;
float ki = 0.0f;
float kd = 0.0f;

unsigned long last_control_time = 0; // in microseconds
float setpoint = 0.0f;               // Degree for IMU, cm for TOF
float sensor_value = 0.0f;
float error_value = 0.0f;
float integral_value = 0.0f;
float derivative_value = 0.0f;
float output_value = 0.0f;
//////////// Global Variables ////////////

//////////// Sample Data ////////////
const int SAMPLE_LEN = 1500;
int SAMPLE_INTERVAL = 1000;          // in microseconds
unsigned long last_sample_time = 0;  // in microseconds
int SAMPLE_DURATION = 5000;          // in milliseconds
unsigned long start_sample_time = 0; // in milliseconds
int sample_count = 0;
bool collecting = false;

// System Buffers
unsigned long time_buffer[SAMPLE_LEN];

// Motor Buffer
float left_pct[SAMPLE_LEN];
float right_pct[SAMPLE_LEN];

// PID Buffer
float sensor_buffer[SAMPLE_LEN];
float error_buffer[SAMPLE_LEN];
float derivative_buffer[SAMPLE_LEN];
float integral_buffer[SAMPLE_LEN];
//////////// Sample Data ////////////

//////////// Commands ////////////
enum CommandTypes
{
    PING,
    START_RECORD,
    STOP_ROBOT,
    SEND_LOG,
    UPDATE_PID,
    SET_DURATION,
    SET_SETPOINT,
    SET_MODE
};
//////////// Commands ////////////

//////////// Control Mode ////////////
enum ControlMode
{
    MODE_POSITION,
    MODE_ORIENTATION
};
ControlMode control_mode = MODE_POSITION;
//////////// Control Mode ////////////

// =========================
// SETUP
// =========================
void setup()
{
    Serial.begin(115200);
    setupBle();
    setupICM();
    setupToF();
    setupMotors();
    led_blink(3, 200);
}

// =========================
// Main Loop
// =========================
void loop()
{
    handleBLE();
    updateSensors();
    if (active)
    {
        // runController();
        if (millis() - start_sample_time >= SAMPLE_DURATION)
            stopRobot();
    }
    if (collecting)
        collectSamples();
}

// =========================
// BLE
// =========================
void handleBLE()
{
    if (!central)
    {
        central = BLE.central();
        if (central)
        {
            Serial.print("Connected to: ");
            Serial.println(central.address());
        }
    }

    if (central && central.connected())
    {
        read_data();
    }
    else if (central && !central.connected())
    {
        Serial.println("Disconnected from: ");
        Serial.print(central.address());
        if (active)
        {
            Serial.println("Failsafe: Connection lost, stopping robot!");
            stopRobot();
        }
        central = BLEDevice();
    }
}

void handleCommand()
{
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    success = robot_cmd.get_command_type(cmd_type);
    // Check if the last tokenization was successful and return if failed
    if (!success)
        return;

    // Handle the command type accordingly
    switch (cmd_type)
    {
    case PING:
    {
        tx_estring_value.clear();
        tx_estring_value.append("PONG");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        break;
    }

    case START_RECORD:
    {
        cleanLog();
        cleanState();

        if (control_mode == MODE_POSITION)
        {
            // distanceSensor1.startRanging();
            distanceSensor2.startRanging();
            Serial.println("Waiting for second ToF reading...");
            while (!distanceSensor2.checkForDataReady())
            {
                delay(1);
            }
            tof2_dist = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
        }
        else if (control_mode == MODE_ORIENTATION)
        {
            imu_init = true;
            while (!myICM.dataReady())
            {
                delay(1);
            }
            updateIMU();
        }

        sample_count = 0;

        last_sample_time = micros();
        last_control_time = micros();
        start_sample_time = millis();

        collecting = true;
        active = true;
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Started Recording");
        break;
    }

    case STOP_ROBOT:
    {
        stopRobot();
        break;
    }

    case SEND_LOG:
    {
        for (int i = 0; i < sample_count; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append((int)time_buffer[i]);
            tx_estring_value.append("|LM: ");
            tx_estring_value.append(left_pct[i]);
            tx_estring_value.append("|RM: ");
            tx_estring_value.append(right_pct[i]);
            tx_estring_value.append("|S: ");
            tx_estring_value.append(sensor_buffer[i]);
            tx_estring_value.append("|E: ");
            tx_estring_value.append(error_buffer[i]);
            tx_estring_value.append("|I: ");
            tx_estring_value.append(integral_buffer[i]);
            tx_estring_value.append("|D: ");
            tx_estring_value.append(derivative_buffer[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            delay(2);
        }
        SERIAL_PORT.print("Sample Count: ");
        SERIAL_PORT.println(sample_count);
        break;
    }

    case UPDATE_PID:
    {
        float new_kp, new_ki, new_kd;
        // Extract the next value from the command string as a float
        success = robot_cmd.get_next_value(new_kp);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_ki);
        if (!success)
            return;
        success = robot_cmd.get_next_value(new_kd);
        if (!success)
            return;

        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        break;
    }

    case SET_DURATION:
    {
        int new_duration;
        success = robot_cmd.get_next_value(new_duration);
        if (!success)
            return;
        SAMPLE_DURATION = new_duration;
        Serial.print("Set Sample Duration to: ");
        Serial.println(SAMPLE_DURATION);
        break;
    }

    case SET_SETPOINT:
    {
        float new_setpoint;
        success = robot_cmd.get_next_value(new_setpoint);
        if (!success)
            return;
        setpoint = new_setpoint;
        Serial.println("Set Setpoint to: ");
        Serial.print(setpoint);
        break;
    }

    case SET_MODE:
    {
        int new_mode;
        success = robot_cmd.get_next_value(new_mode);
        if (!success)
            return;
        switch (ControlMode(new_mode))
        {
        case MODE_POSITION:
            control_mode = MODE_POSITION;
            break;
        case MODE_ORIENTATION:
            control_mode = MODE_ORIENTATION;
            break;
        default:
            Serial.print("Invalid Control Mode: ");
            Serial.println(new_mode);
        }
        break;
    }

    default:
    {
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
    }
    }
}

// =========================
// Controller
// =========================
void runController()
{
    unsigned long current_control_time = micros();
    float dt = (current_control_time - last_control_time) / 1.e6; // in seconds
    if (dt <= 0.0f)
        return;
    last_control_time = current_control_time;

    sensor_value = getSensorValue();
    float new_error = setpoint - sensor_value;

    integral_value += new_error * dt;
    integral_value = constrain(integral_value, -100.0f, 100.0f);
    derivative_value = (new_error - error_value) / dt;
    output_value = kp * new_error + ki * integral_value + kd * derivative_value;
    error_value = new_error;
    applyOutput(output_value);
}

float getSensorValue()
{
    if (control_mode == MODE_POSITION)
    {
        return tof2_dist;
    }
    else if (control_mode == MODE_ORIENTATION)
    {
        return gyr_yaw;
    }
    return 0.0f;
}

void applyOutput(float output)
{
    float power = -output;
    if (power > 100)
        power = 100;
    if (power < -100)
        power = -100;

    if (control_mode == MODE_POSITION)
    {
        left_motor_pct = power;
        right_motor_pct = power;
        setMotors(power, power);
    }
    else if (control_mode == MODE_ORIENTATION)
    {
        left_motor_pct = power;
        right_motor_pct = -power;
        setMotors(power, -power);
    }
    else
    {
        left_motor_pct = 0;
        right_motor_pct = 0;
        setMotors(0, 0);
    }
}

// =========================
// Sensors
// =========================
void updateSensors()
{
    if (myICM.dataReady())
    {
        updateIMU();
    }
    // if (distanceSensor1.checkForDataReady()) {
    //     tof1_dist = distanceSensor1.getDistance();
    //     distanceSensor1.clearInterrupt();
    // }

    if (distanceSensor2.checkForDataReady())
    {
        tof2_dist = distanceSensor2.getDistance();
        distanceSensor2.clearInterrupt();
    }
}

void updateIMU()
{
    unsigned long current_imu_time = micros();
    myICM.getAGMT();
    if (imu_init)
    {
        gyr_yaw = 0.0f;
        imu_init = false;
    }
    else
    {
        float dt = (current_imu_time - last_imu_time) / 1.e6;
        gyr_yaw += (myICM.gyrZ() - gyr_bias_z) * dt;
    }
    last_imu_time = current_imu_time;

    // acc_roll = atan2(myICM.accY(), sqrt(myICM.accX()*myICM.accX() + myICM.accZ()*myICM.accZ())) * 180 / M_PI;
    // acc_pitch = atan2(myICM.accX(), sqrt(myICM.accY()*myICM.accY() + myICM.accZ()*myICM.accZ()))* 180 / M_PI;
    // if(imu_init){
    //     gyr_roll =0.0f;
    //     gyr_pitch = 0.0f;
    //     comp_roll = acc_roll;
    //     comp_pitch = acc_pitch;
    //     imu_init = false;
    // }
    // else
    // {
    //     float dt = (current_imu_time - last_imu_time)/1.e6;
    //     acc_roll = filt_alpha * acc_roll + (1 - filt_alpha) * last_acc_roll;
    //     acc_pitch = filt_alpha * acc_pitch + (1 - filt_alpha) * last_acc_pitch;
    //     gyr_roll = last_gyr_roll + myICM.gyrX()*dt;
    //     gyr_pitch = last_gyr_pitch - myICM.gyrY()*dt;
    //     comp_roll = (1-comp_alpha) * acc_roll + comp_alpha * (last_comp_roll + myICM.gyrX()*dt);
    //     comp_pitch = (1-comp_alpha) * acc_pitch + comp_alpha * (last_comp_pitch - myICM.gyrY()*dt);
    // }
    // last_acc_roll = acc_roll;
    // last_acc_pitch = acc_pitch;
    // last_gyr_roll = gyr_roll;
    // last_gyr_pitch = gyr_pitch;
    // last_comp_roll = comp_roll;
    // last_comp_pitch = comp_pitch;
}

// =========================
// Motors
// =========================
void setMotors(float leftPct, float rightPct)
{
    float leftPWM = 0;
    float rightPWM = 0;
    leftPct = constrain(leftPct, -100.0f, 100.0f);
    rightPct = constrain(rightPct, -100.0f, 100.0f);

    // 1. Process Left Motor
    if (leftPct > 0)
    {
        leftPWM = mapFloat(leftPct, 1, 100, FWD_LEFT_MIN, FWD_LEFT_MAX);
        analogWrite(LEFT_MOTOR_IN1, (int)leftPWM);
        analogWrite(LEFT_MOTOR_IN2, 0);
    }
    else if (leftPct < 0)
    {
        leftPWM = mapFloat(abs(leftPct), 1, 100, REV_LEFT_MIN, REV_LEFT_MAX);
        analogWrite(LEFT_MOTOR_IN1, 0);
        analogWrite(LEFT_MOTOR_IN2, (int)leftPWM);
    }
    else
    {
        // 0% = Stop
        analogWrite(LEFT_MOTOR_IN1, 0);
        analogWrite(LEFT_MOTOR_IN2, 0);
    }

    // 2. Process Right Motor
    if (rightPct > 0)
    {
        rightPWM = mapFloat(rightPct, 1, 100, FWD_RIGHT_MIN, FWD_RIGHT_MAX);
        analogWrite(RIGHT_MOTOR_IN1, (int)rightPWM);
        analogWrite(RIGHT_MOTOR_IN2, 0);
    }
    else if (rightPct < 0)
    {
        rightPWM = mapFloat(abs(rightPct), 1, 100, REV_RIGHT_MIN, REV_RIGHT_MAX);
        analogWrite(RIGHT_MOTOR_IN1, 0);
        analogWrite(RIGHT_MOTOR_IN2, (int)rightPWM);
    }
    else
    {
        // 0% = Stop
        analogWrite(RIGHT_MOTOR_IN1, 0);
        analogWrite(RIGHT_MOTOR_IN2, 0);
    }
}

// void runForward(float pct){setMotors(pct, pct);}
// void runBackward(float pct){setMotors(-pct, -pct);}
// void stopMotors(){setMotors(0, 0);}
// void turnLeft(float pct){setMotors(-pct, pct);}
// void turnRight(float pct){setMotors(pct, -pct);}

// =========================
// Logging
// =========================
void collectSamples()
{
    unsigned long current_time = micros();
    if (current_time - last_sample_time < SAMPLE_INTERVAL)
        return;

    if (sample_count >= SAMPLE_LEN)
    {
        collecting = false;
        Serial.println("Sample buffer full, stopping collection");
        digitalWrite(LED_BUILTIN, LOW);
        return;
    }

    last_sample_time = current_time;

    // System Buffers
    time_buffer[sample_count] = millis() - start_sample_time;

    // Motor Buffer
    left_pct[sample_count] = left_motor_pct;
    right_pct[sample_count] = right_motor_pct;

    // PID Buffer
    sensor_buffer[sample_count] = getSensorValue();
    error_buffer[sample_count] = error_value;
    derivative_buffer[sample_count] = derivative_value;
    integral_buffer[sample_count] = integral_value;

    sample_count++;
}

void cleanLog()
{
    for (int i = 0; i < SAMPLE_LEN; i++)
    {
        time_buffer[i] = 0;
        left_pct[i] = 0.0f;
        right_pct[i] = 0.0f;
        sensor_buffer[i] = 0.0f;
        error_buffer[i] = 0.0f;
        derivative_buffer[i] = 0.0f;
        integral_buffer[i] = 0.0f;
    }
}

void cleanState()
{
    // IMU
    last_imu_time = 0; // in microseconds
    gyr_yaw = 0.0f;
    imu_init = true;

    // TOF
    tof1_dist = 0.0f;
    tof2_dist = 0.0f;

    // Motors
    setMotors(0, 0);
    left_motor_pct = 0.0f;
    right_motor_pct = 0.0f;

    // Controller
    last_control_time = 0;
    sensor_value = 0.0f;
    error_value = 0.0f;
    integral_value = 0.0f;
    derivative_value = 0.0f;
    output_value = 0.0f;
}

// =========================
// Helper Functions
// =========================
void stopRobot()
{
    setMotors(0, 0);
    collecting = false;
    if (control_mode == MODE_POSITION)
    {
        // distanceSensor1.stopRanging();
        distanceSensor2.stopRanging();
    }
    active = false;
    Serial.println("Stop Robot");
}

void setupBle()
{
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void setupICM()
{
    while (!SERIAL_PORT)
    {
        ; // wait for serial port to connect. Needed for native USB
    }

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(WIRE_PORT, AD0_VAL);

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.println("Trying again...");
            delay(500);
        }
        else
        {
            calibrateGyroBias();
            initialized = true;
        }
    }
}

void setupToF()
{
    // Turn off Sensor 2 to prevent I2C address conflicts
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(10);

    // Initialize Sensor 1
    while (distanceSensor1.begin(WIRE_PORT) != 0)
    {
        SERIAL_PORT.println("ToF Sensor 1 failed to begin. Retrying in 500ms...");
        delay(500);
    }

    // Change Sensor 1's I2C address (Default is 0x29, we change it to 0x2A)
    distanceSensor1.setI2CAddress(0x2A << 1);

    // Turn on Sensor 2
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);

    // Initialize Sensor 2
    while (distanceSensor2.begin(WIRE_PORT) != 0)
    {
        SERIAL_PORT.println("ToF Sensor 2 failed to begin. Retrying in 500ms...");
        delay(500);
    }

    distanceSensor1.setDistanceModeLong();
    distanceSensor2.setDistanceModeLong();

    SERIAL_PORT.println("Both ToF Sensors online!");
}

void setupMotors()
{
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    analogWriteResolution(8);
    setMotors(0, 0);
    delay(2000);
}

void calibrateGyroBias()
{
    const int N = 500;
    float sum = 0;
    for (int i = 0; i < N; i++)
    {
        while (!myICM.dataReady())
        {
        }
        myICM.getAGMT();
        sum += myICM.gyrZ();
        delay(5);
    }
    gyr_bias_z = sum / N;
}

void led_blink(int times, int delay_time)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delay_time);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delay_time);
    }
}

void write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000)
        {
            tx_float_value = 0;
        }

        previousMillis = currentMillis;
    }
}

void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written())
    {
        handleCommand();
    }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}