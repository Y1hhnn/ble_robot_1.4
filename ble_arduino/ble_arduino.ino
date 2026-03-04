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
//////////// BLE UUIDs ////////////

//////////// ICM Sensor ////////////
#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM;
//////////// ICM Sensor ////////////

//////////// TOF Sensor ////////////
#define XSHUT_PIN A3    //// XSHUT pin for the second sensor

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
//////////// TOF Sensor ////////////

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

unsigned long loop_count = 0;

// Sample Buffer
const int SAMPLE_LEN = 3000;  
const int SAMPLE_INTERVAL = 1000; // in microseconds
unsigned long last_sample_time = 0;
unsigned long time_buffer[SAMPLE_LEN];
unsigned long temp_buffer[SAMPLE_LEN];

//ToF Sensor Buffer
int distance_buffer1[SAMPLE_LEN];
int distance_buffer2[SAMPLE_LEN];

// IMU Buffers
float raw_acc_x[SAMPLE_LEN];
float raw_acc_y[SAMPLE_LEN];
float raw_acc_z[SAMPLE_LEN];
// float gyro_x[SAMPLE_LEN];
// float gyro_y[SAMPLE_LEN];
// float gyro_z[SAMPLE_LEN];

float raw_acc_roll[SAMPLE_LEN];
float raw_acc_pitch[SAMPLE_LEN];

float gyr_roll[SAMPLE_LEN];
float gyr_pitch[SAMPLE_LEN];
float gyr_yaw[SAMPLE_LEN];

int sample_count = 0;
bool collecting = false;

// Lower-Pass Filter
const float alpha = 0.15; 
// float filt_acc_x[SAMPLE_LEN];
// float filt_acc_y[SAMPLE_LEN];
// float filt_acc_z[SAMPLE_LEN];

float filt_acc_roll[SAMPLE_LEN];
float filt_acc_pitch[SAMPLE_LEN];

float comp_roll[SAMPLE_LEN];
float comp_pitch[SAMPLE_LEN];

//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    START_COLLECT_DATA,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    GET_ACCL_READINGS,
    GET_GYRO_READINGS,
    GET_COMP_READINGS,
    GET_TOF_READINGS
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            float fl_a, fl_b, fl_c;

            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(fl_a);
            if (!success)
                return;
            
            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(fl_b);
            if (!success)
                return;

            // Extract the next value from the command string as a float
            success = robot_cmd.get_next_value(fl_c);
            if (!success)
                return;

            Serial.print("Three Floats: ");
            Serial.print(fl_a);
            Serial.print(", ");
            Serial.print(fl_b);
            Serial.print(", ");
            Serial.println(fl_c);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Robot says -> ");
            Serial.print(tx_estring_value.c_str());
            Serial.println(":)");
            
            break;
        
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;


        /*
         * Write the timestamp on the GATT characteristic BLE_UUID_TX_STRING
         */
        case GET_TIME_MILLIS:
            unsigned long t;
            t = millis();
            
            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append((int)t);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
            Serial.println(tx_estring_value.c_str());
            break;

        /*
         *  Start collecting timestamps in the main loop 
         */
        case START_COLLECT_DATA:
            sample_count = 0;
            loop_count = 0;
            collecting = true;
            last_sample_time = micros();
            digitalWrite(LED_BUILTIN, HIGH); 
            distanceSensor1.startRanging(); 
            distanceSensor2.startRanging();
            break;
        
        /*
         *  Stop collecting timestamps. Loops through time_buffer and sends each data point as a string 
         *  on the GATT characteristic BLE_UUID_TX_STRING
         */
        case SEND_TIME_DATA:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            SERIAL_PORT.print("Loop Count: ");
            SERIAL_PORT.println(loop_count);
            SERIAL_PORT.print(" | Sample Count: ");
            SERIAL_PORT.println(sample_count);
            break;

        /*
         *  Stop collecting temperature. Loops through temp_buffer and sends each data point as 
         *  a string on the GATT characteristic BLE_UUID_TX_STRING
         */
        case GET_TEMP_READINGS:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_estring_value.append("|F: ");
                tx_estring_value.append((int)temp_buffer[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        /*
         *  Stop collecting accelerometer data. Loops through accx, accy and accz buffers (both raw and LPF ones)
         *   and sends each data point as a string on the GATT characteristic BLE_UUID_TX_STRING.
         */
        case GET_ACCL_READINGS:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_estring_value.append("|x: ");
                tx_estring_value.append(raw_acc_x[i]);
                tx_estring_value.append("|y: ");
                tx_estring_value.append(raw_acc_y[i]);
                tx_estring_value.append("|z: ");
                tx_estring_value.append(raw_acc_z[i]);
                tx_estring_value.append("|p: ");
                tx_estring_value.append(raw_acc_pitch[i]);
                tx_estring_value.append("|r: ");
                tx_estring_value.append(raw_acc_roll[i]);
                tx_estring_value.append("|fp: ");
                tx_estring_value.append(filt_acc_pitch[i]);
                tx_estring_value.append("|fr: ");
                tx_estring_value.append(filt_acc_roll[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            SERIAL_PORT.print("Loop Count: ");
            SERIAL_PORT.println(loop_count);
            SERIAL_PORT.print(" | Sample Count: ");
            SERIAL_PORT.print(sample_count);
            break;
        
        /*
         *  Stop collecting gyroscope data. Loops through gyrox, gyroy and gyroz buffers and 
         *  sends each data point as a string on the GATT characteristic BLE_UUID_TX_STRING.
         */
        case GET_GYRO_READINGS:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);

            for (int i = 0; i < sample_count; i++) {

                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                // tx_estring_value.append("|x: ");
                // tx_estring_value.append(gyro_x[i]);
                // tx_estring_value.append("|y: ");
                // tx_estring_value.append(gyro_y[i]);
                // tx_estring_value.append("|z: ");
                // tx_estring_value.append(gyro_z[i]);
                tx_estring_value.append("|r: ");
                tx_estring_value.append(gyr_roll[i]);
                tx_estring_value.append("|p: ");
                tx_estring_value.append(gyr_pitch[i]);
                tx_estring_value.append("|y: ");
                tx_estring_value.append(gyr_yaw[i]); 
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;
        
        /*
         *  Stop collecting complementary filter data. Loops through compl_roll and comp_pitch buffers and 
         *  sends each data point as a string on the GATT characteristic BLE_UUID_TX_STRING.
        */
        case GET_COMP_READINGS:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);

            for (int i = 0; i < sample_count; i++) {

                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_estring_value.append("|r: ");
                tx_estring_value.append(comp_roll[i]);
                tx_estring_value.append("|p: ");
                tx_estring_value.append(comp_pitch[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            break;

        /* 
         *  Stop collecting ToF readings. Loops through distance_buffer and sends each data point as a 
         *  string on the GATT characteristic BLE_UUID_TX_STRING.
         */
        case GET_TOF_READINGS:
            collecting = false;
            digitalWrite(LED_BUILTIN, LOW);
            distanceSensor1.stopRanging();
            distanceSensor2.stopRanging();

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_estring_value.append("|D1: ");
                tx_estring_value.append(distance_buffer1[i]);
                tx_estring_value.append("|D2: ");
                tx_estring_value.append(distance_buffer2[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            SERIAL_PORT.print("Loop Count: ");
            SERIAL_PORT.println(loop_count);
            SERIAL_PORT.print(" | Sample Count: ");
            SERIAL_PORT.print(sample_count);
            break;
        
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);

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

    // ICM Sensor Initialization
    while (!SERIAL_PORT){
        ; // wait for serial port to connect. Needed for native USB
    }

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized){
    
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
            initialized = true;
            // for (int i=0; i<=2;i++){
            //     digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
            //     delay(500);                      // wait for a second
            //     digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
            //     delay(500);
            // }
        }
    }

    // TOF Sensor Initialization
    SERIAL_PORT.println("VL53L1X Qwiic Test");

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
    distanceSensor1.setI2CAddress(0x2A<<1);

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

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();

            if (collecting && sample_count < SAMPLE_LEN) {
                unsigned long current_sample_time = micros();
                loop_count++;

                 if (myICM.dataReady() && distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()){
                    int current_dist1 = distanceSensor1.getDistance();
                    distanceSensor1.clearInterrupt(); 
                    
                    int current_dist2 = distanceSensor2.getDistance();
                    distanceSensor2.clearInterrupt(); 
                    
                    time_buffer[sample_count] = millis();
                    distance_buffer1[sample_count] = current_dist1;
                    distance_buffer2[sample_count] = current_dist2;

                    sample_count++;
                    last_sample_time = current_sample_time;

                    unsigned long current_sample_time = micros();
                    float dt = (current_sample_time - last_sample_time)/1.e6; 
                    myICM.getAGMT();

                    raw_acc_x[sample_count] = myICM.accX();
                    raw_acc_y[sample_count] = myICM.accY();
                    raw_acc_z[sample_count] = myICM.accZ();

                    raw_acc_roll[sample_count] = atan2(myICM.accY(), myICM.accZ()) * 180.0 / M_PI;
                    raw_acc_pitch[sample_count] = atan2(myICM.accX(), myICM.accZ())* 180.0 / M_PI;

                    if (sample_count == 0){
                        filt_acc_roll[sample_count] = raw_acc_roll[sample_count];
                        filt_acc_pitch[sample_count] = raw_acc_pitch[sample_count];

                        gyr_roll[sample_count] = myICM.gyrX()*dt;
                        gyr_pitch[sample_count] = - myICM.gyrY()*dt;
                        gyr_yaw[sample_count] = myICM.gyrZ()*dt;

                        comp_roll[sample_count] = filt_acc_roll[sample_count]*alpha + gyr_roll[sample_count]*(1-alpha);
                        comp_pitch[sample_count] = filt_acc_pitch[sample_count]*alpha + gyr_pitch[sample_count]*(1-alpha);

                    }
                    else {
                        filt_acc_roll[sample_count] = alpha * raw_acc_roll[sample_count] + (1 - alpha) * filt_acc_roll[sample_count-1];
                        filt_acc_pitch[sample_count] = alpha * raw_acc_pitch[sample_count] + (1 - alpha) * filt_acc_pitch[sample_count-1];

                        gyr_roll[sample_count] = myICM.gyrX()*dt + gyr_roll[sample_count-1];
                        gyr_pitch[sample_count] = - myICM.gyrY()*dt - gyr_pitch[sample_count-1];
                        gyr_yaw[sample_count] = myICM.gyrZ()*dt + gyr_yaw[sample_count-1];

                        comp_roll[sample_count] = filt_acc_roll[sample_count]*alpha + (1-alpha)*(comp_roll[sample_count-1] + myICM.gyrX()*dt);
                        comp_pitch[sample_count] = filt_acc_pitch[sample_count]*alpha + (1-alpha)*(comp_pitch[sample_count-1] + myICM.gyrY()*dt);
                    
                    }

                 }
                    
            }
        }

        Serial.println("Disconnected");
    }
}
