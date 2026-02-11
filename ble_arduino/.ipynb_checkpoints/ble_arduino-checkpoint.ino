#include <ICM_20948.h>
#include "math.h"


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

// Sample Buffer
const int SAMPLE_LEN = 100;  
unsigned long time_buffer[SAMPLE_LEN];
unsigned long temp_buffer[SAMPLE_LEN];
float accx_buffer[SAMPLE_LEN];
float accy_buffer[SAMPLE_LEN];
float accz_buffer[SAMPLE_LEN];
int sample_count = 0;
bool collecting = false;

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
            collecting = true;
            break;
        
        /*
         *  Stop collecting timestamps. Loops through time_buffer and sends each data point as a string 
         *  on the GATT characteristic BLE_UUID_TX_STRING
         */
        case SEND_TIME_DATA:
            collecting = false;

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(3);         
            }
            break;

        /*
         *  Stop collecting temperature. Loops through temp_buffer and sends each data point as 
         *  a string on the GATT characteristic BLE_UUID_TX_STRING
         */
        case GET_TEMP_READINGS:
            collecting = false;

            for (int i = 0; i < sample_count; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T: ");
                tx_estring_value.append((int)time_buffer[i]);
                tx_estring_value.append("|F: ");
                tx_estring_value.append((int)temp_buffer[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(3);         
            }
            break;

        /*
         *  Stop collecting temperature. Loops through temp_buffer and sends each data point as 
         *  a string on the GATT characteristic BLE_UUID_TX_STRING
         */
        case GET_ACCL_READINGS:
            collecting = false;

            for (int i = 0; i < sample_count; i++) {
                float accx, accy, accz;
                tx_estring_value.clear();
                tx_estring_value.append("x: ");
                tx_estring_value.append(accx_buffer[i]);
                tx_estring_value.append("|y: ");
                tx_estring_value.append(accy_buffer[i]);
                tx_estring_value.append("|z: ");
                tx_estring_value.append(accz_buffer[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                delay(3);         
            }
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
            for (int i=0; i<=2;i++){
                digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
                delay(500);                      // wait for a second
                digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
                delay(500);
            }
        }
    }
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
                time_buffer[sample_count] = millis();
                temp_buffer[sample_count] = getTempDegF();
                if (myICM.dataReady()){
                    myICM.getAGMT();
                    accx_buffer[sample_count] = myICM.accX();
                    accy_buffer[sample_count] = myICM.accY();
                    accz_buffer[sample_count] = myICM.accZ();
                }
                sample_count++;
            }
        }

        Serial.println("Disconnected");
    }
}
