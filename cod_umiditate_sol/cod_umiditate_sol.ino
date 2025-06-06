#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_MLX90632_Arduino_Library.h"  // RAK12003 IR Temperature sensor

// LoRaWAN credentials - DOUBLE CHECK THESE WITH YOUR TTN APPLICATION
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x18, 0x49, 0x2C};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0x1D, 0xBF, 0xC9, 0xE5, 0x42, 0x23, 0xA2, 0x9E, 0x96, 0x26, 0x17, 0xC6, 0x7B, 0x9C, 0x5B, 0x4C};

// Sensor pins and objects for RAK12023 IO Shield
#define SOIL_POWER_PIN WB_IO3     // Power pin for soil sensor (RAK12023 uses IO3)
#define SOIL_SENSOR_PIN WB_A1     // Analog pin for soil sensor (RAK12023 uses A1)
#define MLX_POWER_PIN WB_IO4      // Power pin for MLX90632 sensor (use IO4 for temp sensor)
MLX90632 mlx;                     // IR temperature sensor object
bool mlxSensorAvailable = false;  // Track if sensor is working

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastJoinAttempt = 0;
const unsigned long SENSOR_INTERVAL = 60000; // Send data every 60 seconds
const unsigned long JOIN_RETRY_INTERVAL = 120000; // Retry join every 2 minutes if failed
bool joinInProgress = false;
int joinAttempts = 0;

// LoRaWAN callback functions
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_unconfirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

// LoRaWAN handler initialization with conservative settings
static lmh_param_t g_lora_param_init = {
    LORAWAN_ADR_OFF,        // Disable ADR for better debugging
    DR_0,                   // Use lowest data rate for maximum range
    LORAWAN_PUBLIC_NETWORK, 
    1,                      // Join trials
    TX_POWER_0,            // Maximum TX power
    LORAWAN_DUTYCYCLE_OFF   // Disable duty cycle for testing
};

static lmh_callback_t g_lora_callbacks = {
    BoardGetBatteryLevel, 
    BoardGetUniqueId, 
    BoardGetRandomSeed,
    lorawan_rx_handler, 
    lorawan_has_joined_handler, 
    lorawan_confirm_class_handler, 
    lorawan_join_failed_handler
};

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("\n=== RAK4630 TTN Sensor Node Debug Version ===");
    Serial.println("Starting initialization...");

    // Print system information
    Serial.print("Board: RAK4630, Firmware: ");
    Serial.println(__VERSION__);
    
    // Initialize basic GPIO first
    Serial.println("\n1. Initializing GPIO...");
    pinMode(SOIL_POWER_PIN, OUTPUT);
    digitalWrite(SOIL_POWER_PIN, LOW);
    pinMode(SOIL_SENSOR_PIN, INPUT);
    pinMode(MLX_POWER_PIN, OUTPUT);
    digitalWrite(MLX_POWER_PIN, HIGH); // OFF initially
    Serial.println("   GPIO initialized successfully");

    // Initialize I2C
    Serial.println("\n2. Initializing I2C...");
    Wire.begin();
    Wire.setClock(100000); // Conservative speed
    Serial.println("   I2C initialized at 100kHz");
    
    // Test MLX sensor with proper error handling
    Serial.println("\n3. Testing MLX90632 temperature sensor...");
    Serial.println("   Powering on sensor...");
    digitalWrite(MLX_POWER_PIN, LOW); // Power ON
    delay(1000); // Give sensor more time to power up
    
    Serial.println("   Attempting sensor initialization...");
    
    // Try to initialize with timeout
    bool sensorSuccess = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        Serial.print("   Attempt ");
        Serial.print(attempt + 1);
        Serial.print("/3: ");
        
        // Try different I2C addresses (0x3A is default, 0x3B is alternative)
        uint8_t addresses[] = {0x3A, 0x3B};
        
        for (int addr = 0; addr < 2; addr++) {
            MLX90632::status sensorStatus;
            if (mlx.begin(addresses[addr], Wire, sensorStatus)) {
                if (sensorStatus == MLX90632::status::SENSOR_SUCCESS) {
                    Serial.print("SUCCESS at address 0x");
                    Serial.println(addresses[addr], HEX);
                    mlxSensorAvailable = true;
                    sensorSuccess = true;
                    break;
                }
            }
        }
        
        if (sensorSuccess) break;
        
        Serial.println("Failed");
        delay(500);
    }
    
    if (mlxSensorAvailable) {
        // Test reading
        float testTemp = mlx.getObjectTemp();
        if (!isnan(testTemp) && testTemp > -50 && testTemp < 150) {
            Serial.print("   Test reading: ");
            Serial.print(testTemp);
            Serial.println("°C");
        } else {
            Serial.println("   Test reading: Invalid, but sensor detected");
        }
    } else {
        Serial.println("   MLX90632 sensor: NOT FOUND (will use default temperature)");
    }
    
    digitalWrite(MLX_POWER_PIN, HIGH); // Power OFF to save energy
    
    // Initialize LoRa hardware
    Serial.println("\n4. Initializing LoRa hardware...");
    Serial.println("   This may take a few seconds...");
    
    int loraInitResult = lora_rak4630_init();
    if (loraInitResult == 0) {
        Serial.println("   LoRa chip: OK");
    } else {
        Serial.print("   LoRa chip: FAILED with code ");
        Serial.println(loraInitResult);
        Serial.println("   CRITICAL ERROR: Check antenna connection!");
        Serial.println("   System will halt here.");
        while(1) {
            Serial.println("   Halted due to LoRa hardware failure");
            delay(10000);
        }
    }

    // Initialize LoRaWAN stack
    Serial.println("\n5. Initializing LoRaWAN stack...");
    lmh_error_status stackResult = lmh_init(&g_lora_callbacks, g_lora_param_init, true, CLASS_A, LORAMAC_REGION_EU868);
    if (stackResult == LMH_SUCCESS) {
        Serial.println("   LoRaWAN stack: OK");
    } else {
        Serial.print("   LoRaWAN stack: FAILED with error ");
        Serial.println(stackResult);
        Serial.println("   CRITICAL ERROR: LoRaWAN stack initialization failed");
        while(1) {
            Serial.println("   Halted due to LoRaWAN stack failure");
            delay(10000);
        }
    }

    // Set credentials
    Serial.println("\n6. Setting LoRaWAN credentials...");
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI); 
    lmh_setAppKey(nodeAppKey);
    
    // Print credentials for verification
    Serial.println("   Device EUI: ");
    Serial.print("   ");
    for (int i = 0; i < 8; i++) {
        if (nodeDeviceEUI[i] < 16) Serial.print("0");
        Serial.print(nodeDeviceEUI[i], HEX);
        if (i < 7) Serial.print(":");
    }
    Serial.println();
    
    Serial.println("   App EUI: ");
    Serial.print("   ");
    for (int i = 0; i < 8; i++) {
        if (nodeAppEUI[i] < 16) Serial.print("0");
        Serial.print(nodeAppEUI[i], HEX);
        if (i < 7) Serial.print(":");
    }
    Serial.println();
    
    Serial.println("   App Key (first 8 bytes): ");
    Serial.print("   ");
    for (int i = 0; i < 8; i++) {
        if (nodeAppKey[i] < 16) Serial.print("0");
        Serial.print(nodeAppKey[i], HEX);
        if (i < 7) Serial.print(":");
    }
    Serial.println("...");

    // Start join procedure
    Serial.println("\n7. Starting OTAA join procedure...");
    Serial.println("   Region: EU868");
    Serial.println("   Class: A");
    Serial.println("   Data Rate: DR0 (SF12/125kHz)");
    Serial.println("   TX Power: 14 dBm");
    Serial.println("   MLX Sensor: " + String(mlxSensorAvailable ? "Available" : "Not Available"));
    Serial.println("   Please wait for join result...");
    
    joinInProgress = true;
    joinAttempts = 1;
    lastJoinAttempt = millis();
    
    Serial.println("   Sending join request...");
    lmh_join();
    
    Serial.println("\n=== Setup Complete - Waiting for Network Join ===");
    Serial.println("If join takes too long, check:");
    Serial.println("- LoRaWAN gateway in range and online");
    Serial.println("- TTN application credentials are correct");
    Serial.println("- Gateway connected to TTN");
    Serial.println("- Frequency plan is EU868");
    Serial.println("- Antenna properly connected");
    Serial.println("- Device EUI matches TTN exactly");
}

void loop() {
    // Handle radio events - CRITICAL for LoRaWAN operation
    Radio.IrqProcess();
    
    // Auto-retry join if failed and enough time has passed
    if (!lmh_join_status_get() && !joinInProgress && 
        (millis() - lastJoinAttempt > JOIN_RETRY_INTERVAL)) {
        
        joinAttempts++;
        Serial.print("\n--- Join Retry Attempt #");
        Serial.print(joinAttempts);
        Serial.println(" ---");
        Serial.println("Previous join failed, retrying...");
        
        joinInProgress = true;
        lastJoinAttempt = millis();
        lmh_join();
    }
    
    // Send sensor data if joined and interval elapsed
    if (lmh_join_status_get() && (millis() - lastSensorRead >= SENSOR_INTERVAL)) {
        send_lora_frame();
        lastSensorRead = millis();
    }
    
    // Print periodic status
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 30000) { // Every 30 seconds
        Serial.print("\n[STATUS] Uptime: ");
        Serial.print(millis()/1000);
        Serial.print("s, Joined: ");
        Serial.print(lmh_join_status_get() ? "YES" : "NO");
        Serial.print(", Join attempts: ");
        Serial.print(joinAttempts);
        Serial.print(", MLX sensor: ");
        Serial.println(mlxSensorAvailable ? "Available" : "Not Available");
        lastStatusPrint = millis();
    }

    delay(100);
}

static void send_lora_frame(void) {
    Serial.println("\n--- Sending Sensor Data ---");

    // Read soil moisture with proper RAK12023 handling
    Serial.println("Reading soil moisture sensor (RAK12023)...");
    
    // Power on soil sensor
    digitalWrite(SOIL_POWER_PIN, HIGH);
    delay(1000); // Give sensor time to stabilize
    
    // Take multiple readings for accuracy
    int soilSum = 0;
    for (int i = 0; i < 5; i++) {
        soilSum += analogRead(SOIL_SENSOR_PIN);
        delay(100);
    }
    int soilRaw = soilSum / 5; // Average reading
    
    // RAK12023 calibration: dry ~3000, wet ~1300 (adjust based on your sensor)
    float soilMoisture = map(soilRaw, 3000, 1300, 0, 100);
    soilMoisture = constrain(soilMoisture, 0, 100);
    
    digitalWrite(SOIL_POWER_PIN, LOW); // Power off to save energy
    
    Serial.print("Soil: ");
    Serial.print(soilMoisture);
    Serial.print("% (raw: ");
    Serial.print(soilRaw);
    Serial.println(")");

    // Read temperature
    float objectTemp = 25.0; // Default fallback
    
    if (mlxSensorAvailable) {
        Serial.println("Reading temperature sensor...");
        digitalWrite(MLX_POWER_PIN, LOW); // Power ON
        delay(500); // Give sensor time to stabilize
        
        float temp = mlx.getObjectTemp();
        if (!isnan(temp) && temp > -40 && temp < 85) {
            objectTemp = temp;
            Serial.print("Temperature: ");
            Serial.print(objectTemp);
            Serial.println("°C");
        } else {
            Serial.println("Temperature: Invalid reading, using default");
        }
        digitalWrite(MLX_POWER_PIN, HIGH); // Power OFF
    } else {
        Serial.println("Temperature: Using default 25.0°C (sensor not available)");
    }

    // Get battery level
    float batteryLevel = BoardGetBatteryLevel();
    Serial.print("Battery: ");
    Serial.print(batteryLevel);
    Serial.println("V");

    // Prepare simplified payload (6 bytes)
    uint8_t data[6];
    uint16_t soilValue = (uint16_t)(soilMoisture * 100);
    int16_t tempValue = (int16_t)(objectTemp * 100);
    uint16_t battValue = (uint16_t)(batteryLevel * 100);
    
    data[0] = (soilValue >> 8) & 0xFF;
    data[1] = soilValue & 0xFF;
    data[2] = (tempValue >> 8) & 0xFF;
    data[3] = tempValue & 0xFF;
    data[4] = (battValue >> 8) & 0xFF;
    data[5] = battValue & 0xFF;

    lmh_app_data_t m_lora_app_data = {data, 6, 2, 0, 0};
    
    Serial.print("Payload: ");
    for (int i = 0; i < 6; i++) {
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    Serial.println("Sending LoRaWAN packet...");
    lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    if (error == LMH_SUCCESS) {
        Serial.println("LoRaWAN packet sent successfully!");
    } else {
        Serial.print("Send failed with error: ");
        Serial.println(error);
    }
}

static void lorawan_has_joined_handler(void) {
    Serial.println("\n🎉 SUCCESS: OTAA JOIN COMPLETED! 🎉");
    Serial.println("Device is now connected to TTN");
    Serial.print("Join attempt #");
    Serial.print(joinAttempts);
    Serial.println(" succeeded");
    Serial.println("Will start sending sensor data every 60 seconds");
    
    joinInProgress = false;
    lmh_class_request(CLASS_A);
    
    // Send first data packet immediately
    lastSensorRead = millis() - SENSOR_INTERVAL;
}

static void lorawan_join_failed_handler(void) {
    Serial.println("\n❌ JOIN FAILED ❌");
    Serial.print("Join attempt #");
    Serial.print(joinAttempts);
    Serial.println(" failed");
    
    Serial.println("\nTroubleshooting checklist:");
    Serial.println("1. Check if LoRaWAN gateway is in range and online");
    Serial.println("2. Verify TTN application credentials match exactly");
    Serial.println("3. Confirm gateway is connected to TTN");
    Serial.println("4. Check antenna connection");
    Serial.println("5. Verify frequency plan is EU868");
    Serial.println("6. Check Device EUI in TTN console");
    
    joinInProgress = false;
    
    // Don't retry immediately, let main loop handle retry timing
    Serial.print("Will retry in ");
    Serial.print(JOIN_RETRY_INTERVAL/1000);
    Serial.println(" seconds...");
}

static void lorawan_rx_handler(lmh_app_data_t *app_data) {
    Serial.println("\n📡 DOWNLINK RECEIVED 📡");
    Serial.print("Port: ");
    Serial.print(app_data->port);
    Serial.print(", Size: ");
    Serial.print(app_data->buffsize);
    Serial.print(", RSSI: ");
    Serial.print(app_data->rssi);
    Serial.print(", SNR: ");
    Serial.println(app_data->snr);
    
    if (app_data->buffsize > 0) {
        Serial.print("Data: ");
        for (uint8_t i = 0; i < app_data->buffsize; i++) {
            if (app_data->buffer[i] < 16) Serial.print("0");
            Serial.print(app_data->buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class) {
    Serial.print("Device class confirmed: ");
    Serial.println(Class == CLASS_A ? "A" : Class == CLASS_B ? "B" : "C");
}

static void lorawan_unconfirm_class_handler(DeviceClass_t Class) {
    Serial.print("Device class switch failed, staying in: ");
    Serial.println(Class == CLASS_A ? "A" : Class == CLASS_B ? "B" : "C");
}