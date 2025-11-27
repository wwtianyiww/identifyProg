//========================
//Largly AI assisted
// especially for printing to terminal and the switch case for command
//========================


#include <Arduino.h>
#include <Wire.h>



const int SHOULDER_INA1 = 19;
const int SHOULDER_INA2 = 18;
const int SHOULDER_PWM = 5;

const int WRIST_INA1 = 25;
const int WRIST_INA2 = 26;
const int WRIST_PWM = 27;

const int SHOULDER_SDA = 21;
const int SHOULDER_SCL = 22;

const int WRIST_SDA = 16;
const int WRIST_SCL = 17;

const int SHOULDER_CURRENT_PIN = 34;
const int WRIST_CURRENT_PIN = 32;

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_ANGLE_REG = 0x0E;
const float CURRENT_SENSITIVITY = 0.185f;
const float ADC_TO_VOLTAGE = 3.3f / 4095.0f;
const float VOLTAGE_OFFSET = 1.65f;


const float SHOULDER_GEAR_RATIO = 1.0f;
const float WRIST_GEAR_RATIO = 4.0f;



enum TestMode {
    IDLE,
    SHOULDER_COMPREHENSIVE,
    WRIST_COMPREHENSIVE
};


const uint16_t SAMPLE_RATE_HZ = 50;
const uint16_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;


const uint8_t PWM_LEVELS[] = {30, 60, 90, 120, 160, 210, 255};
const uint8_t NUM_PWM_LEVELS = 7;
const uint16_t STEP_DURATION_MS = 3000;  // 3 seconds per level (was 8)

const uint32_t TOTAL_TEST_DURATION_MS = (uint32_t)STEP_DURATION_MS * NUM_PWM_LEVELS * 2;
const uint32_t MAX_SAMPLES = 3000;  // set hard limit, there were some issues at around 4000

const float SUPPLY_VOLTAGE = 12.0f;



uint16_t prms_lfsr = 0xACE1;
bool usePRMS = true;

// PRMS random duration settings (in milliseconds)
const uint16_t PRMS_MIN_DURATION_MS = 1000;  // 1 second minimum
const uint16_t PRMS_MAX_DURATION_MS = 4000;  // 4 second maximum
uint32_t currentPhaseDuration = STEP_DURATION_MS;

TestMode currentMode = IDLE;
bool testRunning = false;

// Data arrays
uint16_t sampleCount = 0;
float timeData[MAX_SAMPLES];
float pwmData[MAX_SAMPLES];
float voltageData[MAX_SAMPLES];
int8_t directionData[MAX_SAMPLES];
float angleData[MAX_SAMPLES];          // Joint angle (cumulative, unwrapped)
float encoderAngleData[MAX_SAMPLES];   // Encoder angle (cumulative, unwrapped)
float velocityData[MAX_SAMPLES];
float currentData[MAX_SAMPLES];
uint8_t testPhaseData[MAX_SAMPLES];

// Test state tracking
uint32_t testStartTime = 0;
uint32_t phaseStartTime = 0;
uint32_t lastSampleTime = 0;


float lastWrappedAngle = 0.0f;
float cumulativeEncoderAngle = 0.0f;
float cumulativeJointAngle = 0.0f;
int32_t rotationCount = 0;

uint8_t currentPWMLevel = 0;
uint8_t currentPhase = 0;
bool currentDirection = true;

float shoulderEncoderOffset = 0.0f;
float wristEncoderOffset = 0.0f;

float minAngle = 0.0f;
float maxAngle = 0.0f;
float totalRotation = 0.0f;

float currentGearRatio = 1.0f;


void initializeMotors();
void initializeI2C();
void setMotorPWM(TestMode motor, uint8_t pwm, bool forward);
void stopMotor(TestMode motor);

uint16_t readAS5600Angle(TwoWire& wire);
float convertToAngle(uint16_t rawValue);
float unwrapAngle(float currentWrapped, float lastWrapped, float &cumulative, int32_t &rotCount);
float applyGearRatio(float encoderAngle, float gearRatio);
float readCurrent(int pin);

void startComprehensiveTest(TestMode mode);
void collectSample();
void stopTest();
void updateTestPhase();

void printData();
void clearData();
void printStatistics();
float calculateVelocity(float currentAngle, float lastAngle, float dt);

void handleSerialCommand();
void printMenu();

uint8_t generatePRMS(uint8_t maxValue);



void setup() {
    Serial.begin(9600);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  COMPREHENSIVE SYSTEM IDENTIFICATION");
    Serial.println("  V3: WITH ANGLE UNWRAPPING");
    Serial.println("  Tracks Multi-Rotation Correctly");
    Serial.println("========================================\n");

    initializeMotors();
    initializeI2C();

    delay(500);
    shoulderEncoderOffset = convertToAngle(readAS5600Angle(Wire));
    wristEncoderOffset = convertToAngle(readAS5600Angle(Wire1));

    Serial.println("System initialized successfully!\n");
    Serial.printf("Initial Encoder Readings:\n");
    Serial.printf("  Shoulder (1:1 ratio):  %.2f°\n", shoulderEncoderOffset);
    Serial.printf("  Wrist (4:1 ratio):     %.2f° encoder (%.2f° joint)\n",
                  wristEncoderOffset, wristEncoderOffset / WRIST_GEAR_RATIO);
    Serial.println();

    Serial.println("Gear Ratios:");
    Serial.printf("  Shoulder: %.1f:1 (direct drive)\n", SHOULDER_GEAR_RATIO);
    Serial.printf("  Wrist:    %.1f:1 (geared)\n", WRIST_GEAR_RATIO);
    Serial.println();
    Serial.println("Angle Unwrapping: ENABLED");
    Serial.println("   Tracks cumulative rotation >360°\n");

    Serial.println("Test Coverage:");
    Serial.printf("  - PWM Levels: ");
    for (int i = 0; i < NUM_PWM_LEVELS; i++) {
        Serial.printf("%d ", PWM_LEVELS[i]);
    }
    Serial.println();
    Serial.printf("  - Directions: Forward & Reverse\n");
    Serial.printf("  - Duration per level: %.1f seconds\n", STEP_DURATION_MS / 1000.0f);
    Serial.printf("  - Total test time: %.1f seconds\n", TOTAL_TEST_DURATION_MS / 1000.0f);
    Serial.printf("  - Sample rate: %d Hz\n", SAMPLE_RATE_HZ);
    Serial.printf("  - Expected samples: ~%d\n\n", (TOTAL_TEST_DURATION_MS / SAMPLE_INTERVAL_MS));

    printMenu();
}

void loop() {
    uint32_t currentTime = millis();

    if (Serial.available()) {
        handleSerialCommand();
    }

    if (testRunning) {
        uint32_t elapsedTime = currentTime - testStartTime;

        if (elapsedTime >= TOTAL_TEST_DURATION_MS) {
            stopTest();
            return;
        }

        updateTestPhase();

        if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_MS) {
            lastSampleTime = currentTime;
            collectSample();
        }
    }
}



void initializeMotors() {
    pinMode(SHOULDER_INA1, OUTPUT);
    pinMode(SHOULDER_INA2, OUTPUT);
    pinMode(SHOULDER_PWM, OUTPUT);

    pinMode(WRIST_INA1, OUTPUT);
    pinMode(WRIST_INA2, OUTPUT);
    pinMode(WRIST_PWM, OUTPUT);

    stopMotor(SHOULDER_COMPREHENSIVE);
    stopMotor(WRIST_COMPREHENSIVE);

    Serial.println("Motors initialized");
}

void initializeI2C() {
    Wire.begin(SHOULDER_SDA, SHOULDER_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    Wire1.begin(WRIST_SDA, WRIST_SCL);
    Wire1.setClock(400000);
    Wire1.setTimeOut(50);

    delay(100);

    Wire.beginTransmission(AS5600_ADDR);
    bool shoulderOK = (Wire.endTransmission() == 0);

    Wire1.beginTransmission(AS5600_ADDR);
    bool wristOK = (Wire1.endTransmission() == 0);

    if (shoulderOK) Serial.println("✓ Shoulder encoder detected");
    else Serial.println("✗ WARNING: Shoulder encoder not detected!");

    if (wristOK) Serial.println("✓ Wrist encoder detected");
    else Serial.println("✗ WARNING: Wrist encoder not detected!");
}


void setMotorPWM(TestMode motor, uint8_t pwm, bool forward) {
    if (motor == SHOULDER_COMPREHENSIVE) {
        digitalWrite(SHOULDER_INA1, forward ? HIGH : LOW);
        digitalWrite(SHOULDER_INA2, forward ? LOW : HIGH);
        analogWrite(SHOULDER_PWM, pwm);
    }
    else if (motor == WRIST_COMPREHENSIVE) {
        digitalWrite(WRIST_INA1, forward ? HIGH : LOW);
        digitalWrite(WRIST_INA2, forward ? LOW : HIGH);
        analogWrite(WRIST_PWM, pwm);
    }
}

void stopMotor(TestMode motor) {
    if (motor == SHOULDER_COMPREHENSIVE) {
        analogWrite(SHOULDER_PWM, 0);
        digitalWrite(SHOULDER_INA1, LOW);
        digitalWrite(SHOULDER_INA2, LOW);
    }
    else if (motor == WRIST_COMPREHENSIVE) {
        analogWrite(WRIST_PWM, 0);
        digitalWrite(WRIST_INA1, LOW);
        digitalWrite(WRIST_INA2, LOW);
    }
}



uint16_t readAS5600Angle(TwoWire& wire) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_ANGLE_REG);

    if (wire.endTransmission() != 0) return 0;
    if (wire.requestFrom((int)AS5600_ADDR, 2) != 2) return 0;

    uint16_t highByte = wire.read();
    uint16_t lowByte = wire.read();

    return ((highByte << 8) | lowByte) & 0x0FFF;
}

float convertToAngle(uint16_t rawValue) {
    return (rawValue * 360.0f) / 4096.0f;
}



float unwrapAngle(float currentWrapped, float lastWrapped, float &cumulative, int32_t &rotCount) {


    float delta = currentWrapped - lastWrapped;

    // Detect wrap around 0°/360°
    if (delta > 180.0f) {
        // Wrapped backwards: 10° → 350° (actually -20° change)
        delta -= 360.0f;
        rotCount--;
    } else if (delta < -180.0f) {
        // Wrapped forwards: 350° → 10° (actually +20° change)
        delta += 360.0f;
        rotCount++;
    }

    // Update cumulative angle
    cumulative += delta;

    return cumulative;
}

float applyGearRatio(float encoderAngle, float gearRatio) {
    return encoderAngle / gearRatio;
}

float readCurrent(int pin) {
    int adcValue = analogRead(pin);
    float voltage = adcValue * ADC_TO_VOLTAGE;
    float current = (voltage - VOLTAGE_OFFSET) / CURRENT_SENSITIVITY;
    return current;
}



void startComprehensiveTest(TestMode mode) {
    if (testRunning) {
        Serial.println("Test already running! Send 's' to stop first.");
        return;
    }

    // Reset everything
    sampleCount = 0;
    testStartTime = millis();
    phaseStartTime = testStartTime;
    lastSampleTime = testStartTime;
    testRunning = true;
    currentMode = mode;
    currentPhase = 0;
    currentPWMLevel = 0;
    currentDirection = true;

    prms_lfsr = 0xACE1;

    // Set initial phase duration
    if (usePRMS) {
        // Random duration for first phase in PRMS mode
        currentPhaseDuration = PRMS_MIN_DURATION_MS +
                               (random(PRMS_MAX_DURATION_MS - PRMS_MIN_DURATION_MS + 1));
    } else {
        // Fixed duration in sequential mode
        currentPhaseDuration = STEP_DURATION_MS;
    }

    // Reset unwrapping variables - CRITICAL!
    rotationCount = 0;
    cumulativeEncoderAngle = 0.0f;
    cumulativeJointAngle = 0.0f;

    minAngle = 0.0f;
    maxAngle = 0.0f;
    totalRotation = 0.0f;

    // Get initial angles
    if (mode == SHOULDER_COMPREHENSIVE) {
        currentGearRatio = SHOULDER_GEAR_RATIO;
        uint16_t rawAngle = readAS5600Angle(Wire);
        lastWrappedAngle = convertToAngle(rawAngle) - shoulderEncoderOffset;

        Serial.println("\n*** STARTING SHOULDER COMPREHENSIVE TEST ***");
        Serial.printf("Gear Ratio: %.1f:1\n", SHOULDER_GEAR_RATIO);
    }
    else if (mode == WRIST_COMPREHENSIVE) {
        currentGearRatio = WRIST_GEAR_RATIO;
        uint16_t rawAngle = readAS5600Angle(Wire1);
        lastWrappedAngle = convertToAngle(rawAngle) - wristEncoderOffset;

        Serial.println("\n*** STARTING WRIST COMPREHENSIVE TEST ***");
        Serial.printf("Gear Ratio: %.1f:1 (4 encoder rotations = 1 joint rotation)\n", WRIST_GEAR_RATIO);
        Serial.println("Angle unwrapping: ACTIVE (tracks >360° correctly)");
    }

    Serial.println("\nTest Sequence:");
    Serial.println("Phase | PWM | Dir | Duration");
    Serial.println("------|-----|-----|----------");
    for (int i = 0; i < NUM_PWM_LEVELS; i++) {
        Serial.printf("  %d   | %3d | FWD | %.1fs\n", i, PWM_LEVELS[i], STEP_DURATION_MS/1000.0f);
    }
    for (int i = 0; i < NUM_PWM_LEVELS; i++) {
        Serial.printf("  %d   | %3d | REV | %.1fs\n", i+NUM_PWM_LEVELS, PWM_LEVELS[i], STEP_DURATION_MS/1000.0f);
    }
    Serial.println("\nData collection started...\n");
}
void updateTestPhase() {
    uint32_t currentTime = millis();
    uint32_t phaseElapsed = currentTime - phaseStartTime;

    if (phaseElapsed >= currentPhaseDuration) {
        currentPhase++;
        phaseStartTime = currentTime;

        if (usePRMS) {
            // ===== PSEUDORANDOM MULTILEVEL SEQUENCE MODE =====
            // Randomly select PWM level, direction, and duration
            currentPWMLevel = generatePRMS(NUM_PWM_LEVELS);
            currentDirection = (generatePRMS(2) == 0);  // Random direction

            // Generate random duration between PRMS_MIN_DURATION_MS and PRMS_MAX_DURATION_MS
            currentPhaseDuration = PRMS_MIN_DURATION_MS +
                                   (random(PRMS_MAX_DURATION_MS - PRMS_MIN_DURATION_MS + 1));

            Serial.printf("\n=== Phase %d: PWM=%d, Direction=%s, Duration=%.1fs (PRMS) ===\n",
                         currentPhase, PWM_LEVELS[currentPWMLevel],
                         currentDirection ? "FORWARD" : "REVERSE",
                         currentPhaseDuration / 1000.0f);
        }
        else {
            // ===== ORIGINAL SEQUENTIAL MODE =====
            currentPhaseDuration = STEP_DURATION_MS;  // Use fixed duration in sequential mode

            if (currentPhase < NUM_PWM_LEVELS) {
                currentPWMLevel = currentPhase;
                currentDirection = true;
            }
            else if (currentPhase < NUM_PWM_LEVELS * 2) {
                currentPWMLevel = currentPhase - NUM_PWM_LEVELS;
                currentDirection = false;
            }

            if (currentPhase < NUM_PWM_LEVELS * 2) {
                Serial.printf("\n=== Phase %d: PWM=%d, Direction=%s ===\n",
                             currentPhase, PWM_LEVELS[currentPWMLevel],
                             currentDirection ? "FORWARD" : "REVERSE");
            }
        }
    }
}


uint8_t generatePRMS(uint8_t maxValue) {
    uint16_t bit = ((prms_lfsr >> 0) ^ (prms_lfsr >> 2) ^
                    (prms_lfsr >> 3) ^ (prms_lfsr >> 5)) & 1;

    prms_lfsr = (prms_lfsr << 1) | (bit << 15);

    return prms_lfsr % maxValue;
}


void collectSample() {
    if (sampleCount >= MAX_SAMPLES) {
        Serial.println("WARNING: Data buffer full!");
        stopTest();
        return;
    }

    float wrappedAngle, current;
    uint8_t currentPWM = PWM_LEVELS[currentPWMLevel];

    // Read sensor and unwrap angle
    if (currentMode == SHOULDER_COMPREHENSIVE) {
        setMotorPWM(SHOULDER_COMPREHENSIVE, currentPWM, currentDirection);
        uint16_t rawAngle = readAS5600Angle(Wire);
        wrappedAngle = convertToAngle(rawAngle) - shoulderEncoderOffset;
        current = readCurrent(SHOULDER_CURRENT_PIN);
    }
    else if (currentMode == WRIST_COMPREHENSIVE) {
        setMotorPWM(WRIST_COMPREHENSIVE, currentPWM, currentDirection);
        uint16_t rawAngle = readAS5600Angle(Wire1);
        wrappedAngle = convertToAngle(rawAngle) - wristEncoderOffset;
        current = readCurrent(WRIST_CURRENT_PIN);
    }


    cumulativeEncoderAngle = unwrapAngle(wrappedAngle, lastWrappedAngle, cumulativeEncoderAngle, rotationCount);
    lastWrappedAngle = wrappedAngle;

    // Apply gear ratio to cumulative angle
    cumulativeJointAngle = applyGearRatio(cumulativeEncoderAngle, currentGearRatio);

    // Calculate time and velocity
    float elapsedTime = (millis() - testStartTime) / 1000.0f;
    float dt = SAMPLE_INTERVAL_MS / 1000.0f;


    float velocity = 0.0f;
    if (sampleCount > 0) {
        float lastJointAngle = angleData[sampleCount - 1];
        velocity = (cumulativeJointAngle - lastJointAngle) / dt;
    }

    // Update statistics
    if (cumulativeJointAngle < minAngle) minAngle = cumulativeJointAngle;
    if (cumulativeJointAngle > maxAngle) maxAngle = cumulativeJointAngle;
    totalRotation += abs(velocity * dt);

    // Store data
    timeData[sampleCount] = elapsedTime;
    pwmData[sampleCount] = currentPWM;
    voltageData[sampleCount] = (currentPWM / 255.0f) * SUPPLY_VOLTAGE;
    directionData[sampleCount] = currentDirection ? 1 : -1;
    angleData[sampleCount] = cumulativeJointAngle;        // Unwrapped joint angle
    encoderAngleData[sampleCount] = cumulativeEncoderAngle; // Unwrapped encoder angle
    velocityData[sampleCount] = velocity;
    currentData[sampleCount] = current;
    testPhaseData[sampleCount] = currentPhase;

    sampleCount++;

    // Print progress every 2 seconds
    if (sampleCount % (SAMPLE_RATE_HZ * 2) == 0) {
        Serial.printf("Time: %.1fs | Phase: %d | PWM: %3d | Joint: %7.2f° | Encoder: %7.2f° (%.1f rot) | Vel: %6.2f°/s | I: %.3fA\n",
                     elapsedTime, currentPhase, currentPWM,
                     cumulativeJointAngle, cumulativeEncoderAngle, cumulativeEncoderAngle/360.0f,
                     velocity, current);
    }
}

void stopTest() {
    testRunning = false;

    if (currentMode == SHOULDER_COMPREHENSIVE) {
        stopMotor(SHOULDER_COMPREHENSIVE);
    } else if (currentMode == WRIST_COMPREHENSIVE) {
        stopMotor(WRIST_COMPREHENSIVE);
    }

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║      TEST COMPLETE                     ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("Collected %d samples\n", sampleCount);
    Serial.printf("Test duration: %.2f seconds\n", timeData[sampleCount-1]);

    printStatistics();

    Serial.println("\nSend 'p' to print data in CSV format\n");

    currentMode = IDLE;
}



void printStatistics() {
    Serial.println("\n┌──────────────────────────────────────────┐");
    Serial.println("│       TEST STATISTICS                    │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│ Min Joint Angle:    %9.2f°        │\n", minAngle);
    Serial.printf("│ Max Joint Angle:    %9.2f°        │\n", maxAngle);
    Serial.printf("│ Joint Range:        %9.2f°        │\n", maxAngle - minAngle);
    Serial.printf("│ Total Rotation:     %9.2f°        │\n", totalRotation);
    Serial.printf("│ Gear Ratio:         %9.1f:1      │\n", currentGearRatio);

    if (currentGearRatio > 1.0f) {
        float encoderRange = (maxAngle - minAngle) * currentGearRatio;
        Serial.printf("│ Encoder Rotations:  %9.2f         │\n", encoderRange / 360.0f);
    }

    Serial.printf("│ Phases:             %9d          │\n", NUM_PWM_LEVELS * 2);
    Serial.printf("│ PWM Range:          %3d - %3d            │\n", PWM_LEVELS[0], PWM_LEVELS[NUM_PWM_LEVELS-1]);
    Serial.println("└──────────────────────────────────────────┘\n");

    // Check coverage
    float angleCoverage = (maxAngle - minAngle) / 360.0f * 100.0f;
    Serial.printf("Joint Angular Coverage: %.1f%% of 360°\n", angleCoverage);

    if (angleCoverage > 80.0f) {
        Serial.println("✓ Excellent coverage!");
    } else if (angleCoverage > 50.0f) {
        Serial.println("✓ Good coverage");
    } else {
        Serial.println("⚠ Limited coverage - check mechanical limits or increase PWM");
    }

    Serial.printf("\nActual travel: %.2f degrees\n", maxAngle - minAngle);
}

void printData() {
    if (sampleCount == 0) {
        Serial.println("No data collected yet!");
        return;
    }

    Serial.println("\n========================================");
    Serial.println("  DATA EXPORT (CSV) - UNWRAPPED ANGLES");
    Serial.println("========================================");
    Serial.println("Copy the data below into a .csv file\n");
    Serial.println("---START DATA---");

    Serial.println("Time,PWM,Voltage,Direction,Phase,JointAngle,EncoderAngle,JointVelocity,Current,GearRatio");

    for (uint16_t i = 0; i < sampleCount; i++) {
        Serial.printf("%.4f,%.2f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.6f,%.1f\n",
                     timeData[i],
                     pwmData[i],
                     voltageData[i],
                     directionData[i],
                     testPhaseData[i],
                     angleData[i],
                     encoderAngleData[i],
                     velocityData[i],
                     currentData[i],
                     currentGearRatio);
    }

    Serial.println("---END DATA---\n");
    Serial.printf("Total samples: %d\n", sampleCount);
    Serial.printf("Actual joint travel: %.2f°\n\n", maxAngle - minAngle);

    Serial.println("Note: JointAngle and EncoderAngle are CUMULATIVE (unwrapped)");
    Serial.println("      Can exceed 360° to track multiple rotations correctly.\n");
}

void clearData() {
    sampleCount = 0;
    Serial.println("Data buffer cleared.\n");
}

float calculateVelocity(float currentAngle, float lastAngle, float dt) {
    // No wrapping needed - angles are already cumulative!
    return (currentAngle - lastAngle) / dt;
}


void printMenu() {
    Serial.println("════════════════════════════════════════");
    Serial.println("         COMMANDS");
    Serial.println("════════════════════════════════════════");
    Serial.println("  1 - SHOULDER comprehensive test");
    Serial.println("  2 - WRIST comprehensive test (4:1)");
    Serial.println("  t - Toggle PRMS/Sequential mode");
    Serial.println("  s - STOP test");
    Serial.println("  p - Print CSV data");
    Serial.println("  c - Clear buffer");
    Serial.println("  m - Show menu");
    Serial.println("  r - Recalibrate");
    Serial.println("  i - Test info");
    Serial.println("  g - Gear ratio info");
    Serial.println("════════════════════════════════════════\n");
    Serial.println("Ready...\n");
}

void handleSerialCommand() {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();

    switch (cmd) {
        case '1':
            if (!testRunning) startComprehensiveTest(SHOULDER_COMPREHENSIVE);
            else Serial.println("Test already running!");
            break;
        case '2':
            if (!testRunning) startComprehensiveTest(WRIST_COMPREHENSIVE);
            else Serial.println("Test already running!");
            break;
        case 's':
        case 'S':
            if (testRunning) stopTest();
            else Serial.println("No test running.");
            break;
        case 'p':
        case 'P':
            printData();
            break;
        case 'c':
        case 'C':
            clearData();
            break;
        case 'm':
        case 'M':
            printMenu();
            break;
        case 'r':
        case 'R':
            shoulderEncoderOffset = convertToAngle(readAS5600Angle(Wire));
            wristEncoderOffset = convertToAngle(readAS5600Angle(Wire1));
            Serial.println("Offsets recalibrated!");
            Serial.printf("Shoulder: %.2f°\n", shoulderEncoderOffset);
            Serial.printf("Wrist: %.2f°\n\n", wristEncoderOffset);
            break;
        case 'i':
        case 'I':
            Serial.println("\n┌─────────────────────────────────────┐");
            Serial.println("│       TEST CONFIGURATION            │");
            Serial.println("├─────────────────────────────────────┤");
            Serial.printf("│ PWM Levels: %d                      │\n", NUM_PWM_LEVELS);
            Serial.printf("│ Values: ");
            for (int i = 0; i < NUM_PWM_LEVELS; i++) Serial.printf("%d ", PWM_LEVELS[i]);
            Serial.println("           │");
            Serial.printf("│ Duration/level: %.1fs              │\n", STEP_DURATION_MS/1000.0f);
            Serial.printf("│ Total: %.1fs                        │\n", TOTAL_TEST_DURATION_MS/1000.0f);
            Serial.println("│ Angle Unwrapping: ENABLED           │");
            Serial.println("└─────────────────────────────────────┘\n");
            break;
        case 'g':
        case 'G':
            Serial.println("\n┌─────────────────────────────────────┐");
            Serial.println("│       GEAR RATIO INFO               │");
            Serial.println("├─────────────────────────────────────┤");
            Serial.printf("│ Shoulder: %.1f:1                    │\n", SHOULDER_GEAR_RATIO);
            Serial.printf("│ Wrist:    %.1f:1 (geared)          │\n", WRIST_GEAR_RATIO);
            Serial.println("│ Unwrapping: Tracks >360° correctly  │");
            Serial.println("└─────────────────────────────────────┘\n");
            break;

        case 't':
        case 'T':
            usePRMS = !usePRMS;
            Serial.printf("Test mode: %s\n\n", usePRMS ? "PRMS (Pseudorandom)" : "Sequential");
            break;
    }
}