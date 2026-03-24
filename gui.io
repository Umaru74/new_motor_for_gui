// "Step & Direction" mode needs to be set on MSP software
// HLFB_MODE_STATIC required when MSP HLFB is configured for
//"In Range - Position" or "In Range - Velocity"
// Set the Input Resolution 

#include "ClearCore.h"
#define motor1 ConnectorM0
#define motor2 ConnectorM1
#define motor3 ConnectorM2
#define motor4 ConnectorM3

// Run-time motor count
int MOTOR_COUNT = 0;

#define SerialPort ConnectorUsb

// --- Full pool of all 4 connectors ---
MotorDriver *motorPool[] = {&motor1, &motor2, &motor3, &motor4};

// --- Active motors array (pointer into motorPool, sized at runtime) ---
MotorDriver *motors[4] = {nullptr, nullptr, nullptr, nullptr};

// -----------------------------------------------------------------------
// Hardware Constants
// -----------------------------------------------------------------------
const int32_t encoderResolution = 8000; // maximum encoder resolution for step & direction mode 
const int32_t baudRate          = 115200; // needs to match with host computer USB baudRate

// -----------------------------------------------------------------------
// Default motor configuration 
// -----------------------------------------------------------------------
#define DEFAULT_RPM     2000 // in RPM / sec (CCW unless reversed on MSP software)
#define DEFAULT_ACCEL   200 // in RPM / sec^2
#define DEFAULT_DECEL   200 // in RPM / sec^2
#define DEFAULT_PHASE_0 0.0f // in degrees
#define DEFAULT_PHASE_1 90.0f // in degrees
#define DEFAULT_PHASE_2 180.0f // in degrees
#define DEFAULT_PHASE_3 270.0f // in degrees

// -----------------------------------------------------------------------
// Runtime configuration — changeable via serial
// -----------------------------------------------------------------------
int32_t  velocityRPM       = DEFAULT_RPM;
int32_t  velocityMAX       = DEFAULT_RPM;
uint32_t accelerationLimit = DEFAULT_ACCEL;
uint32_t decelerationLimit = DEFAULT_DECEL;
float    offsets[4]        = {DEFAULT_PHASE_0, DEFAULT_PHASE_1, DEFAULT_PHASE_2, DEFAULT_PHASE_3};

// Serial buffer for multi-char commands
String serialBuffer = "";

// Motor state
bool motorsRunning = false;

// Forward declarations
void PrintAlerts();
void DisableAllMotors();
void EnableAllMotors();
void StopAll();
void PrintStatus();

// Define system states
enum SystemState {
    STATE_IDLE,        // Motors enabled, not yet synced
    STATE_SYNCED,      // Manual sync done, ready for phase
    STATE_PHASE_SET,   // Phase angles applied, ready to run
    STATE_RUNNING,     // Motors spinning
    STATE_HOMED        // Motors homed, back to idle
};

SystemState systemState = STATE_IDLE;

// -----------------------------------------------------------------------
// Startup: Ask the user how many motors to activate (1-4), used in setup()
// -----------------------------------------------------------------------
void PromptMotorCount() {
    SerialPort.SendLine("==============================================");
    SerialPort.SendLine("  ClearCore Motor Controller - Startup");
    SerialPort.SendLine("==============================================");
    SerialPort.SendLine("How many motors will you be testing? (Enter 1, 2, 3, or 4)");

    char response = 0;
    while (true) {
        if (SerialPort.AvailableForRead() > 0) {
            response = SerialPort.CharGet();

            // Accept '1' through '4' only
            if (response >= '1' && response <= '4') {
                MOTOR_COUNT = response - '0'; // Convert ASCII char to integer
                break;
            } else {
                SerialPort.Send("Invalid input '");
                SerialPort.Send(response);
                SerialPort.SendLine("'. Please enter 1, 2, 3, or 4.");
            }
        }
        Delay_ms(10);
    }
    // Populate the active motors[] array from the pool
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i] = motorPool[i];
    }
    SerialPort.Send("Motor count set to: ");
    SerialPort.SendLine(MOTOR_COUNT);
    SerialPort.Send("Active connectors: M0");
    for (int i = 1; i < MOTOR_COUNT; i++) {
        SerialPort.Send(", M");
        SerialPort.Send(i);
    }
    SerialPort.SendLine("");
}

// -----------------------------------------------------------------------
// Configure acceleration, velocity limits for each active motor, HLFB mode
// -----------------------------------------------------------------------
void ConfigureMotors() {
    int32_t accelStepsPerSec_2  = (encoderResolution * accelerationLimit) / 60;
    int32_t velmaxStepsPerSec   = (encoderResolution * velocityMAX) / 60;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->AccelMax(accelStepsPerSec_2);
        motors[i]->VelMax(velmaxStepsPerSec);
        motors[i]->HlfbMode(MotorDriver::HLFB_MODE_STATIC);

        SerialPort.Send("  Motor M");
        SerialPort.Send(i);
        SerialPort.Send(" configured: accel=");
        SerialPort.Send(accelerationLimit);
        SerialPort.Send(" RPM/s^2, velMax=");
        SerialPort.Send(velocityMAX);
        SerialPort.SendLine(" RPM");
    }
}

// -----------------------------------------------------------------------
// Check if all active motors are stationary
// -----------------------------------------------------------------------
bool AllMotorsStationary() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->StatusReg().bit.ReadyState == MotorDriver::MOTOR_DISABLED) {
            continue; // Disabled = stationary by definition
        }
        if (!motors[i]->StepsComplete() || 
            motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            return false;
        }
    }
    return true;
}

// -----------------------------------------------------------------------
// Manual sync: step through each active motor individually
// -----------------------------------------------------------------------
void ManualSyncZero() {
    if (systemState != STATE_IDLE && systemState != STATE_HOMED){
        SerialPort.SendLine("ERROR: Must be in IDLE or HOMED state to sync");
        return;
    }
    SerialPort.SendLine("\n--- SEQUENTIAL MANUAL SYNC START ---");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->EnableRequest(true); // moves to the original home position set on MSP ?

        while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Alert on Motor M");
                SerialPort.Send(i);
                SerialPort.SendLine("! Homing aborted.");
                PrintAlerts();
                return;
            }
            Delay_ms(1);
        }

        motors[i]->PositionRefSet(0);
        Delay_ms(10);

        SerialPort.Send("Motor M");
        SerialPort.Send(i);
        SerialPort.SendLine(" SUCCESS: Enabled and Zeroed.\n");
    }
    systemState = STATE_SYNCED;
    SerialPort.SendLine("--- ALL MOTORS SYNCED ---");
}

// -----------------------------------------------------------------------
// Set phase/angle offsets for all active motors, positionRefSet(0) 
// -----------------------------------------------------------------------
void SetPhaseAngles(const float angles[]) {
    if (systemState != STATE_SYNCED) {
        SerialPort.SendLine("ERROR: Must run Manual Sync 'M' before setting phase.");
        return;
    }

    if (!AllMotorsStationary()) {
        SerialPort.SendLine("ERROR: Cannot set phase while motors are running! Command 'S' first.");
        return;
    }
    SerialPort.SendLine("Applying phase offsets...");

    // Command all motors simultaneously
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // Use float math before casting to int32_t at the end
        int32_t distance = (int32_t)((angles[i] * encoderResolution) / 360.0f);

        SerialPort.Send("Motor ");
        SerialPort.Send(i);
        SerialPort.Send(" target steps: ");
        SerialPort.Send(distance);
        SerialPort.Send(" pulse steps / ");
        SerialPort.Send(angles[i]);
        SerialPort.SendLine(" degrees");

        motors[i]->Move(distance, StepGenerator::MOVE_TARGET_ABSOLUTE);
    }
    while (!AllMotorsStationary()){
        Delay_ms(1);
    }
    systemState = STATE_PHASE_SET;
    SerialPort.SendLine("Phase lock established.");
}

// function: Start rotation at RPM
void StartRotation(int32_t rpm){
    if (systemState != STATE_PHASE_SET && systemState != STATE_SYNCED) {
        SerialPort.SendLine("ERROR: Must sync 'M' before running.");
        return;
    }
    // Convert RPM to steps/sec: (RPM/60)*8000
    int32_t velocityStepsPerSec = (encoderResolution * rpm ) / 60;

    SerialPort.Send("Starting synchronized rotation at RPM: ");
    SerialPort.SendLine(rpm);

    // Command the velocity move to all four motors synchronously
    for (int i = 0; i < MOTOR_COUNT; i++){
         motors[i]-> MoveVelocity(velocityStepsPerSec);
    }
    // set running state before ramp loop so StopAll() works if alert fires 
    motorsRunning = true;
    systemState = STATE_RUNNING;

    // Wait for the step command to ramp up to the commanded velocity
    SerialPort.SendLine("Ramping to speed...");
    uint32_t startTime = Milliseconds();

    bool allAtSpeed = false;
    while (!allAtSpeed) {
        allAtSpeed = true;

        for (int i = 0; i < MOTOR_COUNT; i++) {

            // Check for motor alerts
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert During Ramping: Motor ");
                SerialPort.SendLine(i);
                PrintAlerts();
                StopAll();
                return;
            }

            // Check if motor reached cruise velocity
            if (!motors[i]->CruiseVelocityReached()) {
                allAtSpeed = false;
            }
        }

        // Time Out Protection: 15s
        if (Milliseconds() - startTime > 30000) {
            SerialPort.SendLine("Error: Motors failed to reach target speed!");
            StopAll();
            return;
        }

        Delay_ms(1); // reduce CPU usage
    }
    SerialPort.Send("All motors have hit the desired RPM:");
    SerialPort.SendLine(rpm);
}

// function: Stop all motors smoothly
void StopAll() {
    if (systemState != STATE_RUNNING) {
        SerialPort.SendLine("ERROR: Motors are not running.");
        return;
    }
    SerialPort.SendLine("Initiating controlled stop for all motors...");

    // Convert acceleration limit (RPM/s^2) to pulses / s^2
    int32_t decelStepsPerSec_2 = (encoderResolution * decelerationLimit ) / 60;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        // "MoveStopDecel": Interrupts any current move and commands the motor to stop. 
        motors[i]->MoveStopDecel(decelStepsPerSec_2);
    }

    // Wait for all motors to ramp down and reach zero velocity
    bool allStopped = false;

    while (!allStopped) {
        allStopped = true; 
        for (int i = 0; i < MOTOR_COUNT; i++) {
            // Check alerts during stopping
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("Motor Alert While Stopping: Motor ");
                SerialPort.SendLine(i);
                PrintAlerts();
                systemState = STATE_IDLE;
                motorsRunning = false;
                return;
            }
            // Check if motor stops
            if (!motors[i]->StepsComplete()) {
                allStopped = false;
            }
        }
        Delay_ms(1);
    }
    systemState = STATE_IDLE;
    motorsRunning = false;
    SerialPort.SendLine("All motors have come to a stop.");
}

// Function: Move the motors to the homing position set by PositionRefSet(0)
void HomeAllMotors() {
    if (systemState != STATE_IDLE) {
        SerialPort.SendLine("ERROR: Stop motors before homing.");
        return;
    }
    SerialPort.SendLine("Homing all motors via MSP...");

    // Disable first to trigger MSP homing on re-enable
    DisableAllMotors();
    Delay_ms(500); // Let motors fully de-energize

    // Re-enable triggers MSP homing routine (CW to home)
    EnableAllMotors();

    // Now software zero matches physical home
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->PositionRefSet(0);
    }
    systemState = STATE_HOMED;
    SerialPort.SendLine("All motors homed and zeroed.");
}


//-------------------------------
// EnableAllMotors(): function to enable all the motors, timeout after 10seconds
//-------------------------------
void EnableAllMotors() {
    SerialPort.SendLine("Enabling all motors...");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->EnableRequest(true);
    }

    SerialPort.SendLine("Waiting for HLFB...");

    uint32_t startTime = Milliseconds();
    uint32_t lastStatusTime = Milliseconds();

    // Wait for ALL motors in parallel
    while (!AllMotorsStationary()) {
        // Print status every 1000ms
        if (Milliseconds() - lastStatusTime > 1000) {
            SerialPort.SendLine("Still waiting for HLFB...");
            lastStatusTime = Milliseconds(); // Reset status timer
        }

        // Timeout after 10 seconds
        if (Milliseconds() - startTime > 10000) {
            SerialPort.SendLine("ERROR: Timeout waiting for motors to enable.");
            return;
        }

        // Check for alerts on any motor
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("ERROR: Alert on motor ");
                SerialPort.SendLine(i);
                return;
            }
        }

        Delay_ms(1);
    }

    SerialPort.SendLine("All motors enabled and ready.");
}

void DisableAllMotors() {
    SerialPort.SendLine("Disabling all motors...");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motors[i]->StepsComplete() || motors[i]->VelocityRefCommanded() != 0) {
            SerialPort.SendLine("ERROR: Motion detected! Cannot disable motors while moving.");
            SerialPort.SendLine("Stop the moving motors first before disabling.");
            return;
        }
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i]->EnableRequest(false);
    }

    // Wait for hardware confirmation with timeout
    uint32_t startTime = Milliseconds();
    bool allDisabled = false;

    while (!allDisabled) {
        if (Milliseconds() - startTime > 2000) {
            SerialPort.SendLine("ERROR: Timeout waiting for motors to disable.");
            return;
        }

        allDisabled = true;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i]->StatusReg().bit.ReadyState != MotorDriver::MOTOR_DISABLED) {
                allDisabled = false;
            }
        }
        Delay_ms(1);
    }

    SerialPort.SendLine("All motors disabled and de-energized.");
}

// -----------------------------------------------------------------------
// HandleAlerts(): Clear motor faults by cycling enable and clearing registers
// -----------------------------------------------------------------------
void HandleAlerts() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->AlertReg().bit.MotorFaulted) {
            SerialPort.Send("Fault detected on motor ");
            SerialPort.SendLine(i);

            // Cycle enable to clear physical fault
            motors[i]->EnableRequest(false);
            Delay_ms(100);  // Give motor time to fully de-energize
            motors[i]->EnableRequest(true);

            // Wait for motor to recover before clearing alerts
            uint32_t startTime = Milliseconds();
            while (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED) {
                if (Milliseconds() - startTime > 5000) {
                    SerialPort.Send("ERROR: Motor ");
                    SerialPort.Send(i);
                    SerialPort.SendLine(" did not recover after fault clear.");
                    return;
                }
                Delay_ms(1);
            }

            // Now safe to clear alerts
            motors[i]->ClearAlerts();

            // Verify alerts actually cleared
            if (motors[i]->StatusReg().bit.AlertsPresent) {
                SerialPort.Send("WARNING: Motor ");
                SerialPort.Send(i);
                SerialPort.SendLine(" still has alerts after clear attempt.");
            } else {
                SerialPort.Send("Motor ");
                SerialPort.Send(i);
                SerialPort.SendLine(" fault cleared successfully.");
            }
        }
    }
}

// -----------------------------------------------------------------------
// PrintStatus(): Print all current runtime config
// -----------------------------------------------------------------------
void PrintStatus() {
    SerialPort.Send("State: ");
    switch (systemState) {
        case STATE_IDLE:      SerialPort.SendLine("IDLE");      break;
        case STATE_HOMED:     SerialPort.SendLine("HOMED");     break;
        case STATE_SYNCED:    SerialPort.SendLine("SYNCED");    break;
        case STATE_PHASE_SET: SerialPort.SendLine("PHASE_SET"); break;
        case STATE_RUNNING:   SerialPort.SendLine("RUNNING");   break;
    }
    SerialPort.Send("  Motors: ");        SerialPort.SendLine(MOTOR_COUNT);
    SerialPort.Send("  RPM:    ");        SerialPort.SendLine(velocityRPM);
    SerialPort.Send("  Accel:  ");        SerialPort.SendLine(accelerationLimit);
    SerialPort.Send("  Decel:  ");        SerialPort.SendLine(decelerationLimit);
    SerialPort.Send("  Phases: ");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        SerialPort.Send(offsets[i]);
        if (i < MOTOR_COUNT - 1) SerialPort.Send(", ");
    }
    SerialPort.SendLine(" degrees");
}

// -----------------------------------------------------------------------
// PrintAlerts(): Prints all active alerts for diagnostic purposes
// -----------------------------------------------------------------------
void PrintAlerts() {
    bool anyAlerts = false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i]->StatusReg().bit.AlertsPresent) {
            anyAlerts = true;
            SerialPort.Send("Alerts on Motor ");
            SerialPort.SendLine(i);

            if (motors[i]->AlertReg().bit.MotionCanceledInAlert) 
                SerialPort.SendLine("  - MotionCanceledInAlert");
            if (motors[i]->AlertReg().bit.MotionCanceledPositiveLimit) 
                SerialPort.SendLine("  - MotionCanceledPositiveLimit");
            if (motors[i]->AlertReg().bit.MotionCanceledNegativeLimit) 
                SerialPort.SendLine("  - MotionCanceledNegativeLimit");
            if (motors[i]->AlertReg().bit.MotionCanceledSensorEStop) 
                SerialPort.SendLine("  - MotionCanceledSensorEStop");
            if (motors[i]->AlertReg().bit.MotionCanceledMotorDisabled) 
                SerialPort.SendLine("  - MotionCanceledMotorDisabled");
            if (motors[i]->AlertReg().bit.MotorFaulted) 
                SerialPort.SendLine("  - MotorFaulted (Shutdown)");
        }
    }

    if (!anyAlerts) {
        SerialPort.SendLine("No alerts present on any motor.");
    }
}


// HLFB mode needs to be set for "In Range-Velocity" on MSP for each motor 
bool AllMotorsInRange(){
    for (int i = 0; i < MOTOR_COUNT; i++){
        // HlfbState() returns HLFB_ASSERTED when the motor is physically
        // within the MSP-defined speed window 
        if (motors[i]->HlfbState() != MotorDriver::HLFB_ASSERTED){
            SerialPort.Send("Motor out of range: ");
            SerialPort.SendLine(i);
            return false; // At least one motor has drifted or is still ramping
        }
    }
    return true; // All motors are physically at 2000 RPM (within tolerance)
}

// -----------------------------------------------------------------------
// processCommand(): Handle single-char and multi-char serial commands
// -----------------------------------------------------------------------
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // ── Single character commands ──
    if (cmd.length() == 1) {
        char c = toupper(cmd.charAt(0));
        switch (c) {
            case 'E': SerialPort.SendLine("----Enable All Motors----");    EnableAllMotors();          break;
            case 'D': SerialPort.SendLine("----Disable All Motors----");   DisableAllMotors();         break;
            case 'G': SerialPort.SendLine("----Home All Motors----");      HomeAllMotors();            break;
            case 'M': SerialPort.SendLine("----Manual Sync----");          ManualSyncZero();           break;
            case 'P': SerialPort.SendLine("----Set Phase Angles----");     SetPhaseAngles(offsets);    break;
            case 'R': SerialPort.SendLine("----Start Rotation----");       StartRotation(velocityRPM); break;
            case 'S': SerialPort.SendLine("----Stop All Motors----");      StopAll();                  break;
            case 'A': SerialPort.SendLine("----Print Alerts----");         PrintAlerts();              break;
            case 'H': SerialPort.SendLine("----Handle Alerts----");        HandleAlerts();             break;
            case '?': PrintStatus();                                                                   break;
        }
        return;
    }

    // ── RPM:2000 ──
    if (cmd.startsWith("RPM:")) {
        int32_t val = (int32_t)cmd.substring(4).toFloat();
        if (val < 100 || val > 3000) {
            SerialPort.SendLine("ERROR: RPM must be 100-3000.");
            return;
        }
        velocityRPM = val;
        velocityMAX = val;
        int32_t sps = (encoderResolution * velocityRPM) / 60;
        for (int i = 0; i < MOTOR_COUNT; i++) motors[i]->VelMax(sps);
        SerialPort.Send("RPM set to: ");
        SerialPort.SendLine(velocityRPM);
        return;
    }

    // ── ACCEL:300 ──
    if (cmd.startsWith("ACCEL:")) {
        uint32_t val = (uint32_t)cmd.substring(6).toFloat();
        if (val < 10 || val > 1000) {
            SerialPort.SendLine("ERROR: Accel must be 10-1000.");
            return;
        }
        accelerationLimit = val;
        int32_t sps2 = (encoderResolution * accelerationLimit) / 60;
        for (int i = 0; i < MOTOR_COUNT; i++) motors[i]->AccelMax(sps2);
        SerialPort.Send("Accel set to: ");
        SerialPort.SendLine(accelerationLimit);
        return;
    }

    // ── DECEL:300 ──
    if (cmd.startsWith("DECEL:")) {
        uint32_t val = (uint32_t)cmd.substring(6).toFloat();
        if (val < 10 || val > 1000) {
            SerialPort.SendLine("ERROR: Decel must be 10-1000.");
            return;
        }
        decelerationLimit = val;
        SerialPort.Send("Decel set to: ");
        SerialPort.SendLine(decelerationLimit);
        return;
    }

    // ── PHASE:0,90.5,180,270 ──
    if (cmd.startsWith("PHASE:")) {
        String data = cmd.substring(6);
        float vals[4] = {0, 0, 0, 0};
        int idx = 0, start = 0;
        for (int i = 0; i <= (int)data.length() && idx < 4; i++) {
            if (i == (int)data.length() || data.charAt(i) == ',') {
                vals[idx++] = data.substring(start, i).toFloat();
                start = i + 1;
            }
        }
        for (int i = 0; i < 4; i++) offsets[i] = vals[i];
        SerialPort.Send("Phases set to: ");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            SerialPort.Send(offsets[i]);
            if (i < MOTOR_COUNT - 1) SerialPort.Send(", ");
        }
        SerialPort.SendLine(" degrees");
        return;
    }

    // ── MOTORS:2 ──
    if (cmd.startsWith("MOTORS:")) {
        int val = cmd.substring(7).toInt();
        if (val < 1 || val > 4) {
            SerialPort.SendLine("ERROR: Motor count must be 1-4.");
            return;
        }
        MOTOR_COUNT = val;
        for (int i = 0; i < MOTOR_COUNT; i++) motors[i] = motorPool[i];
        SerialPort.Send("Motor count set to: ");
        SerialPort.SendLine(MOTOR_COUNT);
        return;
    }

    // ── RESET ──
    if (cmd == "RESET") {
        velocityRPM       = DEFAULT_RPM;
        velocityMAX       = DEFAULT_RPM;
        accelerationLimit = DEFAULT_ACCEL;
        decelerationLimit = DEFAULT_DECEL;
        offsets[0] = DEFAULT_PHASE_0;
        offsets[1] = DEFAULT_PHASE_1;
        offsets[2] = DEFAULT_PHASE_2;
        offsets[3] = DEFAULT_PHASE_3;
        int32_t sps  = (encoderResolution * velocityRPM) / 60;
        int32_t sps2 = (encoderResolution * accelerationLimit) / 60;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motors[i]->VelMax(sps);
            motors[i]->AccelMax(sps2);
        }
        SerialPort.SendLine("All settings reset to defaults.");
        PrintStatus();
        return;
    }

    // ── Unknown ──
    SerialPort.Send("ERROR: Unknown command: ");
    SerialPort.SendLine(cmd.c_str());
}
// -----------------------------------------------------------------------
// setup(): Runs when the controller is powered up
// -----------------------------------------------------------------------
void setup() {
    // Set USB serial
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();

    uint32_t start = Milliseconds();
    uint32_t timeout = 5000;
    while (!SerialPort) {
        if (Milliseconds() - start > timeout) {
            break; // Continue even if no terminal is connected
        }
    }
    Delay_ms(200); // Short settle time for the terminal to connect

    // Step 1: Ask how many motors
    PromptMotorCount();

    // Step 2: Apply global mode settings (must be done before per-motor config)
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    // Step 3: Configure only the active motors
    SerialPort.SendLine("Configuring active motors...");
    ConfigureMotors();

    // Step 4: Print default configuration on startup
    SerialPort.SendLine("----------------------------------------------");
    SerialPort.SendLine("Default Configuration:");
    SerialPort.Send("  RPM:   "); SerialPort.SendLine(velocityRPM);
    SerialPort.Send("  Accel: "); SerialPort.SendLine(accelerationLimit);
    SerialPort.Send("  Decel: "); SerialPort.SendLine(decelerationLimit);
    SerialPort.Send("  Phase: ");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        SerialPort.Send(offsets[i]);
        if (i < MOTOR_COUNT - 1) SerialPort.Send(", ");
    }
    SerialPort.SendLine(" degrees");

    // Step 5: Print available commands
    SerialPort.SendLine("----------------------------------------------");
    SerialPort.SendLine("Motion Commands:");
    SerialPort.SendLine("  E = Enable motors");
    SerialPort.SendLine("  D = Disable motors");
    SerialPort.SendLine("  G = Home all motors");
    SerialPort.SendLine("  M = Manual sync");
    SerialPort.SendLine("  P = Set phase angles");
    SerialPort.SendLine("  R = Start rotation");
    SerialPort.SendLine("  S = Stop all motors");
    SerialPort.SendLine("  A = Print alerts");
    SerialPort.SendLine("  H = Handle alerts");
    SerialPort.SendLine("  ? = Print current config");
    SerialPort.SendLine("----------------------------------------------");
    SerialPort.SendLine("Config Commands:");
    SerialPort.SendLine("  RPM:<value>            e.g. RPM:1500");
    SerialPort.SendLine("  ACCEL:<value>          e.g. ACCEL:300");
    SerialPort.SendLine("  DECEL:<value>          e.g. DECEL:150");
    SerialPort.SendLine("  PHASE:m0,m1,m2,m3     e.g. PHASE:0,90.5,180,270");
    SerialPort.SendLine("  MOTORS:<value>         e.g. MOTORS:3");
    SerialPort.SendLine("  RESET                  Restore all defaults");
    SerialPort.SendLine("----------------------------------------------");
    SerialPort.SendLine("Sequence: G → M → P → R → S");
    SerialPort.SendLine("----------------------------------------------");
}

void loop() {
    // 1: Buffered serial read — supports single-char and multi-char commands
    if (SerialPort.AvailableForRead() > 0) {
        char c = SerialPort.CharGet();
        if (c == '\n' || c == '\r') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                processCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            serialBuffer += c;
        }
    }

    // 2: Continuous Monitoring (Only when motors are running)
    if (!AllMotorsStationary() && motorsRunning) {
        if (!AllMotorsInRange()) {
            SerialPort.SendLine("CRITICAL: Synchronization lost or motor stalled!");
            PrintAlerts();
            StopAll();
        }
    }
}
