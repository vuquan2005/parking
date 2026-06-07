#include <Arduino.h>
#include <Wire.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define SDA_PIN 21
#define SCL_PIN 22
#define PCA_ADDR 0x40

#pragma region PCA Helper
/**
 * @brief Write a single byte to a PCA9685 register.
 *
 * @param reg Register address inside the PCA9685.
 * @param data Byte value to write.
 */
void pcaWriteRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

/**
 * @brief Configure the PCA9685 for basic PWM operation.
 *
 * This initializes the PCA9685 mode registers, sets the prescale value,
 * and prepares the board for output on the configured channels.
 */
void setupPCA() {
  // Reset PCA9685 MODE1 register to normal mode.
  pcaWriteRegister(0x00, 0x00);
  delay(10);
  // Enable auto-increment so multiple registers can be written in one I2C
  // sequence.
  pcaWriteRegister(0x00, 0x10);
  // Set prescale for PWM frequency (approx. 50-60 Hz depending on oscillator).
  pcaWriteRegister(0xFE, 5);
  // Enable PCA9685 restart and auto-increment features.
  pcaWriteRegister(0x00, 0xA1);
  // Set all channels to off.
  pcaWriteRegister(0xFD, 0x10);
}

/**
 * @brief Set the PWM duty cycle for a PCA9685 channel.
 *
 * @param channel The PCA channel number (0-15).
 * @param value PWM speed value from 0..255.
 *
 * The function converts an 8-bit speed value into the 12-bit PWM range used
 * by the PCA9685. A value of 255 produces a full-on output.
 */
void pcaSetPWM(uint8_t channel, uint8_t value) {
  uint16_t pwm = (value == 255) ? 4095 : (value * 16);
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * channel);
  Wire.write(0);
  Wire.write(0);
  Wire.write(pwm & 0xFF);
  Wire.write(pwm >> 8);
  Wire.endTransmission();
}
#pragma endregion

/*
 * Motor Classes
 */
#pragma region Motor Classes

/**
 * @brief Direction command characters for a motor.
 *
 * Each motor uses a small command set for forward, backward and stop.
 * The last character of a received command string maps to one of these values.
 */
struct DirectionCodes {
  char forward;
  char backward;
  char stop;
};

/**
 * @brief Abstract base class for all motors.
 *
 * MotorBase provides common runtime tracking, command parsing, and state.
 * Hardware-specific subclasses implement driveImpl() and begin().
 */
class MotorBase {
 public:
  const char* code;      /**< command prefix for this motor, e.g. "11N" */
  uint8_t speedForward;  /**< motor speed for forward direction (0..255) */
  uint8_t speedBackward; /**< motor speed for backward direction (0..255) */
  DirectionCodes dirs;   /**< direction letters assigned to this motor */
  uint32_t maxRunTimeMs; /**< maximum allowed run duration before auto-stop */
  uint32_t runStartMs;   /**< millis() timestamp when motor last started */
  bool isRunning;        /**< true if motor is currently running */
  char activeDirection;  /**< currently active direction code */

  MotorBase(const char* code_, uint8_t speed_, DirectionCodes dirs_,
            uint32_t maxRunTimeMs_ = 0)
      : code(code_),
        speedForward(speed_),
        speedBackward(speed_),
        dirs(dirs_),
        maxRunTimeMs(maxRunTimeMs_),
        runStartMs(0),
        isRunning(false),
        activeDirection(dirs_.stop) {}

  MotorBase(const char* code_, uint8_t speedForward_, uint8_t speedBackward_,
            DirectionCodes dirs_, uint32_t maxRunTimeMs_ = 0)
      : code(code_),
        speedForward(speedForward_),
        speedBackward(speedBackward_),
        dirs(dirs_),
        maxRunTimeMs(maxRunTimeMs_),
        runStartMs(0),
        isRunning(false),
        activeDirection(dirs_.stop) {}

  /**
   * @brief Initialize the motor hardware.
   *
   * Called once from setup() after the PCA and GPIO interfaces are ready.
   */
  virtual void begin() = 0;

  /**
   * @brief Drive the motor and update runtime tracking.
   *
   * If direction is stop, the motor is halted and runtime tracking resets.
   * Otherwise the motor is started and runtime is timestamped.
   */
  void drive(char direction) {
    driveImpl(direction);
    if (direction == dirs.stop) {
      isRunning = false;
      activeDirection = dirs.stop;
    } else {
      isRunning = true;
      activeDirection = direction;
      runStartMs = millis();
    }
  }

  /**
   * @brief Stop the motor if it has run longer than the configured timeout.
   *
   * This is called regularly from loop() to enforce maxRunTimeMs.
   */
  void updateRuntime() {
    if (!isRunning || maxRunTimeMs == 0) return;

    if ((uint32_t)(millis() - runStartMs) >= maxRunTimeMs) {
      drive(dirs.stop);
      Serial.print(F("Motor "));
      Serial.print(code);
      Serial.println(F(": runtime exceeded, stopped"));
    }
  }

  /**
   * @brief Check if a command string addresses this motor.
   *
   * The command must start with this motor's code and end with a known
   * direction.
   */
  virtual bool matchesCommand(const String& command) const {
    if (!command.startsWith(code) || command.length() < strlen(code) + 1)
      return false;
    char dir = command.charAt(command.length() - 1);
    return dir == dirs.forward || dir == dirs.backward || dir == dirs.stop;
  }

  /**
   * @brief Extract the direction character from a command string.
   *
   * For commands targeting this motor, the last character is the direction.
   */
  virtual char commandDirection(const String& command) const {
    return command.charAt(command.length() - 1);
  }

  /**
   * @brief Process a command string if it targets this motor.
   *
   * Returns true when the command was valid and executed.
   */
  virtual bool processCommand(const String& command) {
    if (!matchesCommand(command)) return false;
    drive(commandDirection(command));
    return true;
  }

  /**
   * @brief Return the stop character for this motor.
   *
   * Used when halting all motors.
   */
  virtual char stopDirection() const { return dirs.stop; }

  virtual ~MotorBase() = default;

 protected:
  /**
   * @brief Output a raw drive signal to the motor hardware.
   *
   * Subclasses implement this to actually change motor pins or PCA channels.
   * @param direction One of dirs.forward, dirs.backward or dirs.stop.
   */
  virtual void driveImpl(char direction) = 0;
};

class MotorGPIO : public MotorBase {
 public:
  uint8_t pinA;
  uint8_t pinB;

  /**
   * @brief GPIO-based motor using two output pins.
   *
   * pinA and pinB are driven in a push-pull fashion. One pin is driven with
   * PWM while the other is held LOW to set direction.
   */
  MotorGPIO(const char* code_, uint8_t pinA_, uint8_t pinB_, uint8_t speed_,
            DirectionCodes dirs_, uint32_t maxRunTimeMs_ = 0)
      : MotorBase(code_, speed_, dirs_, maxRunTimeMs_),
        pinA(pinA_),
        pinB(pinB_) {}

  MotorGPIO(const char* code_, uint8_t pinA_, uint8_t pinB_,
            uint8_t speedForward_, uint8_t speedBackward_, DirectionCodes dirs_,
            uint32_t maxRunTimeMs_ = 0)
      : MotorBase(code_, speedForward_, speedBackward_, dirs_, maxRunTimeMs_),
        pinA(pinA_),
        pinB(pinB_) {}

  void begin() override {
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    drive(stopDirection());
  }

 protected:
  void driveImpl(char direction) override {
    if (direction == dirs.forward) {
      analogWrite(pinA, speedForward);
      analogWrite(pinB, 0);
    } else if (direction == dirs.backward) {
      analogWrite(pinA, 0);
      analogWrite(pinB, speedBackward);
    } else {
      analogWrite(pinA, 0);
      analogWrite(pinB, 0);
    }
  }
};

class MotorPCA9685 : public MotorBase {
 public:
  uint8_t chA;
  uint8_t chB;

  /**
   * @brief PCA9685-driven motor channel pair.
   *
   * Uses two PCA channels to control direction by enabling one side and
   * disabling the other.
   */
  MotorPCA9685(const char* code_, uint8_t chA_, uint8_t chB_, uint8_t speed_,
               DirectionCodes dirs_, uint32_t maxRunTimeMs_ = 0)
      : MotorBase(code_, speed_, dirs_, maxRunTimeMs_), chA(chA_), chB(chB_) {}

  MotorPCA9685(const char* code_, uint8_t chA_, uint8_t chB_,
               uint8_t speedForward_, uint8_t speedBackward_,
               DirectionCodes dirs_, uint32_t maxRunTimeMs_ = 0)
      : MotorBase(code_, speedForward_, speedBackward_, dirs_, maxRunTimeMs_),
        chA(chA_),
        chB(chB_) {}

  void begin() override {}

 protected:
  void driveImpl(char direction) override {
    if (direction == dirs.forward) {
      pcaSetPWM(chA, speedForward);
      pcaSetPWM(chB, 0);
    } else if (direction == dirs.backward) {
      pcaSetPWM(chA, 0);
      pcaSetPWM(chB, speedBackward);
    } else {
      pcaSetPWM(chA, 0);
      pcaSetPWM(chB, 0);
    }
  }
};

#pragma endregion
/*
 * End of Motor Classes
 */

#pragma region Motor Config
int toc_do_ngang_tang_1 = 160;
int toc_do_ngang_tang_2 = 150;
int toc_do_keo = 255;

/**
 * @brief Maximum run time for motors using the "N" direction set.
 *
 * Used to auto-stop lifting motors after 5 seconds.
 */
#define MAX_RUN_TIME_MS_N 5000

/**
 * @brief Maximum run time for motors using the "K" direction set.
 *
 * Used to auto-stop winch motors after 10 seconds.
 */
#define MAX_RUN_TIME_MS_K 15000

/**
 * @brief Direction codes for lifting motors.
 *
 * 'P' = forward/up, 'T' = backward/down, 'S' = stop.
 */
const DirectionCodes DIR_N = {'P', 'T', 'S'};

/**
 * @brief Direction codes for pulling motors.
 *
 * 'U' = forward/up, 'D' = backward/down, 'S' = stop.
 */
const DirectionCodes DIR_K = {'U', 'D', 'S'};

MotorPCA9685 motor1("11N", 0, 1, 100, DIR_N, MAX_RUN_TIME_MS_N);
MotorPCA9685 motor2("12N", 2, 3, 110, DIR_N, MAX_RUN_TIME_MS_N);
MotorPCA9685 motor3("13N", 4, 5, 135, DIR_N, MAX_RUN_TIME_MS_N);
MotorPCA9685 motor4("21N", 6, 7, 120, DIR_N, MAX_RUN_TIME_MS_N);
MotorPCA9685 motor5("22N", 8, 9, 140, 120, DIR_N, MAX_RUN_TIME_MS_N);
MotorPCA9685 motor6("23N", 10, 11, 130, DIR_N, MAX_RUN_TIME_MS_N);

MotorGPIO motor7("21K", 32, 33, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor8("22K", 25, 26, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor9("23K", 27, 14, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor10("31K", 18, 19, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor11("32K", 23, 13, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor12("33K", 4, 5, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);
MotorGPIO motor13("34K", 2, 15, toc_do_keo, DIR_K, MAX_RUN_TIME_MS_K);

/**
 * @brief List of all motors managed by the program.
 *
 * Commands are matched against this array in order, so motor codes must be
 * unique.
 */
MotorBase* motors[] = {&motor1,  &motor2,  &motor3, &motor4, &motor5,
                       &motor6,  &motor7,  &motor8, &motor9, &motor10,
                       &motor11, &motor12, &motor13};
#pragma endregion

#pragma region Motor Control
/**
 * @brief Stop every registered motor immediately.
 *
 * This is used as a safe fallback before starting a new motor command,
 * and also for the special "ST" command.
 */
void stopAllMotors() {
  Serial.println(F("Stop all motors!"));
  for (auto motor : motors) motor->drive(motor->stopDirection());
}

/**
 * @brief Handle a command received over Serial or Serial2.
 *
 * Expected command formats:
 *  - "ST" to stop all motors.
 *  - "<motorCode><direction>" where the last character is one of:
 *      forward, backward or stop for that motor's direction set.
 */
void handleCommand(const String& s) {
  if (s.length() < 2) return;
  if (s == "ST") {
    stopAllMotors();
    return;
  }
  for (auto motor : motors) {
    if (motor->matchesCommand(s)) {
      stopAllMotors();
      motor->drive(motor->commandDirection(s));
      Serial.print(F("Processed command: "));
      Serial.println(s);
      return;
    }
  }
  Serial.println(F("ERROR: invalid command or direction!"));
  Serial.print(F("Rejected command: "));
  Serial.println(s);
}
#pragma endregion

#pragma region Setup
void setup() {
  // Các pin bị kéo tự động khi khởi động, đảm bảo chúng ở trạng thái LOW
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(15, LOW);
  digitalWrite(5, LOW);
  // Hơi vô dụng vì sau đó đã được cấu hình lại trong begin() của MotorGPIO, và
  // tắt trong stopAllMotors() Đề xuất dùng PCA module

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Wire.begin(SDA_PIN, SCL_PIN);
  setupPCA();
  for (auto motor : motors) motor->begin();

  delay(100);
  stopAllMotors();
  Serial.println(F("Motor control ready!"));
}
#pragma endregion

#pragma region Loop
void loop() {
  if (Serial2.available() > 0) {
    String masterCommand = Serial2.readStringUntil('\n');
    masterCommand.trim();
    masterCommand.toUpperCase();
    handleCommand(masterCommand);
  }
  if (Serial.available() > 0) {
    String pcCommand = Serial.readStringUntil('\n');
    pcCommand.trim();
    pcCommand.toUpperCase();
    handleCommand(pcCommand);
  }

  for (auto motor : motors) motor->updateRuntime();
}
#pragma endregion
