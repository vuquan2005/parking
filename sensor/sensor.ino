#include <Arduino.h>
#include <initializer_list>

#define DEBOUNCE_DELAY 100
#define SENSOR_CLEAR 1
#define SENSOR_DETECTED 0

#define UART_BAUD_RATE 115200
#define TX_PIN 17
#define RX_PIN 16

#pragma region IRSensorManager
class SensorManager
{
private:
    int *sensorPins;
    int numSensors;
    bool *lastReadState;
    bool *validatedState;
    unsigned long *lastDebounceTime;

    int updateInternal(bool stopAtFirstChange);

public:
    SensorManager(const int pins[], int count);
    SensorManager(std::initializer_list<int> pins);
    ~SensorManager();
    SensorManager(const SensorManager &) = delete;
    SensorManager &operator=(const SensorManager &) = delete;

    void begin();
    bool update();
    int popUpdate();
    void getAllStates(char *buffer, int bufferSize);
    bool getSensorState(int index);
    int getNumSensors() { return numSensors; }
};

SensorManager::SensorManager(const int pins[], int count)
{
    numSensors = (count > 0) ? count : 0;
    if (numSensors > 0)
    {
        sensorPins = new int[numSensors];
        lastReadState = new bool[numSensors];
        validatedState = new bool[numSensors];
        lastDebounceTime = new unsigned long[numSensors];

        for (int i = 0; i < numSensors; i++)
        {
            sensorPins[i] = pins[i];
            lastReadState[i] = SENSOR_CLEAR;
            validatedState[i] = SENSOR_CLEAR;
            lastDebounceTime[i] = 0;
        }
    }
    else
    {
        sensorPins = nullptr;
        lastReadState = nullptr;
        validatedState = nullptr;
        lastDebounceTime = nullptr;
    }
}

SensorManager::SensorManager(std::initializer_list<int> pins)
    : SensorManager(pins.begin(), pins.size())
{
}

SensorManager::~SensorManager()
{
    delete[] sensorPins;
    delete[] lastReadState;
    delete[] validatedState;
    delete[] lastDebounceTime;
}

void SensorManager::begin()
{
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(sensorPins[i], INPUT_PULLUP);
        lastReadState[i] = digitalRead(sensorPins[i]);
        validatedState[i] = lastReadState[i];
    }
}

int SensorManager::updateInternal(bool stopAtFirstChange)
{
    unsigned long currentMillis = millis();
    int changedIndex = -1;

    for (int i = 0; i < numSensors; i++)
    {
        bool currentState = digitalRead(sensorPins[i]);

        if (currentState != lastReadState[i])
        {
            lastDebounceTime[i] = currentMillis;
        }

        if ((currentMillis - lastDebounceTime[i]) > DEBOUNCE_DELAY)
        {
            if (currentState != validatedState[i])
            {
                validatedState[i] = currentState;
                changedIndex = i;
                if (stopAtFirstChange)
                {
                    lastReadState[i] = currentState;
                    return changedIndex;
                }
            }
        }
        lastReadState[i] = currentState;
    }
    return changedIndex;
}

bool SensorManager::update()
{
    return updateInternal(false) != -1;
}

int SensorManager::popUpdate()
{
    return updateInternal(true);
}

void SensorManager::getAllStates(char *buffer, int bufferSize)
{
    if (bufferSize <= 0)
    {
        return;
    }

    int count = (numSensors < bufferSize - 1) ? numSensors : bufferSize - 1;
    for (int i = 0; i < count; i++)
    {
        buffer[i] = (validatedState[i] == SENSOR_DETECTED) ? '1' : '0';
    }
    buffer[count] = '\0';
}

bool SensorManager::getSensorState(int index)
{
    if (index >= 0 && index < numSensors)
    {
        return validatedState[index] == SENSOR_DETECTED;
    }
    return false;
}

#pragma endregion

SensorManager IR1({27, 14, 12, 13});
SensorManager IR2({18, 19, 21});
SensorManager IR3({22, 23, 34, 35}); // đã có pull-up

SensorManager SW1({32, 33, 25, 26});
SensorManager SW2({15, 36, 4, 5});

bool sensorDebug(char *code, SensorManager &manager)
{
    int index = -1;
    bool hasChange = false;

    while ((index = manager.popUpdate()) != -1)
    {
        Serial.print(code);
        Serial.print(index);
        Serial.print(": ");
        Serial.println(manager.getSensorState(index) ? "DETECTED" : "CLEAR");
        hasChange = true;
    }

    if (hasChange)
        return true;
    return false;
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    IR1.begin();
    IR2.begin();
    IR3.begin();
    SW1.begin();
    SW2.begin();

    Serial.println("Sensor system initialized.");
}

void loop()
{
    bool ir1Changed = sensorDebug("IR1", IR1);
    bool ir2Changed = sensorDebug("IR2", IR2);
    bool ir3Changed = sensorDebug("IR3", IR3);

    if (ir1Changed || ir2Changed || ir3Changed)
    {
        char stateBuffer[5];

        Serial2.print("IR");

        IR1.getAllStates(stateBuffer, sizeof(stateBuffer));
        Serial2.print(" ");
        Serial2.print(stateBuffer);

        IR2.getAllStates(stateBuffer, sizeof(stateBuffer));
        Serial2.print(" ");
        Serial2.print(stateBuffer);

        IR3.getAllStates(stateBuffer, sizeof(stateBuffer));
        Serial2.print(" ");
        Serial2.print(stateBuffer);
        Serial2.println();
    }

    bool sw1Changed = sensorDebug("SW1", SW1);
    bool sw2Changed = sensorDebug("SW2", SW2);

    if (sw1Changed || sw2Changed)
    {
        char stateBuffer[5];
        Serial2.print("SW");
        Serial.print("SW");

        SW1.getAllStates(stateBuffer, sizeof(stateBuffer));
        Serial2.print(" ");
        Serial2.print(stateBuffer);
        Serial.print(" ");
        Serial.print(stateBuffer);

        SW2.getAllStates(stateBuffer, sizeof(stateBuffer));
        Serial2.print(" ");
        Serial2.print(stateBuffer);
        Serial2.println();

        Serial.print(" ");
        Serial.println(stateBuffer);

    }
}
