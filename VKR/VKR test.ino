#include "LoRaWan_APP.h"
#include <ModbusMaster.h>
#include <EEPROM.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

RadioEvents_t RadioEvents;

// Структура для хранения конфигурации устройства Modbus
struct DeviceConfig {
    uint8_t deviceType;
    uint8_t deviceID;
    uint16_t startReg;
    uint16_t numRegs;
    uint16_t pollInterval;
    unsigned long lastPollTime;
};

// Константы и размеры
#define MAX_DEVICES 10
#define CONFIG_START_ADDR 0

// Предварительно установленные устройства
const DeviceConfig predefinedDevices[] = {
    {
        .deviceType = 1,                  // Тип устройства (можете поменять по своему усмотрению)
        .deviceID = 1,                    // Slave Address устройства
        .startReg = 100,                  // Начало диапазона регистров
        .numRegs = 1,                     // Сколько регистров читать
        .pollInterval = 10000,            // Интервал опроса (10 секунд)
        .lastPollTime = 0                 // Последний успешный опрос
    },
    {
        .deviceType = 2,                  // Еще один тип устройства
        .deviceID = 2,
        .startReg = 200,
        .numRegs = 1,
        .pollInterval = 15000,
        .lastPollTime = 0
    }
};

// Размеры массива предопределённых устройств
const size_t NUM_PREDEFINED_DEVICES = sizeof(predefinedDevices) / sizeof(DeviceConfig);

// Другие важные константы и переменные
uint8_t devEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appKey[] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };

uint8_t nwkSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };

uint32_t devAddr =  ( uint32_t )0x22222226;

uint16_t userChannelsMask[6] = { 0x0004, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;

uint32_t appTxDutyCycle = 300000;

bool overTheAirActivation = false;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// Глобальные переменные
ModbusMaster node;
DeviceConfig devices[MAX_DEVICES];
uint8_t numDevices = 0;
bool configChanged = false;
bool configMode = false;
static osjob_t sendjob;

// Флаг для проверки успешного присоединения
bool joinedSuccessfully = false;

// Вспомогательные функции
void onEvent();
void OnTxDone();
void prepareTxFrame(uint8_t port);
void loadPredefinedDevices();
void processDeviceData(uint8_t deviceType, uint16_t reg, uint16_t value);
void printConfig();
void handleNormalMode();
void forcePoll(String cmd);
void pollDevice(uint8_t devIndex);


// Инициализация модуля
void setup() {
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, 16, 17); // Hardware Serial для RS485 (GPIO16-RX, GPIO17-TX)
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE); 
    
    // Инициализация Modbus
    node.begin(1, Serial1);

    RadioEvents.TxDone = OnTxDone; // Регистрация только доступного обработчика TxDone

    Radio.Init(&RadioEvents);

    LoRaWAN.init(loraWanClass, loraWanRegion);
    LoRaWAN.setDefaultDR(8); // SF8 — среднее соотношение качества и скорости

    // Загрузка предопределённых устройств
    loadPredefinedDevices();

    // Установка начальных состояний
    deviceState = DEVICE_STATE_INIT;
    joinedSuccessfully = false;
}

// Основной цикл программы
void loop() {
    //handleNormalMode(); // Всё управление осуществляется через метод handleNormalMode()
    onEvent();
}




void OnTxDone() {
  Serial.println("TX Done");
  deviceState = DEVICE_STATE_CYCLE; // Переход к следующему состоянию после успешной передачи
}

void prepareTxFrame(uint8_t port) {
  appDataSize = 4;
  appData[0] = 0x01;
  appData[1] = 0x02;
  appData[2] = 0x03;
  appData[3] = 0x04;
}

// Загрузка предопределённых устройств
void loadPredefinedDevices() {
    memcpy(devices, predefinedDevices, sizeof(predefinedDevices)); // Скопировать предопределённые устройства
    numDevices = NUM_PREDEFINED_DEVICES;                          // Установить количество устройств
}

// Обработчик события основного цикла
void onEvent() {
    switch(deviceState) {
        case DEVICE_STATE_INIT: {
            if (!joinedSuccessfully) { // Проверяем флаг присоединения
                Serial.println("LoRaWAN.join");
                LoRaWAN.join();
                Serial.println("LoRaWAN.joined");
            } else {
                deviceState = DEVICE_STATE_SEND; // Переходим к отправке
            }
            break;
        }
        
        case DEVICE_STATE_SEND: {
            Serial.println("LoRaWAN.send");
            prepareTxFrame(appPort);
            LoRaWAN.send();
            Serial.println("LoRaWAN.sent");
            deviceState = DEVICE_STATE_CYCLE; // Переходим к следующему состоянию
            break;
        }
        
        case DEVICE_STATE_CYCLE: {
            Serial.println("Waiting for next transmission...");
            delay(appTxDutyCycle); // Ждем перед следующей передачей
            deviceState = DEVICE_STATE_SEND; // Повторяем цикл
            break;
        }
        
        default:
        {
            Serial.println(deviceState);
        }
    }
    
    Radio.IrqProcess(); // Обработка прерываний радио-модуля
}

// Процедура обработки данных устройства
void processDeviceData(uint8_t deviceType, uint16_t reg, uint16_t value) {
    // Реализуйте свою логику обработки данных здесь
}

// Печать конфигурации
void printConfig() {
    Serial.println(F("Current configuration:"));
    Serial.print(F("Devices: "));
    Serial.print(numDevices);
    Serial.print(F("/"));
    Serial.println(MAX_DEVICES);
    Serial.println(F("Index | Type | ID | StartReg | NumRegs | Interval"));
    Serial.println(F("---------------------------------------------"));

    for (int i = 0; i < numDevices; i++) {
        Serial.print(i);
        Serial.print(F("     | "));
        Serial.print(devices[i].deviceType);
        Serial.print(F("    | "));
        Serial.print(devices[i].deviceID);
        Serial.print(F("  | "));
        Serial.print(devices[i].startReg);
        Serial.print(F("      | "));
        Serial.print(devices[i].numRegs);
        Serial.print(F("     | "));
        Serial.print(devices[i].pollInterval);
        Serial.println(F("ms"));
    }

    if (configChanged) {
        Serial.println(F("Warning: Config has changes not saved to EEPROM! Use 'save' command."));
    }
}

// Основной режим работы приложения
void handleNormalMode() {
    // Автоматический опрос устройств
    for (int i = 0; i < numDevices; i++) {
        if (millis() - devices[i].lastPollTime >= devices[i].pollInterval) {
            pollDevice(i);
            devices[i].lastPollTime = millis();
        }
    }

    // Обрабатываем возможные команды пользователя
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "list") {
            printConfig();
        }
        else if (cmd.startsWith("poll ")) {
            forcePoll(cmd);
        }
        else {
            Serial.println(F("Only 'list' and 'poll' commands available in normal mode"));
            Serial.println(F("Enter config mode for full configuration"));
        }
    }
}

// Принудительный опрос устройства
void forcePoll(String cmd) {
    int index;
    if (sscanf(cmd.c_str(), "poll %d", &index) == 1) {
        if (index >= 0 && index < numDevices) {
            pollDevice(index);
        }
        else {
            Serial.println(F("Invalid device index"));
        }
    }
    else {
        Serial.println(F("Invalid format. Use: poll <index>"));
    }
}

// Опрос устройства
void pollDevice(uint8_t devIndex) {
    DeviceConfig& cfg = devices[devIndex];
    node.begin(cfg.deviceID, Serial1);

    Serial.print(F("Polling device "));
    Serial.print(cfg.deviceID);
    Serial.print(F(" (Type: "));
    Serial.print(cfg.deviceType);
    Serial.print(F(") Regs "));
    Serial.print(cfg.startReg);
    Serial.print(F("-"));
    Serial.print(cfg.startReg + cfg.numRegs - 1);
    Serial.print(F("... "));

    uint8_t retries = 3;
    bool success = false;

    while (retries-- > 0) {
        uint8_t result = node.readHoldingRegisters(cfg.startReg, cfg.numRegs);
        if (result == node.ku8MBSuccess) {
            success = true;
            break;
        }
        delay(500); // Задержка между попытками
    }

    if (success) {
        Serial.println(F("Success"));
        for (uint16_t i = 0; i < cfg.numRegs; i++) {
            uint16_t value = node.getResponseBuffer(i);
            Serial.print(F("Reg "));
            Serial.print(cfg.startReg + i);
            Serial.print(F(": "));
            Serial.println(value, HEX);
            processDeviceData(cfg.deviceType, cfg.startReg + i, value);
        }
    }
    else {
        Serial.println(F("Modbus error"));
    }
}
