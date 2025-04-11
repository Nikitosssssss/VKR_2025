#include <heltec.h>
#include <ModbusMaster.h>
#include <LoRaWan_APP.h>
#include <EEPROM.h>

// Настройки LoRaWAN
#define LORAWAN_CLASS CLASS_A  // или CLASS_C
#define LORAWAN_REGION RU868
#define LORAWAN_ADR_ON true
#define LORAWAN_NETMODE OTAA  // или ABP

// Настройки RS485
#define RS485_BAUDRATE 9600
#define RS485_DE_PIN 25
#define RS485_RE_PIN 26

ModbusMaster node;
HardwareSerial SerialRS485(1);  // UART1 для RS485

// Структура инструкции Modbus
struct Instruction {
    uint8_t nodeID;
    uint8_t functionCode;
    uint16_t startAddr;
    uint16_t regCount;
    uint32_t pollInterval;
    uint32_t lastPollTime;
    uint8_t data[128];
    uint8_t dataLen;
};

#define MAX_INSTRUCTIONS 32
Instruction instructions[MAX_INSTRUCTIONS];
uint8_t instructionCount = 0;

// Режимы работы
enum Mode { TRANSPARENT, PACKET };   
Mode currentMode = TRANSPARENT;

// AT-команды
String atCommand = "";

void setup() {
    // Инициализация дисплея и Serial
    Heltec.begin(true, false, true);
    Serial.begin(115200);
    SerialRS485.begin(RS485_BAUDRATE, SERIAL_8N1, 16, 17);  // RX, TX

    // Настройка RS485 (DE/RE)
    pinMode(RS485_DE_PIN, OUTPUT);
    pinMode(RS485_RE_PIN, OUTPUT);
    setRS485Receive();

    // Инициализация LoRaWAN
    LoRaWAN.init();
    LoRaWAN.setClass(LORAWAN_CLASS);
    LoRaWAN.setRegion(LORAWAN_REGION);
    LoRaWAN.setAdr(LORAWAN_ADR_ON);

    // Загрузка инструкций из EEPROM
    loadInstructions();

    // Тестовый вывод
    Serial.println("Heltec RS485-LoRaWAN Gateway Ready");
}

void loop() {
    // Обработка LoRaWAN
    LoRaWAN.loop();

    // Опрос Modbus-устройств по расписанию
    pollModbusDevices();

    // Обработка AT-команд
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            processATCommand(atCommand);
            atCommand = "";
        }
        else {
            atCommand += c;
        }
    }
}

// Опрос Modbus-устройств
void pollModbusDevices() {
    uint32_t currentTime = millis();
    for (int i = 0; i < instructionCount; i++) {
        if (currentTime - instructions[i].lastPollTime >= instructions[i].pollInterval) {
            queryModbusDevice(instructions[i]);
            instructions[i].lastPollTime = currentTime;
        }
    }
}

// Запрос к Modbus-устройству
void queryModbusDevice(Instruction& inst) {
    node.begin(inst.nodeID, SerialRS485);
    setRS485Transmit();

    uint8_t result;
    switch (inst.functionCode) {
    case 0x03:  // Read Holding Registers
        result = node.readHoldingRegisters(inst.startAddr, inst.regCount);
        break;
    case 0x04:  // Read Input Registers
        result = node.readInputRegisters(inst.startAddr, inst.regCount);
        break;
    default:
        return;
    }

    setRS485Receive();

    if (result == node.ku8MBSuccess) {
        inst.dataLen = 0;
        for (int j = 0; j < inst.regCount; j++) {
            uint16_t val = node.getResponseBuffer(j);
            inst.data[inst.dataLen++] = val >> 8;
            inst.data[inst.dataLen++] = val & 0xFF;
        }
        sendLoRaData(inst);
    }
}

// Отправка данных через LoRa
void sendLoRaData(Instruction& inst) {
    uint8_t payload[128];
    uint8_t payloadLen = 0;

    if (currentMode == TRANSPARENT) {
        memcpy(payload, inst.data, inst.dataLen);
        payloadLen = inst.dataLen;
    }
    else {  // PACKET mode
        payload[0] = inst.nodeID;
        payload[1] = inst.functionCode;
        payload[2] = inst.startAddr >> 8;
        payload[3] = inst.startAddr & 0xFF;
        payload[4] = inst.regCount >> 8;
        payload[5] = inst.regCount & 0xFF;
        memcpy(payload + 6, inst.data, inst.dataLen);
        payloadLen = 6 + inst.dataLen;
    }

    LoRaWAN.send(payloadLen, payload, LORAWAN_DEFAULT_CONFIRMED_MSG_RETRY, LORAWAN_DEFAULT_SF);
}

// Обработка AT-команд
void processATCommand(String cmd) {
    if (cmd.startsWith("AT+LIST")) {
        listInstructions();
    }
    else if (cmd.startsWith("AT+MODE=")) {
        currentMode = (cmd.substring(8).toInt() == 0) ? TRANSPARENT : PACKET;
        Serial.println("OK");
    }
    else {
        Serial.println("ERROR");
    }
}

// Вывод списка инструкций
void listInstructions() {
    for (int i = 0; i < instructionCount; i++) {
        Serial.print("Instruction ");
        Serial.print(i);
        Serial.print(": NodeID=");
        Serial.print(instructions[i].nodeID);
        Serial.print(", FC=");
        Serial.print(instructions[i].functionCode, HEX);
        Serial.print(", Addr=");
        Serial.print(instructions[i].startAddr);
        Serial.print(", Count=");
        Serial.print(instructions[i].regCount);
        Serial.print(", Interval=");
        Serial.println(instructions[i].pollInterval);
    }
}

// Загрузка инструкций из EEPROM
void loadInstructions() {
    EEPROM.begin(4096);
    instructionCount = EEPROM.read(0);
    for (int i = 0; i < instructionCount; i++) {
        EEPROM.get(1 + i * sizeof(Instruction), instructions[i]);
    }
}

// Управление RS485 (передача/приём)
void setRS485Transmit() {
    digitalWrite(RS485_DE_PIN, HIGH);
    digitalWrite(RS485_RE_PIN, HIGH);
}

void setRS485Receive() {
    digitalWrite(RS485_DE_PIN, LOW);
    digitalWrite(RS485_RE_PIN, LOW);
}