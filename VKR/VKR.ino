#include <SPI.h>           // ���������� ��� ������ � SPI
#include <RH_RF95.h>       // ���������� ��� ������ � RFM95 LoRa
#include <SoftwareSerial.h> // ���������� ��� ������ � ������ ��������
#include <EEPROM.h>        // ���������� ��� ������ � ����������������� �������

// ��������� RFM95 LoRa
#define RFM95_CS 10        // ��� ChipSelect ��� RFM95
#define RFM95_RST 9        // ��� ������ ��� RFM95
#define RFM95_INT 2        // ��� ���������� ��� RFM95

// ��������� RS485
#define RS485_RX_PIN 3     // ������� Rx ��� RS485
#define RS485_TX_PIN 4     // ������� Tx ��� RS485
#define RS485_BAUD_RATE 9600 // �������� �������� ������ �� RS485

// ��������� ������ Modbus
#define MODBUS_DEVICE_ADDR 1  // ����� Modbus-����������
#define REG_START_ADDRESS 0   // ������ ������ ���������
#define REG_COUNT 2           // ������� ��������� ������

// ������� ��������� LoRa RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ������� ����������� �������� ���� ��� RS485
SoftwareSerial rs485Serial(RS485_RX_PIN, RS485_TX_PIN);

// ����-��� ��� Modbus-�������
byte modbusRequest[] = {
    MODBUS_DEVICE_ADDR, // ����� ����������
    0x03,               // ������� ������ ��������� (Function Code 0x03)
    0x00, 0x00,         // ��������� ����� �������� (high-byte, low-byte)
    0x00, REG_COUNT     // ���������� ��������� (high-byte, low-byte)
};

// ��������� ���������� ��� RS485
struct Instruction {
    uint8_t deviceAddress;    // ����� ����������
    uint8_t functionCode;     // ������� Modbus
    uint16_t registerStart;   // ��������� ����� ��������
    uint16_t registerCount;   // ���������� ���������
    uint32_t pollInterval;    // �������� ������ (��)
    uint32_t lastPollTime;    // ����� ���������� ������
};

// ������������ ����� ����������
#define MAX_INSTRUCTIONS 32
Instruction instructions[MAX_INSTRUCTIONS];
uint8_t instructionCount = 0;

// �������� ������ Modbus-��������� (ms)
unsigned long pollInterval = 5000; // �������� ������ (������ 5 ������)
unsigned long lastPollTime = 0;    // ������ ����� ���������� ������

// ������������ ����� ������ ������
#define BUFFER_SIZE 128

// ����� ��� �������� ������
byte buffer[BUFFER_SIZE];

// ������ ������
enum Mode {TRANSPARENT,PACKET};
Mode currentMode = Mode::TRANSPARENT;


// ��� �������� ������
enum TransmissionType { PLAIN, ENCAPSULATED };
TransmissionType transmissionType = TransmissionType::PLAIN;

// ��������
unsigned long lastCheckTime = 0;

// ��������������� �������
void readEEPROM() {
    instructionCount = EEPROM.read(0);
    for (int i = 0; i < instructionCount; i++) {
        EEPROM.get(i * sizeof(Instruction), instructions[i]);
    }
}

void writeEEPROM() {
    EEPROM.put(0, instructionCount);
    for (int i = 0; i < instructionCount; i++) {
        EEPROM.put(i * sizeof(Instruction), instructions[i]);
    }
}

// ����������� ����������
void showInstruction(Instruction instr) {
    Serial.print("Instruction: Device addr=");
    Serial.print(instr.deviceAddress);
    Serial.print(", Func code=");
    Serial.print(instr.functionCode, HEX);
    Serial.print(", Reg start=");
    Serial.print(instr.registerStart);
    Serial.print(", Reg count=");
    Serial.print(instr.registerCount);
    Serial.print(", Poll interval=");
    Serial.print(instr.pollInterval);
    Serial.println(" ms");
}

// ����������� ���� ����������
void listAllInstructions() {
    Serial.println("Stored Instructions:");
    for (int i = 0; i < instructionCount; i++) {
        showInstruction(instructions[i]);
    }
}

// �������������
void setup() {
    // ������������� ����������������� �����
    Serial.begin(115200);

    // ������������� RS485
    rs485Serial.begin(RS485_BAUD_RATE);

    // ������������� RFM95
    while (!rf95.init()) {
        Serial.println("RFM95 initialization failed");
        delay(1000);
    }
    Serial.println("RFM95 initialized");

    // ������������ LoRa-������
    rf95.setFrequency(868.0); // ������� LoRa 868 ��� (������)
    rf95.setTxPower(23);      // �������� �������� (�������� 23 ���)

    // ������������� ��������� ����������
    readEEPROM();

    // �������������� ���������
    Serial.println("Pro Mini LoRa + RS485 gateway ready!");
}

void loop() {
    unsigned long currentMillis = millis();

    // ������������ ���������� ���������� Modbus
    if (currentMillis - lastPollTime >= pollInterval) {
        for (int i = 0; i < instructionCount; i++) {
            if (currentMillis - instructions[i].lastPollTime >= instructions[i].pollInterval) {
                queryModbusDevice(instructions[i]);
                instructions[i].lastPollTime = currentMillis;
            }
        }
        lastPollTime = currentMillis;
    }

    // ��������� ��������� ������ �� LoRa
    checkIncomingLoRaMessages();

    // �������� AT-������
    handleATCommands();

    // ������ �������� ��� ������������
    delay(10);
}

// ����������� ���������� Modbus � ���������� ������
void queryModbusDevice(Instruction instr) {
    // ���������� ������
    byte request[8] = {
        instr.deviceAddress, // ����� ����������
        instr.functionCode,  // ������� Modbus
        highByte(instr.registerStart), lowByte(instr.registerStart), // ��������� ����� ��������
        highByte(instr.registerCount), lowByte(instr.registerCount) // ���������� ���������
    };

    // ����������� ������ �� RS485
    rs485Serial.write(request, sizeof(request));

    // ���� ������ �� ����������
    delay(100);

    // ��������� �����
    int bytesRead = rs485Serial.readBytes(buffer, BUFFER_SIZE);

    // ���������� ���������� ������ ����� LoRa
    if (bytesRead > 0) {
        sendLoRaMessage(buffer, bytesRead);
    }
}

// ���������� ������ ����� LoRa
void sendLoRaMessage(byte* data, int length) {
    if (!rf95.send(data, length)) {
        Serial.println("Failed to send LoRa message");
    }
    else {
        Serial.println("LoRa message sent successfully");
    }

    // ���� ���������� ��������
    rf95.waitPacketSent();
}

// ��������� ����������� ��������� LoRa
void checkIncomingLoRaMessages() {
    if (rf95.available()) {
        // ����� ������
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            Serial.print("Received LoRa message: ");
            Serial.write(buf, len);
            Serial.println();

            // �������� ������ ����������� RS485
            respondToRS485(buf, len);
        }
    }
}

// ���������� ������ ����������� RS485
void respondToRS485(byte* data, int length) {
    rs485Serial.write(data, length);
}

// ��������� AT-������
void handleATCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("AT+SHOW")) {
            listAllInstructions();
        }
        else if (input.startsWith("AT+ADD")) {
            parseAddInstruction(input);
        }
        else if (input.startsWith("AT+SET_MODE")) {
            parseSetMode(input);
        }
    }
}

// ������ ������� ���������� ����������
void parseAddInstruction(String input) {
    int index = input.indexOf(' ');
    if (index != -1) {
        String args = input.substring(index + 1);
        int devAddr = args.toInt();
        args.remove(0, strlen(String(devAddr).c_str()));
        args.trim();
        int funcCode = args.toInt();
        args.remove(0, strlen(String(funcCode).c_str()));
        args.trim();
        int regStart = args.toInt();
        args.remove(0, strlen(String(regStart).c_str()));
        args.trim();
        int regCount = args.toInt();
        args.remove(0, strlen(String(regCount).c_str()));
        args.trim();
        int pollInt = args.toInt();

        Instruction newInstr = {
            devAddr, funcCode, regStart, regCount, pollInt, 0
        };

        if (instructionCount < MAX_INSTRUCTIONS) {
            memcpy(&instructions[instructionCount], &newInstr, sizeof(Instruction)); // �������� ���������
            instructionCount++; // ����������� ������ ����� ����������
            writeEEPROM();
            Serial.println("Instruction added");
        }
        else {
            Serial.println("No more room for instructions");
        }
    }
}

// ������ ������� ����� ������
void parseSetMode(String input) {
    int index = input.indexOf(' ');
    if (index != -1) {
        String arg = input.substring(index + 1);
        if (arg == "PLAIN") {
            transmissionType = TransmissionType::PLAIN;
        }
        else if (arg == "ENCAPSULATED") {
            transmissionType = TransmissionType::ENCAPSULATED;
        }
        Serial.println("Mode changed");
    }
}