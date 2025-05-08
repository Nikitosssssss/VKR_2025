#include <SPI.h>           // Библиотека для работы с SPI
#include <RH_RF95.h>       // Библиотека для работы с RFM95 LoRa
#include <SoftwareSerial.h> // Библиотека для работы с вторым сериалом
#include <EEPROM.h>        // Библиотека для работы с энергонезависимой памятью

// Настройки RFM95 LoRa
#define RFM95_CS 10        // Пин ChipSelect для RFM95
#define RFM95_RST 9        // Пин сброса для RFM95
#define RFM95_INT 2        // Пин прерывания для RFM95

// Настройки RS485
#define RS485_RX_PIN 3     // Контакт Rx для RS485
#define RS485_TX_PIN 4     // Контакт Tx для RS485
#define RS485_BAUD_RATE 9600 // Скорость передачи данных по RS485

// Настройки опроса Modbus
#define MODBUS_DEVICE_ADDR 1  // Адрес Modbus-устройства
#define REG_START_ADDRESS 0   // Начало чтения регистров
#define REG_COUNT 2           // Сколько регистров читать

// Создаем экземпляр LoRa RF95
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Создаем виртуальный серийный порт для RS485
SoftwareSerial rs485Serial(RS485_RX_PIN, RS485_TX_PIN);

// Байт-код для Modbus-запроса
byte modbusRequest[] = {
    MODBUS_DEVICE_ADDR, // Адрес устройства
    0x03,               // Команда чтения регистров (Function Code 0x03)
    0x00, 0x00,         // Начальный адрес регистра (high-byte, low-byte)
    0x00, REG_COUNT     // Количество регистров (high-byte, low-byte)
};

// Структура инструкции для RS485
struct Instruction {
    uint8_t deviceAddress;    // Адрес устройства
    uint8_t functionCode;     // Функция Modbus
    uint16_t registerStart;   // Начальный адрес регистра
    uint16_t registerCount;   // Количество регистров
    uint32_t pollInterval;    // Интервал опроса (мс)
    uint32_t lastPollTime;    // Время последнего опроса
};

// Максимальное число инструкций
#define MAX_INSTRUCTIONS 32
Instruction instructions[MAX_INSTRUCTIONS];
uint8_t instructionCount = 0;

// Интервал опроса Modbus-устройств (ms)
unsigned long pollInterval = 5000; // Интервал опроса (каждый 5 секунд)
unsigned long lastPollTime = 0;    // Хранит время последнего опроса

// Максимальная длина буфера данных
#define BUFFER_SIZE 128

// Буфер для хранения данных
byte buffer[BUFFER_SIZE];

// Режимы работы
enum Mode {TRANSPARENT,PACKET};
Mode currentMode = Mode::TRANSPARENT;


// Тип передачи данных
enum TransmissionType { PLAIN, ENCAPSULATED };
TransmissionType transmissionType = TransmissionType::PLAIN;

// Тайминги
unsigned long lastCheckTime = 0;

// Вспомогательные функции
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

// Отображение инструкции
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

// Отображение всех инструкций
void listAllInstructions() {
    Serial.println("Stored Instructions:");
    for (int i = 0; i < instructionCount; i++) {
        showInstruction(instructions[i]);
    }
}

// Инициализация
void setup() {
    // Инициализация последовательного порта
    Serial.begin(115200);

    // Инициализация RS485
    rs485Serial.begin(RS485_BAUD_RATE);

    // Инициализация RFM95
    while (!rf95.init()) {
        Serial.println("RFM95 initialization failed");
        delay(1000);
    }
    Serial.println("RFM95 initialized");

    // Конфигурация LoRa-модуля
    rf95.setFrequency(868.0); // Частота LoRa 868 МГц (Россия)
    rf95.setTxPower(23);      // Мощность передачи (максимум 23 дБм)

    // Инициализация хранилища инструкций
    readEEPROM();

    // Приветственное сообщение
    Serial.println("Pro Mini LoRa + RS485 gateway ready!");
}

void loop() {
    unsigned long currentMillis = millis();

    // Периодически опрашиваем устройства Modbus
    if (currentMillis - lastPollTime >= pollInterval) {
        for (int i = 0; i < instructionCount; i++) {
            if (currentMillis - instructions[i].lastPollTime >= instructions[i].pollInterval) {
                queryModbusDevice(instructions[i]);
                instructions[i].lastPollTime = currentMillis;
            }
        }
        lastPollTime = currentMillis;
    }

    // Проверяем получение данных по LoRa
    checkIncomingLoRaMessages();

    // Проверка AT-команд
    handleATCommands();

    // Пустая задержка для стабильности
    delay(10);
}

// Запрашивает устройство Modbus и возвращает данные
void queryModbusDevice(Instruction instr) {
    // Составляем запрос
    byte request[8] = {
        instr.deviceAddress, // Адрес устройства
        instr.functionCode,  // Функция Modbus
        highByte(instr.registerStart), lowByte(instr.registerStart), // Начальный адрес регистра
        highByte(instr.registerCount), lowByte(instr.registerCount) // Количество регистров
    };

    // Запрашиваем данные по RS485
    rs485Serial.write(request, sizeof(request));

    // Ждем ответа от устройства
    delay(100);

    // Считываем ответ
    int bytesRead = rs485Serial.readBytes(buffer, BUFFER_SIZE);

    // Отправляем полученные данные через LoRa
    if (bytesRead > 0) {
        sendLoRaMessage(buffer, bytesRead);
    }
}

// Отправляет данные через LoRa
void sendLoRaMessage(byte* data, int length) {
    if (!rf95.send(data, length)) {
        Serial.println("Failed to send LoRa message");
    }
    else {
        Serial.println("LoRa message sent successfully");
    }

    // Ждем завершение передачи
    rf95.waitPacketSent();
}

// Проверяет поступающие сообщения LoRa
void checkIncomingLoRaMessages() {
    if (rf95.available()) {
        // Прием данных
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            Serial.print("Received LoRa message: ");
            Serial.write(buf, len);
            Serial.println();

            // Ответить данным устройствам RS485
            respondToRS485(buf, len);
        }
    }
}

// Отправляет данные устройствам RS485
void respondToRS485(byte* data, int length) {
    rs485Serial.write(data, length);
}

// Обработка AT-команд
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

// Парсим команду добавления инструкции
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
            memcpy(&instructions[instructionCount], &newInstr, sizeof(Instruction)); // Копируем структуру
            instructionCount++; // Увеличиваем индекс после присвоения
            writeEEPROM();
            Serial.println("Instruction added");
        }
        else {
            Serial.println("No more room for instructions");
        }
    }
}

// Парсим команду смены режима
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
