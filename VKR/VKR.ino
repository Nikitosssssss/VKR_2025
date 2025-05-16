#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#include <SoftwareSerial.h>             //   Подключаем библиотеку для работы с программной шиной UART.
#include <iarduino_Modbus.h>            //   Подключаем библиотеку для работы по протоколу Modbus.

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
//TNN - The Node Network. Узел в сети, то есть любое устройство, которое может получать и отправлять данные

//Кроме этого фрагмента FILLMEIN нигде не используется. Возможно нужна была для регрессионного теста
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

//big - endian(aka msb) - формат записи числа, когда старший байт записывается по наименьшему адресу, то есть число 0х12345678 будет записано как 0х12 0х34 0х56 0х78
// LoRaWAN NwkSKey, network session key - network session key, сетевой ключ доступа
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x23 };

// LoRaWAN AppSKey, application session key - ключ доступа приложения
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x23 };

// LoRaWAN end-device address (DevAddr) - адрес устройства
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x22222223; // <-- Change this address for every node!




// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}
//так как регистрация в сети проходит через авторизацию, эти вызовы не используются. Если бы авторизация происходила по запросу динамически, то были бы нужны

static uint8_t mydata[40];
const unsigned long ALARM_DEBOUNCE = 300000; // Задержка между тревогами (300 сек)
unsigned long lastAlarmTime = 0;
static osjob_t sendjob;
unsigned long previousMillis = 0;  // Время последнего переключения
const long interval = 15000;       // Интервал опроса (мс)
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 90;        //раз в 90 сек будет отправлять данные на БС

// Pin mapping
// ===== Пин светодиода =====
// Установка частоты RX2 (в Гц)  
const uint32_t RX2_FREQ = 869100000; // 869.10 МГц  

// Установка Data Rate RX2 (DR0 для RU868)  
const uint8_t RX2_DR = DR_SF12;
// Определяем пины для SoftwareSerial
#define RX_PIN 4
#define TX_PIN 5
//SoftwareSerial     rs485(RX_PIN, TX_PIN);     //   Создаём объект для работы с программной шиной UART-RS485 указывая выводы RX, TX.
ModbusClient       modbus(Serial, 2);           //   Создаём объект для работы по протоколу Modbus указывая объект программной шины UART-RS485 и вывод DE конвертера UART-RS485.
//
#define LED_PIN 13                              // Светодиод
#define ALARM_PIN A0                            // Аналоговый вход для тревоги
#define ALARM_THRESHOLD 500                     // Порог срабатывания (0-1023)
bool ledState = false;
bool alarmTriggered = false;


// Пин-маппинг RFM95
/*const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,//,3//
    .rst = LMIC_UNUSED_PIN,
    .dio = {9, 8, 7}
};*/

const lmic_pinmap lmic_pins = {
    .nss = 10,                      //пин микросхемы. Проверяет, готово ли к работе устройство
    .rxtx = LMIC_UNUSED_PIN,        //пин прехода с TX на RX, переключение режима
    .rst = 9,                       //пин reset
    .dio = {2, 3, LMIC_UNUSED_PIN}, // DIO0, DIO1, DIO2
};
// ===== Прототипы функций =====
void sendStatus(osjob_t* j);
void checkAlarm();
void sendAlarm();

// ===== Отправка статуса =====
void sendStatus(osjob_t* j) {
    uint8_t txData[1] = { ledState ? 0x01 : 0x00 };     //отправка текущего состояния светодиода, включен или выключен
    LMIC_setTxData2(1, txData, sizeof(txData), 0);
    Serial.print("Sent status: ");
    Serial.println(ledState ? "ON" : "OFF");

    // Планируем следующую отправку
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), sendStatus);
}
// ===== Проверка тревоги =====
//Похоже на устранение дребезга контактов. Либо это проверка на устойчивое состояние тревоги(полсекунды)
void checkAlarm() {
    int sensorValue = analogRead(ALARM_PIN);
    if (sensorValue > ALARM_THRESHOLD) {
        if (!alarmTriggered && (millis() - lastAlarmTime > ALARM_DEBOUNCE)) {
            alarmTriggered = true;
            lastAlarmTime = millis();
            sendAlarm();
        }
    }
    else {
        alarmTriggered = false;
    }
}

// ===== Отправка тревоги =====
void sendAlarm() {
    uint8_t alarmData[2] = { 0xFF, 0x01 }; // 0xFF - маркер тревоги, 0x01 - тип (замыкание)
    LMIC_setTxData2(1, alarmData, sizeof(alarmData), 0);
    Serial.println("ALARM SENT: Contact detected!");
}

// Буфер для входящих данных
uint8_t receivedData[64];                   //для полученных данных
uint8_t slaves[8] = { 0,0,0,0,0,0,0,0 };    //подчиненные устройства
uint8_t types[8] = { 0,0,0,0,0,0,0,0 }; 
uint8_t dataLength = 0;
bool newDataReceived = false;

//Параметры:
// ev_t ev - событие, произошедшее в системе
void onEvent(ev_t ev) {
    Serial.print(os_getTime());     //выводит текущее время
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:                       //превышено время выполнения процесса
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));   //обноружен новый маяк(надо понять, что такое маяк)
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));  //сигнал от маяка потерян
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED")); //продалжается отслеживание сигнала маяка
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));        //попытка устройства подключиться к сети
        break;
    case EV_JOINED:                             
        Serial.println(F("EV_JOINED"));         //успешно подключились
        break;  
        /*
         This event is defined but not used in the code. No
         point in wasting codespace on it.

         case EV_RFU1:
             Serial.println(F("EV_RFU1"));
             break;
        */
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));    //не удалось подключиться
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));  //повторная попытка подключения не удалась
        break;
    case EV_TXCOMPLETE:                         //завершение передачи данных
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        u1_t dLen = LMIC.dataLen;       //LMIC - принятые данные
        Serial.println(dLen);
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.println(F("Received "));         //выводит длину полезной нагрузки в полученном собщении
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        //зачем-то дублируется вывод
        if (LMIC.txrxFlags & TXRX_ACK)              //LMIC.txrxFlags - набор флагов, TXRX_ACK - было получено подтверждение успешной доставки
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.print(F("Received "));           //выводит длину полезной нагрузки в полученном собщении
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
            Serial.print("Port: ");                 //выводит порт
            Serial.println(LMIC.frame[(LMIC.dataBeg - 1)]);
            Serial.print("Data: ");                 //выводит данные, полученные по этому порту
            // char rec_data = (LMIC.frame + LMIC.dataBeg);
            for (int i = 0; i < LMIC.dataLen; ++i) {
                if (i != 0)
                    Serial.print(",");
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
            }
            Serial.println();
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);    //планирует следующую передачу данных

        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));         //потеряна синхронизация времени с сетью(на устройстве одно время, в сети другое)
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));              //произошел сброс системы(либо сбой, либо перезагрузка)
        break;
    case EV_RXCOMPLETE:                             //были получены данные через окно приема RX
        // data received in ping slot
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println(F("Received ack"));
        }
        if (LMIC.dataLen) {
            // Данные получены
            dataLength = LMIC.dataLen;  
            //LMIC.frame - кадр, заголовок+полезная нагрузка
            //LMIC.dataBeg - начало полезных данных. Смещение, до которого идет заголовок, а после которого полезные данные
            memcpy(receivedData, LMIC.frame + LMIC.dataBeg, dataLength);    //в массив сохраняются полученные данные
            newDataReceived = true;                 //флажок, были ли получены новые данные

            Serial.print(F("Received "));           //выводит данные на консоль
            Serial.print(dataLength);
            Serial.print(F(" bytes of payload: "));
            for (int i = 0; i < dataLength; i++) {
                Serial.print(receivedData[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));          //потеря соединения с сетью
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));         //соединение с сетью восстановлено
        break;
        /*
         This event is defined but not used in the code. No
         point in wasting codespace on it.

         case EV_SCAN_FOUND:
            Serial.println(F("EV_SCAN_FOUND"));
            break;
        */
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));            //начало передачи данных
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));         //отмена передачи данных
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        Serial.println(F("EX_RXSTART"));            //начало приема данных
        break;
    case EV_JOIN_TXCOMPLETE:                        // завершение отправки специального пакета, который инициирует процедуру присоединения устройства к сети LoRaWAN
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));         //запрос на присоединение был отправлен, но ответ не пришел
        break;
    default:
        Serial.print(F("Unknown event: "));         //неизвестное событие
        Serial.println((unsigned)ev);
        break;
    }
}

//Функция опрашивает подключенные Modbus устройства и сохраняет полученные данные
void pollData() {
    int i = 0;
    for (i = 0; i < 40; i++) { mydata[i] = 0xff; } //массив заполняется нулями, так он инициализируется или очищаяется 
    i = 0;
    int32_t regDI = modbus.holdingRegisterRead(1, 144); // Читаем из модуля с адресом 1, значение регистра "Holding Register" с адресом 144.
    uint16_t di = regDI;
    if (di >= 0) {
        mydata[0] = 1;      //скорее всего номер датчика
        mydata[1] = 1;
        mydata[2] = (uint8_t)(di >> 8);     // Старший байт
        mydata[3] = (uint8_t)(di);          // Младший байт
    }                                       // Сохраняем прочитанное значение в переменную i.
    else {
        mydata[2] = 0xff;       // Старший байт
        mydata[3] = 0xff;
    }    // Младший байт}        // Выводим сообщение о ошибке чтения.
    i = 0;//Опрос AI8 с адресом 2
    //если вощникла ошибка
    if (!modbus.requestFrom(19, HOLDING_REGISTERS, 0, 8))   //19 - адрес датчика
    {
        while (i < 8) {                                     //опрос 8-ми регистров HOLDING_REGISTERS
            uint16_t reg = -1;                              //имитирует код ошибки            
            //mydata[10]=2;
            mydata[4] = 2;      //номер датчика
            mydata[5] = 2;
            mydata[i * 2 + 6] = (uint8_t)(reg >> 8);    // Старший байт
            mydata[i * 2 + 1 + 6] = (uint8_t)(reg);     // Младший байт
            i++;
        }
        //Serial.print("Err");
    } //   Функция modbus.requestFrom() возвращает количество прочитанных значений, или 0 при неудаче.
    else {
        //если ошибки  нет, данные записываются
        while (modbus.available()) {
            uint16_t reg = 0;
            mydata[4] = 2;          //почему-то адрес 19, а здесь записывается 2
            mydata[5] = 2;
            reg = modbus.read();
            //mydata[10]=1;
            //записываются байты 6-21
            mydata[i * 2 + 6] = (uint8_t)(reg >> 8);    // Старший байт
            mydata[i * 2 + 1 + 6] = (uint8_t)(reg);     // Младший байт
            //Serial.print((String)" " + reg); 
            i++;
        }
    } //   Читаем полученные значения функцией modbus.read() проверяя их наличие функцией modbus.available().
    i = 0;//Опрос DS8 с адресом 3
    //то же самое для второго датчика
    if (!modbus.requestFrom(3, HOLDING_REGISTERS, 0, 8)) {
        while (i < 8) {
            uint16_t reg = -1;//random(0, 6000);
            //mydata[10]=2;
            mydata[22] = 3;
            mydata[23] = 3;
            mydata[i * 2 + 6 + 18] = (uint8_t)(reg >> 8);    // Старший байт
            mydata[i * 2 + 1 + 6 + 18] = (uint8_t)(reg);     // Младший байт
            i++;
        }
        //Serial.print("Err");
    } //   Функция modbus.requestFrom() возвращает количество прочитанных значений, или 0 при неудаче.
    else {
        while (modbus.available()) {
            uint16_t reg = 0;
            mydata[22] = 3;
            mydata[23] = 3;
            reg = modbus.read();
            //mydata[10]=1;
            mydata[i * 2 + 6 + 18] = (uint8_t)(reg >> 8);    // Старший байт
            mydata[i * 2 + 1 + 6 + 18] = (uint8_t)(reg);     // Младший байт
            //Serial.print((String)" " + reg); 
            i++;
        }
    } //   Читаем полученные значения функцией modbus.read() проверяя их наличие функцией modbus.available().
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {                    //проверка, идет ли сейчас прием или передача данных
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else {
        // Prepare upstream data transmission at the next possible time.
        //mydata[0]=1;mydata[1]=2;mydata[2]=3;mydata[3]=4;
        //pollData();
        int i = 0;
        for (i = 0; i < 40; i++) { mydata[i] = 0xff; }  //очистка массива
        mydata[0] = 0x10;                               //скорее всего заголовок
        //1 - порт, mydata - массив данных, sizeof(mydata) - 1 размер данных, 0 - опции, дополнительные настройки
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);      
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    //    pinMode(13, OUTPUT);

    delay(100);     // per sample code on RF_95 test
    //Serial.println(F("Starting"));
    pinMode(LED_PIN, OUTPUT);       //настройка порта светодиода на выход, чтобы он мог загораться
    pinMode(ALARM_PIN, INPUT);      //
    digitalWrite(LED_PIN, LOW);     //включение светодиода
#ifdef VCC_ENABLE                   //шина питания
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);    //подключается питание к компонентам
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(9600);     //9600 бит в сек - скорость передачи данных для UART
    //rs485.begin(9600); // while(!rs485 );                                         //   Инициируем работу с программной шиной UATR-RS485 указав её скорость.
    modbus.begin();                             //   Инициируем работу по протоколу Modbus.
    modbus.setTimeout(200);                     //   Указываем максимальное время ожидания ответа по протоколу Modbus.
    modbus.setDelay(40);                        //   Указываем минимальный интервал между отправляемыми сообщениями по протоколу Modbus.
    modbus.setTypeMB(MODBUS_RTU);               //   Указываем тип протокола Modbus: MODBUS_RTU (по умолчанию), или MODBUS_ASCII.

    //pollData();

        // LMIC init
    os_init();          //инициализация стека LoRaWAN
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM      //постоянная флэш память
// On AVR, these values are stored in flash and only copied to RAM
// once. Copy them to a temporary buffer here, LMIC_setSession will
// copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(22, DEVADDR, nwkskey, appskey);         //установка ключей для авторизации в сети
#else
// If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(22, DEVADDR, NWKSKEY, APPSKEY);         //если этой памяти нет, то просто авторизуемся
#endif

    //     #if defined(CFG_eu868)
    //настройка каналов, 1 на FSK модуляцию для большей гибкости и совместимости с устройствами
    //DR_RANGE_MAP(DR_SF12, DR_SF7) - диапазон допустимых коэффициентов разрешения, BAND_CENTI - лимит на частоту отправки данных, 1% от общего времени пользования канала
    LMIC_setupChannel(0, 864100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 864300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 864500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 864700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 864900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 869100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 868900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 869000000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(8, 869300000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
    //настройки скорости LoRa
    LMIC_setLinkCheckMode(0);       //проверка качества связи
    LMIC_setAdrMode(0);             //автоматическое управление мощностью передачи

    LMIC_setDrTxpow(DR_SF7, 14);    //мощность передачи и коэффициент распространения для канала ответов
    LMIC.dn2Freq = RX2_FREQ;        //настройка RX2
    LMIC.dn2Dr = RX2_DR;
    //LMICeu868_setRx1Params()
    // Start job
    do_send(&sendjob);              //отправка пакета

}
void loop() {
    unsigned long now;
    now = millis();         //время с момента старта программы(в мс) 
    if (now - previousMillis >= interval) {         //проверка временного интервала
        previousMillis = now;      // Обновляем время опроса
        //pollData();
        /*Serial.print("HoldingRegisters = {");                                                       //
            if (!modbus.requestFrom(1, HOLDING_REGISTERS, 144, 1)) { Serial.print("Err"); } //   Функция modbus.requestFrom() возвращает количество прочитанных значений, или 0 при неудаче.
            else { while (modbus.available()) { Serial.print((String)" " + modbus.read()); } } //   Читаем полученные значения функцией modbus.read() проверяя их наличие функцией modbus.available().
            Serial.println(" }");                                                       //*/
    }

    //мигание светодиода, примерно 2 раза в секунду меняется состояние
    if ((now & 512) != 0) {
        digitalWrite(13, HIGH);
    }
    else {
        digitalWrite(13, LOW);
    }

    os_runloop_once();      //обслуживание задач, поставленных в очередь
    checkAlarm(); // Постоянная проверка тревоги

}
