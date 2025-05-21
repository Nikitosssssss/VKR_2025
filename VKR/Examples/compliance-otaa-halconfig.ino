

#include <Arduino.h>

#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_lorawan_compliance.h>

#include <SPI.h>
class cEventQueue;

#define APPLICATION_VERSION ARDUINO_LMIC_VERSION_CALC(3,0,99,10)

void os_getArtEui(u1_t* buf) { memset(buf, 0, 8); buf[0] = 1; }


void os_getDevEui(u1_t* buf) { memset(buf, 0, 8); buf[0] = 1; }

void os_getDevKey(u1_t* buf) { memset(buf, 0, 16); buf[15] = 2; }

static uint8_t mydata[] = { 0xCA, 0xFE, 0xF0, 0x0D };
static osjob_t sendjob;

const unsigned TX_INTERVAL = 5; //интервал передач на сервер

// флаг для режима тестирования
bool g_fTestMode = false;

lmic_event_cb_t myEventCb;
lmic_rxmessage_cb_t myRxMessageCb;

const char* const evNames[] = { LMIC_EVENT_NAME_TABLE__INIT };

static void rtccount_begin();
static uint16_t rtccount_read();

#define NEED_USBD_LL_ConnectionState    0
#ifdef ARDUINO_ARCH_STM32
# ifdef _mcci_arduino_version
#  if _mcci_arduino_version < _mcci_arduino_version_calc(2, 5, 0, 10)
#   undef NEED_USBD_LL_ConnectionState
#   define NEED_USBD_LL_ConnectionState 1
#  endif // _mcci_arduino_version < _mcci_arduino_version_calc(2, 5, 0, 10)
# endif // def _mcci_arduino_version
#endif // def ARDUINO_ARCH_STM32

#define NEED_STM32_ClockCalibration    0
#ifdef ARDUINO_ARCH_STM32
# ifdef _mcci_arduino_version
#  if _mcci_arduino_version <= _mcci_arduino_version_calc(2, 5, 0, 10)
#   undef NEED_STM32_ClockCalibration
#   define NEED_STM32_ClockCalibration 1
#  endif // _mcci_arduino_version <= _mcci_arduino_version_calc(2, 5, 0, 10)
# endif // def _mcci_arduino_version
# define SUPPORT_STM32_ClockCalibration 1
#else
# define SUPPORT_STM32_ClockCalibration 0
#endif // def ARDUINO_ARCH_STM32

/*

Name:	myEventCb()

Function:
        lmic_event_cb_t myEventCb;

        extern "C" { void myEventCb(void *pUserData, ev_t ev); }

Description:
        This function is registered for event notifications from the LMIC
        during setup() processing. Its main job is to display events in a
        user-friendly way.

Returns:
        No explicit result.

*/

static osjobcbfn_t eventjob_cb;

class cEventQueue {
public:
    cEventQueue() {};
    ~cEventQueue() {};

    struct eventnode_t {    //это узел очереди событий
        osjob_t     job;        //задание ОС
        ev_t        event;      //событие
        const char* pMessage;   //сообщение события
        uint32_t    datum;      //дополнительные данные
        ostime_t    time;       //дальше временные метки событий отправки и получения данных и флаги
        ostime_t    txend;
        ostime_t    rxtime;
        ostime_t    globalDutyAvail;
        u4_t        nLateRx;
        ostime_t    ticksLateRx;
        u4_t        freq;
        u2_t        rtccount;
        u2_t        opmode;
        u2_t        fcntDn;
        u2_t        fcntUp;
        rxsyms_t    rxsyms;
        rps_t       rps;
        u1_t        txChnl;
        u1_t        datarate;
        u1_t        txrxFlags;
        u1_t        saveIrqFlags;
    };

    bool getEvent(eventnode_t& node) {
        if (m_head == m_tail) {     //если позиция заголовка очереди равна позиции хвоста, то очередь пустая
            return false;
        }
        node = m_queue[m_head];     //получаем узел из очереди, сохраняем его
        if (++m_head == sizeof(m_queue) / sizeof(m_queue[0])) {         //удаляем элемент из очереди, увеличивая его индекс
            m_head = 0;
        }
        return true;
    }
    //добавляет событие в очередь
    bool putEvent(ev_t event, const char* pMessage = nullptr, uint32_t datum = 0) {
        auto i = m_tail + 1;                //вычисляем новую позицию для элемента
        if (i == sizeof(m_queue) / sizeof(m_queue[0])) {        //если дошли до конца очереди, то начинаем с начала
            i = 0;
        }
        if (i != m_head) {                      //проверка, есть ли свободное место в очереди(если i = 0, то места нет)
            auto const pn = &m_queue[m_tail];
            pn->job = LMIC.osjob;               //задание ОС
            pn->time = os_getTime();            //текущее время
            pn->rtccount = rtccount_read();     //значение RTC-счетчика
            pn->txend = LMIC.txend;             //время окончания передачи
            pn->rxtime = LMIC.rxtime;           //время приема сигнала
            pn->globalDutyAvail = LMIC.globalDutyAvail;     //доступность глобального ресурса
            pn->event = event;                  //тип события
            pn->pMessage = pMessage;            //Сообщение события
            pn->datum = datum;                  //дополнительные данные
            pn->nLateRx = LMIC.radio.rxlate_count;          //Количество поздних приёмов
            pn->ticksLateRx = LMIC.radio.rxlate_ticks;      //Тики задержки приёма
            pn->freq = LMIC.freq;                // Частота
            pn->opmode = LMIC.opmode;            // Режим работы устройства
            pn->fcntDn = (u2_t)LMIC.seqnoDn;     // Последовательный номер нисходящего потока
            pn->fcntUp = (u2_t)LMIC.seqnoUp;     // Последовательный номер восходящего потока
            pn->rxsyms = LMIC.rxsyms;            // Символы приёма
            pn->rps = LMIC.rps;                  // Параметры радиосигнала
            pn->txChnl = LMIC.txChnl;            // Канал передачи
            pn->datarate = LMIC.datarate;        // Скорость передачи данных
            pn->txrxFlags = LMIC.txrxFlags;       // Флаги передачи/приёма
            pn->saveIrqFlags = LMIC.saveIrqFlags; // Сохранённые флаги прерывания

            m_tail = i;                          // Устанавливаем новую позицию хвоста
            return true;                        
        }
        else {
            return false;
        }
    }

private:
    unsigned m_head, m_tail;        //начало и конец очереди
    eventnode_t m_queue[32];        //очередь
    osjob_t m_job;                  //задание ОС
};

cEventQueue eventQueue;

#if LMIC_ENABLE_event_logging       //логирование событий
extern "C" {
    void LMICOS_logEvent(const char* pMessage);     //простое событие
    void LMICOS_logEventUint32(const char* pMessage, uint32_t datum);   //событие с бдоп данными
}

void LMICOS_logEvent(const char* pMessage)      //добавляет событие в очередь на логирование
{
    eventQueue.putEvent(ev_t(-1), pMessage);
}

void LMICOS_logEventUint32(const char* pMessage, uint32_t datum)
{
    eventQueue.putEvent(ev_t(-2), pMessage, datum);
}
#endif // LMIC_ENABLE_event_logging

hal_failure_handler_t log_assertion;

void log_assertion(const char* pMessage, uint16_t line) {       //логирование ошибок
    eventQueue.putEvent(ev_t(-3), pMessage, line);              //добавляет ошибку в журнал логирования
    eventPrintAll();                                            //выводит все события
    Serial.println(F("***HALTED BY ASSERT***"));                //программа останавливается
    while (true)
        yield();
}

bool lastWasTxStart;
uint32_t lastTxStartTime;

void myEventCb(void* pUserData, ev_t ev) {
    eventQueue.putEvent(ev);                    // Добавляет событие в очередь

    if (ev == EV_TXSTART) {                     // Начало передачи
        lastWasTxStart = true;
        lastTxStartTime = millis();             // Запоминаем текущее время старта передачи
    }
    else if (ev == EV_RXSTART) {                // Начало приёма
        lastWasTxStart = false;
    }

    if (ev == EV_JOINING) {                     // Процесс присоединения к сети
        setupForNetwork(true);
    }
    else if (ev == EV_JOINED) {                 // Устройство присоединилось к сети
        setupForNetwork(false);
    }
}

void eventPrint(cEventQueue::eventnode_t& e);        // Печатает одно событие
void printFcnts(cEventQueue::eventnode_t& e);        // Вспомогательная печать последовательности кадров
void printTxend(cEventQueue::eventnode_t& e);        // Печать завершения передачи
void printRxtime(cEventQueue::eventnode_t& e);       // Печать времени приёма
void printLateStats(cEventQueue::eventnode_t& e);    // Печать статистики задержек

void eventPrintAll(void) {
    while (eventPrintOne()) {}                       // Выводит события, пока они есть
}

bool eventPrintOne(void) {
    cEventQueue::eventnode_t e;
    if (eventQueue.getEvent(e)) {                   
        eventPrint(e);                              // Печатает первое событие из очереди
        return true;
    }
    else {
        return false;                               // Нет больше событий
    }
}

static void eventjob_cb(osjob_t* j) {               //Выводит все события
    eventPrintAll();
}

const char* getSfName(rps_t rps) {      //возвращает строку коэффициентов расширения спектра из структуры настроек радио-интерфейса
    const char* const t[] = { "FSK", "SF7", "SF8", "SF9", "SF10", "SF11", "SF12", "SFrfu" };
    return t[getSf(rps)];
}

const char* getBwName(rps_t rps) {      //выводит строку с настройками полосы пропускания
    const char* const t[] = { "BW125", "BW250", "BW500", "BWrfu" };
    return t[getBw(rps)];
}

const char* getCrName(rps_t rps) {      //выводит строку коэффициента кодирования
    const char* const t[] = { "CR 4/5", "CR 4/6", "CR 4/7", "CR 4/8" };
    return t[getCr(rps)];
}

const char* getCrcName(rps_t rps) {     //определяет наличие CRC(контроль целостности)
    return getNocrc(rps) ? "NoCrc" : "Crc";
}

void printHex2(unsigned v) {            //печатает число в 16-ричном формате
    v &= 0xff;
    if (v < 16)                         //дополняет ведущими 0
        Serial.print('0');
    Serial.print(v, HEX);
}

void printHex4(unsigned v) {            //печатает число в 16-ричном формате
    printHex2(v >> 8u);
    printHex2(v);
}

void printSpace(void) {                 //Выводит пробел
    Serial.print(' ');
}

void printFreq(u4_t freq) {             //выводит частоту канала
    Serial.print(F(": freq="));
    Serial.print(freq / 1000000);
    Serial.print('.');
    Serial.print((freq % 1000000) / 100000);
}

void printRps(rps_t rps) {              //выводит настройки радиоинтерфейса
    Serial.print(F(" rps=0x")); printHex2(rps);
    Serial.print(F(" (")); Serial.print(getSfName(rps));
    printSpace(); Serial.print(getBwName(rps));
    printSpace(); Serial.print(getCrName(rps));
    printSpace(); Serial.print(getCrcName(rps));
    Serial.print(F(" IH=")); Serial.print(unsigned(getIh(rps)));
    Serial.print(')');
}

void printOpmode(uint16_t opmode, char sep = ',') {     //выводит режим работы
    if (sep != 0)
        Serial.print(sep);
    Serial.print(F(" opmode=")); Serial.print(opmode, HEX);
}
        
void printTxend(cEventQueue::eventnode_t& e) {          //детали передачи данных
    Serial.print(F(", txend=")); Serial.print(e.txend);     //время оконччания передачи
    Serial.print(F(", avail=")); Serial.print(e.globalDutyAvail);
}

void printRxtime(cEventQueue::eventnode_t& e) {         //выводит время приема
    Serial.print(F(", rxtime=")); Serial.print(e.rxtime);
}

void printTxChnl(u1_t txChnl) {     //выводит номер активного канала
    Serial.print(F(": ch="));
    Serial.print(unsigned(txChnl));
}

void printDatarate(u1_t datarate) {     //скорость передачи данных
    Serial.print(F(", datarate=")); Serial.print(unsigned(datarate));
}

void printTxrxflags(u1_t txrxFlags) {       //флаги передачи и приема
    Serial.print(F(", txrxFlags=0x")); printHex2(txrxFlags);
    if (txrxFlags & TXRX_ACK)
        Serial.print(F("; Received ack"));
}

void printSaveIrqFlags(u1_t saveIrqFlags) {         //содержимое регистра, сохранённого при обработке прерывания
    Serial.print(F(", saveIrqFlags 0x"));
    printHex2(saveIrqFlags);
}

void printLateStats(cEventQueue::eventnode_t& e) {      //статистика задержек приёма
    Serial.print(F(", nLateRx="));
    Serial.print(e.nLateRx);
    Serial.print(F(" ticks="));
    Serial.print(e.ticksLateRx);
}

void printFcnts(cEventQueue::eventnode_t& e) {      //текущие номера восходящих и нисходящих кадров, помогающие отслеживать целостность протокола
    printHex4(e.fcntUp);                            // Восходящий кадровый счётчик
    Serial.print(F(", FcntDn="));
    printHex4(e.fcntDn);                            // Нисходящий кадровый счётчик
}

#if LMIC_ENABLE_event_logging
// dump all the registers.
void printAllRegisters(void) {
    uint8_t regbuf[0x80];                               // Буфер для хранения значений регистров
    regbuf[0] = 0;                                      // Первоначальная инициализация буфера нулевым значением
    hal_spi_read(1, regbuf + 1, sizeof(regbuf) - 1);    // Читаем регистры начиная с адреса 1

    for (unsigned i = 0; i < sizeof(regbuf); ++i) {
        if (i % 16 == 0) {
            printNl();                                  // Новая строка каждые 16 байтов
            printHex2(i);                               // Печать адреса регистра
        }

        Serial.print(((i % 8) == 0) ? F(" - ") : F(" "));   // Форматирование вывода
        printHex2(regbuf[i]);                               // Печать значения регистра
    }

    // reset the radio, just in case the register dump caused issues.
    hal_pin_rst(0);                               // Сбрасываем модуль в неактивное состояние
    delay(2);                                     // Пауза 2 мс
    hal_pin_rst(2);                               // Включаем снова
    delay(6);                                     // Пауза 6 мс

    // restore the radio to idle.
    const uint8_t opmode = 0x88;                  // Режим ожидания LoRa
    hal_spi_write(0x81, &opmode, 1);              // Отправляем команду в регистр OpMode
}
#endif

void printNl(void) {        //пустая строка, перенос каретки
    Serial.println();
}

void eventPrint(cEventQueue::eventnode_t& e) {  //приниммает какое-то событие
    ev_t ev = e.event;          //получаем имя события

    Serial.print(e.time);       //выводим время события в мс
    Serial.print(F(" ("));
    Serial.print(osticks2ms(e.time));
#if SUPPORT_STM32_ClockCalibration      //если есть поддержка калибровки часов, то выводим показания таймера
    Serial.print(F(" ms, lptim1="));
    Serial.print(e.rtccount);
    Serial.print(F("): "));
#else
    Serial.print(F(" ms): "));
#endif

    if (ev == ev_t(-1) || ev == ev_t(-2)) {
        Serial.print(e.pMessage);           // Выводим само сообщение

        if (ev == ev_t(-2)) {
            Serial.print(F(", datum=0x")); Serial.print(e.datum, HEX); // Выводим дополнительный параметр
        }

        printOpmode(e.opmode, '.'); // Выводим режим работы
    }
    else if (ev == ev_t(-3)) {      //если это ошибка, выводим информацию об ошибке
        Serial.print(e.pMessage);
        Serial.print(F(", line ")); Serial.print(e.datum);
        printFreq(e.freq);
        printTxend(e);
        printTxChnl(e.txChnl);
        printRps(e.rps);
        printOpmode(e.opmode);
        printTxrxflags(e.txrxFlags);
        printSaveIrqFlags(e.saveIrqFlags);
        printLateStats(e);
#if LMIC_ENABLE_event_logging
        printAllRegisters();
#endif
    }
    else {          //иначе обрабатываем событие, это не ошибка и не лог
        if (ev < sizeof(evNames) / sizeof(evNames[0])) {
            Serial.print(evNames[ev]);          //выводим имя события
        }
        else {
            Serial.print(F("Unknown event: "));     //неизвестное событие
            Serial.print((unsigned)ev);
        }
        switch (ev) {
        case EV_SCAN_TIMEOUT:
            break;
        case EV_BEACON_FOUND:
            break;
        case EV_BEACON_MISSED:
            break;
        case EV_BEACON_TRACKED:
            break;
        case EV_JOINING:
            break;

        case EV_JOINED:     //успешное соединение
            printTxChnl(e.txChnl);
            printNl();
            do {            //сохранение информации о сеансе. Делается 1 раз. Зачем do while - непонятно
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);      //получаем ключи сеанса, выводим их
                Serial.print(F("netid: "));
                Serial.println(netid, DEC);
                Serial.print(F("devaddr: "));
                Serial.println(devaddr, HEX);
                Serial.print(F("artKey: "));
                for (size_t i = 0; i < sizeof(artKey); ++i) {
                    if (i != 0)
                        Serial.print('-');
                    printHex2(artKey[i]);
                }
                printNl();
                Serial.print(F("nwkKey: "));
                for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                    if (i != 0)
                        Serial.print('-');
                    printHex2(nwkKey[i]);
                }
            } while (0);
            break;
            /*
            || This event is defined but not used in the code. No
            || point in wasting codespace on it.
            ||
            || case EV_RFU1:
            ||     Serial.println(F("EV_RFU1"));
            ||     break;
            */
        case EV_JOIN_FAILED:            //ошибка присоединения
            // print out rx info
            printFreq(e.freq);
            printRps(e.rps);
            printOpmode(e.opmode);
#if LMIC_ENABLE_event_logging
            printAllRegisters();
#endif
            break;

        case EV_REJOIN_FAILED:
            // this event means that someone tried a rejoin, and it failed.
            // it doesn't really mean anything bad, it's just advisory.
            break;

        case EV_TXCOMPLETE:         //завершение отправки данных
            printTxChnl(e.txChnl);
            printRps(e.rps);
            printTxrxflags(e.txrxFlags);
            printFcnts(e);
            printTxend(e);
            printLateStats(e);
            break;
        case EV_LOST_TSYNC:
            break;
        case EV_RESET:
            break;
        case EV_RXCOMPLETE:
            break;
        case EV_LINK_DEAD:
            break;
        case EV_LINK_ALIVE:
            break;
            /*
            || This event is defined but not used in the code. No
            || point in wasting codespace on it.
            ||
            || case EV_SCAN_FOUND:
            ||    Serial.println(F("EV_SCAN_FOUND"));
            ||    break;
            */
        case EV_TXSTART:        //начало отправки данных
            // this event tells us that a transmit is about to start.
            // but printing here is bad for timing.
            printTxChnl(e.txChnl);
            printRps(e.rps);
            printDatarate(e.datarate);
            printOpmode(e.opmode);
            printTxend(e);
            break;

        case EV_RXSTART:            //начало приема данных
            printFreq(e.freq);
            printRps(e.rps);
            printDatarate(e.datarate);
            printOpmode(e.opmode);
            printTxend(e);
            printRxtime(e);
            Serial.print(F(", rxsyms=")); Serial.print(unsigned(e.rxsyms));
            break;

        case EV_JOIN_TXCOMPLETE:    
            printSaveIrqFlags(e.saveIrqFlags);
            printLateStats(e);
            break;

        default:
            break;
        }
    }
    printNl();
}

/*

Name:   myRxMessageCb()

Function:
        Handle received LoRaWAN downlink messages.

Definition:
        lmic_rxmessage_cb_t myRxMessageCb;

        extern "C" {
            void myRxMessageCb(
                void *pUserData,
                uint8_t port,
                const uint8_t *pMessage,
                size_t nMessage
                );
        }

Description:
        This function is called whenever a non-Join downlink message
        is received over LoRaWAN by LMIC. Its job is to invoke the
        compliance handler (if compliance support is needed), and
        then decode any non-compliance messages.

Returns:
        No explicit result.

*/

void myRxMessageCb(
    void* pUserData,       // Пользовательские данные (не используются в данной реализации)
    uint8_t port,          // Порт, на котором получено сообщение
    const uint8_t* pMessage,// Указатель на принятое сообщение
    size_t nMessage        // Длина принятого сообщения
) {
    // Выполнить стандартную проверку соответствия принятому сообщению спецификации LoRaWAN
    lmic_compliance_rx_action_t const action = LMIC_complianceRxMessage(port, pMessage, nMessage);

    // Действия в зависимости от полученного ответа
    switch (action) {
    case LMIC_COMPLIANCE_RX_ACTION_START: {  // Команда перехода в тестовый режим
        Serial.println(F("Enter test mode")); // Вывод уведомления о входе в тестовый режим
        os_clearCallback(&sendjob);           // Удаляем ранее запланированные задания отправки
        g_fTestMode = true;                   // Переводим систему в тестовый режим
        return;                               // Выходим из функции
    }
    case LMIC_COMPLIANCE_RX_ACTION_END: {    // Команда выхода из тестового режима
        Serial.println(F("Exit test mode"));  // Вывод уведомления о выходе из тестового режима
        g_fTestMode = false;                  // Переключаемся обратно в обычный режим
        // Планируем новое задание отправки через интервал TX_INTERVAL секунд
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        return;                               // Выходим из функции
    }
    case LMIC_COMPLIANCE_RX_ACTION_IGNORE: { // Игнорируемое сообщение
        if (port == LORAWAN_PORT_COMPLIANCE) {// Если порт предназначен для тестов
            Serial.print(F("Received test packet 0x")); // Выводим уведомление о тестовом пакете
            if (nMessage > 0)                     // Если длина сообщения положительна
                printHex2(pMessage[0]);           // Выводим первый байт сообщения в hex
            Serial.print(F(" length "));          // Выводим длину сообщения
            Serial.println((unsigned)nMessage);   // Выводим количество полученных байт
        }
        return;                                   // Выходим из функции
    }
    default:                                     // Любое другое действие
        // Просто продолжаем дальше
        break;
    }

    // Стандартный вывод информации о сообщении вне теста
    Serial.print(F("Received message on port ")); // Выводим полученный порт
    Serial.print(port);
    Serial.print(F(": "));
    Serial.print(unsigned(nMessage));            // Выводим длину сообщения
    Serial.println(F(" bytes"));                 // Завершаем вывод
}

lmic_txmessage_cb_t sendComplete;

void do_send(osjob_t* j) {
    // Проверяем, есть ли активные процессы передачи или приема
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending")); // Есть незавершенная передача или прием
        sendComplete(j, 0);                           // Сообщаем о завершении с ошибкой
    }
    else if (g_fTestMode) {
        Serial.println(F("test mode, not sending"));   // Тестовый режим активирован, ничего не отправляем
    }
    else {
        // Готовимся отправить данные вверх по потоку в ближайшее возможное время
        if (LMIC_sendWithCallback_strict(1, mydata, sizeof(mydata), 0, sendComplete, j) == 0) {
            Serial.println(F("Packet queued"));       // Пакет поставлен в очередь
        }
        else {
            Serial.println(F("Packet queue failure; sleeping")); // Ошибка постановки в очередь
            sendComplete(j, 0);                          // Сообщаем о завершении с ошибкой
        }
    }
}

void sendComplete(
    void* pUserData,
    int fSuccess
) {
    osjob_t* const j = (osjob_t*)pUserData; // Преобразование пользовательских данных в ос-задание

    if (!fSuccess)
        Serial.println(F("sendComplete: uplink failed")); // Отправка завершилась неудачей

    if (!g_fTestMode) {
        // Планируем следующее задание на отправку спустя указанный интервал
        os_setTimedCallback(j, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
}

void myFail(const char* pMessage) {
    pinMode(LED_BUILTIN, OUTPUT); // Настраиваем светодиод для сигнализации ошибки
    for (;;) {
        // Постоянно выводим сообщение об ошибке
        Serial.println(pMessage);
        // Мигание светодиодом для привлечения внимания
        for (int i = 0; i < 5; ++i) {
            digitalWrite(LED_BUILTIN, 1); // Включаем светодиод
            delay(100);                   // Задержка на включение
            digitalWrite(LED_BUILTIN, 0); // Выключаем светодиод
            delay(900);                   // Более длительная пауза
        }
    }
}

void setup() {
    delay(5000);                                            // Ждем 5 секунд после включения, чтобы стабилизировалось питание
    while (!Serial);                                      // Ожидаем готовности последовательного порта (Serial)
    Serial.begin(115200);                                  // Инициализируем последовательный порт на скорости 115200 бит/с

    setup_printSignOn();                                    // Показываем информацию о старте и версию прошивки
    setup_calibrateSystemClock();                           // Калибруем внутренний тактовый генератор (если применимо)

    // Инициализация LMIC с настройками платформы
    const auto pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();// Получаем таблицу соединений для нашей платы

    // Если карта распиновки не найдена, завершаться с ошибкой
    if (pPinMap == nullptr) {
        myFail("board not known to library; add pinmap or update getconfig_thisboard.cpp"); // Выход с ошибкой
    }

    // Регистрируем обработчик исключительных ситуаций
    hal_set_failure_handler(log_assertion);                 // Настраиваем функцию-обработчик ошибок

    // Инициализируем нижний уровень ОС и аппаратуру
    os_init_ex(pPinMap);                                    // Инициализация операционной системы и аппаратуры с учётом карты контактов

    // Регистрируем обратные вызовы для обработки событий и входящих сообщений
    if (!(LMIC_registerRxMessageCb(myRxMessageCb, nullptr) && LMIC_registerEventCb(myEventCb, nullptr))) {
        myFail("couldn't register callbacks");              // Если регистрация обработчиков неудачна, завершить программу
    }

    // Сброс состояния стека MAC
    LMIC_reset();                                           // Сбрасываем стек MAC, удаляя любые старые сессии и отложенные передачи

    // Настройка параметров сети перед процедурой подключения
    setupForNetwork(false);                                 // Настройка сетей (может включать выбор субполосы и т.п.)

    // Запускаем первую отправку данных (это запустит автоматический процесс Over-the-Air Activation (OTAA))
    do_send(&sendjob);                                      // Ставим задачу на отправку данных
}

void setup_printSignOnDashLine(void)        //просто рисует длинную линию, чтобы визуально разделить текст
{
    for (unsigned i = 0; i < 78; ++i)
        Serial.print('-');

    printNl();
}

static constexpr const char* filebasename2(const char* s, const char* p) {      //выделяет имя файла из полного пути
    return p[0] == '\0' ? s :
        (p[0] == '/' || p[0] == '\\') ? filebasename2(p + 1, p + 1) :
        filebasename2(s, p + 1);
}

static constexpr const char* filebasename(const char* s)                        //обертка для filebasename2
{
    return filebasename2(s, s);
}

void printVersionFragment(char sep, uint8_t v) {            //выводит один байт, предварительно добавив разделитель (точку, пробел и т.д.) при необходимости
    if (sep != 0) {
        Serial.print(sep);
    }
    Serial.print(unsigned(v));
}

void printVersion(uint32_t v) {                 //выводит полную версию программного продукта
    printVersionFragment(0, uint8_t(v >> 24u));   // Старший байт
    printVersionFragment('.', uint8_t(v >> 16u)); // Срединный старший байт
    printVersionFragment('.', uint8_t(v >> 8u));  // Срединный младший байт
    if (uint8_t(v) != 0) {                       // Последний байт, если ненулевой
        printVersionFragment('.', uint8_t(v));
    }
}

void setup_printSignOn() {      //красиво выводит данные о прошивке
    printNl();                                // Пустой ряд
    setup_printSignOnDashLine();              // Верхняя рамка из '-'

    Serial.println(filebasename(__FILE__));   // Выводим имя файла текущей прошивки
    Serial.print(F("Version "));              // Заголовок версии
    printVersion(APPLICATION_VERSION);        // Выводим версию прошивки
    Serial.print(F("\nLMIC version "));       // Заголовок версии библиотеки LMIC
    printVersion(ARDUINO_LMIC_VERSION);       // Выводим версию библиотеки LMIC
    Serial.print(F(" configured for region ")); // Заголовок региона
    Serial.print(CFG_region);                 // Выводим регион (EU868, AS923 и т.д.)
    Serial.println(F(".\nRemember to select 'Line Ending: Newline' at the bottom of the monitor window.")); // Рекомендуем выбрать правильный Line Ending

    setup_printSignOnDashLine();              // Нижняя рамка из '-'
    printNl();                                // Ещё один пустой ряд
}

void setupForNetwork(bool preJoin) {        //выполняет специальную подготовку перед попыткой присоединиться к сети
#if CFG_LMIC_US_like
    LMIC_selectSubBand(0);
#endif
}

void loop() {
    os_runloop_once();                    // Запуск одного шага основной операционной петли LMIC

    // Проверка на таймаут передачи
    if (lastWasTxStart && millis() - lastTxStartTime > 10000) {
        /* ugh. TX timed out */           // Таймаут передачи данных
        Serial.println(F("Tx timed out")); // Сообщаем о произошедшем таймауте
#if LMIC_ENABLE_event_logging           // Если включена запись событий
        printAllRegisters();             // Выводим содержимое всех регистров
#endif
        LMIC_clrTxData();                // Очищаем данные передачи
        lastWasTxStart = false;          // Снимаем флаг активности передачи
    }

    // Проверка отсутствия активных задач и своевременности
    if ((LMIC.opmode & OP_TXRXPEND) == 0 && // Если нет активной передачи или приёма
        !os_queryTimeCriticalJobs(ms2osticks(1000))) { // И нет срочных задач на ближайшие секунды
        eventPrintAll();                 // Выводим все имеющиеся события
    }
}

// there's a problem with running 2.5 of the MCCI STM32 BSPs;
// hack around it.
#if NEED_USBD_LL_ConnectionState
uint32_t USBD_LL_ConnectionState(void) {
    return 1;
}
#endif // NEED_USBD_LL_ConnectionState

static constexpr bool kMustCalibrateLSE = NEED_STM32_ClockCalibration;          // _mcci_arduino_version indicates that LSE clock is used.
static constexpr bool kCanCalibrateLSE = SUPPORT_STM32_ClockCalibration;

void setup_calibrateSystemClock(void) {
    if (kMustCalibrateLSE) {                  // Необходима обязательная калибровка часов
        Serial.println("need to calibrate clock"); // Выводим сообщение о необходимости калибровки
#if NEED_STM32_ClockCalibration              // Если платформа STM32 требует калибровки
        Stm32_CalibrateSystemClock();         // Осуществляем калибровку часов на платформе STM32
#endif // NEED_STM32_ClockCalibration
        Serial.println("setting LPTIM1");     // Сообщаем о подготовке Low Power Timer
        // Устанавливаем погрешность часов равной 0.4% (стандартная величина)
        LMIC_setClockError(4 * MAX_CLOCK_ERROR / 1000); // Максимально допустимая ошибка составляет примерно 0.4%
        rtccount_begin();                     // Начинаем отсчёт времени RTC
    }
    else if (kCanCalibrateLSE) {             // Возможна калибровка часов
        Serial.println("assuming BIOS has calibrated clock, setting LPTIM1");
        LMIC_setClockError(4 * MAX_CLOCK_ERROR / 1000); // Предполагаем, что часы уже откалиброваны, задаём стандартные 0.4%
        rtccount_begin();                     // Начинаем отсчёт времени RTC
    }
    else {                                   // Никакая калибровка не предусмотрена
        Serial.println("calibration not supported"); // Сообщаем, что калибровка не поддерживается
    }
}

#if NEED_STM32_ClockCalibration //если нужна калибровка часов

// RTC needs to be initialized before we calibrate the clock.
bool rtcbegin() {           //инициализация RTC
    RTC_TimeTypeDef	Time;
    RTC_DateTypeDef	Date;
    uint32_t RtcClock;
    RTC_HandleTypeDef	hRtc;

    memset(&hRtc, 0, sizeof(hRtc));     //Устанавливаем 0, чтобы не было мусора

    hRtc.Instance = RTC;
    hRtc.Init.HourFormat = RTC_HOURFORMAT_24;
    RtcClock = __HAL_RCC_GET_RTC_SOURCE();      //определение источника тактирования
    if (RtcClock == RCC_RTCCLKSOURCE_LSI)       //настройка предделителей
    {
        hRtc.Init.AsynchPrediv = 37 - 1; /* 37kHz / 37 = 1000Hz */
        hRtc.Init.SynchPrediv = 1000 - 1; /* 1000Hz / 1000 = 1Hz */
    }
    else if (RtcClock == RCC_RTCCLKSOURCE_LSE)
    {
        hRtc.Init.AsynchPrediv = 128 - 1; /* 32768Hz / 128 = 256Hz */
        hRtc.Init.SynchPrediv = 256 - 1; /* 256Hz / 256 = 1Hz */
    }
    else
    {
        /*
        || use HSE clock --
        || we don't support use of HSE as RTC because it's connected to
        || TCXO_OUT, and that's controlled by the LoRaWAN software.
        */
        Serial.println(
            " HSE can not be used for RTC clock!"
        );
        return false;
    }


    hRtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hRtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hRtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hRtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hRtc) != HAL_OK)
    {
        Serial.println(
            "HAL_RTC_Init() failed"
        );
        return false;
    }

    /* Initialize RTC and set the Time and Date */
    if (HAL_RTCEx_BKUPRead(&hRtc, RTC_BKP_DR0) != 0x32F2)
    {
        Time.Hours = 0x0;
        Time.Minutes = 0x0;
        Time.Seconds = 0x0;
        Time.SubSeconds = 0x0;
        Time.TimeFormat = RTC_HOURFORMAT12_AM;
        Time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        Time.StoreOperation = RTC_STOREOPERATION_RESET;

        if (HAL_RTC_SetTime(
            &hRtc,
            &Time,
            RTC_FORMAT_BIN
        ) != HAL_OK)
        {
            Serial.print(
                "HAL_RTC_SetTime() failed"
            );
            return false;
        }

        /* Sunday 1st January 2017 */
        Date.WeekDay = RTC_WEEKDAY_SUNDAY;
        Date.Month = RTC_MONTH_JANUARY;
        Date.Date = 0x1;
        Date.Year = 0x0;

        if (HAL_RTC_SetDate(
            &hRtc,
            &Date,
            RTC_FORMAT_BIN
        ) != HAL_OK)
        {
            Serial.print(
                "HAL_RTC_SetDate() failed"
            );
            return false;
        }

        HAL_RTCEx_BKUPWrite(&hRtc, RTC_BKP_DR0, 0x32F2);
    }

    /* Enable Direct Read of the calendar registers (not through Shadow) */
    HAL_RTCEx_EnableBypassShadow(&hRtc);

    HAL_RTC_DeactivateAlarm(&hRtc, RTC_ALARM_A);
    return true;
}

extern "C" {

    static volatile uint32_t* gs_pAlarm;
    static RTC_HandleTypeDef* gs_phRtc;

    void RTC_IRQHandler(void)
    {
        HAL_RTC_AlarmIRQHandler(gs_phRtc);
    }

    void HAL_RTC_AlarmAEventCallback(
        RTC_HandleTypeDef* hRtc
    )
    {
        if (gs_pAlarm)
            *gs_pAlarm = 1;
    }

    void HAL_RTC_MspInit(
        RTC_HandleTypeDef* hRtc
    )
    {
        if (hRtc->Instance == RTC)
        {
            /* USER CODE BEGIN RTC_MspInit 0 */

            /* USER CODE END RTC_MspInit 0 */
            /* Peripheral clock enable */
            __HAL_RCC_RTC_ENABLE();
            /* USER CODE BEGIN RTC_MspInit 1 */
            HAL_NVIC_SetPriority(RTC_IRQn, TICK_INT_PRIORITY, 0U);
            HAL_NVIC_EnableIRQ(RTC_IRQn);
            /* USER CODE END RTC_MspInit 1 */
        }
    }

    void HAL_RTC_MspDeInit(
        RTC_HandleTypeDef* hRtc
    )
    {
        if (hRtc->Instance == RTC)
        {
            /* USER CODE BEGIN RTC_MspDeInit 0 */
            HAL_NVIC_DisableIRQ(RTC_IRQn);
            /* USER CODE END RTC_MspDeInit 0 */
            /* Peripheral clock disable */
            __HAL_RCC_RTC_DISABLE();
            /* USER CODE BEGIN RTC_MspDeInit 1 */

            /* USER CODE END RTC_MspDeInit 1 */
        }
    }

    uint32_t HAL_AddTick(
        uint32_t delta
    )
    {
        extern __IO uint32_t uwTick;
        // copy old interrupt-enable state to flags.
        uint32_t const flags = __get_PRIMASK();

        // disable interrupts
        __set_PRIMASK(1);

        // observe uwTick, and advance it.
        uint32_t const tickCount = uwTick + delta;

        // save uwTick
        uwTick = tickCount;

        // restore interrupts (does nothing if ints were disabled on entry)
        __set_PRIMASK(flags);

        // return the new value of uwTick.
        return tickCount;
    }

} /* extern "C" */

uint32_t Stm32_CalibrateSystemClock(void)
{
    uint32_t Calib;
    uint32_t CalibNew;
    uint32_t CalibLow;
    uint32_t CalibHigh;
    uint32_t mSecond;
    uint32_t mSecondNew;
    uint32_t mSecondLow;
    uint32_t mSecondHigh;
    bool fHaveSeenLow;
    bool fHaveSeenHigh;
    const bool fCalibrateMSI = HAL_RCC_GetHCLKFreq() < 16000000;

    if (!rtcbegin()) {
        return 0;
    }

    if (fCalibrateMSI)
    {
        Calib = (RCC->ICSCR & RCC_ICSCR_MSITRIM) >> 24;
    }
    else
    {
        Calib = (RCC->ICSCR & RCC_ICSCR_HSITRIM) >> 8;
    }

    /* preapre to loop, setting suitable defaults */
    CalibNew = Calib;
    CalibLow = 0;
    CalibHigh = 0;
    mSecondLow = 0;
    mSecondHigh = 2000000;
    fHaveSeenLow = fHaveSeenHigh = false;

    /* loop until we have a new value */
    do {
        /* meassure the # of millis per RTC second */
        mSecond = MeasureMicrosPerRtcSecond();

        /* invariant: */
        if (Calib == CalibNew)
            mSecondNew = mSecond;

        /* if mSecond is low, this meaans we must increase the system clock */
        if (mSecond <= 1000000)
        {
            Serial.print('-');
            /*
            || the following condition establishes that we're
            || below the target frequency, but closer than we've been
            || before (mSecondLow is the previous "low" limit). If
            || so, we reduce the limit, and capture the "low" calibration
            || value.
            */
            if (mSecond > mSecondLow)
            {
                mSecondLow = mSecond;
                CalibLow = Calib; /* save previous calibration value */
                fHaveSeenLow = true;
            }

            /*
            || if we are low, and we have never exceeded the high limit,
            || we can  increase the clock.
            */
            if (!fHaveSeenHigh)
            {
                if (fCalibrateMSI)
                {
                    if (Calib < 0xFF)
                    {
                        ++Calib;
                        __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(Calib);
                    }
                    else
                        break;
                }
                else
                {
                    if (Calib < 0x1F)
                    {
                        ++Calib;
                        __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(Calib);
                    }
                    else
                    {
                        break;
                    }
                }

                /* let the clock settle */
                delay(500);
            }
        }

        /* if mSecond is high, we must reduce the system clock */
        else
        {
            Serial.print('+');
            /*
            || the following condition establishes that we're
            || above the target frequency, but closer than we've been
            || before (mSecondHigh is the previous "high" limit). If
            || so, we reduce the limit, and capture the calibration
            || value.
            */
            if (mSecond < mSecondHigh)
            {
                mSecondHigh = mSecond;
                CalibHigh = Calib;
                fHaveSeenHigh = true;
            }

            /*
            || if we are above the target frequency, and we have
            || never raised the frequence, we can lower the
            || frequency
            */
            if (!fHaveSeenLow)
            {
                if (Calib == 0)
                    break;

                --Calib;
                if (fCalibrateMSI)
                {
                    __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(Calib);
                }
                else
                {
                    __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(Calib);
                }
                delay(500);
            }
        }
    } while ((Calib != CalibNew) &&
        (!fHaveSeenLow || !fHaveSeenHigh));

    //
    // We are going to take higher calibration value first and
    // it allows us not to call LMIC_setClockError().
    //
    if (fHaveSeenHigh)
    {
        mSecondNew = mSecondHigh;
        CalibNew = CalibHigh;
    }
    else if (fHaveSeenLow)
    {
        mSecondNew = mSecondLow;
        CalibNew = CalibLow;
    }
    else
    {
        // Use original value
        Serial.println(
            "?CalibrateSystemClock: can't calibrate"
        );
    }

    if (CalibNew != Calib)
    {
        Serial.print(CalibNew < Calib ? '+' : '-');
        if (fCalibrateMSI)
        {
            __HAL_RCC_MSI_CALIBRATIONVALUE_ADJUST(CalibNew);
        }
        else
        {
            __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(CalibNew);
        }
        delay(500);
    }

    Serial.print(" 0x");
    Serial.println(CalibNew, HEX);
    return CalibNew;
}

uint32_t
MeasureMicrosPerRtcSecond(
    void
)
{
    uint32_t second;
    uint32_t now;
    uint32_t start;
    uint32_t end;

    /* get the starting time */
    second = RTC->TR & (RTC_TR_ST | RTC_TR_SU);

    /* wait for a new second to start, and capture millis() in start */
    do {
        now = RTC->TR & (RTC_TR_ST | RTC_TR_SU);
        start = micros();
    } while (second == now);

    /* update our second of interest */
    second = now;

    /* no point in watching the register until we get close */
    delay(500);

    /* wait for the next second to start, and capture millis() */
    do {
        now = RTC->TR & (RTC_TR_ST | RTC_TR_SU);
        end = micros();
    } while (second == now);

    /* return the delta */
    return end - start;
}
#endif // NEED_STM32_ClockCalibration

#if SUPPORT_STM32_ClockCalibration
static void rtccount_begin()
{
    // enable clock to LPTIM1
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    auto const pLptim = LPTIM1;

    // set LPTIM1 clock to LSE clock.
    __HAL_RCC_LPTIM1_CONFIG(RCC_LPTIM1CLKSOURCE_LSE);

    // disable everything so we can tweak the CFGR
    pLptim->CR = 0;

    // disable interrupts (needs to be done while disabled globally)
    pLptim->IER = 0;

    // upcount from selected internal clock (which is LSE)
    auto rCfg = pLptim->CFGR & ~0x01FEEEDF;
    rCfg |= 0;
    pLptim->CFGR = rCfg;

    // enable the counter but don't start it
    pLptim->CR = LPTIM_CR_ENABLE;
    delayMicroseconds(100);

    // set ARR to max value so we can count from 0 to 0xFFFF.
    // must be done after enabling.
    pLptim->ARR = 0xFFFF;

    // start in continuous mode.
    pLptim->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
}

static uint16_t rtccount_read()
{
    auto const pLptim = LPTIM1;
    uint32_t v1, v2;

    for (v1 = pLptim->CNT & 0xFFFF; (v2 = pLptim->CNT & 0xFFFF) != v1; v1 = v2)
        /* loop */;

    return (uint16_t)v1;
}

#else
static void rtccount_begin()
{
    // nothing
}

static uint16_t rtccount_read()
{
    return 0;
}
#endif