
#include <ModbusRTUSlave.h>

 // Настройка соответствия между выводами микроконтроллера и виртуальными регистрами Modbus
//цифровые выходы
int digitalOutputsPins[] = {
#if defined(PIN_Q0_4)
  Q0_0, Q0_1, Q0_2, Q0_3, Q0_4,    
#endif
};
//цифровые входы
int digitalInputsPins[] = {
#if defined(PIN_I0_6)
  I0_0, I0_1, I0_2, I0_3, I0_4, I0_5, I0_6,
#endif
};
//Аналоговые выходы
int analogOutputsPins[] = {
#if defined(PIN_Q0_7)
  A0_5, A0_6, A0_7,
#endif
};
//Аналоговые входы
int analogInputsPins[] = {
#if defined(PIN_I0_12)
  I0_7, I0_8, I0_9, I0_10, I0_11, I0_12,
#endif
};

//вычисление количества элементов в каждом массиве
#define numDigitalOutputs int(sizeof(digitalOutputsPins) / sizeof(int))
#define numDigitalInputs int(sizeof(digitalInputsPins) / sizeof(int))
#define numAnalogOutputs int(sizeof(analogOutputsPins) / sizeof(int))
#define numAnalogInputs int(sizeof(analogInputsPins) / sizeof(int))

//массивы для хранения состояний каналов
bool digitalOutputs[numDigitalOutputs];
bool digitalInputs[numDigitalInputs];
uint16_t analogOutputs[numAnalogOutputs];
uint16_t analogInputs[numAnalogInputs];

// Выбор интерфейса
#if defined HAVE_RS485_HARD
#include <RS485.h>
ModbusRTUSlave modbus(RS485, 31);

#elif defined HAVE_RS232_HARD
#include <RS232.h>
ModbusRTUSlave modbus(RS232, 31);

#else
ModbusRTUSlave modbus(Serial1, 31);
#endif

const uint32_t baudrate = 38400UL;  //скорость обмена

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600UL);

    // Массивы заполняются начальными значениями
    for (int i = 0; i < numDigitalOutputs; ++i) {
        digitalOutputs[i] = false;
        digitalWrite(digitalOutputsPins[i], digitalOutputs[i]);
    }
    for (int i = 0; i < numDigitalInputs; ++i) {
        digitalInputs[i] = digitalRead(digitalInputsPins[i]);
    }
    for (int i = 0; i < numAnalogOutputs; ++i) {
        analogOutputs[i] = 0;
        analogWrite(analogOutputsPins[i], analogOutputs[i]);
    }
    for (int i = 0; i < numAnalogInputs; ++i) {
        analogInputs[i] = analogRead(analogInputsPins[i]);
    }

    // Выбор протокола передачи данных в зависимости от типа интерфейса. SERIAL_8E1 - настройка формата кадра. Пакет состоит из 8 бит, тип проверки четности - четный, 1 стоп бит
#if defined HAVE_RS485_HARD
    RS485.begin(baudrate, HALFDUPLEX, SERIAL_8E1);
#elif defined HAVE_RS232_HARD
    RS232.begin(baudrate, SERIAL_8E1);
#else
    Serial1.begin(baudrate, SERIAL_8E1);
#endif

    // создаем объект modbus slave
    modbus.begin(baudrate);

    //сопоставление областей объекта modbus с областями памяти
    modbus.setCoils(digitalOutputs, numDigitalOutputs);             //цифровые выходы - катушки
    modbus.setDiscreteInputs(digitalInputs, numDigitalInputs);      //цифровые входы - дискретные входы
    modbus.setHoldingRegisters(analogOutputs, numAnalogOutputs);    //регистры хранения - аналоговые выходы
    modbus.setInputRegisters(analogInputs, numAnalogInputs);        //входные регистры - аналоговые входы
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    // обновление значений входов
    for (int i = 0; i < numDigitalInputs; ++i) {
        digitalInputs[i] = digitalRead(digitalInputsPins[i]);       //Сохраняем состояния входов
    }
    for (int i = 0; i < numAnalogInputs; ++i) {
        analogInputs[i] = analogRead(analogInputsPins[i]);
    }

    //синхронизирует фактическое состояние системы с состоянием в виде регистров Modbus, так как дискретные входы и входные регистры недоступны для записи
    modbus.update();

    // обновление значений выходов, катушки и регистры хранения доступны для записи, тут можно сразу записать
    for (int i = 0; i < numDigitalOutputs; ++i) {
        digitalWrite(digitalOutputsPins[i], digitalOutputs[i]);
    }
    for (int i = 0; i < numAnalogOutputs; ++i) {
        analogWrite(analogOutputsPins[i], analogOutputs[i]);
    }
}