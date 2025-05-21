
//Функция считывает значения дискретных входов
#include <ModbusRTUMaster.h>


// Выбирается тип интерфейса
#if defined HAVE_RS485_HARD
#include <RS485.h>
ModbusRTUMaster master(RS485);

#elif defined HAVE_RS232_HARD
#include <RS232.h>
ModbusRTUMaster master(RS232);

#else
ModbusRTUMaster master(Serial1);
#endif


#define DISCRETE_INPUTS_TO_READ 7       //дискретные входы отражают статусы физических входов, подключенных к устройству. Они предназначены только для чтения      
uint32_t lastSentTime = 0UL;
const uint32_t baudrate = 38400UL;

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(9600UL);

    // Выбор протокола передачи данных в зависимости от типа интерфейса. SERIAL_8E1 - настройка формата кадра. Пакет состоит из 8 бит, тип проверки четности - четный, 1 стоп бит
#if defined HAVE_RS485_HARD
    RS485.begin(baudrate, HALFDUPLEX, SERIAL_8E1);
#elif defined HAVE_RS232_HARD
    RS232.begin(baudrate, SERIAL_8E1);
#else
    Serial1.begin(baudrate, SERIAL_8E1);
#endif

    master.begin(baudrate); //создание объекта modbus master
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    // Каждую секунду делается запрос
    if (millis() - lastSentTime > 1000) {
        //читаем дискретные входы, начиная с 0-вого, 7 штук
        if (!master.readDiscreteInputs(31, 0, DISCRETE_INPUTS_TO_READ)) {
            // обработка ошибки
        }

        lastSentTime = millis();
    }

    // Check available responses often
    if (master.isWaitingResponse()) {
        ModbusResponse response = master.available();
        if (response) {
            if (response.hasError()) {
                //Обработка ответа. Если ответ доступен, но содержит ошибки, то получаем код ошибки и выводим на консоль
                Serial.print("Error ");
                Serial.println(response.getErrorCode());
            }
            else {
                // Если пришел ответ без ошибок, то выводим значения дискретных входов на консоль.
                Serial.print("Discrete inputs values: ");
                for (int i = 0; i < DISCRETE_INPUTS_TO_READ; ++i) {
                    Serial.print(response.isDiscreteInputSet(i));
                    Serial.print(',');
                }
                Serial.println();
            }
        }
    }
}