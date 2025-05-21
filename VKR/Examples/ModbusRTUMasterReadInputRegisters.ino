
//записываем данные в один регистр хранения

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
    
    if (millis() - lastSentTime > 1000) {
        // в регистр 0 записываем значение 1000
        if (!master.writeSingleRegister(31, 0, 1000)) {
            //обработка ошибки
        }

        lastSentTime = millis();
    }

    // проверяем, есть ли ответ от устройства
    if (master.isWaitingResponse()) {
        ModbusResponse response = master.available();
        if (response) {
            if (response.hasError()) {
                Serial.print("Error ");
                Serial.println(response.getErrorCode());
            }
            else {
                
                if (response.hasError()) {
                    // Выводим ошибку и ее код
                    Serial.print("Error ");
                    Serial.println(response.getErrorCode());
                }
                else {
                    Serial.println("Done");
                }
            }
        }
    }
}