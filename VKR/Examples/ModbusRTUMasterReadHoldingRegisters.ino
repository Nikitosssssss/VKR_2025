
//Читает значения регистров хранения


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

#define REGISTERS_TO_READ 3             //количество регистров, которые нужно читать. Регистр хранения - ячейка памяти длиной 16 бит, доступная для чтения и записи
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
    // каждую секунду отправляется запрос
    if (millis() - lastSentTime > 1000) {
        //считываем регистры хранения
        if (!master.readHoldingRegisters(31, 0, REGISTERS_TO_READ)) {
            //Обработка ошибки
        }

        lastSentTime = millis();
    }

    // проверяем, есть ли ответ от устройства
    if (master.isWaitingResponse()) {
        ModbusResponse response = master.available();
        if (response) {
            if (response.hasError()) {
                //Если ответ содержит ошибку, то выводим ее код
                Serial.print("Error ");
                Serial.println(response.getErrorCode());
            }
            else {
                //Код дублируется, это не нужно здесь, мне кажется
                    //if (response.hasError()) {
                    
                    //    Serial.print("Error ");
                    //    Serial.println(response.getErrorCode());
                    //}
                //else {
                    Serial.print("Holding Register values: ");
                    for (int i = 0; i < REGISTERS_TO_READ; ++i) {
                        Serial.print(response.getRegister(i));  //скорее всего ответ приходит в виде массива значений, поэтому получаем отдельно значение регистра
                        Serial.print(',');
                    }
                    Serial.println();
                //}
            }
        }
    }
}