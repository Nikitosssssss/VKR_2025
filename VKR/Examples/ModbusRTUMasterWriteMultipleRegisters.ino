//Эта функция реализует запрос значений катушек от серверного устройства(Master) клиентскому (Slave). 

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

/*
    катушка это цифровой аналог электрических контактов. 
    Логический сигнал, который используется для представления состояния элемента, включен или выключен.
    По сути каждая катушка представляет собой бит данных.
*/

#define COILS_TO_READ 5                 //количетсво катушек, которые нужно читать
uint32_t lastSentTime = 0UL;            
const uint32_t baudrate = 38400UL;      //скорость обмена данными

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
        //запрос на чтение 5-ти катушеек, начиная с 0-вой. 0, 1 , 2, 3, 4.
        // функции чтения и записи Modbus работают ассинхронно.
        if (!master.readCoils(31, 0, COILS_TO_READ)) {
            //Если произошла ошибка при работе функции, она вернет false. Здесь должен быть блок обработки ошибок
        }

        lastSentTime = millis();    //получаем текущее время
    }

    // проверяем, есть ли ответ от устройства
    if (master.isWaitingResponse()) {
        ModbusResponse response = master.available();       //попытка получить доступный ответ
        if (response) {     //если ответ получен
            if (response.hasError()) {          //видимо флаг, который говорит о том, что в ответ пришла ошибка
                //Можно использовать функцию response.getErrorCode(), чтобы получить код ошибки
            }
            else {
                //Если ошибок нет, то выводим значение катушек(битов)
                for (byte i = 0; i < COILS_TO_READ; i++) {
                    bool coil = response.isCoilSet(i);

                    Serial.print("Coil ");
                    Serial.print(i);
                    Serial.print(": ");
                    Serial.println(coil);
                }
            }
        }
    }
}