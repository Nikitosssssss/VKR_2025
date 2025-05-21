
//���������� ������ � ���� ������� ��������

#include <ModbusRTUMaster.h>

// ���������� ��� ����������
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

    // ����� ��������� �������� ������ � ����������� �� ���� ����������. SERIAL_8E1 - ��������� ������� �����. ����� ������� �� 8 ���, ��� �������� �������� - ������, 1 ���� ���
#if defined HAVE_RS485_HARD
    RS485.begin(baudrate, HALFDUPLEX, SERIAL_8E1);
#elif defined HAVE_RS232_HARD
    RS232.begin(baudrate, SERIAL_8E1);
#else
    Serial1.begin(baudrate, SERIAL_8E1);
#endif

    master.begin(baudrate); //�������� ������� modbus master
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    
    if (millis() - lastSentTime > 1000) {
        // � ������� 0 ���������� �������� 1000
        if (!master.writeSingleRegister(31, 0, 1000)) {
            //��������� ������
        }

        lastSentTime = millis();
    }

    // ���������, ���� �� ����� �� ����������
    if (master.isWaitingResponse()) {
        ModbusResponse response = master.available();
        if (response) {
            if (response.hasError()) {
                Serial.print("Error ");
                Serial.println(response.getErrorCode());
            }
            else {
                
                if (response.hasError()) {
                    // ������� ������ � �� ���
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