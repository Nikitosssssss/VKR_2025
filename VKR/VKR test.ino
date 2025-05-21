#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// LoRaWAN NwkSKey, network session key - network session key, ������� ���� �������
static const PROGMEM u1_t NWKSKEY[16] = { 0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26 };

// LoRaWAN AppSKey, application session key - ���� ������� ����������
static const u1_t PROGMEM APPSKEY[16] = { 0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x26 };

// LoRaWAN end-device address (DevAddr) - ����� ����������
static const u4_t DEVADDR = 0x22222226; 


static uint8_t mydata[4];           //������ ������, ���� ���������� ��������� �����
static osjob_t sendjob;

const unsigned TX_INTERVAL = 30;        //��� � 30 ��� ����� ���������� ������ �� ��
const unsigned port = 1;                //���� ��� ��������
 

// ��������� ������� RX2 (� ��)  
const uint32_t RX2_FREQ = 869100000; // 869.1 ���  

// ��������� Data Rate RX2 (DR0 ��� RU868)  
const uint8_t RX2_DR = DR_SF12;


// ���-������� RFM95
// �� ����, ��� ����� ��������� ����
const lmic_pinmap lmic_pins = {
    .nss = 10,                      //��� ����������. ���������, ������ �� � ������ ����������
    .rxtx = LMIC_UNUSED_PIN,        //��� ������� � TX �� RX, ������������ ������
    .rst = 9,                       //��� reset
    .dio = {2, 3, LMIC_UNUSED_PIN}, // DIO0, DIO1, DIO2
};
// ===== ��������� ������� =====
void sendStatus(osjob_t* j);
void checkAlarm();
void sendAlarm();


uint8_t dataLength = 0;
bool newDataReceived = false;

//���������:
// ev_t ev - �������, ������������ � �������
void onEvent(ev_t ev) {
    Serial.print(os_getTime());     //������� ������� �����
    Serial.print(": ");
    switch (ev) {
    case EV_TXCOMPLETE:         //���� �������� ��������� �������, �� ������� ���������
        Serial.println("EV_TXCOMPLETE");
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println("Received ACK");
        if (LMIC.dataLen) {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        //����� ������������� ��������� ��������
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;

    case EV_JOINED:
        Serial.println("EV_JOINED");
        break;

    case EV_RXCOMPLETE:
        Serial.println("EV_RXCOMPLETE");
        break;

    case EV_JOIN_FAILED:
        Serial.println("EV_JOIN_FAILED");
        break;

    default:
        Serial.print("Unknown event: ");
        Serial.println(ev);
        break;
    }
}

//���������� ��������� 1
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {         //��������, ���� �� ������ ����� ��� �������� ������
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else {
        mydata[0] = 1;      //������ �������� ������-�� �������
        mydata[1] = 2;
        mydata[2] = 3;
        mydata[3] = 4;
        // mydata - ������ ������, sizeof(mydata) - ������ ������, 0 - �����, �������������� ���������
        LMIC_setTxData2(port, mydata, sizeof(mydata), 0);      
        Serial.println(F("Packet queued"));
    }
}

void setup() {
  
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Initializing...");
    os_init();
    LMIC_reset();
  

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);    //���������� ��������� ��� �����������

    // ��������� �������
    LMIC_setupChannel(0, 864100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(1, 864300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 864500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(3, 864700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(4, 864900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(5, 869100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(6, 868900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(7, 869000000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
    LMIC_setupChannel(8, 869300000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF12, 14);

    do_send(&sendjob);              //�������� ������

}
void loop() {
    
    os_runloop_once();     //������������ �����, ������������ � �������
  
}