
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>



#define TX_INTERVAL 2000        // �������� �������� 2 ���
#define RX_RSSI_INTERVAL 100    // �������� ��������� ������

 // ����� ����������� ���������
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = { 25,    // DIO0 (IRQ) is D25
                 26,    // DIO1 is D26
                 27,    // DIO2 is D27
               },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000     // 8MHz
};
#else
# error "Unknown target"
#endif

//�������� ������ ��� ��������� ����������
void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

//� ���� ������� ������� �� ���������������
void onEvent(ev_t ev) {
}

extern "C" {
    void lmic_printf(const char* fmt, ...);
};
//������ ���������� ���������� � ���������������� ����
void lmic_printf(const char* fmt, ...) {
    if (!Serial.dtr())
        return;

    char buf[256];                      // ����� ��� ������������ ������
    va_list ap;

    va_start(ap, fmt);                  // �������� ������ ���������� �������
    vsnprintf(buf, sizeof(buf) - 1, fmt, ap); // ����������� ������
    va_end(ap);

    buf[sizeof(buf) - 1] = '\0';        // ����������� ������� ����������� ������
    if (Serial.dtr())                   // ���� ������ ������ � ����������������� �����
        Serial.print(buf);              // ������� �������������� ������
}

osjob_t txjob;                         // ������ ������� ��� ��������
osjob_t timeoutjob;                    // ������ ������� ��������
static void tx_func(osjob_t* job);      // �������� ������� ��������

//�������� ������
void tx(const char* str, osjobcb_t func) {
    os_radio(RADIO_RST);                     // ������������� ����� �������� (���� �� �������)
    delay(1);                               // ��������� ��������, ����� ��������� ����� ����� �� ������ �����

    LMIC.dataLen = 0;                       // �������� ����� ������
    while (*str)                             // �������� ������ �� ������ � �����
        LMIC.frame[LMIC.dataLen++] = *str++; // ������ ����� �������� � ����

    LMIC.osjob.func = func;                 // ��������� callback-������� ��� ���������� ��������
    os_radio(RADIO_TX);                     // ������������� � ����� ��������
    Serial.println("TX");                   // ������� � �������� ����������� � ��������
}

// ������� � ����� ������
void rx(osjobcb_t func) {
    LMIC.osjob.func = func;                 // ��������� ������� ��������� ������ ��� ��������� �����
    LMIC.rxtime = os_getTime();             // ����� ������ ����� ��������� � ������� ��������
    os_radio(RADIO_RXON);                   // ���������� ���������� ����
    Serial.println("RX");                   // ������� ����������� � �������� � ����� �����
}

//����-��� ������
static void rxtimeout_func(osjob_t* job) {
    digitalWrite(LED_BUILTIN, LOW);         // ��������� ��������� (��������� ������������ �����)
}

//������ �� ���������� �����
static void rx_func(osjob_t* job) {
    digitalWrite(LED_BUILTIN, LOW);         // ��������� ��������� (����� ���������� ����������� �����)
    delay(10);                              // ������� �����
    digitalWrite(LED_BUILTIN, HIGH);        // �������� ��������� ������� (����� ����������� ����)

    // ������ ������ ���������� ���������� ������ 3 ������� ��������
    os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3 * TX_INTERVAL), rxtimeout_func);

    // ��������� ��������� �������� �������� ���������� ��������� ��������
    os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL / 2), tx_func);

    // ���������� � ���������� ������
    Serial.print("Got ");                   // ������� ���������� � �������� ������
    Serial.print(LMIC.dataLen);             // ����� ��������� ������
    Serial.println(" bytes");
    Serial.write(LMIC.frame, LMIC.dataLen); // ���������� ��� �������� �����
    Serial.println();

    // ������������ � ����� ����� �����
    rx(rx_func);                           // ��������� �������� �����
}

static void txdone_func(osjob_t* job) {
    rx(rx_func);                            // ����� ����� ���������� �������� ��������� � ����� �����
}

//�������� ������
static void tx_func(osjob_t* job) {
    tx("Hello, world!", txdone_func);      // ���������� ������ 'Hello, world!' � ��������� ���������� ��������
    // �������� ��������� ������ �������� � ��������� ���������, ����� �������� ��������
    os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}


void setup() {
 
    delay(3000);

    //����, ���� ��������� ���������������� ����
    while (!Serial.dtr())
        /* wait for the PC */;
    //������������� ����������������� �����
    Serial.begin(115200);
    Serial.println("Starting");

#ifdef VCC_ENABLE   //��� ��������� ���� ����� �������� �������� ������� �������� �������
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
#endif

    pinMode(LED_BUILTIN, OUTPUT);   //��������� ���������� �� �����

    // initialize runtime env
    os_init();

    // ������������� �������� �����
#ifdef ARDUINO_ARCH_STM32
    LMIC_setClockError(10 * 65536 / 100);
#endif
    
#if defined(CFG_eu868)  //����� ��������� ��� ������ ��������
    
    LMIC.freq = 869525000;
   
    LMIC.datarate = DR_SF9;

    LMIC.txpow = 27;
#elif defined(CFG_us915)
   
    const static bool fDownlink = false;

    // the downlink channel to be used.
    const static uint8_t kDownlinkChannel = 3;

    // the uplink channel to be used.
    const static uint8_t kUplinkChannel = 8 + 3;

    // this is automatically set to the proper bandwidth in kHz,
    // based on the selected channel.
    uint32_t uBandwidth;

    if (!fDownlink)
    {
        if (kUplinkChannel < 64)
        {
            LMIC.freq = US915_125kHz_UPFBASE +
                kUplinkChannel * US915_125kHz_UPFSTEP;
            uBandwidth = 125;
        }
        else
        {
            LMIC.freq = US915_500kHz_UPFBASE +
                (kUplinkChannel - 64) * US915_500kHz_UPFSTEP;
            uBandwidth = 500;
        }
    }
    else
    {
        // downlink channel
        LMIC.freq = US915_500kHz_DNFBASE +
            kDownlinkChannel * US915_500kHz_DNFSTEP;
        uBandwidth = 500;
    }

    // Use a suitable spreading factor
    if (uBandwidth < 500)
        LMIC.datarate = US915_DR_SF7;         // DR4
    else
        LMIC.datarate = US915_DR_SF12CR;      // DR8

    // default tx power for US: 21 dBm
    LMIC.txpow = 21;
#elif defined(CFG_au915)
   
    const static bool fDownlink = false;

    // the downlink channel to be used.
    const static uint8_t kDownlinkChannel = 3;

    // the uplink channel to be used.
    const static uint8_t kUplinkChannel = 8 + 3;

    // this is automatically set to the proper bandwidth in kHz,
    // based on the selected channel.
    uint32_t uBandwidth;

    if (!fDownlink)
    {
        if (kUplinkChannel < 64)
        {
            LMIC.freq = AU915_125kHz_UPFBASE +
                kUplinkChannel * AU915_125kHz_UPFSTEP;
            uBandwidth = 125;
        }
        else
        {
            LMIC.freq = AU915_500kHz_UPFBASE +
                (kUplinkChannel - 64) * AU915_500kHz_UPFSTEP;
            uBandwidth = 500;
        }
    }
    else
    {
        // downlink channel
        LMIC.freq = AU915_500kHz_DNFBASE +
            kDownlinkChannel * AU915_500kHz_DNFSTEP;
        uBandwidth = 500;
    }

    // Use a suitable spreading factor
    if (uBandwidth < 500)
        LMIC.datarate = AU915_DR_SF7;         // DR4
    else
        LMIC.datarate = AU915_DR_SF12CR;      // DR8

    // default tx power for AU: 30 dBm
    LMIC.txpow = 30;
#elif defined(CFG_as923)
  
    const static uint8_t kChannel = 0;
    uint32_t uBandwidth;

    LMIC.freq = AS923_F1 + kChannel * 200000;
    uBandwidth = 125;

    // Use a suitable spreading factor
    if (uBandwidth == 125)
        LMIC.datarate = AS923_DR_SF7;         // DR7
    else
        LMIC.datarate = AS923_DR_SF7B;        // DR8

    // default tx power for AS: 21 dBm
    LMIC.txpow = 16;

    if (LMIC_COUNTRY_CODE == LMIC_COUNTRY_CODE_JP)
    {
        LMIC.lbt_ticks = us2osticks(AS923JP_LBT_US);
        LMIC.lbt_dbmax = AS923JP_LBT_DB_MAX;
    }
#elif defined(CFG_kr920)
  
    const static uint8_t kChannel = 0;
    uint32_t uBandwidth;

    LMIC.freq = KR920_F1 + kChannel * 200000;
    uBandwidth = 125;

    LMIC.datarate = KR920_DR_SF7;         // DR7
    // default tx power for KR: 14 dBm
    LMIC.txpow = KR920_TX_EIRP_MAX_DBM;
    if (LMIC.freq < KR920_F14DBM)
        LMIC.txpow = KR920_TX_EIRP_MAX_DBM_LOW;

    LMIC.lbt_ticks = us2osticks(KR920_LBT_US);
    LMIC.lbt_dbmax = KR920_LBT_DB_MAX;
#elif defined(CFG_in866)
   
    const static uint8_t kChannel = 0;
    uint32_t uBandwidth;

    LMIC.freq = IN866_F1 + kChannel * 200000;
    uBandwidth = 125;

    LMIC.datarate = IN866_DR_SF7;         // DR7
    // default tx power for IN: 30 dBm
    LMIC.txpow = IN866_TX_EIRP_MAX_DBM;
#else
# error Unsupported LMIC regional configuration.
#endif


    // disable RX IQ inversion
    LMIC.noRXIQinversion = true;

    // This sets CR 4/5, BW125 (except for EU/AS923 DR_SF7B, which uses BW250)
    LMIC.rps = updr2rps(LMIC.datarate);
        
    Serial.print("Frequency: "); Serial.print(LMIC.freq / 1000000);         //����� ��������
    Serial.print("."); Serial.print((LMIC.freq / 100000) % 10);
    Serial.print("MHz");
    Serial.print("  LMIC.datarate: "); Serial.print(LMIC.datarate);
    Serial.print("  LMIC.txpow: "); Serial.println(LMIC.txpow);
    Serial.println("Started");
    Serial.flush();

    // setup initial job
    os_setCallback(&txjob, tx_func);        //������ ��������
}

void loop() {
    // execute scheduled jobs and events
    os_runloop_once();
}