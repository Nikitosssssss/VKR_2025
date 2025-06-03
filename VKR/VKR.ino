#include "LoRaWan_APP.h"
#include <iarduino_Modbus.h> 

ModbusClient modbus(Serial, 1);  
/* OTAA para*/
uint8_t devEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26};
uint8_t appSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26};
uint32_t devAddr =  ( uint32_t )0x22222226;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;
const unsigned long readSensorInterval = 10000;  

/*OTAA or ABP*/
bool overTheAirActivation = false;

/*ADR enable*/
bool loraWanAdr = true;


/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

int lastResult;
long middleValue=0;
int count=0;
int allCount = 0;
int loses = 0;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
    appDataSize = 4;
    appData[0] = 0x01;
    appData[1] = middleValue;
    appData[2] = count;
    appData[3] = 0x04;
    middleValue = 0;
}



//downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
  uint32_t color=mcpsIndication->Buffer[0]<<16|mcpsIndication->Buffer[1]<<8|mcpsIndication->Buffer[2];
#if(LoraWan_RGB==1)
  turnOnRGB(color,5000);
  turnOffRGB();
#endif
}
int readSensor(void);

RTC_DATA_ATTR bool firstrun = true;

void setup() {
  Serial.begin(9600, SERIAL_8N1, 45, 43); 
  //Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  modbus.begin();
  modbus.setTimeout(10);
  modbus.setDelay(4);
  modbus.setTypeMB(MODBUS_RTU);
  if(firstrun)
  {
    LoRaWAN.displayMcuInit();
    firstrun = false;
  }

}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(LORAWAN_DEVEUI_AUTO)
			LoRaWAN.generateDeveuiByChipID();
#endif
			LoRaWAN.init(loraWanClass,loraWanRegion);
			//both set join DR and DR when ADR off 
			LoRaWAN.setDefaultDR(3);
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.displayJoining();
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
      int rezult = readSensor();
			LoRaWAN.displaySending();
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.displayAck();
			LoRaWAN.sleep(loraWanClass);
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}


int readSensor(void) {
  int result = 0;
  allCount +=1;
  if (!modbus.requestFrom(1, HOLDING_REGISTERS, 0x90, 1)) {
    loses +=1;
    return -1;           // Ошибка отправки команды
  }

  while (modbus.available()) {
    result = modbus.read();
  }
  middleValue += result;
  count +=1;
  
  return result;
}

