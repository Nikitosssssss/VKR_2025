#include "LoRaWan_APP.h"
#include <iarduino_Modbus.h> 
#include <Wire.h>                
#include "HT_SSD1306Wire.h"

static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); 
ModbusClient modbus(Serial, 1);  
/* OTAA para*/
uint8_t devEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appEui[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };
uint8_t appKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26};
uint8_t appSKey[] = { 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x26};
uint32_t devAddr =  ( uint32_t )0x2222226;

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

uint8_t allCount = 0;
uint8_t loses = 0;
uint8_t slaveID =0;
uint8_t speedOfModbus =0;
uint8_t value =0;


/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
    appDataSize = 4;
    appData[0] = slaveID;
    appData[1] = speedOfModbus;
    appData[2] = value;
    appData[3] = allCount-loses;
}



//downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication){
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

//временная метка для задержки между чтением по modbus
unsigned long previousMillis;

void setup() {
  Serial.begin(9600, SERIAL_8N1, 45, 43); 
  //Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  modbus.begin();
  modbus.setTimeout(10);
  modbus.setDelay(4);
  modbus.setTypeMB(MODBUS_RTU);
	display.init();
  display.flipScreenVertically();

  for(int i = 0; i <4; i++)
  {
    appData[i] = 0;
  }

  // if(firstrun)
  // {
  //   LoRaWAN.displayMcuInit();
  //   firstrun = false;
  // }

}

void loop()
{
  //onEvent();
	readSensor();
	displayStart();
	delay(5000);
	display.clear();
  displayStart();
  delay(5000);
  display.clear();
}


int readSensor(void) {
  allCount +=1;
  //считываем slave id 
  if (!modbus.requestFrom(1, HOLDING_REGISTERS, 0xfd, 1)) {
    loses +=1;
    return 0;           // Ошибка отправки команды
  }
  while(modbus.available()) {
    slaveID = modbus.read();
  }
  //небольшая задержка между чтениями
  previousMillis = millis();
  while(millis() - previousMillis <1000){};
  //скорость отправки, 3 = 9600
  if (!modbus.requestFrom(1, HOLDING_REGISTERS, 0xfe, 1)) {
    loses +=1;
    return 0;           // Ошибка отправки команды
  }
  while(modbus.available()) {
    speedOfModbus = modbus.read();
  }
  //небольшая задержка между чтениями
  previousMillis = millis();
  while(millis() - previousMillis <1000){};
  //считываем данные
  if (!modbus.requestFrom(1, HOLDING_REGISTERS, 0x90, 1)) {
    loses +=1;
    return 0;           // Ошибка отправки команды
  }
  while(modbus.available()) {
    value = modbus.read();
  }
  return 1;
}

void displayStart(void){
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Start");
  display.display();
  display.drawString(0, 12, "Registers = ");
  display.display();
  display.drawString(0, 24, String(slaveID));  
  display.drawString(15, 24, String(speedOfModbus));  
  display.drawString(30, 24, String(value));  
  display.drawString(50, 24, String(allCount-loses));  
  display.display();
}

void onEvent(){
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
      if(readSensor()==1){
			  LoRaWAN.displaySending();
			  prepareTxFrame( appPort );
			  LoRaWAN.send();
      }
      loses =0;
      allCount=0;
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

