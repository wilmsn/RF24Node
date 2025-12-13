
/**
 * @file Node.ino
 * @brief A generic Arduino Program / Template to build the Software für ATMega Nodes
 * 
 * A Node for the rf24hub.
 * Can be configurated to you needs
 * 
 * Build in Parts (cut and pasteable): see page Parts.txt
 * 
 * On Branch: RF24Node  => entw  !!!!!
 * 
 */
//****************************************************
// Select / enable your in "Node_settings" defined Nodes here
// Select only one at one time !!!!
// Node 200
//#define AUSSENTHERMOMETER1
// Node 201
//#define AUSSENTHERMOMETER2
// Node 100
//#define BASTELZIMMERTHERMOMETER
// Node 101
//#define NODE_101
// Node 102
//#define SCHLAFZIMMERTHERMOMETER
// Node 103
//#define KUECHENTHERMOMETER
// Node 104
#define GAESTEZIMMERTHERMOMETER
// Node 105
//#define BASTELZIMMERTHERMOMETER_SW
// Node 106
//#define KUGELNODE1
// Node 107
//#define KUGELNODE2
// Node 108
//#define MW_WOHNZIMMER
// Node 110
//#define ANKLEIDEZIMMERTHERMOMETER    
// Node 111
//#define GASZAEHLERNODE
//----Testnodes-----
// Node 198
//#define ZAEHLER_TEST_198
// Node 240
//#define TESTNODE_240
// ------ End of configuration part ------------
//****************************************************
// Default settings and settings for the individual nodes are in "Node_settings.h"
#include "Node_settings.h"
//-----------------------------------------------------
//*****************************************************
#ifndef RF24NODE
#error "Node nicht definiert"
#endif
#include <avr/pgmspace.h>
#include <SPI.h>
#include <sleeplib.h>
#include <Vcc.h>
#include <EEPROM.h>
#include "dataformat.h"
#include "version.h"
#include "config.h"
#include "rf24_config.h"
#include <nRF24L01.h>
#include <RF24.h>

#if defined(DEBUG_SERIAL)
#include "printf.h"
#endif

#if defined(DISPLAY_5110)
#include "LCD5110.h"
#endif

#if defined(DEBUG_DISPLAY_5110)
#include "LCD5110_Logwriter.h"
#endif

#if defined(SENSOR_18B20)
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#if defined(SENSOR_BOSCH)
#include <BMX_sensor.h>
#endif

#if defined(SENSOR_HTU2X)
#include <htu2x_sensor.h>
#endif

#if defined(SENSOR_AHT20)
#include <aht20_sensor.h>
#endif

// ----- End of Includes ------------------------

Vcc vcc(1.0);
float u_ref;

ISR(WDT_vect) { watchdogEvent(); }

#if defined(DEBUG_DISPLAY)
LCD5110 lcd(N5110_RST,N5110_CE,N5110_DC,N5110_DIN,N5110_CLK);
#endif

#if defined(DISPLAY_ALL)
bool displayIsSleeping = false;
#if defined(DISPLAY_5110)
LCD5110 lcd(N5110_RST,N5110_CE,N5110_DC,N5110_DIN,N5110_CLK);
#endif
#endif

#if defined(SENSOR_DUMMY)
float temp_dummy;
#endif

#if defined(SENSOR_18B20)
OneWire oneWire(SENSOR_18B20); 
DallasTemperature s_18b20(&oneWire);
DeviceAddress sensorAddress;
float temp_18b20;
#endif

#if defined(SENSOR_BOSCH)
BMX_SENSOR bosch;
float temp_bosch, pres_bosch, humi_bosch;
#endif

#if defined(SENSOR_HTU2X)
HTU2X htu2x;
float temp_htu2x, humi_htu2x;
#endif

#if defined(SENSOR_AHT20)
AHT20 aht20;
float temp_aht20, humi_aht20;
#endif

#if defined (ZAEHLER)
float          zaehlerF;
uint32_t       zaehlerUI;
uint8_t        statusStack;
unsigned long  sumSleepTime_ms;
#endif

payload_t r_payload, s_payload;
uint8_t     payloadNo = 0;

uint8_t     rf24_node2hub[] = RF24_NODE2HUB;
uint8_t     rf24_hub2node[] = RF24_HUB2NODE;

struct eeprom_t {
   uint16_t versionnumber;
   uint16_t senddelay;
   float    volt_fac;
   float    volt_off;
   float    volt_lv;
   uint16_t sleeptime_lv;
   uint16_t sleeptime;
   uint16_t sleep4ms_fac;
   uint16_t emptyloops;
   uint8_t  max_sendcount;
   uint8_t  max_stopcount;
#if defined(DISPLAY_ALL)
   uint8_t  contrast;
   uint8_t  brightnes;
#endif
   uint8_t  pa_level;
};
eeprom_t eeprom;

boolean             low_voltage_flag = false;
boolean             exec_pingTest = false;
boolean             exec_RegTrans = false;
boolean             set_default = false;
float               cur_voltage;
float               vcc_mess;
uint16_t            loopcount;
uint16_t            max_loopcount;
int                 sleeptime_kor;
uint8_t             last_orderno = 0;
uint8_t             msg_id = 0;
uint8_t             heartbeatno=0;
boolean             monitormode = false;
unsigned long       tempsleeptime_ms;

//Some Var for restore after sleep of display
#if defined(DISPLAY_ALL)
#if defined(DISPLAY_LAYOUT_TEMPHUMI)
#endif
#if defined(DISPLAY_LAYOUT_DEFAULT)
float               field1_val, field2_val, field3_val, field4_val;
bool                field1_set = false;
bool                field2_set = false;
bool                field3_set = false;
bool                field4_set = false;
#endif
boolean             display_on = true;
#endif


// nRF24L01(+) radio attached using Getting Started board 
// Usage: radio(CE_pin, CS_pin)
RF24 radio(RADIO_CE_PIN,RADIO_CSN_PIN);

void get_sensordata(void) {
// Read Voltage and calculate battery
  u_ref = vcc.Read_Volts();
#if defined(BATTERY_READ_EXTERNAL)
  vcc_mess = (float)analogRead(BATTERY_READ_EXTERNAL) / 1024.0 * u_ref
              * (float)(BATTERY_VOLTAGEDIVIDER_R1 + BATTERY_VOLTAGEDIVIDER_R2) / (float)BATTERY_VOLTAGEDIVIDER_R2;
#else
  vcc_mess = u_ref;
#endif
  cur_voltage = ( vcc_mess * eeprom.volt_fac ) + eeprom.volt_off;
  low_voltage_flag = (eeprom.volt_lv > 1.5) && (cur_voltage <= eeprom.volt_lv);
#if defined(DEBUG_SERIAL_SENSOR)
   Serial.print("Volt (gemessen): ");
   Serial.println(vcc_mess);
   Serial.print("Volt Faktor: ");
   Serial.println(eeprom.volt_fac);
   Serial.print("Volt Offset: ");
   Serial.println(eeprom.volt_off);
   Serial.print("Volt: ");
   Serial.println(cur_voltage);
   Serial.print("Low Voltage Flag: ");
   if ( low_voltage_flag ) {       
     Serial.println("set");
   } else {
     Serial.println("not set");
   }
#endif  
// Ende Voltage

// Sensor Dummy
#if defined(SENSOR_DUMMY)
    temp_dummy=DUMMY_TEMP;
#define DISPLAY_TEMP temp_dummy
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.println("Dummysensor");
    Serial.print("Temp: ");
    Serial.print(temp_dummy);
#endif
#endif
// ENDE: Sensor Dummy

// Sensor Dallas 18B20
#if defined(SENSOR_18B20)
  s_18b20.requestTemperatures(); // Send the command to get temperatures
  SLEEPTYPE(SENSOR_18B20_DELAYTIME);
  delay(2);
  temp_18b20=s_18b20.getTempCByIndex(0);
#define DISPLAY_TEMP temp_18b20
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.println("18B20");
    Serial.print("Temp: ");
    Serial.print(temp_18b20);
#endif
#endif
// ENDE: Sensor Dallas 18B20

// Sensor Bosch BMP180; BMP280; BME280
#if defined(SENSOR_BOSCH)
#if defined(DEBUG_SERIAL_SENSOR)
  if (bosch.isBMP180()) Serial.println("BMP180");
  if (bosch.isBMP280()) Serial.println("BMP280");
  if (bosch.isBME280()) Serial.println("BME280"); 
#endif
  bosch.startSingleMeasure();
  if (bosch.hasTemperature() ) {
    temp_bosch = bosch.getTemperature();
#define DISPLAY_TEMP temp_bosch
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("Temp: ");
    Serial.println(temp_bosch);
#endif
  }
  if (bosch.hasPressure() ) {
    pres_bosch = bosch.getPressureAtSealevel(ALTITUDEABOVESEALEVEL);
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("Pres: ");
    Serial.println(pres_bosch);
#endif
  }
  if (bosch.hasHumidity() )  {
    humi_bosch = bosch.getHumidity();
#define DISPLAY_HUMI humi_bosch
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("Humi: ");
    Serial.println(humi_bosch);
#endif
  }
#endif
// ENDE: Sensor Bosch BMP180; BMP280; BME280

// Sensor HTU2X
#if defined(SENSOR_HTU2X)
  htu2x.startSingleMeasure();
  temp_htu2x = htu2x.getTemperature();
#define DISPLAY_TEMP temp_htu2x
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.println("HTU2X");
    Serial.print("Temp: ");
    Serial.println(temp_htu2x);
#endif
  humi_htu2x = htu2x.getHumidity();
#define DISPLAY_HUMI humi_htu2x
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("Humi: ");
    Serial.println(humi_htu2x);
#endif
#endif
// ENDE: Sensor HTU2X

// Sensor AHT20
#if defined(SENSOR_AHT20)
  aht20.startSingleMeasure();
  temp_aht20 = aht20.getTemperature();
#define DISPLAY_TEMP temp_aht20
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.println("AHT20");
    Serial.print("Temp: ");
    Serial.println(temp_aht20);
#endif
  humi_aht20 = aht20.getHumidity();
#define DISPLAY_HUMI humi_aht20
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("Humi: ");
    Serial.println(humi_aht20);
#endif
#endif
// ENDE: Sensor AHT20
}

uint32_t action_loop(uint32_t data) {
  uint32_t retval = 0;
  int      intval;
  uint8_t  channel = getChannel(data);
#if defined(DEBUG_SERIAL_PROC)
    Serial.print("Processing: Channel: ");
    Serial.println(getChannel(data));
#endif
    switch (channel) {
#if defined(DISPLAY_ALL)
#if defined(DISPLAY_LAYOUT_DEFAULT)
      case 21:
      {
        // Set field 1
        getValue(data, &field1_val);
        field1_set = true;
#if defined(DEBUG_SERIAL_DISPLAY)
        Serial.print("Data Field1: ");
        Serial.println(field1_val);
#endif
      }
      break;
      case 22:
      {
        // Set field 2
        getValue(data, &field2_val);
        field2_set = true;;
#if defined(DEBUG_SERIAL_DISPLAY)
        Serial.print("Data Field2: ");
        Serial.println(field2_val);
#endif
      }
      break;
      case 23:
      {
        // Set field 3
        getValue(data, &field3_val);
        field3_set = true;;
#if defined(DEBUG_SERIAL_DISPLAY)
        Serial.print("Data Field3: ");
        Serial.println(field3_val);
#endif
      }
      break;
      case 24:
      {
        // Set field 4
        getValue(data, &field4_val);
        field4_set = true;;
#if defined(DEBUG_SERIAL_DISPLAY)
        Serial.print("Data Field4: ");
        Serial.println(field4_val);
#endif
      }
      break;
#endif
      case 51:
      {
        uint16_t val;
        getValue(data, &val);
        // Displaylight ON <-> OFF
        if ( val & 0x01 ) {
          digitalWrite(STATUSLED,STATUSLED_ON);
        } else  {
          digitalWrite(STATUSLED,STATUSLED_OFF);
        }
      }
      break;
      case 52:
      {
        // Display Sleepmode ON <-> OFF
        uint16_t val;
        getValue(data, &val);
        display_sleep( val == 0x00 );
      }
      break;
#endif
#if defined (ZAEHLER)
      case ZAEHLER_LO_SET_CHANNEL:
      {
        uint16_t val;
        getValue(data, &val);
        zaehlerUI = zaehlerUI & 0xffff0000;
        zaehlerUI = zaehlerUI | val;
        zaehlerF = (float)zaehlerUI / 100;
      }
      break;
      case ZAEHLER_HI_SET_CHANNEL:
      {
        uint16_t val;
        uint32_t val1;
        getValue(data, &val);
        zaehlerUI = zaehlerUI & 0x0000ffff;
        val1 = val;
        zaehlerUI = zaehlerUI | (val1 << 16);
        zaehlerF = (float)zaehlerUI / 100;
      }
      break;
#endif
      case REG_TRANSREG:
      {
        exec_RegTrans = true;
      }
      break;
      case REG_TRANSPA:
      {
        exec_pingTest = true;
      }
      break;
      case REG_DEFAULT:
      {
        set_default = true;
      }
      case REG_SLEEPTIME:
      {
        // sleeptime in sec!
        uint16_t val;
        getValue(data, &val);
        if ( val >= 10 && val <= 32400) {
          eeprom.sleeptime = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_SLEEP4MS_FAC:
      {
        // sleeptime adjust in sec!
        int16_t val;
        getValue(data, &val);
        if (val >= 500 && val <= 2000) {
          eeprom.sleep4ms_fac = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_EMPTYLOOPS:
      {
        // emptyloops - number of loops without sending to hub / messure and display only!
        uint16_t val;
        getValue(data, &val);
        if (val < 21) {
          eeprom.emptyloops=(uint8_t)val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_SLEEPTIME_KOR:
      {
        // sleeptime_kor: onetime adjust of sleeptime, will be reset to 0 after use
        int16_t val;
        getValue(data, &val);
        if (val > -1001 && val < 1001) {
          sleeptime_kor = val;
        }
      }
      break;
      case REG_SENDDELAY:
      {
        // senddelay in millisec.
        uint16_t val;
        getValue(data, &val);
        if (val > 49 && val < 1001) {
          eeprom.senddelay = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_MAX_SENDCOUNT:
      {
        // max_sendcount: numbers of attempts to send for normal messages
        uint16_t val;
        getValue(data, &val);
        if (val > 0 && val < 21) {
          eeprom.max_sendcount = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_MAX_STOPCOUNT:
      {
      // max_stopcount: numbers of attempts to send for stop messages
        uint16_t val;
        getValue(data, &val);
        if (val > 0 && val < 21) {
          eeprom.max_stopcount = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_VOLT_FAC:
      {
        // Volt_fac - V = Vmess * Volt_fac
        float val;
        getValue(data, &val);
        if (val >= 0.1 && val <= 10) {
          eeprom.volt_fac = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_VOLT_OFF:
      {
        // Volt_off - V = (Vmess * Volt_fac) + Volt_off
        float val;
        getValue(data, &val);
        if (val >= -10 && val <= 10) {
          eeprom.volt_off = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_VOLT_LV:
      {
        // Low Voltage Level
        float val;
        getValue(data, &val);
        if (val >= 1 && val <= 5) {
          eeprom.volt_lv = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_SLEEPTIME_LV:
      {
        // Low Voltage send interval
        uint16_t val;
        getValue(data, &val);
        if ( val > 9 && val < 32401) {
          eeprom.sleeptime_lv = val;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_PALEVEL:
      {
        // PA Level
        uint16_t val;
        getValue(data, &val);
        if (val > 0 && val < 5) {
          eeprom.pa_level = val;
          radio.setPALevel( eeprom.pa_level) ;
          EEPROM.put(0, eeprom);
        }
      }
      break;
      case REG_SW:
      {
        uint16_t swversion = SWVERSION;
        data = calcTransportValue(REG_SW, swversion);
      }
      break;
#if defined(DISPLAY_ALL)
      case REG_CONTRAST:
      {
        uint16_t val;
        getValue(data, &val);
        if ( val >= 0 && val <= 100 ) {
          eeprom.contrast = val;
          EEPROM.put(0, eeprom);
#if defined(DISPLAY_5110)
          lcd.setContrast(val);          
#endif
        }
        data = calcTransportValue(REG_CONTRAST, eeprom.contrast);
      }
      break;
      case REG_BRIGHTNES:
      {
        uint16_t val;
        getValue(data, &val);
        if ( val >= 0 && val <= 100 ) {
          eeprom.brightnes = val;
          EEPROM.put(0, eeprom);
        }
        data = calcTransportValue(REG_BRIGHTNES, eeprom.brightnes);
      }
      break;
#endif
    }
    return data;
}

void init_eeprom(bool reset_eeprom) {
// EEPROM_VERSION == 0 disables EEPROM !!!!
  EEPROM.get(0, eeprom);
  if (eeprom.versionnumber != EEPROM_VERSION || EEPROM_VERSION == 0 || reset_eeprom) {
    eeprom.versionnumber    = EEPROM_VERSION;
#if defined(DISPLAY_ALL)
    eeprom.brightnes        = BRIGHTNES;
    eeprom.contrast         = CONTRAST;
#endif
    eeprom.sleeptime        = SLEEPTIME;
    eeprom.sleep4ms_fac     = SLEEP4MS_FAC;
    eeprom.emptyloops       = EMPTYLOOPS;
    eeprom.senddelay        = SENDDELAY;
    eeprom.max_sendcount    = MAX_SENDCOUNT;
    eeprom.max_stopcount    = MAX_STOPCOUNT;
    eeprom.volt_fac         = VOLT_FAC;
    eeprom.volt_off         = VOLT_OFF;
    eeprom.volt_lv          = VOLT_LV;
    eeprom.pa_level         = PA_LEVEL;
    eeprom.sleeptime_lv     = SLEEPTIME_LV;
    if (EEPROM_VERSION > 0) EEPROM.put(0, eeprom);
  }
}

void setup(void) {
  delay(50);
#if defined(DEBUG_SERIAL)
  Serial.begin(115200);
  printf_begin();
  Serial.println("Programmstart");
#endif
  
#if defined(STATUSLED)
  pinMode(STATUSLED, OUTPUT);
  digitalWrite(STATUSLED,STATUSLED_ON);
#endif

#if defined (ZAEHLER)
  pinMode(ZAEHLER_PIN, INPUT_PULLUP);
#if defined (ZAEHLER_START)
  zaehlerUI = ZAEHLER_START;
  zaehlerF = ZAEHLER_START / 100.0;
#else
  zaehlerF = 0;
  zaehlerUI = 0;
#endif
  sumSleepTime_ms = 0;
  statusStack = 0;
#endif

// Init EEPROM
  init_eeprom(false);

  SPI.begin();

#if defined(SENSOR_18B20)
  s_18b20.begin();
  s_18b20.setWaitForConversion(false);
  for(byte i=0; i<s_18b20.getDeviceCount(); i++) {
      if(s_18b20.getAddress(sensorAddress, i)) {
        s_18b20.setResolution(sensorAddress, SENSOR_18B20_RESOLUTION);
      }
    }
#endif

#if defined(SENSOR_BOSCH)
  bosch.begin();
#endif

#if defined(SENSOR_AHT20)
  aht20.begin();
#endif

#if defined(SENSOR_HTU2X)
  htu2x.begin();
#endif

  radio.begin();
  delay(100);
  radio.setChannel(RF24_CHANNEL);
  radio.setDataRate(RF24_SPEED);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(0, 0);
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(rf24_node2hub);
  radio.openReadingPipe(1,rf24_hub2node);
  delay(100);
#if defined(DEBUG_SERIAL_RADIO_DETAILS)
  radio.printDetails();
#endif

  delay(100);

#if defined(DEBUG_DISPLAY_5110)
  lcd.begin();
  lcd.clear();
#endif

#if defined(DISPLAY_ALL)
#if defined(DISPLAY_5110)
  lcd.begin();
//  lcd.setContrast(eeprom.contrast);
  lcd.setContrast(40);
  lcd.setFont(LCD5110::small);
  lcd.clear();
  lcd.println();
  lcd.print("Node: ");
  lcd.println(RF24NODE);
  lcd.println();
  lcd.println("    SW:");
  lcd.println(SWVERSIONSTR);
  lcd.draw();
#if defined(MONITOR)
  monitor(1000);
#endif
#endif //DISPLAY_5110
#endif //DISPLAY_ALL
// on init send config to hub
  pingTest();
  sendRegister();
  loopcount = 0;
#if defined(STATUSLED)
  digitalWrite(STATUSLED,STATUSLED_OFF);
#endif
#if defined(DEBUG_SERIAL)
  Serial.println("Ende setup()");
#endif
}

// Start of DISPLAY_ALL Block
#if defined(DISPLAY_ALL)

void monitor(uint32_t delaytime) {
#if defined(MONITOR)
  const char string_0[] PROGMEM = "SW Version: ";
  const char string_1[] PROGMEM = "Temp: ";
  const char string_2[] PROGMEM = "Ubatt: ";
  const char string_3[] PROGMEM = "Loops: ";
  const char string_4[] PROGMEM = "Send: ";
  const char string_5[] PROGMEM = "RF24 Nw: ";
  const char string_6[] PROGMEM = "Node: ";
  const char string_7[] PROGMEM = "Channel: ";
  const char string_8[] PROGMEM = "Kontrast: ";
#if defined(DISPLAY_5110)
  lcd.setFont(LCD5110::small);
  lcd.clear();
  lcd.println();
  lcd.println(string_0);
  lcd.println();
  lcd.print("    ");
  lcd.print(SWVERSION);
  lcd.draw();
  SLEEPTYPE(delaytime);
  delay(1);
  lcd.clear();
  get_sensordata();
  lcd.print(string_1);
  lcd.println(temp,1);
  get_voltage();
  lcd.print(string_2);
  lcd.print(cur_voltage,1);
  lcd.print("/");
  lcd.println(vcc.Read_Volts(),1);
  lcd.print(string_3);
  lcd.print(eeprom.sleeptime_sec);
  lcd.print("/");
  lcd.println(eeprom.emptyloops);
  lcd.print(string_4);
  lcd.print(eeprom.senddelay);
  lcd.print("/");
  lcd.print(eeprom.max_sendcount);
  lcd.print("/");
  lcd.print(eeprom.max_stopcount);
  lcd.draw();
  SLEEPTYPE(delaytime);
  delay(1);
  lcd.clear();
  lcd.println(string_5);
  lcd.print(string_6);
  lcd.println(RF24NODE);
  lcd.print(string_7);
  lcd.print(RF24_CHANNEL);
  lcd.draw();
  SLEEPTYPE(delaytime);
  lcd.clear();
  lcd.print(string_8);
  lcd.print(eeprom.contrast);
  lcd.setFont(LCD5110::big);
  for (int i=0; i<100; i+=5) {
    lcd.setCursor(25,20);
    lcd.print(i);
    lcd.setContrast(i);
    lcd.draw();
    delay(300);
  }
  SLEEPTYPE(5000);
  lcd.setContrast(eeprom.contrast);
  delay(1);
  lcd.clear();
#endif
#endif

} // END monitor()

void display_fill() {
  if ( ! low_voltage_flag ) {
    draw_battery(cur_voltage);
    draw_hb_countdown((uint8_t) 8 * (1- ((float)loopcount / eeprom.emptyloops)) );
    draw_temp(DISPLAY_TEMP);
#if defined(DISPLAY_LAYOUT_TEMPHUMI)
    draw_humi(DISPLAY_HUMI);
#endif
#if defined(DISPLAY_LAYOUT_DEFAULT)
    print_field(field1_set,field1_val,1);
    print_field(field2_set,field2_val,2);
    print_field(field3_set,field3_val,3);
    print_field(field4_set,field4_val,4);
#endif
  }
}

void display_sleep(boolean displayGotoSleep) {
  if ( displayIsSleeping != displayGotoSleep ) {
    if ( displayGotoSleep ) { // Display go to sleep
#if defined(DISPLAY_5110)
      lcd.off();
#endif
      displayIsSleeping = true;
    } else {
      if ( ! low_voltage_flag ) {
#if defined(DISPLAY_5110)
        lcd.on();
#endif
        displayIsSleeping = false;
        display_fill();
      }
    }
  }
}

void draw_hb_countdown(uint8_t watermark) {
  if ( display_on ) {
    if ( watermark > 8 ) watermark = 8;
#if defined(DISPLAY_5110)
    lcd.drawRect(HB_X0,HB_Y0,4,8,false,true,false);
    lcd.drawRect(HB_X0, HB_Y0 + 8 - watermark, 4, watermark, true, true, true);
#endif
  }
}

void draw_therm() {
// Ein Thermometersymbol 4*8 Pixel
  if ( display_on ) {
#if defined(DISPLAY_5110)
    lcd.drawRect(THERM_X0+1,THERM_Y0,1,3);
    lcd.drawRect(THERM_X0,THERM_Y0+4,3,2);
#endif
  }
}

void draw_temp(float t) {
  if ( display_on ) {
#if defined(DISPLAY_5110)
    lcd.setFont(LCD5110::big);
    lcd.setCursor(0,0);
    if ((t >= 0) && (t < 10)) {
      lcd.print(" ");
    }
    lcd.print(t,1);
    lcd.print("*");
#endif
  }
}

void draw_humi(float h) {
  if ( display_on ) {
#if defined(DISPLAY_5110)
    lcd.setFont(LCD5110::medium);
    lcd.setCursor(50,30);
    lcd.print(h,0);
    lcd.print("%");
#endif
  }
}

void print_field(bool isSet, float val, int field) {
  uint8_t x0, y0, x1, y1;
  x1=41;
  y1=10;
  switch (field) {
    case 1: x0=0; y0=27;  break;
    case 2: x0=42; y0=27; break;
    case 3: x0=0; y0=37;  break;
    case 4: x0=42; y0=37; break;
  }
#if defined(DISPLAY_5110)
  lcd.drawRect(x0,y0,x1,y1,true,true,false);
  lcd.setFont(LCD5110::small);
  lcd.setCursor(x0+7,y0+2);
#endif
  if(isSet) {
    if ( val > 99.9 ) {
      if (val > 999) {
#if defined(DISPLAY_5110)
        lcd.print(val,0);
#endif
      } else {
#if defined(DISPLAY_5110)
        lcd.print(" ");
        lcd.print(val,0);
#endif   
      }
    } else {
      if (val >= 10) {
#if defined(DISPLAY_5110)
        lcd.print(val,1);
#endif
      } else {
#if defined(DISPLAY_5110)
        lcd.print(val,2);
#endif
      }
    }
  }
}

void draw_battery(float u) {
  if ( display_on ) {
#if defined(DISPLAY_5110)
    BATT_X0,BATT_Y0,
// Das Batteriesymbol ist 10*5 Pixel
    // Clear the drawing field
    //lcd.clrRect(x,y,9,5);
    // Drawing a symbol of an battery
    // Size: 10x5 pixel
    // at position x and y
    lcd.drawRect(BATT_X0+2,BATT_Y0,7,4,true,true,false);
    lcd.drawRect(BATT_X0,BATT_Y0+1,1,2,true,true,false);
    if ( u > U1 ) {
      lcd.drawRect(BATT_X0+8,BATT_Y0+1,1,2,true,true,true);
    } else {
      lcd.setPixel(BATT_X0+3,BATT_Y0);
      lcd.setPixel(BATT_X0+4,BATT_Y0+1);
      lcd.setPixel(BATT_X0+5,BATT_Y0+2);
      lcd.setPixel(BATT_X0+6,BATT_Y0+3);
      lcd.setPixel(BATT_X0+7,BATT_Y0+4);
    }
    if ( u > U2 ) lcd.drawRect(BATT_X0+6,BATT_Y0+1,1,2,true,true,true);
    if ( u > U3 ) lcd.drawRect(BATT_X0+4,BATT_Y0+1,1,2,true,true,true);
    if ( u > U4 ) lcd.drawRect(BATT_X0+2,BATT_Y0+1,1,2,true,true,true);
#endif
  }
}

void draw_antenna() {
  if ( display_on ) {
#if defined(DISPLAY_5110)
    // Drawing a symbol of an antenna
    // Size: 10x10 pixel
    // at position x and y
    lcd.setPixel(ANT_X0+7,ANT_Y0+0);
    lcd.setPixel(ANT_X0+1,ANT_Y0+1);
    lcd.setPixel(ANT_X0+8,ANT_Y0+1);
    lcd.setPixel(ANT_X0+0,ANT_Y0+2);
    lcd.setPixel(ANT_X0+3,ANT_Y0+2);
    lcd.setPixel(ANT_X0+6,ANT_Y0+2);
    lcd.setPixel(ANT_X0+9,ANT_Y0+2);
    lcd.setPixel(ANT_X0+0,ANT_Y0+3);
    lcd.setPixel(ANT_X0+2,ANT_Y0+3);
    lcd.setPixel(ANT_X0+7,ANT_Y0+3);
    lcd.setPixel(ANT_X0+9,ANT_Y0+3);
    lcd.setPixel(ANT_X0+0,ANT_Y0+4);
    lcd.setPixel(ANT_X0+2,ANT_Y0+4);
    lcd.setPixel(ANT_X0+4,ANT_Y0+4);
    lcd.setPixel(ANT_X0+5,ANT_Y0+4);
    lcd.setPixel(ANT_X0+7,ANT_Y0+4);
    lcd.setPixel(ANT_X0+9,ANT_Y0+4);
    lcd.setPixel(ANT_X0+0,ANT_Y0+5);
    lcd.setPixel(ANT_X0+2,ANT_Y0+5);
    lcd.setPixel(ANT_X0+4,ANT_Y0+5);
    lcd.setPixel(ANT_X0+5,ANT_Y0+5);
    lcd.setPixel(ANT_X0+7,ANT_Y0+5);
    lcd.setPixel(ANT_X0+9,ANT_Y0+5);
    lcd.setPixel(ANT_X0+0,ANT_Y0+6);
    lcd.setPixel(ANT_X0+3,ANT_Y0+6);
    lcd.setPixel(ANT_X0+4,ANT_Y0+6);
    lcd.setPixel(ANT_X0+5,ANT_Y0+6);
    lcd.setPixel(ANT_X0+6,ANT_Y0+6);
    lcd.setPixel(ANT_X0+9,ANT_Y0+6);
    lcd.setPixel(ANT_X0+1,ANT_Y0+7);
    lcd.setPixel(ANT_X0+4,ANT_Y0+7);
    lcd.setPixel(ANT_X0+5,ANT_Y0+7);
    lcd.setPixel(ANT_X0+8,ANT_Y0+7);
    lcd.setPixel(ANT_X0+4,ANT_Y0+8);
    lcd.setPixel(ANT_X0+5,ANT_Y0+8);
    lcd.setPixel(ANT_X0+4,ANT_Y0+9);
    lcd.setPixel(ANT_X0+5,ANT_Y0+9);
#endif
  }
}

void display_clear() {
#if defined(DISPLAY_5110)
    lcd.clear();
#endif
}

void display_refresh() {
#if defined(DISPLAY_5110)
    lcd.draw();
#endif
}
#endif //DISPLAY_ALL
// End of DISPLAY_ALL Block

void payloadInitData(void) {
  s_payload.data1 = 0;
  s_payload.data2 = 0;
  s_payload.data3 = 0;
  s_payload.data4 = 0;
  s_payload.data5 = 0;
  s_payload.data6 = 0;
}

#if defined(DEBUG_SERIAL_RADIO_RXTX)
void printPayloadData(uint32_t pldata) {
    char buf[20];
    Serial.print("(");
    Serial.print(getChannel(pldata));
    Serial.print("/");
    Serial.print(unpackTransportValue(pldata,buf));
    Serial.print(")");
}

void printPayload(payload_t* pl) {
    Serial.print(" N:");
    Serial.print(pl->node_id);
    Serial.print(" I:");
    Serial.print(pl->msg_id);
    Serial.print(" T:");
    Serial.print(pl->msg_type);
    Serial.print(" O:");
    Serial.print(pl->orderno);
    Serial.print(" ");
    printPayloadData(pl->data1);
    printPayloadData(pl->data2);
    printPayloadData(pl->data3);
    printPayloadData(pl->data4);
    printPayloadData(pl->data5);
    printPayloadData(pl->data6);
    Serial.println();
}
#endif

uint8_t mk_flags(bool last_msg) {
  uint8_t retval = PAYLOAD_FLAG_EMPTY;
  if (low_voltage_flag) retval |= PAYLOAD_FLAG_LOWVOLTAGE;
  if (last_msg) retval |= PAYLOAD_FLAG_LASTMESSAGE;
  return retval;
}

void payload_data(uint8_t* pos, uint8_t channel, float value) {
  if ( (*pos) == 7 ) {
    do_transmit(eeprom.max_sendcount, payloadNo == 0 ? PAYLOAD_TYPE_HB : PAYLOAD_TYPE_HB_F, mk_flags(false), payloadNo, heartbeatno);
// Hub needs some time to prcess data !!!
    delay(500);
    payloadInitData();
    (*pos) = 1;
    payloadNo++;
  }
  switch (*pos) {
    case 1:
      s_payload.data1 = calcTransportValue(channel, value);
    break;
    case 2:
      s_payload.data2 = calcTransportValue(channel, value);
    break;
    case 3:
      s_payload.data3 = calcTransportValue(channel, value);
    break;
    case 4:
      s_payload.data4 = calcTransportValue(channel, value);
    break;
    case 5:
      s_payload.data5 = calcTransportValue(channel, value);
    break;
    case 6:
      s_payload.data6 = calcTransportValue(channel, value);
    break;
  }
  (*pos)++;
}

/*
 * Ping Test durchführen
 * Ergebnis ist die geringste Sendeleistung die vom Hub erkannt wurde
 */

void pingTest(void) {
  radio.setPALevel( RF24_PA_MAX) ;
  do_transmit(3, PAYLOAD_TYPE_PING_POW_MAX, mk_flags(false), 0, 251);
  radio.setPALevel( RF24_PA_HIGH) ;
  do_transmit(3, PAYLOAD_TYPE_PING_POW_HIGH, mk_flags(false), 0, 252);
  radio.setPALevel( RF24_PA_LOW) ;
  do_transmit(3, PAYLOAD_TYPE_PING_POW_LOW, mk_flags(false), 0, 253);
  radio.setPALevel( RF24_PA_MIN) ;
  do_transmit(3, PAYLOAD_TYPE_PING_POW_MIN, mk_flags(false), 0, 254);
  radio.setPALevel( RF24_PA_MAX) ;
  do_transmit(3, PAYLOAD_TYPE_PING_END, mk_flags(false), 0, 255);
  radio.setPALevel( eeprom.pa_level ); 
  exec_pingTest = false;
}

void sendRegister(uint32_t data, uint8_t* pos, uint8_t* hbno) {
  if ((*pos) == 1) payloadInitData();
  switch (*pos) {
    case 1: s_payload.data1 = data;  break;
    case 2: s_payload.data2 = data;  break;
    case 3: s_payload.data3 = data;  break;
    case 4: s_payload.data4 = data;  break;
    case 5: s_payload.data5 = data;  break;
    case 6: s_payload.data6 = data;  break;
  }
  (*pos)++;
  if ((*pos) == 7) {
    do_transmit(3,PAYLOAD_TYPE_INIT,mk_flags(false),0, *hbno);
    (*hbno)++;
    (*pos) = 1;
// Hub needs some time to prcess data !!!
    delay(1000);
  }
}

void sendRegister(void) {
  uint8_t pos = 1;
  uint8_t hbno = 241;
  sendRegister(calcTransportValue(REG_VOLT_FAC, eeprom.volt_fac),&pos,&hbno);
  sendRegister(calcTransportValue(REG_VOLT_OFF, eeprom.volt_off),&pos,&hbno);
  sendRegister(calcTransportValue(REG_VOLT_LV, eeprom.volt_lv),&pos,&hbno);
  sendRegister(calcTransportValue(REG_SLEEP4MS_FAC, eeprom.sleep4ms_fac),&pos,&hbno);
  sendRegister(calcTransportValue(REG_SLEEPTIME_LV, eeprom.sleeptime_lv),&pos,&hbno);
  sendRegister(calcTransportValue(REG_SW, SWVERSION),&pos,&hbno);
  sendRegister(calcTransportValue(REG_SLEEPTIME, eeprom.sleeptime),&pos,&hbno);
  sendRegister(calcTransportValue(REG_EMPTYLOOPS, eeprom.emptyloops),&pos,&hbno);
  sendRegister(calcTransportValue(REG_SENDDELAY, eeprom.senddelay),&pos,&hbno);
  sendRegister(calcTransportValue(REG_MAX_SENDCOUNT, eeprom.max_sendcount),&pos,&hbno);
  sendRegister(calcTransportValue(REG_MAX_STOPCOUNT, eeprom.max_stopcount),&pos,&hbno);
  sendRegister(calcTransportValue(REG_PALEVEL, eeprom.pa_level),&pos,&hbno);
#if defined(DISPLAY_5110)
  sendRegister(calcTransportValue(REG_CONTRAST, eeprom.contrast),&pos,&hbno);
#endif
#if defined(DISPLAY_XXXX)
  sendRegister(calcTransportValue(REG_BRIGHTNES, eeprom.brightnes),&pos,&hbno);
#endif
  if ( pos > 1 ) do_transmit(3,PAYLOAD_TYPE_INIT,PAYLOAD_FLAG_EMPTY,0, hbno);
  exec_RegTrans = false;
}

void prep_data(uint8_t msg_type, uint8_t msg_flags, ONR_DATTYPE orderno, uint8_t myheartbeatno) {
  s_payload.node_id = RF24NODE;
  s_payload.msg_id = 0;
  s_payload.msg_type = msg_type;
  s_payload.msg_flags = msg_flags;
  s_payload.orderno = orderno;
  if (myheartbeatno > 0) {
    s_payload.heartbeatno = myheartbeatno;
  } else {
    s_payload.heartbeatno = heartbeatno;
  }
}

void do_transmit(uint8_t max_tx_loopcount, uint8_t msg_type, uint8_t msg_flags, ONR_DATTYPE orderno, uint8_t myheartbeatno) {
// ToDo: Verarbeitung des MSG_FLAGS !!!!
    unsigned long start_ts;
    uint8_t tx_loopcount = 0;
    bool doLoop = true;
    start_ts = millis();
    prep_data(msg_type, msg_flags, orderno, myheartbeatno);
    while ( tx_loopcount < max_tx_loopcount ) {
      s_payload.msg_id++;
      radio.stopListening();
      delay(1);
#if defined(DEBUG_SERIAL_RADIO_RXTX)
      Serial.print("TX: ");
      printPayload(&s_payload);
#endif
#if defined(DEBUG_STATUSLED_RADIO_TX)
      digitalWrite(STATUSLED, STATUSLED_ON);
#endif
      radio.write(&s_payload, sizeof(s_payload));
      delay(1);
      radio.startListening();
      delay(1);
      start_ts = millis();
      doLoop = true;
#if defined(DEBUG_SERIAL_RADIO)
      Serial.print("RX: ");
      Serial.println(start_ts);
#endif
      while ( (millis() < (start_ts + eeprom.senddelay) ) && doLoop ) {
#if defined(DEBUG_SERIAL_RADIO)
        unsigned long temp_ts = 0;
        if ( temp_ts + 100 < millis() ) {
          temp_ts = millis();
          Serial.print(".");
        }
#endif
        if ( radio.available() ) {
          radio.read(&r_payload, sizeof(r_payload));
#if defined(DEBUG_SERIAL_RADIO)
          Serial.println("Radio: got message");
#endif
#if defined(DEBUG_SERIAL_RADIO_RXTX)
          Serial.println();
          printPayload(&r_payload);
          if (r_payload.node_id != RF24NODE) {
            Serial.println("Wrong Node, Message dropped!");
          }
          if (r_payload.orderno == last_orderno) {
            Serial.println("Ordernumber already processed!");
          }
#endif
          if (r_payload.node_id == RF24NODE && r_payload.orderno != last_orderno) {
            last_orderno = r_payload.orderno;
            switch(r_payload.msg_type) {
              case PAYLOAD_TYPE_DAT:
                if (r_payload.data1 > 0) { s_payload.data1 = action_loop(r_payload.data1); } else { s_payload.data1 = 0; }
                if (r_payload.data2 > 0) { s_payload.data2 = action_loop(r_payload.data2); } else { s_payload.data2 = 0; }
                if (r_payload.data3 > 0) { s_payload.data3 = action_loop(r_payload.data3); } else { s_payload.data3 = 0; }
                if (r_payload.data4 > 0) { s_payload.data4 = action_loop(r_payload.data4); } else { s_payload.data4 = 0; }
                if (r_payload.data5 > 0) { s_payload.data5 = action_loop(r_payload.data5); } else { s_payload.data5 = 0; }
                if (r_payload.data6 > 0) { s_payload.data6 = action_loop(r_payload.data6); } else { s_payload.data6 = 0; }
                prep_data(PAYLOAD_TYPE_DATRESP,PAYLOAD_FLAG_LASTMESSAGE,r_payload.orderno, 0);
                tx_loopcount = 0;
                if (r_payload.msg_flags & PAYLOAD_FLAG_LASTMESSAGE ) {
                  // Wenn das LASTMESSAGEFLAG gesetzt ist sollte nur noch eine Endmessage kommen
                  // ==> Zeit wird verkürzt
                  max_tx_loopcount = eeprom.max_stopcount;
                } else {
                  max_tx_loopcount = eeprom.max_sendcount;
                }
                doLoop = false;
              break;
              case PAYLOAD_TYPE_HB_RESP:
              case PAYLOAD_TYPE_HB_F_RESP:
              case PAYLOAD_TYPE_DATSTOP:
                  tx_loopcount = max_tx_loopcount;
                  doLoop = false;
              break;
            }
          }
        }  //radio.available
      }
#if defined(DEBUG_STATUSLED_RADIO_TX)
      digitalWrite(STATUSLED, STATUSLED_OFF);
#endif
      tx_loopcount++;
    }
}

void exec_jobs(void) {
  // Test if there are some jobs to do
  if (set_default) {
    init_eeprom(true);
    exec_RegTrans = true;
  }
  if (exec_pingTest) {
    pingTest();
    delay(200);
  }
  if (exec_RegTrans) {
    sendRegister();
    delay(200);
  }
}

void loop(void) {
#if defined(ZAEHLER)
  if (loopcount == 0)
#endif
  get_sensordata();
  if ( loopcount == 0) {
#if defined(DEBUG_SERIAL_RADIO)
    delay(100);
    Serial.println("Loop: Radio wakeup");
#endif
    payloadNo = 0;
    heartbeatno++;
    if ( heartbeatno > 200 ) heartbeatno = 1;
    payloadInitData();
    radio.powerUp();
    delay(1);
    radio.startListening();
    radio.openReadingPipe(1,rf24_hub2node);
    delay(1);

    // Empty FiFo Buffer from old transmissions
    while ( radio.available() ) {
      radio.read(&r_payload, sizeof(r_payload));
      delay(10);
    }
    uint8_t pos=1;
    payload_data(&pos, SENSOR_BATT, cur_voltage);
#if defined(SENSOR_DUMMY)
    payload_data(&pos, TEMP_DUMMY_CHANNEL, temp_dummy);
#endif
#if defined(SENSOR_18B20)
    payload_data(&pos, TEMP_18B20_CHANNEL, temp_18b20);
#endif
#if defined(SENSOR_BOSCH)
    if ( bosch.hasTemperature() ) payload_data(&pos, TEMP_BOSCH_CHANNEL, temp_bosch);
    if ( bosch.hasPressure() )  payload_data(&pos, PRES_BOSCH_CHANNEL, pres_bosch);
    if ( bosch.hasHumidity() )  payload_data(&pos, HUMI_BOSCH_CHANNEL, humi_bosch);
#endif
#if defined(SENSOR_HTU2X)
    payload_data(&pos, TEMP_HTU2X_CHANNEL, temp_htu2x);
    payload_data(&pos, HUMI_HTU2X_CHANNEL, humi_htu2x);
#endif
#if defined(SENSOR_AHT20)
    payload_data(&pos, TEMP_AHT20_CHANNEL, temp_aht20);
    payload_data(&pos, HUMI_AHT20_CHANNEL, humi_aht20);
#endif
#if defined(ZAEHLER)
    payload_data(&pos,ZAEHLER_CHANNEL,zaehlerF);
    uint16_t val;
    val = zaehlerUI;
    payload_data(&pos,ZAEHLER_LO_CHANNEL,val);
    val = zaehlerUI >> 16;
    payload_data(&pos,ZAEHLER_HI_CHANNEL,val);
#if defined(DEBUG_SERIAL_SENSOR)
    Serial.print("zaehler: ");
    Serial.print("Rohdaten: ");
    Serial.print(zaehlerUI);
    Serial.print("aufbereitet: ");
    Serial.println(zaehlerF);
#endif
#endif
#if defined(DEBUG_SERIAL_RADIO)
    Serial.println("Loop: Radio start transmit");
#endif
#if defined(DEBUG_STATUSLED_RADIO)
    digitalWrite(STATUSLED, STATUSLED_ON);
#endif
    do_transmit(eeprom.max_sendcount, payloadNo == 0 ? PAYLOAD_TYPE_HB : PAYLOAD_TYPE_HB_F, mk_flags(true), 0, 0);
    exec_jobs();
#if defined(DEBUG_STATUSLED_RADIO)
    digitalWrite(STATUSLED, STATUSLED_OFF);
#endif
#if defined(DEBUG_SERIAL_RADIO)
    Serial.println("Loop: Radio end transmit");
#endif
    radio.stopListening();
    delay(1);
    radio.powerDown();
#if defined(DEBUG_SERIAL_RADIO)
    Serial.println("Loop: Radio Sleep");
    delay(100);
#endif
  }
#if defined(DISPLAY_ALL)
  if (low_voltage_flag) {
    display_sleep(true);
  } else {  //low_voltage_flag: Regular voltage handling 
// regular Voltage Handling
    display_sleep(false);
    display_clear();
    display_fill();
    display_refresh();
  } // END regular Voltage Handling
#endif // DISPLAY_ALL
  if (low_voltage_flag) {
    // Low Voltage Schlafzeit in Sek. * Korrekturfaktor
    tempsleeptime_ms = (unsigned long)eeprom.sleeptime_lv * eeprom.sleep4ms_fac;
  } else {
    // (regelmaessige Schlafzeit in Sek. + einmalige Korrektur) * Korrekturfaktor
    tempsleeptime_ms = ((unsigned long)eeprom.sleeptime + sleeptime_kor) * eeprom.sleep4ms_fac;
  }
  sleeptime_kor = 0;
#ifdef ZAEHLER  // Sonderhehandlung für Zähler
  while (sumSleepTime_ms < tempsleeptime_ms) {
    bool pinstate = digitalRead(ZAEHLER_PIN);
    if ( (statusStack & 0b00001111) == 0b00001111 && (pinstate == LOW) ) {
      zaehlerUI++;
      zaehlerF = (float)zaehlerUI / 100.0;
    }
    statusStack = (statusStack << 1) | (pinstate? 0b00000001:0b00000000);
    SLEEPTYPE(61);  //sleep 60ms
    sumSleepTime_ms += 60;
  }
  sumSleepTime_ms = 0;
#else
/*
 * Behandlung normale Nodes
 */
    SLEEPTYPE(tempsleeptime_ms);
/*
 * ENDE Trennung: Zählernodes / normale Nodes
 */
#endif   //ZAEHLER
  loopcount++;
  if ( loopcount > eeprom.emptyloops ) loopcount = 0;
#ifdef DEBUG_SERIAL
  Serial.print("Loopcount: ");
  Serial.println(loopcount);
#endif
}
