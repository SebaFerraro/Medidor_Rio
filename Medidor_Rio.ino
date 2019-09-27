#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <lmic.h>
#include <hal/hal.h>
#include <esp_adc_cal.h>


#define LVL_PWR 13
#define DHT_PWR 15
#define DHT_DATA 22

#define DHTTYPE DHT11 
#define ANALOG_PIN_0 34

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600      /* Time ESP32 will go to sleep (in seconds) */

#define RXD2 16
#define TXD2 17
#define SONARE 5
#define KEEPALIVE  10
/*
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
//# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif
*/
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x37, 0x3D, 0xD9, 0x5F, 0xA9, 0x23, 0xCE, 0x6E, 0x92, 0x61, 0x5C, 0x6E, 0x3B, 0x1D, 0x1C, 0xF5 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x22, 0x02, 0x24, 0x3B, 0x92, 0x45, 0x45, 0x22, 0x9A, 0x70, 0x97, 0x33, 0x83, 0xCA, 0x4A, 0x9E };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
#ifndef COMPILE_REGRESSION_TEST
static const u4_t DEVADDR = 0x26011BF9;
#else
static const u4_t DEVADDR = 0;
#endif

#define LMIC_DEBUG_LEVEL 2

// The LORAWAN session state is saved in EEPROM
// Here just the frame sequence number is saved as its ABP with a Single Channel Gateway,
// See http://forum.thethingsnetwork.org/t/new-backend-how-to-connect/1983/91 
#define INITED_FLAG 0x12344321 // <<<< Change this to have the frame sequence restart from zero  

typedef struct {
  uint32_t inited;
  u4_t seqnoUp;
} LORA_STATE;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// payload to send to TTN gateway
static uint8_t payload[9];
static osjob_t sendjob;
float h=0;
float t=0;
float v=0;
long flag_TXCOMPLETE=50000000;
//float lvl=0;
static int taskCore = 0;
char bufferT[28]="";

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR float lvl = 0;
RTC_DATA_ATTR int times = 0;
RTC_DATA_ATTR LORA_STATE loraState;

const unsigned TX_INTERVAL = 30;

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
    .nss = 21,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {26, 27, 25},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

DHT dht(DHT_DATA, DHTTYPE);

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            //if (LMIC.dataLen) {
            Serial.println(LMIC.dataLen);
            flag_TXCOMPLETE=0;
            
            if (LMIC.dataLen != 0 || LMIC.dataBeg != 0) {
              Serial.println(F("*******************Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload*************************************"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            Serial.println("Send EVENTO  " + String(flag_TXCOMPLETE) +" Seconds");
            delay(500);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // read the temperature from the DHT22
        

        // read the humidity from the DHT22
        float rTemperature =t;
        float rHumidity =h;
        float rVolt =v;
        float rArio =lvl;
        // adjust for the f2sflt16 range (-1 to 1)
        rHumidity = rHumidity / 100;
        rVolt = rVolt / 10;
        rTemperature =rTemperature /100;
        rArio =rArio /10000;
        // float -> int
        // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
        uint16_t payloadTemp = LMIC_f2sflt16(rTemperature);
        // int -> bytes
        byte tempLow = lowByte(payloadTemp);
        byte tempHigh = highByte(payloadTemp);
        // place the bytes into the payload
        payload[0] = tempLow;
        payload[1] = tempHigh;

        // float -> int
        payloadTemp = LMIC_f2sflt16(rHumidity);
        // int -> bytes
        byte humidLow = lowByte(payloadTemp);
        byte humidHigh = highByte(payloadTemp);
        payload[2] = humidLow;
        payload[3] = humidHigh;

        payloadTemp = LMIC_f2sflt16(rVolt);
        tempLow = lowByte(payloadTemp);
        tempHigh = highByte(payloadTemp);
        payload[4] = tempLow;
        payload[5] = tempHigh;

        payloadTemp = LMIC_f2sflt16(rArio);
        tempLow = lowByte(payloadTemp);
        tempHigh = highByte(payloadTemp);
        payload[6] = tempLow;
        payload[7] = tempHigh;
        
        Serial.println( sizeof(payload)-1);
        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        Serial.println(LMIC.freq);
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
       
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void Lee_Valores(){
  int n=3;
  while (n>0){
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println("Error obteniendo los datos del sensor DHT11");
      h=0;
      t=0;
      n--;
      delay(100);
    }else{
      n=0;
    }
  }
  v =get_vcc();
  lvl=Sonar_Medir();
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Nivel = ");
  Serial.print(lvl);
  Serial.print(" Battery Voltage = ");
  Serial.println(v);
 }
  
float Sonar_Medir() { //Choose Serial1 or Serial2 as required
  float promedio=0;
  int n=0;
  String phrase;
  char c;
  int valor=0;

  digitalWrite(SONARE,LOW);
  Serial.println("SONARE LOW");
  delay(100);
  digitalWrite(SONARE,HIGH);
  Serial.println("SONARE HIGH");
  delay(3000);
  digitalWrite(SONARE,LOW);
  Serial.println("SONARE LOW");
  delay(100);
  while (Serial2.available()) {
    c=(char(Serial2.read()));
    if (c!='R'){
      phrase = String(phrase + c);  
    }else{
      valor=phrase.toInt();
      Serial.print("Valor :");
      Serial.println(valor);
      if((valor!=9999) && (valor!=0)){
        promedio=promedio+valor;
        n++;
     }
      phrase="";
    }
    
  }
  if (n>0){
    promedio=promedio/n;
    Serial.print("Promedio :");
    Serial.println(promedio);
  }
  return promedio;  
}



void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

float get_vcc(){
  int steps=analogRead(ANALOG_PIN_0);
  Serial.print("Voltage Steps = ");
  Serial.println(steps);
  float VBAT = 5.2f * 2 * float(steps) / 4095.0f;  // LiPo battery
  return VBAT;
}
void radio_off(){
  Serial.println("** Stopping WiFi+BT");
  WiFi.mode(WIFI_OFF);
  btStop();
}

void setup() { 
  Serial.begin(115200);  
  radio_off();
  pinMode(SONARE,OUTPUT);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
  adc_power_on();
  pinMode(LVL_PWR,OUTPUT);             
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  digitalWrite(LVL_PWR, HIGH);
  analogReadResolution(12); //12 bits
  //analogSetAttenuation(ADC_11db);  //For all pins
  //analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  Serial.println(F("LMIC"));
  if (loraState.inited != INITED_FLAG) {
    loraState.inited = INITED_FLAG;
    loraState.seqnoUp = 0;
  }
  delay(400);
  dht.begin();
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    
  //LMIC_selectSubBand(1);
  //for (int b = 0; b < 8; ++b) {
  //    LMIC_disableSubBand(b);
  // }
  // We'll disable all 72 channels used by TTN
  for (int c = 0; c < 72; c++){
    LMIC_disableChannel(c);
    Serial.print("LMIC disableChannel");
    Serial.println(c);
  }
  // Then enable the channel(s) you want to use
  //LMIC_enableChannel(8); // 903.9 MHz
  LMIC_enableChannel(8); // 903.9 MHz
  // We'll only enable Channel 16 (905.5Mhz) since we're transmitting on a single-channel
  //LMIC_enableChannel(3);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  //LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,20);

  LMIC.seqnoUp = loraState.seqnoUp;
    
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +" Seconds");
  Lee_Valores();
  Serial.println("Send job inicio" + String(flag_TXCOMPLETE) +" Seconds");
  delay(400);
  do_send(&sendjob);
  while(flag_TXCOMPLETE > 0){
     os_runloop_once();
     flag_TXCOMPLETE--;
  }
  Serial.println("Send job complete " + String(flag_TXCOMPLETE) +" Seconds");
  flag_TXCOMPLETE=50000000;
  //delay(200);
}


void loop() {
  
  // Start job
//  for(sale=0; sale < 1000; sale ++){
//    os_runloop_once();
//  }
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  Serial.println("Duerme.");
  digitalWrite(DHT_PWR, LOW);
  digitalWrite(LVL_PWR, LOW);
  
  adc_power_off();
  delay(1000);
  loraState.seqnoUp = LMIC.seqnoUp;
  Serial.print("loraState.seqnoUp = "); Serial.println(loraState.seqnoUp);    
  Serial.print("Up time "); Serial.print(millis());
  os_radio(RADIO_RST);
  delay(1000);
  esp_deep_sleep_start();
 }
