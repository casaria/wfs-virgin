
#define SubscribeTopic "WFS"
//#define MQTTServer "ccc.casaria.net"
#define UNIT "MASTER"
/*
 * Project wfs-virgin
 * Description:
 * Author:
 * Date:
 */

// This #include statement was automatically added by the Particle IDE.
#include <DS18B20.h>
#include <Particle.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_PCA9685.h>

//#include <PCF8574.h>

// This #include statement was automatically added by the Particle IDE.
#include <CASARIA_MCP3428.h>

#include <application.h>
#include <spark_wiring_i2c.h>

// MCP3428 I2C
//address is 0x68(104)
#define Addr 0x68

// This #include statement was automatically added by the Particlee IDE.
#include <CASARIA_MCP23017.h>

#include <math.h>

#include <MQTT.h>

//#include "DS18.h"

/*
TODO
- BME/BMP280
- Compressor 1
- Runtime using Tstamp
- issue with fan, crash, bettee Watchd dog
    - MQTT with SSL/TLS Qos2, locaL FALLBACK SERVER3
    
    
    */

#define damper2_1 0
#define damper2_2 1
#define damper1_1 2
#define damper1_2 3
#define txv1 4
#define txv2 8

/*
#define RelayFan1 1 //if ((InReverse1) && (msReverse1 > 10000) && (psi[4]>320)) {
#define RelayCompr1 2
#define RelayCool1 3
#define RelayBypassDamper1 4

#define RelayFan2 7
#define RelayCompr2 8
#define RelayCool2 5
#define RelayBypassDamper2 6
*/

#define RelayFan1 7 //if ((InReverse1) && (msReverse1 > 10000) && (psi[4]>320)) {
#define RelayCompr1 6
#define RelayCool1 14
#define RelayBypassDamper1 13

#define RelayFan2 1
#define RelayCompr2 1
#define RelayCool2 1
#define RelayBypassDamper2 1

static boolean InReverse1 = 0;
static boolean InReverse2 = 0;
static boolean HaltTemp = 0;
static int DefrostLevel1 = 0;
static int DefrostLevel2 = 0;

//uint16_t damperval11, damperval12, damperval21, damperval22;

const int MAXRETRY = 3;
const int pinOneWire = D4;
const int pinLED = D7;
//const int speakerPin = D2;
const int MQTT_LED = D7;
//const int PB_LED= D3;90
//const int PB_SW = A7;
//const int slaveSelectPinA = A2;

char* MQTTServer = "ccc.casaria.net";
const uint32_t msPressureSampleTime = 1000;
const uint32_t msRelaySampleTime = 2000; //
const uint32_t msTempSampleTime =8950;
const uint32_t msPublishTime = 5000;          //30000
const uint32_t msPublishTime2 = 8000;         //30000
const uint32_t msModulateDamperTime = 120000; //30000
const uint32_t msCompressor1Startlock = 5000; //30000
const uint32_t msCompressor2Startlock = 5000; //30000
const uint32_t defrostTimer = 5800000;        //1800000
const uint32_t MQTTConnectCheckTimer = 3000;
static uint32_t msDefrost1 = 0;
static uint32_t msDefrost2 = 0;

static float settemp1 = 8.00;
static float settemp2 = 8.00;

uint32_t msStartReverse1;
uint32_t msStartReverse2;
static int status10 = 0x00;
static int prevStatus10 = 0;

static int cmdPosArray[6];

static bool MAINTENANCE_ON_UNIT1 = FALSE;
static bool MAINTENANCE_ON_UNIT2 = FALSE;

enum mode_u
{
  off = 1,
  fullAuto = 2,
  noAutoDefrost = 3,
  manual = 4
};

mode_u ModeUnit1;
// SYSTEM_THREAD(ENABLED);
void dummy();

// Initialize objects from the lib

//ApplicationWatchdog wd(3000 , System.reset());
//void UnitMode1(mode_u mode);
void MQTTcallback1(char *topic, byte *ngth);
int cmdDefrost1(String szDefrost1);
int cmdDefrost2(String szDefrost1);
void PerformDefrost1(int advanceToState);
void PerformDefrost2(int advanceToState);

// declare a global watchdog instance
// reset the system after 15 seconds if the application is unresponsive
//ApplicationWatchdog wd(90000,System.reset(), 1424);
const int nSENSORS = 14; //14
const int damperDefault1 = 3000;
const int damperDefault2 = 2800;
String sz18B20AddressInfo[nSENSORS];
//[(1 * nSENSORS)+1];


ApplicationWatchdog wd(60000, System.reset, 1424);
CASARIA_MCP23017 relays;

String szInfo;
DS18B20 ds18b20(pinOneWire);
retained uint8_t sensorAddresses[nSENSORS][8];

double celsius[nSENSORS];
double temp;

//double celsius[nSENSORS] = {};
//uint8_t addr[8][12];

CASARIA_MCP3428 mcp1(0x68);
CASARIA_MCP3428 mcp2(0x6E);

MQTT MQTTclient(MQTTServer, 1883, MQTTcallback);
uint16_t qos2messageid1 = 0;

//DS18 sensor(pinOneWire);

Adafruit_PCA9685 damper = Adafruit_PCA9685(0x40, true); // Use the default address, but also turn on debugging

//CASARIA_MCP23017 relays; //not used0

// PCF8574 I2C address is 0x24 (36)
#define Addr8574 0x24
static byte relayMap = 0xFF;

void dummy()
{
}

void MQTTcallback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  memcpy(p, payload, length);
  wd.checkin(); // resets the AWDT count
  p[length] = NULL;
  String message(p);

  //message.getBytes(payload,length); does not work
  //message.toCharArray(ColorMode,length+1);
  if (strcmp(topic, "WFS/CONTROL1/TXV1") == 0)
  {
    cmdTXV1(message);
  }
  else if (strcmp(topic, "WFS/CONTROL1/TXV2") == 0)
  {
    cmdTXV2(message);
  }
  else if ((strcmp(topic, "WFS/CONTROL1/damper1_1") == 0))
  {
    damper.setVal(damper1_1, message.toInt());
  }
  else if ((strcmp(topic, "WFS/CONTROL1/damper1_2") == 0))
  {
    damper.setVal(damper1_2, message.toInt());
  }
  else if ((strcmp(topic, "WFS/CONTROL1/damper2_1") == 0))
  {
    damper.setVal(damper2_1, message.toInt());
  }
  else if ((strcmp(topic, "WFS/CONTROL1/damper2_2") == 0))
  {
    damper.setVal(damper2_2, message.toInt());
  }
  else if ((strcmp(topic, "WFS/CONTROL1/InitializeTemp") == 0))
  {
    cmdTempReset(message);
    publishOneWireAddresses();
  }
  else if ((strcmp(topic, "WFS/CONTROL1/defrost1") == 0))
  {
    cmdDefrost1(message);
  }
  else if ((strcmp(topic, "WFS/CONTROL1/defrost2") == 0))
  {
    cmdDefrost2(message);
  }
  else if ((strcmp(topic, "WFS/CONTROL1/MAINTENANCE1") == 0))
  {
    MAINTENANCE_ON_UNIT1 = (message.toInt() >= 1) ? TRUE : FALSE;
  }
  else if ((strcmp(topic, "WFS/CONTROL1/MAINTENANCE2") == 0))
  {
    MAINTENANCE_ON_UNIT2 = (message.toInt() >= 1) ? TRUE : FALSE;
  }
  else if ((strcmp(topic, "WFS/CONTROL1/statusRequest") == 0))
  {
    publishOneWireAddresses();
  }
  else if ((strcmp(topic, "WFS/CONTROL1/manual") == 0))
  {
    //.UnitMode1(manual);
  }
  else
  {
    //UnitMode1(fullAuto);
  }
  if ((ModeUnit1 == manual) & (strcmp(topic, "WFS/CONTROL1/compressor") == 0))
  {
    if (message == "on")
    {
      // turnOnRelay(RelayCompr1);
      turnOnRelay(RelayFan1);
    }
    if (message == "off")
    {
      // turnOffRelay(RelayCompr1);
      turnOffRelay(RelayFan1);
    }
  }
}

void MQTTConnect()
{

  // connect to the server
  digitalWrite(MQTT_LED, LOW);
  wd.checkin();
  MQTTclient.connect(String(SubscribeTopic) + String(UNIT));
  // add qos callback. If don't add qoscallback, ACK message from MQTT server is ignored.
  MQTTclient.addQosCallback(qoscallback); // publish/subscribe
  if (MQTTclient.isConnected())
  {
    digitalWrite(MQTT_LED, HIGH);
    MQTTclient.subscribe(String(SubscribeTopic) + "/#", MQTT::QOS2);
  }
}

// QOS ack callback.
// if application use QOS1 or QOS2, MQTT server sendback ack message id.
void qoscallback(unsigned int messageid)
{
  // Serial.print("Ack Message Id:");
  //Serial.println(messageid);
  /*
    if (messageid == qos2messageid1) {
        //Serial.println("Release QoS2 Message");
        MQTTclient1.publishRelease(qos2messageid1);
    }
    */
}

void MQTTpublish(String subTopic, const char *payload)
{
  uint16_t messageid;
  if (MQTTclient.isConnected())
  {
    /*
       if (!latch) {
         
           memcpy(state,"OPEN", 5); 
          } else {
            memcpy(state,"CLOSED", 7); 
        }
            
      */
    MQTTclient.publish(String(SubscribeTopic) + "/" + String(UNIT) + "/" + subTopic, (uint8_t *)payload, strlen(payload), true, MQTT::QOS2, false, &messageid);

    // save QoS2 message id as global parameter.
    qos2messageid1 = messageid;
  }
}

int b1status = 0;
int b2status = 0;
byte error;
int8_t mcp1address;
long Raw_adc[8];
float psi[8];
const int psiFS[8]{5000, 300, 300, 0, 500, 0, 0, 0};

//HP2,   LP2

int cmdSetDamper(String command)
{
  // Wire.reset();
  String valueString;
  for (int i = 0; i < 4; i++)
  {
    valueString = command.substring(i * 4, (i * 4 + 4));
    cmdPosArray[i] = valueString.toInt();
    //TXV2 PWM output is 8 (skipped 3)
    if (cmdPosArray[i] < 4096)
      damper.setVal(i + ((i < 6) ? 0 : 3), cmdPosArray[i]);
  }
  return 1;
}



int cmdTXV2(String command)
{
  int TXVpos;
  TXVpos = command.toInt();
  if (TXVpos <= 4095)
  {
    damper.setVal(txv2, TXVpos);
  }
  return TXVpos;
}

int cmdTXV1(String command)
{
  int TXVpos;
  TXVpos = command.toInt();
  if (TXVpos <= 4095)
  {
    damper.setVal(txv1, TXVpos);
  }
  return TXVpos;
}

int cmdDefrost1(String szDefrost1)
{
  DefrostLevel1 = szDefrost1.toInt();
  if (DefrostLevel1 == 0)
    cmdStopDefrost1("0");
  return DefrostLevel1;
}

int cmdStopDefrost1(String command)
{
  turnOnRelay(RelayCool1);
  turnOnRelay(RelayFan1);
  turnOnRelay(RelayBypassDamper1);
  InReverse1 = 0;
  MQTTpublish("defrost1", String(DefrostLevel1, DEC));

  PerformDefrost1(8);
  //msDefrost2 = millis() + defrostTimer -100000;
  return 0;
}

int cmdStopDefrost2(String command)
{
  turnOnRelay(RelayCool2);
  turnOnRelay(RelayFan2);
  turnOnRelay(RelayBypassDamper2);
  InReverse2 = 0;
  MQTTpublish("defrost2", String(DefrostLevel2, DEC));

  PerformDefrost2(8);
  return 0;
}

int cmdDefrost2(String szDefrost2)
{

  DefrostLevel2 = szDefrost2.toInt();
  if (DefrostLevel2 == 0)
    cmdStopDefrost2("0");
  return DefrostLevel2;
}

int cmdTempReset(String command)
{
  HaltTemp = TRUE;
  relayOff8574();
  delay(6000);
  //Particle.process();
  //find all temp sensors
  for (int i = 0; i < nSENSORS; i++)
    sz18B20AddressInfo[i] = "";
  wd.checkin();
  delay(10);
  ds18b20.resetsearch();
  delay(200); // initialise for sensor search

  for (int i = 0; i < nSENSORS; i++)
  {                                     // try to read the sensor addre
    ds18b20.search(sensorAddresses[i]); // and if available store
    //String address((char*)sensorAddresses[i]);
    delay(300);
    celsius[i] = 0;
    wd.checkin();
  }
  wd.checkin();
  /* for (int i = 0; i< nSENSORS; i++){
      for (int x=0; x < 8; x++){
        sz18B20AddressInfo[i].concat(String::format("%02x", sensorAddresses[i][x]) );
      }
  }*/
  HaltTemp = FALSE;
  relayOn8574();
  return 0;
}

void relayOff8574()
{
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // SeAdressslect GPIO as input
  relayMap = 0xff;

  Wire.write(relayMap); //Wire.write(0xff);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
}
void relayOn8574()
{
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  relayMap = 0x00;
  Wire.write(relayMap); //Wire.write(0xff);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
}

void PerformDefrost2(int advanceToState)
{
  static int defstate = 0, prevDefstate = 0;
  static uint32_t msStartTime = 0;
  static uint16_t damper1 = 3000;
  static uint16_t damper2 = 3000;
  static uint16_t txvValue = 3800;
  if ((defstate != prevDefstate) || (advanceToState != 0))
  {
    MQTTpublish("defrost2State", String::format("defstate: %d  Elapsed: %d ", defstate, msStartTime - millis()));
    prevDefstate = defstate;
  }
  if (advanceToState != 0)
    defstate = advanceToState;
  switch (defstate)
  {
  case 0:

    if (DefrostLevel2 > 0)
      defstate = 1;
    break;
  case 1:
    damper1 = damper.getVal(damper2_1);
    damper2 = damper.getVal(damper2_2);
    txvValue = damper.getVal(txv2);
    msStartTime = millis();
    turnOffRelay(RelayCool2);
    turnOffRelay(RelayFan2);
    turnOffRelay(RelayBypassDamper2);
    InReverse2 = !InReverse2;
    if (InReverse2)
      msStartReverse2 = millis();
    if (DefrostLevel2 >= 3)
    {
      msStartTime = msStartTime - 2000;
    }
    //wait for dampers
    damper.setVal(damper2_2, 500);
    damper.setVal(damper2_1, 500);
    MQTTpublish("defrost2", String(DefrostLevel2, DEC));
    defstate++;
    break;
  case 2:
    if (psi[0] > 435)
      turnOffRelay(RelayCompr2);
    if (millis() > (msStartTime + 5000))
    {
      defstate++;
      turnOffRelay(RelayCompr2);
    }
    break;
  case 3:
    //wait for damprt clodse with fan off/compressor off
    if (millis() > (msStartTime + 75000))
    {
      defstate = 5;
    }
    break;
  case 5:;
    turnOnRelay(RelayFan2);
    turnOffRelay(RelayCool2); //reverse heat - cool
    turnOnRelay(RelayCompr2);
    defstate++;
    break;
  case 6:
    //run in reverse with dampers clsoed, bypass open
    if (psi[0] > 435)
      turnOffRelay(RelayCompr2);
    if (millis() > (msStartTime + 145000 + DefrostLevel1 * 12000))
      defstate++;
    break;
  case 7:
    // back to cooling mode
    InReverse2 = FALSE;
    if (psi[0] > 435)
      turnOffRelay(RelayCompr2);
    damper.setVal(txv2, txvValue);
    if (millis() > (msStartTime + 165000 + DefrostLevel2 * 12000))
      defstate++;
    break;
  case 8:
    //restore dampers to previous position
    //restore txv
    turnOnRelay(RelayFan2);
    turnOnRelay(RelayCool2);
    turnOnRelay(RelayCompr2);   
    damper.setVal(damper2_2, damper2);
    damper.setVal(damper2_1, damper1);
    damper.setVal(txv2, txvValue);
    if (millis() > (msStartTime + 176000 + DefrostLevel2 * 12000))
      defstate++;
    break;
  case 9:
    turnOnRelay(RelayBypassDamper2);
    if (millis() > (msStartTime + 200000 + DefrostLevel2 * 12000))
    {
      defstate++;
      turnOnRelay(RelayCompr2);
      turnOnRelay(RelayFan2);
    }
    break;
  case 10:
    if (millis() > (msStartTime + 95000))
      defstate++;
    MQTTpublish("defrost2", String(DefrostLevel2, DEC));
    //MQTTpublish("defrost2 / STATE", concat(String(DefrostLevel2, DEC), VVVVVV       break;
  case 11:
    DefrostLevel2 = 0;
    defstate = 0;
    MQTTpublish("defrost2", String(DefrostLevel2, DEC));
  }
}

void PerformDefrost1(int advanceToState)
{
  static int defstate = 0, prevDefstate = 0;
  static uint32_t msStartTime = 0;
  static uint16_t damper1 = 3000;
  static uint16_t damper2 = 3000;
  static uint16_t txv1Value = 3000;
  if ((defstate != prevDefstate) || (advanceToState != 0))
  {
    MQTTpublish("defros1State", String::format("defstate: %d  Elapsed: %d ", defstate, msStartTime - millis()));
    prevDefstate = defstate;
  }
  if (advanceToState != 0)
    defstate = advanceToState;
  switch (defstate)
  {

  case 0:
    if (DefrostLevel1 > 0)
      defstate = 1;
    break;
  case 1:
    txv1Value = damper.getVal(txv1);
    damper1 = damper.getVal(damper1_1);
    damper2 = damper.getVal(damper1_2);
    msStartTime = millis();
    damper.setVal(txv1, 4095);
    turnOffRelay(RelayCool1);
    turnOffRelay(RelayFan1);
    turnOffRelay(RelayBypassDamper1);
    InReverse1 = !InReverse1;
    if (InReverse1)
      msStartReverse1 = millis();
    if (DefrostLevel1 >= 2)
    {
      msStartTime = msStartTime - 25000;
    }
    //wait for dampers
    damper.setVal(damper1_2, 500);
    damper.setVal(damper1_1, 500);
    MQTTpublish("defrost1", String(DefrostLevel1, DEC));
    defstate++;
    break;
  case 2:
    if (psi[4] > 425)
      turnOffRelay(RelayCompr1);
    if (millis() > (msStartTime + 5000))
    {
      defstate++;
      turnOffRelay(RelayCompr1);
    }
    break;
  case 3:
    //wait for damprt clodse with fan off/compressor off
    if (millis() > (msStartTime + 85000))
    {
      defstate = 5;
    }
    break;
  case 5:;
    turnOnRelay(RelayFan1);
    turnOffRelay(RelayCool1); //reverse heat - cool
    turnOnRelay(RelayCompr1);
    defstate++;
    break;
  case 6:
    //run in reverse with dampers clsoed, bypass open
    if (millis() > (msStartTime + 185000 + DefrostLevel1 * 10000))
      defstate++;
    break;
  case 7:
    // back to cooling mode
    turnOnRelay(RelayFan1);
    turnOnRelay(RelayCool1);
    InReverse1 = FALSE;
    turnOnRelay(RelayCompr1);
    damper.setVal(txv1, txv1Value);
    if (millis() > (msStartTime + 215000 + DefrostLevel1 * 10000))
      defstate++;
    break;
  case 8:
    //restore dampers to previous position
    //restore txv
    damper.setVal(damper1_2, damper2);
    damper.setVal(damper1_1, damper1);
    damper.setVal(txv1, txv1Value);
    if (millis() > (msStartTime + 245000 + DefrostLevel1 * 10000))
      defstate++;
    break;
  case 9:
    turnOnRelay(RelayBypassDamper1);
    if (millis() > (msStartTime + 265000 + DefrostLevel1 * 10000))
    {
      defstate++;
      turnOnRelay(RelayCompr1);
      turnOnRelay(RelayFan1);
    }
    break;
  case 10:
    if (millis() > (msStartTime + 275000))
      defstate++;
    break;
  case 11:
    DefrostLevel1 = 0;
    defstate = 0;
    MQTTpublish("defrost1", String(DefrostLevel1, DEC));
    // msDefrost1 = millis() + defrostTimer;
  }
}

STARTUP(System.enableFeature(FEATURE_RESET_INFO));
/****************** SETUP  INITIALIZATION  SETUP  ************************/
/****************** SETUP  INITIALIZATION  SETUP  ************************/
/****************** SETUP  INITIALIZATION  SETUP  ************************/
/****************** SETUP  INITIALIZATION  SETUP  ************************/
void setup(void)
{
  relays.setAddress(0x20);
  relays.setRelays(16);
  relays.setOutputs(0x00, 0x00);
  //relays.turnOnAllRelays();
  relays.reAssignmentMap[4] = 4;
  delay(2000);
  // relays.setOutput(4);
  relays.init();
  // pinMode(RelayPowerOn, OUTPUT);
  //pinMode(PIRtrigger, INPUT);

  //If this is a 32 channel relay board, the A0 address jumper is ALWAYS 1 on the second chipset, so should never be set here on the first

  Particle.function("RelayControl", triggerRelay);
  //   Particle.function("ColorCommand", cmdColor);
  Particle.variable("Bank_1", b1status);
  Particle.variable("Bank_2", b2status);
  //Particle.variable("cumulative1RT", cumulative1RT);
  //Particle.variable("cumulative2RT", cumulative2RT);
  //Particle.variable("Input Status", b4status);
  relays.turnOffAllRelays();

  if (System.resetReason() == RESET_REASON_PANIC)
  {
    System.enterSafeMode();
  }
  //Particle.function("SetDamper", cmdSetDamper);

  //cmdStopDefrost1("abort");
  //cmdStopDefrost2("abort");
  //Wire.begin();
  // Start I2C Transmission
  //Wire.beginTransmission(Addr);  // Continuous conversion mode, Channel-1, 12-bit resolution
  //Wire.write(0x10);
  // Stop I2C Transmission
  //Wire.endTransmission();
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  relayMap = 0xFF;
  Wire.write(relayMap); //Wire.write(0xff);13
                        // Relay8574(1, 1);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
  //UnitMode1(manual);
  relayOff8574();
  delay(5000);
  //find all temp sensors
  wd.checkin();

  ds18b20.resetsearch();
  delay(300); // initialise for sensor search
  for (int i = 0; i < nSENSORS; i++)
  {                                     // try to read the sensor addre
    ds18b20.search(sensorAddresses[i]); // and if available store
    delay(400);
    celsius[i] = 0;
  }
  wd.checkin();

  //relays.setAddress(0x20);
  //relays.setRelays(16);
  //relays.setOutputs(0x00, 0x00);

  //relays.setOutput(4);
  //relays.init();

  //If this is a 32 channel relay board, the A0 address jumper is ALWAYS set on the second chipset, so should never be set here on the first

  //Particle.function("TempReset", cmdTempReset);
  //Particle.function("TXV1", cmdTXV1);
  //Particle.function("TXV2", cmdTXV2);
  //Particle.function("DEFROST1", cmdDefrost1);
  //Particle.function("DEFROST2", cmdDefrost2);

  // Particle.variable("Bank_1", b1status);
  // Particle.variable("Bank_2", b2status);
  //Particle.variable("Input Status", b4status);
  //relays.turnOffAllRelays();
  pinMode(pinLED, OUTPUT);

  //delay(500);
  // Start serial communication and set baud rate = 9600
  //Serial.begin(9600);
  // Set variable

  damper.begin();          // This calls Wire.begin()
  damper.setPWMFreq(1500); // Maximum PWM frequency is 1600

  damper.setVal(damper2_2, 480);
  damper.setVal(damper2_1, 480);
  damper.setVal(damper1_2, 3801);
  damper.setVal(damper1_1, 3661);

  turnOnRelay(RelayBypassDamper1);
  turnOnRelay(RelayBypassDamper2);
  // Initialise I2C communication as MASTER
  wd.checkin();
  //UnitMode1(manual);
  cmdTempReset("");
   wd.checkin();
}

void publishDebug()
{
  // char szInfo[200];fWFSZ
}

double getTemp(uint8_t addr[8])
{
  double _temp;
  int i = 0;

  do
  {
    //delay(20);
    _temp = ds18b20.getTemperature(addr);
  } while (!ds18b20.crcCheck() && MAXRETRY > i++);

  if (i >= MAXRETRY)

  {
    _temp = -99;
    // Serial.println("Invalid reading");
  }
  return _temp;
}

void publishData()
{
  char szInfo[220];
  snprintf(szInfo, sizeof(szInfo), "%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f", celsius[0], celsius[1], celsius[2], celsius[3],
           celsius[4], celsius[5], celsius[6], celsius[7], celsius[8], celsius[9], celsius[10], celsius[11], celsius[10], celsius[11], celsius[12], celsius[13], celsius[14]);
  Particle.publish("dsTEMPS", szInfo, PRIVATE);

  /*
  for (int i = 0; i < nSENSORS; i++) {   //ry to read the sen-sor addre
    snprintf(szInfo, sizeof(szInfo), "%s %s %s %s %s %s %s %s %s %s %s %s", sensorAddresses[0],sensorAddresses[1], sensorAddresses[2], sensorAddresses[3],
      sensorAddresses[4], sensorAddresses[5], sensorAddresses[6], sensorAddresses[7], sensorAddresses[8],sensorAddresses[9],sensorAddresses[10],sensorAddresses[11]);
    Particle.publish("dsAddress", szInfo, PRIVATE);
  
  }
  */
}

void publishPressure()
{
  char szInfo[200];
  uint16_t d11, d12, d21, d22, tx1, tx2;
  d11 = damper.getVal(damper1_1);
  d12 = damper.getVal(damper1_2);
  d21 = damper.getVal(damper2_1);
  d22 = damper.getVal(damper2_2);
  tx1 = damper.getVal(txv1);
  tx2 = damper.getVal(txv2);
  //  snprintf(szInfo, sizeof(szInfo), "%d&%d&%d&%d&%d&%d&%d&%d", Raw_adc[0], Raw_adc[1], Raw_adc[2], Raw_adc[3], Raw_adc[4], Raw_adc[5], Raw_adc[6], Raw_adc[7]);
  // Particle.publish("dsAnalogin",szInfo, PRIVATE );
  snprintf(szInfo, sizeof(szInfo), "%.2f&%.2f&%.2f&%.2f&%.2f&%4d&%4d&%4d&%4d&%4d&%4d", psi[0], psi[1], psi[2], psi[3], psi[4], d11, d12, d21, d22, tx1, tx2);
  Particle.publish("ds_psi", szInfo, PRIVATE);
}

void publishStatus()
{

  status10 = 0;
  status10 = status10 | (((bool)HaltTemp) << 0);
  status10 = status10 | (((DefrostLevel1 >= 1) ? 1 : 0) << 1);
  status10 = status10 | (((DefrostLevel2 >= 1) ? 1 : 0) << 2);

  if (status10 != prevStatus10)
  {
    MQTTpublish("status10", String(status10, BIN));
    prevStatus10 = status10;
  }
}

void publishOneWireAddresses()
{
  // Particle.publish("ds_18B20Addr", sz18B20AddressInfo, PRIVATE);
  for (int i = 0; i < nSENSORS; i++)
  {
    MQTTpublish(String::format("D18B22address%d", i), String::format("%s", sz18B20AddressInfo[i]));
  }
  MQTTpublish("status10", String(status10, BIN));
}
/*
void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switcnsh(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);1

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}
*/

/*
void getTemp(){
  static int i=0;


  if (sensor.read()) {
        
    // Do something cool with the temperature
    if (i>11) i=0;
    celsius[i]= sensor.celsius();
      sensor.addr(addr[i]);
    i++;
    printDebugInfo();

  // If sensor.read() didn't return true you can try again later
  // This next block helps debug what's wrong.
  // It's not needed for the sensor to work properly
  } else {
    // Once all sensors have been read you'll get searchDone() == true
    // Next time read() is called the first sensor is read again
    i++;
    if (sensor.searchDone()) {
      Serial.println("No more addresses.");
      // Avoid excessive printing when no sensors are connected
      delay(250);

    // Something went wrong
    } else {
      printDebugInfo();
    }
  }
    
}
*/
int ModulateDamper(uint8_t damperNum, int min = 1200, int max = 4095, int by = 100)
{
  static int16_t dutyCycle;
  static boolean goup = FALSE;

  // Edit this if you want to edit the brightness step size
  if (dutyCycle > max)
  {
    dutyCycle = max;
    goup = FALSE;
  }
  if (dutyCycle <= min)
  {
    dutyCycle = min;
    goup = TRUE;
  }
  if (goup)
    dutyCycle = +by;
  else
    dutyCycle = -by;

  damper.setVal(damperNum, dutyCycle);
  return dutyCycle;
}

void relayOp(int relay, int op)
{
 /* if (relay > 8)
  {
  }
  else
  {

 byte rbit = (1 << (relay - 1));
    Wire.beginTransmission(Addr8574);
    // Select GPIO as input
    //(toggle) ? Wire.write(0x55) : Wire.write(0xAA);
    //Wire.write(0x00);
    // Stop I2C transmission
    relayMap = bitop(relayMap, rbit, op);
    Wire.write(relayMap);
    Wire.endTransmission(); */
    relays.relayOp(relay, op);
  
}

byte bitop(byte b1, byte b2, int op)
{
  switch (op)
  {
  case 1:
    return b1 | b2;
  case 2:
    return b1 & ~b2;
  case 3:
    return b1 ^ b2;
  }
  return 0;
}

void turnOnRelay(int relay)
{
  relayOp(relay, 2);
}
void turnOffRelay(int relay)
{
  relayOp(relay, 1);
}
void toggleRelay(int relay)
{
  relayOp(relay, 3);
}

void Relay8574(byte bitnum, boolean value)
{
  static bool toggle;

  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  //(toggle) ? Wire.write(0x55) : Wire.write(0xAA);
  //Wire.write(0x00);
  // Stop I2C transmission
  Wire.write(0x00);
  relayMap = 0x00;
  Wire.endTransmission();
  toggle = !toggle;
}

void loop()
{
  static uint32_t msTempSample = 0;
  static uint32_t msPressureSample = 0;
  static uint32_t msMQTTConnectCheck = 0;
  static uint32_t msRelaySample = 0;
  static uint32_t msPublish = 0;
  static uint32_t msPublish2 = 0;
  static uint32_t msModulateDamper = 0;
  static uint32_t msReverse1 = 0;
  static uint32_t msReverse2 = 0;
  static bool FirstTime = TRUE;
  static uint32_t now;
  static int sensorNumber = 0;
  static int i;
 

 if(!DefrostLevel1) turnOnRelay(RelayBypassDamper1);





 if(!DefrostLevel2) turnOnRelay(RelayBypassDamper2);



  //turnOffRelay(RelayCompr1);
  //turnOffRelay(RelayFan1);
  if (MQTTclient.isConnected())
    MQTTclient.loop();

  if (Particle.connected() && FirstTime)
  {
    Particle.publish("RESETREASON", String(System.resetReason(), DEC), PRIVATE);
    FirstTime = FALSE;
  }

  now = millis();

  wd.checkin();

  if (msDefrost1 == 0)
    msDefrost1 = now + defrostTimer;

  if (msDefrost2 == 0)
    msDefrost2 = now + defrostTimer;

  wd.checkin();

  if (psi[0] > 450)
    turnOffRelay(RelayCompr1);
  // if (psi[4] > 450)    turnOffRelay(RelayCompr2);

  if (now - msRelaySample >= msRelaySampleTime)
  {
    msRelaySample = millis();
  }

  if (now - msDefrost1 >= defrostTimer)
  {
    if ((celsius[0] < 1.0) && (DefrostLevel1 == 0))
      cmdDefrost1("5");
    msDefrost1 = millis();
  }

  if (now - msDefrost2 >= defrostTimer)
  {
    if ((celsius[2] < 1.0) && (DefrostLevel2 == 0))
      cmdDefrost2("5");
    msDefrost2 = millis();
  }

  wd.checkin();

  if (now - msMQTTConnectCheck >= MQTTConnectCheckTimer)
  {

    if (!MQTTclient.isConnected())
    { //cloud connect
      MQTTConnect();
    }
    else
    {
      digitalWrite(MQTT_LED, HIGH);
    }
    msMQTTConnectCheck = millis();
  }
  wd.checkin();

  if (now - msTempSample >= msTempSampleTime)
  {
    if (!HaltTemp)
    {
      for (int i = 0; i < nSENSORS; i++)
      {
        temp = getTemp(sensorAddresses[i]);

        if (!isnan(temp))
        {
          celsius[i] = temp;
        }
        else
        {
          temp = getTemp(sensorAddresses[i]);
          if (!isnan(temp))
            celsius[i] = temp;
        }

        wd.checkin();
      }
    }
    msTempSample = millis();
  }
  /*    if (!HaltTemp){
        if (i < nSENSORS) 
        {
          temp = getTemp(sensorAddresses[i]);
      
          if (!isnan(temp))
          {
            celsius[i] = temp;
          }
          else
          {
            celsius[i] = -99;
          }
          i++; 
        } else {
          i = 0;
        } 
      }
  
  }*/
  wd.checkin();

  if (now - msPressureSample >= msPressureSampleTime)
  {
    getMCP();
    msPressureSample = millis();
    msReverse1 = millis() - msStartReverse1;
    msReverse2 = millis() - msStartReverse2;

    wd.checkin();
    /*if ((InReverse1) && (msReverse1 > 12000) && (psi[4] > 290))
    {
      //if ( (psi[4]>300)) {
    if ((InReverse2) && (msReverse2 > 12000) && (psi[0] > 290))
    {
      //  if ( (psi[0]>300)) {
      cmdStopDefrost2("OFF");
    } */
  }

  PerformDefrost1(0);
   wd.checkin();
  PerformDefrost2(0);

  if (now - msPublish2 >= msPublishTime2)
  {
    msPublish2 = millis();
    wd.checkin();
    publishPressure();
    wd.checkin();
    MQTTpublish("damper1_1", String(damper.getVal(damper1_1), DEC));
    MQTTpublish("damper1_2", String(damper.getVal(damper1_2), DEC));
    MQTTpublish("damper2_1", String(damper.getVal(damper2_1), DEC));
    MQTTpublish("damper2_2", String(damper.getVal(damper2_2), DEC));
    MQTTpublish("txv1", String(damper.getVal(txv1), DEC));
    MQTTpublish("txv2", String(damper.getVal(txv2), DEC));

    publishStatus();
    //publishDebug();
  }

  if (now - msModulateDamper >= msModulateDamperTime)
  {
    msModulateDamper = millis();
    //  ModulateDamper(damper1_1 ,1200, 3200, 100);
    // ModulateDamper(damper1_2 ,1500, 3200, 100);
    //  ModulateDamper(damper2_1 ,1200, 3200, 100);
    //  ModulateDamper(damper2_2 ,1500, 3200, 100);
    //
  }
  if (now - msPublish >= msPublishTime)
  {
    publishData();
    wd.checkin();
    //ModulateDamper();
    msPublish = millis();
  }
  wd.checkin(); // resets the AWDT count
  if (MAINTENANCE_ON_UNIT1)
  {
    turnOffRelay(RelayCompr1);
    turnOffRelay(RelayFan1);

    damper.setVal(damper1_2, 500);
    damper.setVal(damper1_1, 500);
  }
  if (MAINTENANCE_ON_UNIT2)
  {
    turnOffRelay(RelayCompr2);
    turnOffRelay(RelayFan2);

    damper.setVal(damper2_2, 500);
    damper.setVal(damper2_1, 500);
  }
}

void getMCP()
{

  // Distributed with a free-will license.
  // Use it any way you want, prof it or free, provided it fits in the licenses of its associated works.
  // MCP3428
  // This code is designed to work with the MCP3428_I2CADC I2C Mini Module available from ControlEverything.com.
  // https://www.controleverything.com/content/Analog-Digital-Converters?sku=MCP3428_I2CADC#tabs-0-product_tabset-2

  byte error;
  int8_t address;

  address = mcp1.devAddr;
  // The i2c_scanner uses the  return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {

    for (int i = 1; i <= 4; i++)
    {
      // MCP3428 is configured to channel i with 12 bits resolution, continuous mode and gain defined to 1
      // This arrangement of the mentioned paarmeters can be changed as per convenience
      mcp1.SetConfiguration(i, 16, 1, 2);
      Raw_adc[i - 1] = mcp1.readADC();
      // Note that the library waits for a complete conversion
      psi[i - 1] = ((float(Raw_adc[i - 1])) - 5813) / (29390 - 5813) * psiFS[i - 1];
      // raw_adc = raw_adc * LSB(1 mV)/PGA for PGA = 1;       // 12-bit Resolution
      // raw_adc = raw_adc * LSB(250 µV)/PGA for PGA = 1;     // 14-bit Resolution
      // raw_adc = raw_adc * LSB(62.5 µV)/PGA for PGA = 1;    // 16-bit Resolution
    } //calibration/linearizatiom
    //CALIBRATION
    // correct zero offset
    if (psi[0] < 600)
    {
      psi[0] = psi[0] + 15;
    }
  }
  wd.checkin();
  address = mcp2.devAddr;
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
    for (int i = 4; i <= 8; i++)
    {
      // MCP3428 is configured to channel i with 12 bits resolution, continuous mode and gain defined to 1
      // This arrangement of the mentioed paarmeters can be changed as per convenience
      mcp2.SetConfiguration(i, 16, 1, 2);

      // Note that the library waits for a complete conversion
      Raw_adc[i - 1] = mcp2.readADC();
      psi[i - 1] = ((float(Raw_adc[i - 1])) - 5813) / (29390 - 5813) * psiFS[i - 1];
    }
    //calibration/linearizatiom
    //CALIBRATION
    // correct zero offset error
  }
}

String relayCmd(String acommand)
{
  int relay = 0;
  int bank = 0;
  int op = 0;
  int test = 0;
  bool all = false;
  int p = acommand.indexOf(" ");

  bool relay_next = false;
  String word;
  if (p > -1)
  {
    while (acommand.length() > 0)
    {
      if (p > -1)
      {
        word = acommand.substring(0, p);
        acommand = acommand.substring(p + 1);
        //preservedCommand = command.substring(0, p);
        p = acommand.indexOf(" ");
      }
      else
      {
        word = acommand;
        // command = "";
        //preservedCommand  = word;
      }

      if (word.equalsIgnoreCase("on") || word.equalsIgnoreCase("activate"))
      {
        if (op == 0 || bank > 0)
          op = 2;
      }
      if (word.equalsIgnoreCase("off") || word.equalsIgnoreCase("deactivate"))
      {
        op = 1;
      }
      if (word.equalsIgnoreCase("toggle") || word.equalsIgnoreCase("flip"))
      {
        op = 3;
      }
      if (relay_next)
      {

        if (test == 0)
        {
          test = word.toInt();
        }
        if (test > 0)
        {
          relay = test;
          relay_next = false;
        }
      }

      if (word.equalsIgnoreCase("relay") || word.equalsIgnoreCase("output"))
      {
        relay_next = true;
      }

      if (word.equalsIgnoreCase("bank"))
      {
        all = true;
      }
      if (word.equalsIgnoreCase("all"))
      {
        all = true;
      }
    }
  }
  if (all)
  {
    return acommand;
  }
  else
  {
    if (bank > 0)
      relay += ((bank - 1) * 8);
    relays.relayOp(relay, op);
    return "";
  }
}

int triggerRelay(String command)
{

  //relays.relayTalk(relayCmd(command));
  relays.relayTalk(command);
  return 1;
}
