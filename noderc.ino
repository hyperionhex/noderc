/*
  =====================================================================
      RC CAR PRO - ROBUST WiFi VERSION
      NodeMCU ESP8266 + L298N + 2 Yellow TT Motors + Front Caster
  =====================================================================

  DIRECT CONNECTION
  -----------------
  WiFi:
      RC_Car

  Password:
      12345678

  Open:
      http://192.168.4.1


  OPTIONAL HOME WIFI
  ------------------
  Press "HOME WIFI" in the Web UI.

  Enter:
      WiFi SSID
      WiFi Password

  Once connected:
      http://rccar.local

  IMPORTANT:
  RC_Car AP always remains the recovery network.


  =====================================================================
  WIRING
  =====================================================================

  NodeMCU              L298N
  ----------------------------------
  D1 / GPIO5    ---->  IN1
  D2 / GPIO4    ---->  IN2

  D5 / GPIO14   ---->  IN3
  D6 / GPIO12   ---->  IN4

  D7 / GPIO13   ---->  ENA
  D0 / GPIO16   ---->  ENB

  GND           ---->  GND
  VIN           <----  L298N 5V


  REMOVE:
      ENA jumper
      ENB jumper


  MOTORS:
      OUT1 + OUT2 -> Left TT motor
      OUT3 + OUT4 -> Right TT motor


  BATTERY:
      2S + -> L298N VIN / +12V
      2S - -> L298N GND


  =====================================================================
  REQUIRED LIBRARY
  =====================================================================

  Arduino Library Manager:

      WebSockets
      by Markus Sattler

  =====================================================================
*/


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#include <stddef.h>


// =====================================================================
// BASIC WIFI
// =====================================================================

const char* AP_SSID = "RC_Car";
const char* AP_PASSWORD = "12345678";

const char* MDNS_HOST = "rccar";


IPAddress AP_IP(
  192, 168, 4, 1
);

IPAddress AP_GATEWAY(
  192, 168, 4, 1
);

IPAddress AP_SUBNET(
  255, 255, 255, 0
);


ESP8266WebServer server(80);

WebSocketsServer webSocket(81);


// =====================================================================
// MOTOR PINS
// =====================================================================

#define IN1 D1
#define IN2 D2
#define ENA D7

#define IN3 D5
#define IN4 D6
#define ENB D0


// =====================================================================
// PERSISTENT CONFIGURATION
// =====================================================================

#define EEPROM_SIZE 192

#define CONFIG_MAGIC   0x52434334UL
#define CONFIG_VERSION 1


struct PersistentConfig
{
  uint32_t magic;

  uint16_t version;

  uint8_t apChannel;

  uint8_t homeConfigured;

  char homeSSID[33];

  char homePassword[65];

  uint32_t checksum;
};


PersistentConfig persistentConfig;


// =====================================================================
// WIFI RADIO SETTINGS
// =====================================================================

uint8_t apChannel = 1;

float wifiTxPower = 14.0;


// =====================================================================
// HOME WIFI STATE
// =====================================================================

bool homeTestActive = false;

char testSSID[33] = {0};

char testPassword[65] = {0};

unsigned long homeTestStart = 0;


bool pendingHomeConnect = false;

unsigned long pendingHomeConnectAt = 0;


bool pendingForgetHome = false;

unsigned long pendingForgetAt = 0;


bool reconnectOldHomePending = false;

unsigned long reconnectOldHomeAt = 0;


unsigned long lastSavedHomeAttempt = 0;


bool previousHomeConnected = false;

bool mdnsRunning = false;


String homeStatusMessage =
  "Home WiFi not configured";


// =====================================================================
// AP CHANNEL CHANGE STATE
// =====================================================================

bool pendingChannelChange = false;

uint8_t requestedChannel = 1;

unsigned long requestedChannelAt = 0;


// =====================================================================
// CAR CONFIG
// =====================================================================

int speedPercent = 70;

int turnStrength = 70;


int leftTrim = 100;

int rightTrim = 100;


/*
   Acceleration ramp.

   Low:
      smoother
      less current spike

   High:
      faster response
*/

int rampStep = 10;


// 0 = Pivot
// 1 = Smooth

int steeringMode = 1;


/*
   Current physical build:

   LEFT previously turned RIGHT.

   Keep true.
*/

bool swapSteering = true;


bool ecoMode = true;

int ecoSpeedLimit = 85;


// =====================================================================
// FAILSAFE
// =====================================================================

const unsigned long CONTROL_TIMEOUT = 700;

unsigned long lastControlPacket = 0;


const unsigned long AP_HEALTH_INTERVAL = 5000;

unsigned long lastAPHealthCheck = 0;


// =====================================================================
// MOTOR STATE
// =====================================================================

char activeCommand = 'S';


int currentLeft = 0;

int currentRight = 0;


int targetLeft = 0;

int targetRight = 0;


unsigned long lastRampUpdate = 0;

const unsigned long RAMP_INTERVAL = 8;


// =====================================================================
// CHECKSUM
// =====================================================================

uint32_t calculateConfigChecksum(
  const PersistentConfig& cfg
)
{
  const uint8_t* data =
    reinterpret_cast<const uint8_t*>(
      &cfg
    );


  size_t length =
    offsetof(
      PersistentConfig,
      checksum
    );


  uint32_t hash =
    2166136261UL;


  for(
    size_t i = 0;
    i < length;
    i++
  )
  {
    hash ^=
      data[i];


    hash *=
      16777619UL;
  }


  return hash;
}


// =====================================================================
// DEFAULT CONFIG
// =====================================================================

void setDefaultPersistentConfig()
{
  memset(
    &persistentConfig,
    0,
    sizeof(persistentConfig)
  );


  persistentConfig.magic =
    CONFIG_MAGIC;


  persistentConfig.version =
    CONFIG_VERSION;


  persistentConfig.apChannel =
    1;


  persistentConfig.homeConfigured =
    0;


  persistentConfig.checksum =
    calculateConfigChecksum(
      persistentConfig
    );
}


// =====================================================================
// LOAD CONFIG
// =====================================================================

void loadPersistentConfig()
{
  EEPROM.begin(
    EEPROM_SIZE
  );


  EEPROM.get(
    0,
    persistentConfig
  );


  bool valid =
    (
      persistentConfig.magic
      ==
      CONFIG_MAGIC
    )

    &&

    (
      persistentConfig.version
      ==
      CONFIG_VERSION
    )

    &&

    (
         persistentConfig.apChannel == 1
      || persistentConfig.apChannel == 6
      || persistentConfig.apChannel == 11
    )

    &&

    (
      persistentConfig.checksum
      ==
      calculateConfigChecksum(
        persistentConfig
      )
    );


  if(!valid)
  {
    setDefaultPersistentConfig();


    EEPROM.put(
      0,
      persistentConfig
    );


    EEPROM.commit();
  }


  apChannel =
    persistentConfig.apChannel;
}


// =====================================================================
// SAVE CONFIG
// =====================================================================

bool savePersistentConfig()
{
  persistentConfig.magic =
    CONFIG_MAGIC;


  persistentConfig.version =
    CONFIG_VERSION;


  persistentConfig.apChannel =
    apChannel;


  persistentConfig.checksum =
    calculateConfigChecksum(
      persistentConfig
    );


  EEPROM.put(
    0,
    persistentConfig
  );


  return EEPROM.commit();
}


// =====================================================================
// IS HOME WIFI SAVED?
// =====================================================================

bool savedHomeWiFiExists()
{
  return (
    persistentConfig.homeConfigured
    ==
    1

    &&

    strlen(
      persistentConfig.homeSSID
    )
    >
    0
  );
}


// =====================================================================
// MOTOR HELPERS
// =====================================================================

int percentToPWM(
  int percent
)
{
  percent =
    constrain(
      percent,
      0,
      100
    );


  return map(
    percent,
    0,
    100,
    0,
    255
  );
}


int applyTrim(
  int pwm,
  int trim
)
{
  long output =
    (
      (long)pwm
      *
      trim
    )
    /
    100;


  return constrain(
    output,
    -255,
    255
  );
}


// =====================================================================
// LEFT MOTOR
// =====================================================================

void setLeftMotor(
  int pwm
)
{
  pwm =
    constrain(
      pwm,
      -255,
      255
    );


  if(pwm > 0)
  {
    digitalWrite(
      IN1,
      HIGH
    );


    digitalWrite(
      IN2,
      LOW
    );


    analogWrite(
      ENA,
      pwm
    );
  }


  else if(pwm < 0)
  {
    digitalWrite(
      IN1,
      LOW
    );


    digitalWrite(
      IN2,
      HIGH
    );


    analogWrite(
      ENA,
      -pwm
    );
  }


  else
  {
    analogWrite(
      ENA,
      0
    );


    digitalWrite(
      IN1,
      LOW
    );


    digitalWrite(
      IN2,
      LOW
    );
  }
}


// =====================================================================
// RIGHT MOTOR
// =====================================================================

void setRightMotor(
  int pwm
)
{
  pwm =
    constrain(
      pwm,
      -255,
      255
    );


  /*
     Right motor is mounted opposite
     to the left motor.
  */

  if(pwm > 0)
  {
    digitalWrite(
      IN3,
      LOW
    );


    digitalWrite(
      IN4,
      HIGH
    );


    analogWrite(
      ENB,
      pwm
    );
  }


  else if(pwm < 0)
  {
    digitalWrite(
      IN3,
      HIGH
    );


    digitalWrite(
      IN4,
      LOW
    );


    analogWrite(
      ENB,
      -pwm
    );
  }


  else
  {
    analogWrite(
      ENB,
      0
    );


    digitalWrite(
      IN3,
      LOW
    );


    digitalWrite(
      IN4,
      LOW
    );
  }
}


// =====================================================================
// HARD STOP
// =====================================================================

void stopCar()
{
  targetLeft = 0;

  targetRight = 0;


  currentLeft = 0;

  currentRight = 0;


  activeCommand = 'S';


  analogWrite(
    ENA,
    0
  );


  analogWrite(
    ENB,
    0
  );


  digitalWrite(
    IN1,
    LOW
  );


  digitalWrite(
    IN2,
    LOW
  );


  digitalWrite(
    IN3,
    LOW
  );


  digitalWrite(
    IN4,
    LOW
  );
}


// =====================================================================
// SAFE RAMP
// =====================================================================

int rampMotor(
  int current,
  int target
)
{
  int safeTarget =
    target;


  /*
     Never directly reverse a motor.

     +PWM -> 0 -> -PWM
  */

  if(
       (
         current > 0
         &&
         target < 0
       )

    ||

       (
         current < 0
         &&
         target > 0
       )
  )
  {
    safeTarget =
      0;
  }


  if(
    current
    <
    safeTarget
  )
  {
    current +=
      rampStep;


    if(
      current
      >
      safeTarget
    )
    {
      current =
        safeTarget;
    }
  }


  else if(
    current
    >
    safeTarget
  )
  {
    current -=
      rampStep;


    if(
      current
      <
      safeTarget
    )
    {
      current =
        safeTarget;
    }
  }


  return current;
}


// =====================================================================
// UPDATE MOTORS
// =====================================================================

void updateMotors()
{
  unsigned long now =
    millis();


  if(
    now
    -
    lastRampUpdate
    <
    RAMP_INTERVAL
  )
  {
    return;
  }


  lastRampUpdate =
    now;


  currentLeft =
    rampMotor(
      currentLeft,
      targetLeft
    );


  currentRight =
    rampMotor(
      currentRight,
      targetRight
    );


  setLeftMotor(
    currentLeft
  );


  setRightMotor(
    currentRight
  );
}


// =====================================================================
// LEFT / RIGHT CORRECTION
// =====================================================================

char correctSteering(
  char command
)
{
  if(!swapSteering)
  {
    return command;
  }


  switch(command)
  {
    case 'L':
      return 'R';


    case 'R':
      return 'L';


    case 'G':
      return 'H';


    case 'H':
      return 'G';


    case 'I':
      return 'J';


    case 'J':
      return 'I';
  }


  return command;
}


// =====================================================================
// DRIVE CALCULATION
// =====================================================================

void calculateDrive(
  char userCommand
)
{
  char command =
    correctSteering(
      userCommand
    );


  int actualSpeed =
    speedPercent;


  if(ecoMode)
  {
    actualSpeed =
      min(
        actualSpeed,
        ecoSpeedLimit
      );
  }


  int base =
    percentToPWM(
      actualSpeed
    );


  int inside =
    (
      (long)base
      *
      (100 - turnStrength)
    )
    /
    100;


  int pivot =
    (
      (long)base
      *
      turnStrength
    )
    /
    100;


  /*
     Reduce pivot current in ECO mode.
  */

  if(ecoMode)
  {
    pivot =
      min(
        pivot,
        percentToPWM(65)
      );
  }


  int left =
    0;


  int right =
    0;


  switch(command)
  {
    case 'F':

      left =
        base;

      right =
        base;

      break;


    case 'B':

      left =
        -base;

      right =
        -base;

      break;


    case 'L':

      if(
        steeringMode
        ==
        0
      )
      {
        left =
          -pivot;


        right =
          pivot;
      }

      else
      {
        left =
          inside;


        right =
          base;
      }

      break;


    case 'R':

      if(
        steeringMode
        ==
        0
      )
      {
        left =
          pivot;


        right =
          -pivot;
      }

      else
      {
        left =
          base;


        right =
          inside;
      }

      break;


    case 'G':

      left =
        inside;


      right =
        base;

      break;


    case 'H':

      left =
        base;


      right =
        inside;

      break;


    case 'I':

      left =
        -inside;


      right =
        -base;

      break;


    case 'J':

      left =
        -base;


      right =
        -inside;

      break;
  }


  targetLeft =
    applyTrim(
      left,
      leftTrim
    );


  targetRight =
    applyTrim(
      right,
      rightTrim
    );
}


// =====================================================================
// VALID DRIVE COMMAND
// =====================================================================

bool validDriveCommand(
  char command
)
{
  return (
       command == 'F'
    || command == 'B'
    || command == 'L'
    || command == 'R'
    || command == 'G'
    || command == 'H'
    || command == 'I'
    || command == 'J'
    || command == 'S'
  );
}


// =====================================================================
// DRIVE COMMAND
// =====================================================================

void driveCommand(
  char command
)
{
  if(
    !validDriveCommand(
      command
    )
  )
  {
    stopCar();

    return;
  }


  lastControlPacket =
    millis();


  if(
    command
    ==
    'S'
  )
  {
    stopCar();

    return;
  }


  activeCommand =
    command;


  calculateDrive(
    command
  );
}


// =====================================================================
// START / REBUILD RC_Car AP
// =====================================================================

bool startSoftAP(
  bool stationAlsoEnabled
)
{
  stopCar();


  if(stationAlsoEnabled)
  {
    WiFi.mode(
      WIFI_AP_STA
    );
  }

  else
  {
    WiFi.mode(
      WIFI_AP
    );
  }


  WiFi.setSleepMode(
    WIFI_NONE_SLEEP
  );


  WiFi.setPhyMode(
    WIFI_PHY_MODE_11G
  );


  WiFi.setOutputPower(
    wifiTxPower
  );


  if(
    !WiFi.softAPConfig(
      AP_IP,
      AP_GATEWAY,
      AP_SUBNET
    )
  )
  {
    return false;
  }


  bool success =
    WiFi.softAP(
      AP_SSID,
      AP_PASSWORD,
      apChannel,
      false,
      4
    );


  return success;
}


// =====================================================================
// MDNS
// =====================================================================

void stopMDNS()
{
  if(
    MDNS.isRunning()
  )
  {
    MDNS.close();
  }


  mdnsRunning =
    false;
}


void startMDNS()
{
  if(
    WiFi.status()
    !=
    WL_CONNECTED
  )
  {
    return;
  }


  stopMDNS();


  if(
    MDNS.begin(
      MDNS_HOST
    )
  )
  {
    MDNS.addService(
      "http",
      "tcp",
      80
    );


    MDNS.addService(
      "ws",
      "tcp",
      81
    );


    mdnsRunning =
      true;


    Serial.println(
      "mDNS ready:"
    );


    Serial.println(
      "http://rccar.local"
    );
  }
}


// =====================================================================
// BEGIN HOME WIFI CONNECTION
// =====================================================================

void beginHomeWiFi(
  const char* ssid,
  const char* password
)
{
  stopCar();


  /*
     Keep RC_Car AP alive while adding
     station mode.
  */

  WiFi.mode(
    WIFI_AP_STA
  );


  WiFi.hostname(
    MDNS_HOST
  );


  WiFi.setAutoReconnect(
    false
  );


  WiFi.disconnect(
    false
  );


  delay(20);


  WiFi.begin(
    ssid,
    password
  );


  lastSavedHomeAttempt =
    millis();
}


// =====================================================================
// START TESTING NEW HOME WIFI
// =====================================================================

void startPendingHomeTest()
{
  if(
    !pendingHomeConnect
  )
  {
    return;
  }


  if(
    millis()
    <
    pendingHomeConnectAt
  )
  {
    return;
  }


  pendingHomeConnect =
    false;


  homeTestActive =
    true;


  homeTestStart =
    millis();


  homeStatusMessage =
    "Connecting to Home WiFi...";


  beginHomeWiFi(
    testSSID,
    testPassword
  );
}


// =====================================================================
// FORGET HOME WIFI
// =====================================================================

void processPendingForget()
{
  if(
    !pendingForgetHome
  )
  {
    return;
  }


  if(
    millis()
    <
    pendingForgetAt
  )
  {
    return;
  }


  pendingForgetHome =
    false;


  stopCar();


  homeTestActive =
    false;


  reconnectOldHomePending =
    false;


  stopMDNS();


  WiFi.disconnect(
    true
  );


  memset(
    persistentConfig.homeSSID,
    0,
    sizeof(
      persistentConfig.homeSSID
    )
  );


  memset(
    persistentConfig.homePassword,
    0,
    sizeof(
      persistentConfig.homePassword
    )
  );


  persistentConfig.homeConfigured =
    0;


  savePersistentConfig();


  homeStatusMessage =
    "Home WiFi forgotten";


  /*
     Return to clean AP-only mode.
  */

  startSoftAP(
    false
  );
}


// =====================================================================
// MANAGE HOME WIFI
// =====================================================================

void manageHomeWiFi()
{
  bool connected =
    (
      WiFi.status()
      ==
      WL_CONNECTED
    );


  // -----------------------------------------------------------------
  // JUST CONNECTED
  // -----------------------------------------------------------------

  if(
    connected
    &&
    !previousHomeConnected
  )
  {
    previousHomeConnected =
      true;


    startMDNS();


    Serial.println(
      "Home WiFi connected"
    );


    Serial.print(
      "Home IP: "
    );


    Serial.println(
      WiFi.localIP()
    );
  }


  // -----------------------------------------------------------------
  // JUST DISCONNECTED
  // -----------------------------------------------------------------

  if(
    !connected
    &&
    previousHomeConnected
  )
  {
    previousHomeConnected =
      false;


    stopMDNS();


    homeStatusMessage =
      "Home WiFi disconnected";
  }


  // -----------------------------------------------------------------
  // TEST SUCCEEDED
  // -----------------------------------------------------------------

  if(
    connected
    &&
    homeTestActive
  )
  {
    memset(
      persistentConfig.homeSSID,
      0,
      sizeof(
        persistentConfig.homeSSID
      )
    );


    memset(
      persistentConfig.homePassword,
      0,
      sizeof(
        persistentConfig.homePassword
      )
    );


    strncpy(
      persistentConfig.homeSSID,
      testSSID,
      sizeof(
        persistentConfig.homeSSID
      )
      -
      1
    );


    strncpy(
      persistentConfig.homePassword,
      testPassword,
      sizeof(
        persistentConfig.homePassword
      )
      -
      1
    );


    persistentConfig.homeConfigured =
      1;


    savePersistentConfig();


    homeTestActive =
      false;


    memset(
      testSSID,
      0,
      sizeof(testSSID)
    );


    memset(
      testPassword,
      0,
      sizeof(testPassword)
    );


    homeStatusMessage =
      "Connected and saved";


    return;
  }


  // -----------------------------------------------------------------
  // NEW CREDENTIAL TEST FAILED
  // -----------------------------------------------------------------

  if(
    homeTestActive
    &&
    !connected
    &&
    (
      millis()
      -
      homeTestStart
      >
      18000
    )
  )
  {
    homeTestActive =
      false;


    WiFi.disconnect(
      false
    );


    homeStatusMessage =
      "Connection failed. RC_Car is still available.";


    memset(
      testSSID,
      0,
      sizeof(testSSID)
    );


    memset(
      testPassword,
      0,
      sizeof(testPassword)
    );


    /*
       If old saved credentials exist,
       restore them.

       Otherwise go back to AP-only mode.
    */

    if(
      savedHomeWiFiExists()
    )
    {
      reconnectOldHomePending =
        true;


      reconnectOldHomeAt =
        millis()
        +
        1500;
    }

    else
    {
      startSoftAP(
        false
      );
    }


    return;
  }


  // -----------------------------------------------------------------
  // RESTORE PREVIOUS SAVED NETWORK
  // -----------------------------------------------------------------

  if(
    reconnectOldHomePending
    &&
    millis()
    >=
    reconnectOldHomeAt
  )
  {
    reconnectOldHomePending =
      false;


    beginHomeWiFi(
      persistentConfig.homeSSID,
      persistentConfig.homePassword
    );


    homeStatusMessage =
      "Restoring previous Home WiFi...";


    return;
  }


  // -----------------------------------------------------------------
  // RETRY SAVED HOME WIFI
  //
  // Don't channel-hop while car is driving.
  // -----------------------------------------------------------------

  if(
    !connected
    &&
    !homeTestActive
    &&
    !pendingHomeConnect
    &&
    savedHomeWiFiExists()
    &&
    activeCommand == 'S'
    &&
    (
      millis()
      -
      lastSavedHomeAttempt
      >
      30000
    )
  )
  {
    beginHomeWiFi(
      persistentConfig.homeSSID,
      persistentConfig.homePassword
    );


    homeStatusMessage =
      "Reconnecting Home WiFi...";
  }
}


// =====================================================================
// AP CHANNEL CHANGE
// =====================================================================

void processChannelChange()
{
  if(
    !pendingChannelChange
  )
  {
    return;
  }


  if(
    millis()
    <
    requestedChannelAt
  )
  {
    return;
  }


  pendingChannelChange =
    false;


  /*
     Router owns channel when station
     is connected.
  */

  if(
    WiFi.status()
    ==
    WL_CONNECTED
  )
  {
    return;
  }


  if(
       requestedChannel != 1
    && requestedChannel != 6
    && requestedChannel != 11
  )
  {
    return;
  }


  stopCar();


  apChannel =
    requestedChannel;


  persistentConfig.apChannel =
    apChannel;


  savePersistentConfig();


  WiFi.softAPdisconnect(
    true
  );


  delay(50);


  bool needsSTA =
       savedHomeWiFiExists()
    || homeTestActive
    || pendingHomeConnect;


  if(
    !startSoftAP(
      needsSTA
    )
  )
  {
    delay(100);


    ESP.restart();
  }
}


// =====================================================================
// AP HEALTH WATCHDOG
// =====================================================================

void checkAccessPointHealth()
{
  unsigned long now =
    millis();


  if(
    now
    -
    lastAPHealthCheck
    <
    AP_HEALTH_INTERVAL
  )
  {
    return;
  }


  lastAPHealthCheck =
    now;


  WiFiMode_t mode =
    WiFi.getMode();


  bool hasAP =
       mode == WIFI_AP
    || mode == WIFI_AP_STA;


  bool correctIP =
    (
      WiFi.softAPIP()
      ==
      AP_IP
    );


  if(
    hasAP
    &&
    correctIP
  )
  {
    return;
  }


  Serial.println(
    "AP watchdog recovery"
  );


  stopCar();


  bool needsSTA =
       savedHomeWiFiExists()
    || homeTestActive
    || pendingHomeConnect;


  if(
    !startSoftAP(
      needsSTA
    )
  )
  {
    delay(100);


    ESP.restart();
  }
}


// =====================================================================
// JSON ESCAPE
// =====================================================================

String jsonEscape(
  const String& input
)
{
  String output;


  output.reserve(
    input.length()
    +
    10
  );


  for(
    unsigned int i = 0;
    i < input.length();
    i++
  )
  {
    char c =
      input[i];


    if(c == '"')
    {
      output +=
        "\\\"";
    }


    else if(c == '\\')
    {
      output +=
        "\\\\";
    }


    else if(
      (uint8_t)c
      >=
      32
    )
    {
      output +=
        c;
    }
  }


  return output;
}


// =====================================================================
// WEB PAGE
// =====================================================================

const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>

<head>

<meta charset="UTF-8">

<meta
 name="viewport"
 content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">

<meta
 name="theme-color"
 content="#07110e">

<title>RC Car Pro</title>


<style>

:root{
 --accent:#21e28b;
 --background:#07110e;
 --panel:#101d18;
 --panel2:#172923;
 --text:#f3fff8;
 --muted:#82948c;
 --red:#ff4057;
 --orange:#ffad32;
}

*{
 box-sizing:border-box;
 -webkit-user-select:none;
 user-select:none;
 -webkit-tap-highlight-color:transparent;
}

html,
body{
 width:100%;
 height:100%;
 margin:0;
 overflow:hidden;
}

body{
 color:var(--text);
 font-family:system-ui,-apple-system,BlinkMacSystemFont,sans-serif;
 background:
 radial-gradient(circle at 50% -20%,#1d4b3a,var(--background) 46%);
 touch-action:none;
}

.app{
 height:100%;
 max-width:760px;
 margin:auto;
 padding:11px;
 display:flex;
 flex-direction:column;
}


/* HEADER */

header{
 min-height:58px;
 display:flex;
 align-items:center;
 justify-content:space-between;
}

.title{
 font-size:20px;
 font-weight:900;
}

.subtitle{
 margin-top:3px;
 color:var(--muted);
 font-size:11px;
}

.dot{
 display:inline-block;
 width:8px;
 height:8px;
 margin-right:5px;
 border-radius:50%;
 background:#777;
}

.dot.ws{
 background:var(--accent);
 box-shadow:0 0 12px var(--accent);
}

.dot.http{
 background:var(--orange);
 box-shadow:0 0 10px #ffad3266;
}

.topButtons{
 display:flex;
 gap:7px;
}

.iconButton{
 height:42px;
 padding:0 12px;
 border-radius:13px;
 border:1px solid #ffffff16;
 background:var(--panel);
 color:white;
 font-weight:800;
}


/* STATS */

.stats{
 display:grid;
 grid-template-columns:repeat(5,1fr);
 gap:6px;
 margin:6px 0 9px;
}

.stat{
 padding:7px 2px;
 border-radius:11px;
 text-align:center;
 background:#ffffff07;
 border:1px solid #ffffff0d;
}

.stat strong{
 display:block;
 color:var(--accent);
 font-size:13px;
}

.stat span{
 color:var(--muted);
 font-size:8px;
 text-transform:uppercase;
}


/* MAIN */

main{
 flex:1;
 min-height:0;
 display:flex;
 align-items:center;
 justify-content:space-around;
 gap:12px;
}


/* PAD */

.pad{
 display:grid;
 grid-template-columns:repeat(3,75px);
 grid-template-rows:repeat(3,75px);
 gap:8px;
}

.driveButton{
 border:1px solid #ffffff15;
 border-radius:20px;
 background:linear-gradient(145deg,#182a23,#0b1511);
 color:white;
 font-size:26px;
 touch-action:none;
 box-shadow:0 7px 16px #0005;
}

.driveButton.active{
 background:var(--accent);
 color:#00150a;
 transform:scale(.92);
}

.stopButton{
 background:#401019;
 color:#ff8090;
}


/* SIDE */

.side{
 width:180px;
 display:flex;
 flex-direction:column;
 gap:8px;
}

.card{
 padding:13px;
 border-radius:17px;
 background:#ffffff08;
 border:1px solid #ffffff0e;
}

.row{
 display:flex;
 justify-content:space-between;
 align-items:center;
 gap:8px;
}

.speedNumber{
 color:var(--accent);
 font-size:28px;
 font-weight:900;
}

input[type=range]{
 width:100%;
 accent-color:var(--accent);
}

.modeButtons{
 display:grid;
 grid-template-columns:1fr 1fr;
 gap:6px;
 margin-top:8px;
}

.smallButton{
 padding:9px 4px;
 border-radius:10px;
 border:1px solid #ffffff13;
 background:var(--panel2);
 color:white;
 font-weight:700;
 font-size:11px;
}

.smallButton.selected{
 background:var(--accent);
 color:#00150a;
}

.wifiButton{
 height:48px;
 border:1px solid #45a7ff66;
 border-radius:15px;
 background:#102537;
 color:#8dcbff;
 font-weight:900;
}

.emergencyButton{
 height:50px;
 border:1px solid #ff5568;
 border-radius:15px;
 background:#3b1018;
 color:#ff8492;
 font-weight:900;
}


/* MODAL */

.overlay{
 position:fixed;
 inset:0;
 display:none;
 align-items:flex-end;
 justify-content:center;
 z-index:30;
 background:#000c;
 backdrop-filter:blur(6px);
}

.overlay.show{
 display:flex;
}

.panel{
 width:min(650px,100%);
 max-height:93vh;
 overflow:auto;
 padding:18px;
 background:#0c1713;
 border-radius:24px 24px 0 0;
}

.panelHeader{
 display:flex;
 justify-content:space-between;
 align-items:center;
 position:sticky;
 top:-18px;
 padding:12px 0;
 background:#0c1713;
 z-index:2;
}

.closeButton{
 width:38px;
 height:38px;
 border:0;
 border-radius:11px;
 background:#ffffff10;
 color:white;
}

.sectionTitle{
 margin:17px 0 6px;
 color:var(--accent);
 font-size:11px;
 font-weight:900;
 letter-spacing:1px;
 text-transform:uppercase;
}

.setting{
 padding:11px 0;
 border-bottom:1px solid #ffffff0c;
}

.setting label{
 font-size:14px;
 font-weight:700;
}

.hint{
 margin-top:3px;
 color:var(--muted);
 font-size:10px;
}

.value{
 color:var(--accent);
 font-weight:800;
}

select,
input[type=text],
input[type=password]{
 padding:10px;
 border-radius:10px;
 border:1px solid #ffffff16;
 background:var(--panel2);
 color:white;
}

.fullInput{
 width:100%;
 margin-top:8px;
 font-size:16px;
}

.action{
 width:100%;
 margin-top:9px;
 padding:12px;
 border:0;
 border-radius:11px;
 background:var(--accent);
 color:#00150a;
 font-weight:900;
}

.secondary{
 color:white;
 background:var(--panel2);
 border:1px solid #ffffff16;
}

.danger{
 color:#ff8996;
 background:#351018;
 border:1px solid #67202b;
}

.statusBox{
 margin-top:8px;
 padding:12px;
 border-radius:14px;
 border:1px solid #ffffff0e;
 background:#ffffff07;
}

.good{
 color:var(--accent);
}

.bad{
 color:#ff8492;
}

.networkList{
 display:grid;
 gap:5px;
 margin-top:8px;
}

.networkItem{
 width:100%;
 padding:10px;
 text-align:left;
 border:1px solid #ffffff10;
 border-radius:9px;
 background:var(--panel2);
 color:white;
}

.address{
 color:var(--accent);
 font-weight:800;
 word-break:break-all;
}


/* RESPONSIVE */

@media(max-width:520px){

 .pad{
  grid-template-columns:repeat(3,66px);
  grid-template-rows:repeat(3,66px);
 }

 .side{
  width:132px;
 }

 .stats{
  grid-template-columns:repeat(5,1fr);
 }

}


@media(max-height:520px){

 .stats{
  display:none;
 }

 header{
  min-height:42px;
 }

 .pad{
  grid-template-columns:repeat(3,60px);
  grid-template-rows:repeat(3,60px);
 }

}

</style>

</head>


<body>


<div class="app">


<header>

<div>

<div class="title">
🏎️ RC CAR PRO
</div>

<div class="subtitle">

<span
 class="dot"
 id="connectionDot">
</span>

<span id="connectionText">
Starting...
</span>

</div>

</div>


<div class="topButtons">

<button
 class="iconButton"
 id="homeWifiMainButton">
📶 WiFi
</button>

<button
 class="iconButton"
 id="settingsMainButton">
⚙
</button>

</div>

</header>



<div class="stats">

<div class="stat">
<strong id="latencyStat">--</strong>
<span>Latency</span>
</div>

<div class="stat">
<strong id="transportStat">--</strong>
<span>Control</span>
</div>

<div class="stat">
<strong id="clientStat">--</strong>
<span>Clients</span>
</div>

<div class="stat">
<strong id="channelStat">--</strong>
<span>Channel</span>
</div>

<div class="stat">
<strong id="networkStat">AP</strong>
<span>Network</span>
</div>

</div>



<main>


<div class="pad">

<button
 class="driveButton"
 data-drive="G">
↖
</button>

<button
 class="driveButton"
 data-drive="F">
▲
</button>

<button
 class="driveButton"
 data-drive="H">
↗
</button>

<button
 class="driveButton"
 data-drive="L">
◀
</button>

<button
 class="driveButton stopButton"
 id="centerStopButton">
■
</button>

<button
 class="driveButton"
 data-drive="R">
▶
</button>

<button
 class="driveButton"
 data-drive="I">
↙
</button>

<button
 class="driveButton"
 data-drive="B">
▼
</button>

<button
 class="driveButton"
 data-drive="J">
↘
</button>

</div>



<div class="side">


<div class="card">

<div class="row">

<span>SPEED</span>

<span
 class="speedNumber"
 id="speedNumber">
70
</span>

</div>


<input
 id="speedSlider"
 type="range"
 min="20"
 max="100"
 value="70">


<div class="modeButtons">

<button
 class="smallButton selected"
 id="ecoModeButton">
🌿 ECO
</button>

<button
 class="smallButton"
 id="sportModeButton">
⚡ SPORT
</button>

</div>

</div>


<button
 class="wifiButton"
 id="homeWifiSideButton">
📶 HOME WIFI
</button>


<button
 class="emergencyButton"
 id="emergencyStopButton">
⛔ STOP
</button>


</div>


</main>

</div>



<!-- ==============================================================
     HOME WIFI PANEL
================================================================ -->


<div
 class="overlay"
 id="wifiOverlay">


<div class="panel">


<div class="panelHeader">

<div>

<h2 style="margin:0">
📶 Home WiFi
</h2>

<div class="hint">
RC_Car direct WiFi remains your recovery network.
</div>

</div>


<button
 class="closeButton"
 id="closeWifiPanel">
✕
</button>

</div>



<div class="statusBox">

<div class="row">

<b>Home WiFi</b>

<span
 id="homeWifiStatus">
Checking...
</span>

</div>


<div
 class="hint"
 id="homeWifiMessage">
</div>


<div
 id="homeWifiDetails"
 style="margin-top:9px">
</div>

</div>



<div class="sectionTitle">
Connect to router
</div>


<button
 class="action secondary"
 id="scanNetworksButton">
📡 SCAN AVAILABLE WIFI
</button>


<div
 class="networkList"
 id="networkList">
</div>



<div class="setting">

<label for="ssidInput">
WiFi Name / SSID
</label>

<input
 class="fullInput"
 id="ssidInput"
 type="text"
 maxlength="32"
 autocomplete="off"
 placeholder="Enter your home WiFi name">

</div>



<div class="setting">

<label for="passwordInput">
WiFi Password
</label>

<input
 class="fullInput"
 id="passwordInput"
 type="password"
 maxlength="64"
 autocomplete="off"
 placeholder="Enter your WiFi password">

</div>



<div
 class="row"
 style="margin-top:9px">

<label for="showPasswordCheckbox">
Show Password
</label>

<input
 id="showPasswordCheckbox"
 type="checkbox">

</div>



<button
 class="action"
 id="connectWifiButton">
CONNECT & TEST
</button>


<div class="hint">
Credentials are saved only after the ESP8266 successfully connects.
A wrong password will not overwrite the previous working network.
</div>



<div class="sectionTitle">
Addresses
</div>


<div class="statusBox">

<div>
Direct connection:
</div>

<div class="address">
http://192.168.4.1
</div>


<div style="margin-top:9px">
Home network:
</div>

<div class="address">
http://rccar.local
</div>


<div
 class="hint"
 id="homeAddressHint">

rccar.local becomes available after Home WiFi connects.

</div>

</div>



<button
 class="action danger"
 id="forgetWifiButton">
FORGET SAVED HOME WIFI
</button>


</div>

</div>



<!-- ==============================================================
     CAR SETTINGS
================================================================ -->


<div
 class="overlay"
 id="settingsOverlay">


<div class="panel">


<div class="panelHeader">

<h2 style="margin:0">
⚙ Car Settings
</h2>


<button
 class="closeButton"
 id="closeSettingsPanel">
✕
</button>

</div>



<div class="sectionTitle">
Driving
</div>


<div class="setting">

<div class="row">

<div>

<label>
Steering
</label>

<div class="hint">
Smooth uses less power.
</div>

</div>


<select id="steeringSelect">

<option value="1">
Smooth
</option>

<option value="0">
Pivot
</option>

</select>

</div>

</div>



<div class="setting">

<div class="row">

<label>
Turn Strength
</label>

<span
 class="value"
 id="turnValue">
70%
</span>

</div>

<input
 id="turnSlider"
 type="range"
 min="30"
 max="100"
 value="70">

</div>



<div class="setting">

<div class="row">

<label>
Acceleration
</label>

<span
 class="value"
 id="rampValue">
10
</span>

</div>

<input
 id="rampSlider"
 type="range"
 min="4"
 max="30"
 value="10">

</div>



<div class="setting">

<div class="row">

<label>
Left Motor Trim
</label>

<span
 class="value"
 id="leftTrimValue">
100%
</span>

</div>

<input
 id="leftTrimSlider"
 type="range"
 min="70"
 max="110"
 value="100">

</div>



<div class="setting">

<div class="row">

<label>
Right Motor Trim
</label>

<span
 class="value"
 id="rightTrimValue">
100%
</span>

</div>

<input
 id="rightTrimSlider"
 type="range"
 min="70"
 max="110"
 value="100">

</div>



<div class="setting">

<div class="row">

<label>
ECO Speed Limit
</label>

<span
 class="value"
 id="ecoLimitValue">
85%
</span>

</div>

<input
 id="ecoLimitSlider"
 type="range"
 min="50"
 max="100"
 value="85">

</div>



<div class="setting">

<div class="row">

<div>

<label>
Swap Left / Right
</label>

<div class="hint">
Enabled for your current motor orientation.
</div>

</div>


<input
 id="swapSteeringCheckbox"
 type="checkbox"
 checked>

</div>

</div>



<div class="sectionTitle">
Direct RC_Car WiFi
</div>


<div class="setting">

<div class="row">

<div>

<label>
WiFi Channel
</label>

<div class="hint">
Used when Home WiFi is disconnected.
</div>

</div>


<select id="channelSelect">

<option value="1">
Channel 1
</option>

<option value="6">
Channel 6
</option>

<option value="11">
Channel 11
</option>

</select>

</div>


<button
 class="action"
 id="applyChannelButton">
APPLY CHANNEL
</button>


<div
 class="hint"
 id="channelMessage">
</div>

</div>


</div>

</div>



<script>

"use strict";


// ==================================================================
// DOM
// ==================================================================

function byId(id)
{
 return document.getElementById(id);
}


const ui =
{
 dot:
  byId("connectionDot"),

 connection:
  byId("connectionText"),

 latency:
  byId("latencyStat"),

 transport:
  byId("transportStat"),

 clients:
  byId("clientStat"),

 channel:
  byId("channelStat"),

 network:
  byId("networkStat"),


 speedSlider:
  byId("speedSlider"),

 speedNumber:
  byId("speedNumber"),

 eco:
  byId("ecoModeButton"),

 sport:
  byId("sportModeButton"),


 stop:
  byId("centerStopButton"),

 emergency:
  byId("emergencyStopButton"),


 wifiMain:
  byId("homeWifiMainButton"),

 wifiSide:
  byId("homeWifiSideButton"),

 wifiOverlay:
  byId("wifiOverlay"),

 wifiClose:
  byId("closeWifiPanel"),


 settingsMain:
  byId("settingsMainButton"),

 settingsOverlay:
  byId("settingsOverlay"),

 settingsClose:
  byId("closeSettingsPanel"),


 homeStatus:
  byId("homeWifiStatus"),

 homeMessage:
  byId("homeWifiMessage"),

 homeDetails:
  byId("homeWifiDetails"),

 homeAddressHint:
  byId("homeAddressHint"),


 scan:
  byId("scanNetworksButton"),

 networkList:
  byId("networkList"),

 ssid:
  byId("ssidInput"),

 password:
  byId("passwordInput"),

 showPassword:
  byId("showPasswordCheckbox"),

 connectWifi:
  byId("connectWifiButton"),

 forgetWifi:
  byId("forgetWifiButton"),


 steering:
  byId("steeringSelect"),

 turn:
  byId("turnSlider"),

 turnValue:
  byId("turnValue"),

 ramp:
  byId("rampSlider"),

 rampValue:
  byId("rampValue"),

 leftTrim:
  byId("leftTrimSlider"),

 leftTrimValue:
  byId("leftTrimValue"),

 rightTrim:
  byId("rightTrimSlider"),

 rightTrimValue:
  byId("rightTrimValue"),

 ecoLimit:
  byId("ecoLimitSlider"),

 ecoLimitValue:
  byId("ecoLimitValue"),

 swap:
  byId("swapSteeringCheckbox"),


 channelSelect:
  byId("channelSelect"),

 applyChannel:
  byId("applyChannelButton"),

 channelMessage:
  byId("channelMessage")
};


// ==================================================================
// LOCAL SETTINGS
// ==================================================================

const defaults =
{
 speed:70,

 turn:70,

 ramp:10,

 leftTrim:100,

 rightTrim:100,

 steering:1,

 swap:true,

 eco:true,

 ecoLimit:85
};


let cfg =
 Object.assign(
  {},
  defaults
 );


try
{
 const saved =
  JSON.parse(
   localStorage.getItem(
    "RC_CAR_CONFIG"
   )
  );


 if(saved)
 {
  Object.assign(
   cfg,
   saved
  );
 }
}
catch(error)
{
}


function saveLocalConfig()
{
 localStorage.setItem(
  "RC_CAR_CONFIG",
  JSON.stringify(cfg)
 );
}


// ==================================================================
// APPLY LOCAL UI
// ==================================================================

function applyLocalUI()
{
 ui.speedSlider.value =
  cfg.speed;


 ui.speedNumber.textContent =
  cfg.speed;


 ui.turn.value =
  cfg.turn;


 ui.turnValue.textContent =
  cfg.turn + "%";


 ui.ramp.value =
  cfg.ramp;


 ui.rampValue.textContent =
  cfg.ramp;


 ui.leftTrim.value =
  cfg.leftTrim;


 ui.leftTrimValue.textContent =
  cfg.leftTrim + "%";


 ui.rightTrim.value =
  cfg.rightTrim;


 ui.rightTrimValue.textContent =
  cfg.rightTrim + "%";


 ui.ecoLimit.value =
  cfg.ecoLimit;


 ui.ecoLimitValue.textContent =
  cfg.ecoLimit + "%";


 ui.steering.value =
  cfg.steering;


 ui.swap.checked =
  cfg.swap;


 ui.eco.classList.toggle(
  "selected",
  cfg.eco
 );


 ui.sport.classList.toggle(
  "selected",
  !cfg.eco
 );
}


// ==================================================================
// WEBSOCKET / HTTP FALLBACK
// ==================================================================

let ws = null;

let reconnectTimer = null;

let controlTransport =
 "HTTP";


function websocketReady()
{
 return (
  ws
  &&
  ws.readyState
  ===
  WebSocket.OPEN
 );
}


function showWebSocketReady()
{
 controlTransport =
  "WS";


 ui.dot.className =
  "dot ws";


 ui.connection.textContent =
  "WEBSOCKET • READY";


 ui.transport.textContent =
  "WS";
}


function showHTTPReady()
{
 if(websocketReady())
 {
  return;
 }


 controlTransport =
  "HTTP";


 ui.dot.className =
  "dot http";


 ui.connection.textContent =
  "HTTP BACKUP • READY";


 ui.transport.textContent =
  "HTTP";
}


function showOffline()
{
 if(websocketReady())
 {
  return;
 }


 ui.dot.className =
  "dot";


 ui.connection.textContent =
  "RECONNECTING...";


 ui.transport.textContent =
  "--";
}


// ==================================================================
// STATE
// ==================================================================

function applyState(state)
{
 ui.clients.textContent =
  state.clients;


 ui.channel.textContent =
  "CH" + state.channel;


 ui.channelSelect.value =
  state.savedChannel;


 if(state.homeConnected)
 {
  ui.network.textContent =
   "HOME";


  ui.channelSelect.disabled =
   true;


  ui.applyChannel.disabled =
   true;


  ui.channelMessage.textContent =
   "Home router currently controls the radio channel: CH "
   +
   state.channel;
 }

 else
 {
  ui.network.textContent =
   "AP";


  ui.channelSelect.disabled =
   false;


  ui.applyChannel.disabled =
   false;


  ui.channelMessage.textContent =
   "Try channels 1, 6 and 11 and keep the lowest-latency one.";
 }
}


// ==================================================================
// HTTP HEALTH CHECK
// ==================================================================

async function probeHTTP()
{
 try
 {
  const response =
   await fetch(
    "/api/state",
    {
     cache:"no-store"
    }
   );


  if(!response.ok)
  {
   throw new Error(
    "HTTP unavailable"
   );
  }


  const state =
   await response.json();


  applyState(
   state
  );


  if(!websocketReady())
  {
   showHTTPReady();
  }


  return true;
 }

 catch(error)
 {
  if(!websocketReady())
  {
   showOffline();
  }


  return false;
 }
}


// ==================================================================
// WEBSOCKET
// ==================================================================

function connectWebSocket()
{
 clearTimeout(
  reconnectTimer
 );


 try
 {
  ws =
   new WebSocket(
    "ws://"
    +
    location.hostname
    +
    ":81/"
   );
 }

 catch(error)
 {
  showHTTPReady();

  scheduleWebSocketReconnect();

  return;
 }


 ws.onopen =
  function()
  {
   showWebSocketReady();


   sendRuntimeConfig();


   sendSpeed(
    cfg.speed
   );
  };


 ws.onclose =
  function()
  {
   stopDrive();


   probeHTTP();


   scheduleWebSocketReconnect();
  };


 ws.onerror =
  function()
  {
   try
   {
    ws.close();
   }
   catch(error)
   {
   }
  };


 ws.onmessage =
  function(event)
  {
   const message =
    event.data;


   if(
    message.startsWith(
     "P:"
    )
   )
   {
    const start =
     Number(
      message.substring(2)
     );


    ui.latency.textContent =
     (
      Date.now()
      -
      start
     )
     +
     "ms";


    return;
   }


   if(
    message.startsWith(
     "CHANGING:"
    )
   )
   {
    ui.channelMessage.textContent =
     "Switching to channel "
     +
     message.split(":")[1]
     +
     "...";


    return;
   }


   if(
    message.startsWith(
     "CHANNEL_LOCKED:"
    )
   )
   {
    ui.channelMessage.textContent =
     "Home router controls channel "
     +
     message.split(":")[1];


    return;
   }
  };
}


function scheduleWebSocketReconnect()
{
 clearTimeout(
  reconnectTimer
 );


 reconnectTimer =
  setTimeout(
   connectWebSocket,
   1000
  );
}


// ==================================================================
// DRIVE TRANSPORT
// ==================================================================

function sendDrive(command)
{
 if(websocketReady())
 {
  ws.send(
   "D:"
   +
   command
  );


  return;
 }


 fetch(
  "/api/drive?c="
  +
  encodeURIComponent(command),

  {
   cache:"no-store"
  }

 ).catch(
  function(){}
 );
}


function sendSpeed(value)
{
 if(websocketReady())
 {
  ws.send(
   "S:"
   +
   value
  );


  return;
 }


 fetch(
  "/api/speed?v="
  +
  value,

  {
   cache:"no-store"
  }

 ).catch(
  function(){}
 );
}


// ==================================================================
// CONFIG SEND
// ==================================================================

let configTimer = null;


function sendRuntimeConfig()
{
 clearTimeout(
  configTimer
 );


 configTimer =
  setTimeout(
   function()
   {
    if(websocketReady())
    {
     ws.send(
      "C:"
      +
      cfg.turn + ","
      +
      cfg.leftTrim + ","
      +
      cfg.rightTrim + ","
      +
      cfg.ramp + ","
      +
      cfg.steering + ","
      +
      (cfg.swap ? 1 : 0) + ","
      +
      (cfg.eco ? 1 : 0) + ","
      +
      cfg.ecoLimit
     );
    }

    else
    {
     const url =
      "/api/config?"
      +
      "turn=" + cfg.turn
      +
      "&lt=" + cfg.leftTrim
      +
      "&rt=" + cfg.rightTrim
      +
      "&ramp=" + cfg.ramp
      +
      "&steer=" + cfg.steering
      +
      "&swap=" + (cfg.swap ? 1 : 0)
      +
      "&eco=" + (cfg.eco ? 1 : 0)
      +
      "&limit=" + cfg.ecoLimit;


     fetch(
      url,
      {
       cache:"no-store"
      }

     ).catch(
      function(){}
     );
    }
   },

   50
  );
}


// ==================================================================
// DRIVE BUTTONS
// ==================================================================

let driveTimer = null;

let activeButton = null;

let currentDriveCommand = "S";


function startDrive(
 command,
 button
)
{
 clearInterval(
  driveTimer
 );


 currentDriveCommand =
  command;


 if(activeButton)
 {
  activeButton.classList.remove(
   "active"
  );
 }


 activeButton =
  button;


 if(activeButton)
 {
  activeButton.classList.add(
   "active"
  );
 }


 sendDrive(
  command
 );


 driveTimer =
  setInterval(
   function()
   {
    sendDrive(
     currentDriveCommand
    );
   },

   websocketReady()
   ?
   120
   :
   200
  );
}


function stopDrive()
{
 clearInterval(
  driveTimer
 );


 driveTimer =
  null;


 currentDriveCommand =
  "S";


 if(activeButton)
 {
  activeButton.classList.remove(
   "active"
  );
 }


 activeButton =
  null;


 sendDrive(
  "S"
 );
}


document
.querySelectorAll(
 ".driveButton[data-drive]"
)
.forEach(
 function(button)
 {
  button.addEventListener(
   "pointerdown",

   function(event)
   {
    event.preventDefault();


    startDrive(
     button.dataset.drive,
     button
    );
   }
  );
 }
);


window.addEventListener(
 "pointerup",
 stopDrive
);


window.addEventListener(
 "pointercancel",
 stopDrive
);


ui.stop.addEventListener(
 "pointerdown",

 function(event)
 {
  event.preventDefault();

  stopDrive();
 }
);


ui.emergency.addEventListener(
 "pointerdown",

 function(event)
 {
  event.preventDefault();


  stopDrive();


  if(websocketReady())
  {
   ws.send(
    "X"
   );
  }

  else
  {
   fetch(
    "/api/stop",
    {
     cache:"no-store"
    }
   ).catch(
    function(){}
   );
  }


  if(navigator.vibrate)
  {
   navigator.vibrate(
    40
   );
  }
 }
);


// ==================================================================
// SPEED
// ==================================================================

ui.speedSlider.addEventListener(
 "input",

 function()
 {
  cfg.speed =
   Number(
    ui.speedSlider.value
   );


  ui.speedNumber.textContent =
   cfg.speed;


  saveLocalConfig();


  sendSpeed(
   cfg.speed
  );
 }
);


// ==================================================================
// ECO / SPORT
// ==================================================================

ui.eco.addEventListener(
 "click",

 function()
 {
  cfg.eco =
   true;


  saveLocalConfig();


  applyLocalUI();


  sendRuntimeConfig();
 }
);


ui.sport.addEventListener(
 "click",

 function()
 {
  cfg.eco =
   false;


  saveLocalConfig();


  applyLocalUI();


  sendRuntimeConfig();
 }
);


// ==================================================================
// OVERLAYS
// ==================================================================

function openWifiPanel()
{
 stopDrive();


 ui.wifiOverlay.classList.add(
  "show"
 );


 refreshHomeStatus();
}


ui.wifiMain.addEventListener(
 "click",
 openWifiPanel
);


ui.wifiSide.addEventListener(
 "click",
 openWifiPanel
);


ui.wifiClose.addEventListener(
 "click",

 function()
 {
  ui.wifiOverlay.classList.remove(
   "show"
  );
 }
);


ui.settingsMain.addEventListener(
 "click",

 function()
 {
  stopDrive();


  ui.settingsOverlay.classList.add(
   "show"
  );
 }
);


ui.settingsClose.addEventListener(
 "click",

 function()
 {
  ui.settingsOverlay.classList.remove(
   "show"
  );
 }
);


// ==================================================================
// CAR SETTINGS
// ==================================================================

ui.turn.addEventListener(
 "input",

 function()
 {
  cfg.turn =
   Number(
    ui.turn.value
   );


  saveLocalConfig();

  applyLocalUI();

  sendRuntimeConfig();
 }
);


ui.ramp.addEventListener(
 "input",

 function()
 {
  cfg.ramp =
   Number(
    ui.ramp.value
   );


  saveLocalConfig();

  applyLocalUI();

  sendRuntimeConfig();
 }
);


ui.leftTrim.addEventListener(
 "input",

 function()
 {
  cfg.leftTrim =
   Number(
    ui.leftTrim.value
   );


  saveLocalConfig();

  applyLocalUI();

  sendRuntimeConfig();
 }
);


ui.rightTrim.addEventListener(
 "input",

 function()
 {
  cfg.rightTrim =
   Number(
    ui.rightTrim.value
   );


  saveLocalConfig();

  applyLocalUI();

  sendRuntimeConfig();
 }
);


ui.ecoLimit.addEventListener(
 "input",

 function()
 {
  cfg.ecoLimit =
   Number(
    ui.ecoLimit.value
   );


  saveLocalConfig();

  applyLocalUI();

  sendRuntimeConfig();
 }
);


ui.steering.addEventListener(
 "change",

 function()
 {
  stopDrive();


  cfg.steering =
   Number(
    ui.steering.value
   );


  saveLocalConfig();

  sendRuntimeConfig();
 }
);


ui.swap.addEventListener(
 "change",

 function()
 {
  stopDrive();


  cfg.swap =
   ui.swap.checked;


  saveLocalConfig();

  sendRuntimeConfig();
 }
);


// ==================================================================
// CHANNEL
// ==================================================================

ui.applyChannel.addEventListener(
 "click",

 function()
 {
  stopDrive();


  const value =
   Number(
    ui.channelSelect.value
   );


  ui.channelMessage.textContent =
   "Applying channel...";


  if(websocketReady())
  {
   ws.send(
    "W:"
    +
    value
   );
  }

  else
  {
   fetch(
    "/api/channel?v="
    +
    value,

    {
     cache:"no-store"
    }

   ).catch(
    function(){}
   );
  }
 }
);


// ==================================================================
// HOME WIFI STATUS
// ==================================================================

async function refreshHomeStatus()
{
 try
 {
  const response =
   await fetch(
    "/api/wifi/status",

    {
     cache:"no-store"
    }
   );


  const data =
   await response.json();


  if(data.connected)
  {
   ui.homeStatus.textContent =
    "CONNECTED";


   ui.homeStatus.className =
    "good";


   ui.homeMessage.textContent =
    data.message;


   ui.homeDetails.innerHTML =
    "SSID: "
    +
    escapeHTML(data.ssid)
    +
    "<br>"
    +
    "IP: "
    +
    escapeHTML(data.ip)
    +
    "<br>"
    +
    "Signal: "
    +
    data.rssi
    +
    " dBm"
    +
    "<br>"
    +
    "Channel: "
    +
    data.channel;


   ui.homeAddressHint.innerHTML =
    "Open <b>http://rccar.local</b> or <b>http://"
    +
    escapeHTML(data.ip)
    +
    "</b> from devices on the same Home WiFi.";
  }

  else
  {
   ui.homeStatus.textContent =
    data.testing
    ?
    "CONNECTING..."
    :
    (
     data.configured
     ?
     "DISCONNECTED"
     :
     "NOT CONFIGURED"
    );


   ui.homeStatus.className =
    data.testing
    ?
    ""
    :
    (
     data.configured
     ?
     "bad"
     :
     ""
    );


   ui.homeMessage.textContent =
    data.message;


   ui.homeDetails.innerHTML =
    data.configured
    ?
    (
     "Saved SSID: "
     +
     escapeHTML(data.ssid)
    )
    :
    "No Home WiFi saved.";


   ui.homeAddressHint.textContent =
    "rccar.local becomes available after Home WiFi successfully connects.";
  }
 }

 catch(error)
 {
  ui.homeStatus.textContent =
   "STATUS ERROR";
 }
}


// ==================================================================
// WIFI SCAN
// ==================================================================

ui.scan.addEventListener(
 "click",

 async function()
 {
  stopDrive();


  ui.networkList.innerHTML =
   '<div class="hint">Scanning WiFi networks...</div>';


  ui.scan.disabled =
   true;


  try
  {
   const response =
    await fetch(
     "/api/wifi/scan",

     {
      cache:"no-store"
     }
    );


   const networks =
    await response.json();


   ui.networkList.innerHTML =
    "";


   networks.forEach(
    function(network)
    {
     const button =
      document.createElement(
       "button"
      );


     button.className =
      "networkItem";


     button.textContent =
      network.ssid
      +
      " • "
      +
      network.rssi
      +
      " dBm"
      +
      " • CH"
      +
      network.channel
      +
      (
       network.open
       ?
       " • OPEN"
       :
       " • 🔒"
      );


     button.addEventListener(
      "click",

      function()
      {
       ui.ssid.value =
        network.ssid;


       ui.networkList.innerHTML =
        '<div class="hint">Selected: '
        +
        escapeHTML(network.ssid)
        +
        '</div>';
      }
     );


     ui.networkList.appendChild(
      button
     );
    }
   );


   if(networks.length === 0)
   {
    ui.networkList.innerHTML =
     '<div class="hint">No networks found.</div>';
   }
  }

  catch(error)
  {
   ui.networkList.innerHTML =
    '<div class="hint">Scan failed. You can still enter SSID manually.</div>';
  }


  ui.scan.disabled =
   false;
 }
);


// ==================================================================
// SHOW PASSWORD
// ==================================================================

ui.showPassword.addEventListener(
 "change",

 function()
 {
  ui.password.type =
   ui.showPassword.checked
   ?
   "text"
   :
   "password";
 }
);


// ==================================================================
// CONNECT HOME WIFI
// ==================================================================

ui.connectWifi.addEventListener(
 "click",

 async function()
 {
  stopDrive();


  const ssid =
   ui.ssid.value.trim();


  const password =
   ui.password.value;


  if(!ssid)
  {
   ui.homeMessage.textContent =
    "Enter a WiFi name / SSID.";

   return;
  }


  if(
   password.length > 0
   &&
   password.length < 8
  )
  {
   ui.homeMessage.textContent =
    "Password must normally be at least 8 characters. Leave it blank only for an open network.";

   return;
  }


  ui.connectWifi.disabled =
   true;


  ui.homeMessage.textContent =
   "Sending credentials...";


  const data =
   new URLSearchParams();


  data.set(
   "ssid",
   ssid
  );


  data.set(
   "pass",
   password
  );


  try
  {
   const response =
    await fetch(
     "/api/wifi/connect",

     {
      method:"POST",

      headers:
      {
       "Content-Type":
        "application/x-www-form-urlencoded"
      },

      body:
       data.toString()
     }
    );


   ui.homeMessage.textContent =
    await response.text();


   /*
      The AP may briefly move to the
      router channel.

      Browser/WS automatically reconnects.
   */

   setTimeout(
    refreshHomeStatus,
    1500
   );


   setTimeout(
    refreshHomeStatus,
    5000
   );
  }

  catch(error)
  {
   ui.homeMessage.textContent =
    "Connection request was interrupted. RC_Car recovery WiFi should still remain available.";
  }


  ui.connectWifi.disabled =
   false;
 }
);


// ==================================================================
// FORGET WIFI
// ==================================================================

ui.forgetWifi.addEventListener(
 "click",

 async function()
 {
  stopDrive();


  if(
   !confirm(
    "Forget saved Home WiFi? RC_Car direct WiFi will remain available."
   )
  )
  {
   return;
  }


  try
  {
   await fetch(
    "/api/wifi/forget",

    {
     method:"POST"
    }
   );


   ui.ssid.value =
    "";


   ui.password.value =
    "";


   ui.homeMessage.textContent =
    "Forgetting Home WiFi...";


   setTimeout(
    refreshHomeStatus,
    1200
   );
  }

  catch(error)
  {
  }
 }
);


// ==================================================================
// PING
// ==================================================================

async function measureLatency()
{
 const start =
  Date.now();


 if(websocketReady())
 {
  ws.send(
   "P:"
   +
   start
  );


  return;
 }


 try
 {
  await fetch(
   "/api/ping?t="
   +
   start,

   {
    cache:"no-store"
   }
  );


  ui.latency.textContent =
   (
    Date.now()
    -
    start
   )
   +
   "ms";
 }

 catch(error)
 {
 }
}


// ==================================================================
// HTML ESCAPE
// ==================================================================

function escapeHTML(text)
{
 const div =
  document.createElement(
   "div"
  );


 div.textContent =
  text;


 return div.innerHTML;
}


// ==================================================================
// SAFETY EVENTS
// ==================================================================

document.addEventListener(
 "visibilitychange",

 function()
 {
  if(document.hidden)
  {
   stopDrive();
  }
 }
);


window.addEventListener(
 "pagehide",
 stopDrive
);


window.addEventListener(
 "blur",
 stopDrive
);


window.addEventListener(
 "contextmenu",

 function(event)
 {
  event.preventDefault();
 }
);


// ==================================================================
// START
// ==================================================================

applyLocalUI();


probeHTTP();


connectWebSocket();


refreshHomeStatus();


setInterval(
 measureLatency,
 1800
);


setInterval(
 probeHTTP,
 3500
);


setInterval(
 refreshHomeStatus,
 5000
);

</script>

</body>

</html>
)HTML";


// =====================================================================
// WEB ROOT
// =====================================================================

void handleRoot()
{
  server.sendHeader(
    "Cache-Control",
    "no-store, no-cache, must-revalidate"
  );


  server.sendHeader(
    "Pragma",
    "no-cache"
  );


  server.send_P(
    200,
    "text/html",
    PAGE
  );
}


// =====================================================================
// BUILD GENERAL STATE JSON
// =====================================================================

String buildStateJSON()
{
  bool homeConnected =
    (
      WiFi.status()
      ==
      WL_CONNECTED
    );


  int actualChannel =
    homeConnected
    ?
    WiFi.channel()
    :
    apChannel;


  String json;


  json.reserve(
    160
  );


  json +=
    "{";


  json +=
    "\"clients\":";


  json +=
    String(
      WiFi.softAPgetStationNum()
    );


  json +=
    ",\"channel\":";


  json +=
    String(
      actualChannel
    );


  json +=
    ",\"savedChannel\":";


  json +=
    String(
      apChannel
    );


  json +=
    ",\"homeConnected\":";


  json +=
    (
      homeConnected
      ?
      "true"
      :
      "false"
    );


  json +=
    ",\"heap\":";


  json +=
    String(
      ESP.getFreeHeap()
    );


  json +=
    "}";


  return json;
}


// =====================================================================
// STATE API
// =====================================================================

void handleState()
{
  server.sendHeader(
    "Cache-Control",
    "no-store"
  );


  server.send(
    200,
    "application/json",
    buildStateJSON()
  );
}


// =====================================================================
// HTTP DRIVE FALLBACK
// =====================================================================

void handleDriveHTTP()
{
  if(
    !server.hasArg("c")
  )
  {
    server.send(
      400,
      "text/plain",
      "Missing command"
    );


    return;
  }


  String value =
    server.arg("c");


  if(
    value.length()
    ==
    0
  )
  {
    server.send(
      400,
      "text/plain",
      "Invalid command"
    );


    return;
  }


  driveCommand(
    value.charAt(0)
  );


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// =====================================================================
// HTTP SPEED
// =====================================================================

void handleSpeedHTTP()
{
  if(
    server.hasArg("v")
  )
  {
    speedPercent =
      constrain(
        server.arg("v").toInt(),
        20,
        100
      );


    if(
      activeCommand
      !=
      'S'
    )
    {
      calculateDrive(
        activeCommand
      );
    }
  }


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// =====================================================================
// RUNTIME CONFIG APPLY
// =====================================================================

void applyRuntimeConfig(
  int turn,
  int lt,
  int rt,
  int ramp,
  int steering,
  int swap,
  int eco,
  int limit
)
{
  turnStrength =
    constrain(
      turn,
      30,
      100
    );


  leftTrim =
    constrain(
      lt,
      70,
      110
    );


  rightTrim =
    constrain(
      rt,
      70,
      110
    );


  rampStep =
    constrain(
      ramp,
      4,
      30
    );


  steeringMode =
    constrain(
      steering,
      0,
      1
    );


  swapSteering =
    (
      swap
      !=
      0
    );


  ecoMode =
    (
      eco
      !=
      0
    );


  ecoSpeedLimit =
    constrain(
      limit,
      50,
      100
    );


  if(
    activeCommand
    !=
    'S'
  )
  {
    calculateDrive(
      activeCommand
    );
  }
}


// =====================================================================
// HTTP CONFIG FALLBACK
// =====================================================================

void handleConfigHTTP()
{
  if(
       !server.hasArg("turn")
    || !server.hasArg("lt")
    || !server.hasArg("rt")
    || !server.hasArg("ramp")
    || !server.hasArg("steer")
    || !server.hasArg("swap")
    || !server.hasArg("eco")
    || !server.hasArg("limit")
  )
  {
    server.send(
      400,
      "text/plain",
      "Missing config"
    );


    return;
  }


  applyRuntimeConfig(

    server.arg("turn").toInt(),

    server.arg("lt").toInt(),

    server.arg("rt").toInt(),

    server.arg("ramp").toInt(),

    server.arg("steer").toInt(),

    server.arg("swap").toInt(),

    server.arg("eco").toInt(),

    server.arg("limit").toInt()

  );


  server.send(
    200,
    "text/plain",
    "OK"
  );
}


// =====================================================================
// HTTP STOP
// =====================================================================

void handleStopHTTP()
{
  stopCar();


  server.send(
    200,
    "text/plain",
    "STOPPED"
  );
}


// =====================================================================
// HTTP PING
// =====================================================================

void handlePing()
{
  if(
    server.hasArg("t")
  )
  {
    server.send(
      200,
      "text/plain",
      server.arg("t")
    );
  }

  else
  {
    server.send(
      200,
      "text/plain",
      "PONG"
    );
  }
}


// =====================================================================
// HOME WIFI STATUS
// =====================================================================

void handleHomeWiFiStatus()
{
  bool connected =
    (
      WiFi.status()
      ==
      WL_CONNECTED
    );


  String ssid;


  if(connected)
  {
    ssid =
      WiFi.SSID();
  }


  else if(
    savedHomeWiFiExists()
  )
  {
    ssid =
      persistentConfig.homeSSID;
  }


  String json;


  json.reserve(
    320
  );


  json +=
    "{";


  json +=
    "\"configured\":";


  json +=
    (
      savedHomeWiFiExists()
      ?
      "true"
      :
      "false"
    );


  json +=
    ",\"testing\":";


  json +=
    (
      homeTestActive
      ||
      pendingHomeConnect
      ?
      "true"
      :
      "false"
    );


  json +=
    ",\"connected\":";


  json +=
    (
      connected
      ?
      "true"
      :
      "false"
    );


  json +=
    ",\"ssid\":\"";


  json +=
    jsonEscape(
      ssid
    );


  json +=
    "\"";


  json +=
    ",\"ip\":\"";


  if(connected)
  {
    json +=
      WiFi.localIP().toString();
  }


  json +=
    "\"";


  json +=
    ",\"rssi\":";


  json +=
    (
      connected
      ?
      String(
        WiFi.RSSI()
      )
      :
      String(0)
    );


  json +=
    ",\"channel\":";


  json +=
    String(
      connected
      ?
      WiFi.channel()
      :
      apChannel
    );


  json +=
    ",\"message\":\"";


  json +=
    jsonEscape(
      homeStatusMessage
    );


  json +=
    "\"";


  json +=
    "}";


  server.sendHeader(
    "Cache-Control",
    "no-store"
  );


  server.send(
    200,
    "application/json",
    json
  );
}


// =====================================================================
// WIFI SCAN
// =====================================================================

void handleWiFiScan()
{
  stopCar();


  /*
     Temporarily enable STA capability
     so scanning works.

     AP stays enabled.
  */

  if(
    WiFi.getMode()
    ==
    WIFI_AP
  )
  {
    WiFi.mode(
      WIFI_AP_STA
    );
  }


  int networkCount =
    WiFi.scanNetworks(
      false,
      true
    );


  String json;


  json.reserve(
    1600
  );


  json +=
    "[";


  int added =
    0;


  if(
    networkCount
    >
    0
  )
  {
    for(
      int i = 0;
      i < networkCount && added < 15;
      i++
    )
    {
      String ssid =
        WiFi.SSID(i);


      if(
        ssid.length()
        ==
        0
      )
      {
        continue;
      }


      if(
        added
        >
        0
      )
      {
        json +=
          ",";
      }


      json +=
        "{";


      json +=
        "\"ssid\":\"";


      json +=
        jsonEscape(
          ssid
        );


      json +=
        "\"";


      json +=
        ",\"rssi\":";


      json +=
        String(
          WiFi.RSSI(i)
        );


      json +=
        ",\"channel\":";


      json +=
        String(
          WiFi.channel(i)
        );


      json +=
        ",\"open\":";


      json +=
        (
          WiFi.encryptionType(i)
          ==
          ENC_TYPE_NONE
          ?
          "true"
          :
          "false"
        );


      json +=
        "}";


      added++;
    }
  }


  json +=
    "]";


  WiFi.scanDelete();


  server.sendHeader(
    "Cache-Control",
    "no-store"
  );


  server.send(
    200,
    "application/json",
    json
  );
}


// =====================================================================
// REQUEST HOME WIFI CONNECTION
// =====================================================================

void handleHomeWiFiConnect()
{
  stopCar();


  if(
    !server.hasArg("ssid")
  )
  {
    server.send(
      400,
      "text/plain",
      "Missing WiFi name"
    );


    return;
  }


  String ssid =
    server.arg("ssid");


  String password =
    server.arg("pass");


  ssid.trim();


  if(
    ssid.length() < 1
    ||
    ssid.length() > 32
  )
  {
    server.send(
      400,
      "text/plain",
      "WiFi name must be 1-32 characters"
    );


    return;
  }


  if(
    password.length()
    >
    64
  )
  {
    server.send(
      400,
      "text/plain",
      "Password is too long"
    );


    return;
  }


  if(
    password.length() > 0
    &&
    password.length() < 8
  )
  {
    server.send(
      400,
      "text/plain",
      "Password must be at least 8 characters, or blank for an open network"
    );


    return;
  }


  memset(
    testSSID,
    0,
    sizeof(testSSID)
  );


  memset(
    testPassword,
    0,
    sizeof(testPassword)
  );


  strncpy(
    testSSID,
    ssid.c_str(),
    sizeof(testSSID)-1
  );


  strncpy(
    testPassword,
    password.c_str(),
    sizeof(testPassword)-1
  );


  /*
     Important:

     Respond to browser FIRST.

     Then start station connection after
     800ms from loop().

     This reduces the chance of losing
     the HTTP acknowledgement when the
     ESP radio moves to the router channel.
  */

  pendingHomeConnect =
    true;


  pendingHomeConnectAt =
    millis()
    +
    800;


  homeStatusMessage =
    "Preparing Home WiFi connection...";


  server.send(
    200,
    "text/plain",
    "Credentials received. Testing connection now..."
  );
}


// =====================================================================
// FORGET HOME WIFI
// =====================================================================

void handleForgetHomeWiFi()
{
  stopCar();


  /*
     Same idea:

     reply first,
     change radio state afterwards.
  */

  pendingForgetHome =
    true;


  pendingForgetAt =
    millis()
    +
    500;


  server.send(
    200,
    "text/plain",
    "Forgetting Home WiFi..."
  );
}


// =====================================================================
// CHANNEL API
// =====================================================================

void handleChannelHTTP()
{
  if(
    WiFi.status()
    ==
    WL_CONNECTED
  )
  {
    server.send(
      409,
      "text/plain",
      "Home router controls the channel"
    );


    return;
  }


  if(
    !server.hasArg("v")
  )
  {
    server.send(
      400,
      "text/plain",
      "Missing channel"
    );


    return;
  }


  int channel =
    server.arg("v").toInt();


  if(
       channel != 1
    && channel != 6
    && channel != 11
  )
  {
    server.send(
      400,
      "text/plain",
      "Invalid channel"
    );


    return;
  }


  requestedChannel =
    channel;


  requestedChannelAt =
    millis()
    +
    600;


  pendingChannelChange =
    true;


  server.send(
    200,
    "text/plain",
    "Changing channel..."
  );
}


// =====================================================================
// WEBSOCKET
// =====================================================================

void webSocketEvent(
  uint8_t client,
  WStype_t type,
  uint8_t* payload,
  size_t length
)
{
  if(
    type
    ==
    WStype_CONNECTED
  )
  {
    webSocket.sendTXT(
      client,
      "READY"
    );


    return;
  }


  if(
    type
    ==
    WStype_DISCONNECTED
  )
  {
    /*
       No immediate stop here because browsers
       can reconnect momentarily during AP/STA
       channel transitions.

       CONTROL_TIMEOUT is the independent
       motor failsafe.
    */

    return;
  }


  if(
    type
    !=
    WStype_TEXT
  )
  {
    return;
  }


  if(
    length
    ==
    0
  )
  {
    return;
  }


  char message[120];


  size_t copyLength =
    min(
      length,
      sizeof(message)-1
    );


  memcpy(
    message,
    payload,
    copyLength
  );


  message[copyLength] =
    '\0';


  // -----------------------------------------------------------------
  // DRIVE
  // -----------------------------------------------------------------

  if(
    message[0] == 'D'
    &&
    message[1] == ':'
  )
  {
    driveCommand(
      message[2]
    );


    return;
  }


  // -----------------------------------------------------------------
  // EMERGENCY STOP
  // -----------------------------------------------------------------

  if(
    message[0]
    ==
    'X'
  )
  {
    stopCar();


    return;
  }


  // -----------------------------------------------------------------
  // SPEED
  // -----------------------------------------------------------------

  if(
    message[0] == 'S'
    &&
    message[1] == ':'
  )
  {
    speedPercent =
      constrain(
        atoi(
          message+2
        ),
        20,
        100
      );


    if(
      activeCommand
      !=
      'S'
    )
    {
      calculateDrive(
        activeCommand
      );
    }


    return;
  }


  // -----------------------------------------------------------------
  // CONFIG
  // -----------------------------------------------------------------

  if(
    message[0] == 'C'
    &&
    message[1] == ':'
  )
  {
    int turn;

    int lt;

    int rt;

    int ramp;

    int steering;

    int swap;

    int eco;

    int limit;


    int count =
      sscanf(

        message+2,

        "%d,%d,%d,%d,%d,%d,%d,%d",

        &turn,

        &lt,

        &rt,

        &ramp,

        &steering,

        &swap,

        &eco,

        &limit

      );


    if(
      count
      ==
      8
    )
    {
      applyRuntimeConfig(

        turn,

        lt,

        rt,

        ramp,

        steering,

        swap,

        eco,

        limit

      );
    }


    return;
  }


  // -----------------------------------------------------------------
  // PING
  // -----------------------------------------------------------------

  if(
    message[0] == 'P'
    &&
    message[1] == ':'
  )
  {
    webSocket.sendTXT(
      client,
      message
    );


    return;
  }


  // -----------------------------------------------------------------
  // CHANNEL
  // -----------------------------------------------------------------

  if(
    message[0] == 'W'
    &&
    message[1] == ':'
  )
  {
    int channel =
      atoi(
        message+2
      );


    if(
      WiFi.status()
      ==
      WL_CONNECTED
    )
    {
      char response[32];


      snprintf(
        response,
        sizeof(response),
        "CHANNEL_LOCKED:%d",
        WiFi.channel()
      );


      webSocket.sendTXT(
        client,
        response
      );


      return;
    }


    if(
         channel != 1
      && channel != 6
      && channel != 11
    )
    {
      return;
    }


    requestedChannel =
      channel;


    requestedChannelAt =
      millis()
      +
      600;


    pendingChannelChange =
      true;


    char response[24];


    snprintf(
      response,
      sizeof(response),
      "CHANGING:%d",
      channel
    );


    webSocket.sendTXT(
      client,
      response
    );


    return;
  }
}


// =====================================================================
// SETUP
// =====================================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(
    100
  );


  Serial.println();

  Serial.println(
    "======================================"
  );

  Serial.println(
    "       RC CAR PRO - BOOTING"
  );

  Serial.println(
    "======================================"
  );


  // -----------------------------------------------------------------
  // LOAD SETTINGS
  // -----------------------------------------------------------------

  loadPersistentConfig();


  // -----------------------------------------------------------------
  // MOTOR PINS
  // -----------------------------------------------------------------

  pinMode(
    IN1,
    OUTPUT
  );


  pinMode(
    IN2,
    OUTPUT
  );


  pinMode(
    IN3,
    OUTPUT
  );


  pinMode(
    IN4,
    OUTPUT
  );


  pinMode(
    ENA,
    OUTPUT
  );


  pinMode(
    ENB,
    OUTPUT
  );


  analogWriteRange(
    255
  );


  analogWriteFreq(
    1000
  );


  stopCar();


  // -----------------------------------------------------------------
  // WIFI BEHAVIOUR
  // -----------------------------------------------------------------

  WiFi.persistent(
    false
  );


  /*
     We manage reconnection ourselves so the
     radio does not unexpectedly channel-hop
     while the car is moving.
  */

  WiFi.setAutoConnect(
    false
  );


  WiFi.setAutoReconnect(
    false
  );


  WiFiClient::setDefaultNoDelay(
    true
  );


  WiFi.hostname(
    MDNS_HOST
  );


  // -----------------------------------------------------------------
  // START AP
  // -----------------------------------------------------------------

  bool apStarted =
    false;


  for(
    int attempt = 0;
    attempt < 3;
    attempt++
  )
  {
    if(
      startSoftAP(
        savedHomeWiFiExists()
      )
    )
    {
      apStarted =
        true;


      break;
    }


    delay(
      100
    );
  }


  if(!apStarted)
  {
    Serial.println(
      "AP failed - rebooting"
    );


    delay(
      300
    );


    ESP.restart();
  }


  // -----------------------------------------------------------------
  // WEB ROUTES
  // -----------------------------------------------------------------

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );


  server.on(
    "/api/state",
    HTTP_GET,
    handleState
  );


  server.on(
    "/api/drive",
    HTTP_GET,
    handleDriveHTTP
  );


  server.on(
    "/api/speed",
    HTTP_GET,
    handleSpeedHTTP
  );


  server.on(
    "/api/config",
    HTTP_GET,
    handleConfigHTTP
  );


  server.on(
    "/api/stop",
    HTTP_GET,
    handleStopHTTP
  );


  server.on(
    "/api/ping",
    HTTP_GET,
    handlePing
  );


  server.on(
    "/api/channel",
    HTTP_GET,
    handleChannelHTTP
  );


  server.on(
    "/api/wifi/status",
    HTTP_GET,
    handleHomeWiFiStatus
  );


  server.on(
    "/api/wifi/scan",
    HTTP_GET,
    handleWiFiScan
  );


  server.on(
    "/api/wifi/connect",
    HTTP_POST,
    handleHomeWiFiConnect
  );


  server.on(
    "/api/wifi/forget",
    HTTP_POST,
    handleForgetHomeWiFi
  );


  server.onNotFound(
    handleRoot
  );


  server.begin();


  // -----------------------------------------------------------------
  // WEBSOCKET
  // -----------------------------------------------------------------

  webSocket.begin();


  webSocket.onEvent(
    webSocketEvent
  );


  // -----------------------------------------------------------------
  // SAVED HOME WIFI
  // -----------------------------------------------------------------

  if(
    savedHomeWiFiExists()
  )
  {
    homeStatusMessage =
      "Connecting saved Home WiFi...";


    beginHomeWiFi(

      persistentConfig.homeSSID,

      persistentConfig.homePassword

    );
  }

  else
  {
    /*
       YOUR CURRENT CASE:

       No Home WiFi is configured.

       Stay AP-only.
    */

    homeStatusMessage =
      "No Home WiFi configured. RC_Car direct mode active.";
  }


  // -----------------------------------------------------------------
  // DEBUG
  // -----------------------------------------------------------------

  Serial.println(
    "RC_Car AP: READY"
  );


  Serial.print(
    "SSID: "
  );


  Serial.println(
    AP_SSID
  );


  Serial.print(
    "AP IP: "
  );


  Serial.println(
    WiFi.softAPIP()
  );


  Serial.print(
    "Preferred AP Channel: "
  );


  Serial.println(
    apChannel
  );


  Serial.print(
    "Home WiFi saved: "
  );


  Serial.println(
    savedHomeWiFiExists()
    ?
    "YES"
    :
    "NO"
  );


  Serial.println(
    "HTTP: port 80"
  );


  Serial.println(
    "WebSocket: port 81"
  );


  Serial.println(
    "Open http://192.168.4.1"
  );


  Serial.println(
    "======================================"
  );


  lastControlPacket =
    millis();


  lastSavedHomeAttempt =
    millis();
}


// =====================================================================
// LOOP
// =====================================================================

void loop()
{
  // HTTP

  server.handleClient();


  // WebSocket

  webSocket.loop();


  // mDNS

  if(
    mdnsRunning
    &&
    MDNS.isRunning()
  )
  {
    MDNS.update();
  }


  // Motors

  updateMotors();


  // -----------------------------------------------------------------
  // MOTOR FAILSAFE
  // -----------------------------------------------------------------

  if(
    activeCommand
    !=
    'S'
  )
  {
    if(
      millis()
      -
      lastControlPacket
      >
      CONTROL_TIMEOUT
    )
    {
      stopCar();
    }
  }


  // -----------------------------------------------------------------
  // NETWORK ACTIONS
  // -----------------------------------------------------------------

  startPendingHomeTest();


  processPendingForget();


  manageHomeWiFi();


  processChannelChange();


  checkAccessPointHealth();


  // -----------------------------------------------------------------
  // ESP8266 WIFI / WATCHDOG
  // -----------------------------------------------------------------

  yield();
}
