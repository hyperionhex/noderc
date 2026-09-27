/*
  =====================================================================
        ROBUST LOW-LATENCY WIFI RC CAR
        NodeMCU ESP8266 + L298N + 2 TT Motors + Front Caster
  =====================================================================

  WiFi:
      SSID     : RC_Car
      Password : 12345678

  Controller:
      http://192.168.4.1

  REQUIRED LIBRARY:
      WebSockets by Markus Sattler

  =====================================================================
  WIRING
  =====================================================================

  NodeMCU ESP8266            L298N
  ---------------------------------------
  D1 / GPIO5      -------->  IN1
  D2 / GPIO4      -------->  IN2

  D5 / GPIO14     -------->  IN3
  D6 / GPIO12     -------->  IN4

  D7 / GPIO13     -------->  ENA
  D0 / GPIO16     -------->  ENB

  GND             -------->  GND
  VIN             <--------  L298N 5V

  REMOVE:
      ENA jumper
      ENB jumper

  KEEP:
      L298N 5V regulator jumper if powering NodeMCU from L298N 5V.

  Motor A:
      OUT1 + OUT2 -> Left motor

  Motor B:
      OUT3 + OUT4 -> Right motor

  Battery:
      2S Lithium positive -> L298N +12V / VIN
      2S Lithium negative -> L298N GND

  =====================================================================
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>


// =====================================================================
// WIFI
// =====================================================================

const char* AP_SSID = "RC_Car";
const char* AP_PASS = "12345678";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

ESP8266WebServer server(80);
WebSocketsServer webSocket(81);


// =====================================================================
// MOTOR PINS
// =====================================================================

// LEFT MOTOR
#define IN1 D1
#define IN2 D2
#define ENA D7

// RIGHT MOTOR
#define IN3 D5
#define IN4 D6
#define ENB D0


// =====================================================================
// EEPROM
// =====================================================================

#define EEPROM_SIZE       16
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_CH_ADDR    1
#define EEPROM_MAGIC      0xA7

uint8_t apChannel = 1;


// =====================================================================
// WIFI SETTINGS
// =====================================================================

// Adjustable from WebUI.
//
// Values supported in UI:
// 8.5
// 11
// 14
// 17.5
// 20.5 dBm

float wifiTxPower = 14.0;


// =====================================================================
// CAR SETTINGS
// =====================================================================

int speedPercent = 70;

int turnStrength = 70;

// Straight-line calibration
int leftTrim  = 100;
int rightTrim = 100;

// Smooth acceleration
int rampStep = 10;

// 0 = Pivot steering
// 1 = Smooth steering
int steeringMode = 1;

// Your current physical car has reversed L/R,
// therefore this starts enabled.
bool swapSteering = true;

// Battery-saving motor mode
bool ecoMode = true;

int ecoSpeedLimit = 85;


// =====================================================================
// SAFETY SETTINGS
// =====================================================================

// Browser heartbeat = 100 ms.
//
// If communication stops for >450ms,
// motors are killed by ESP itself.
const unsigned long CONTROL_TIMEOUT = 450;

unsigned long lastControlPacket = 0;


// AP health check
const unsigned long AP_CHECK_INTERVAL = 5000;

unsigned long lastAPCheck = 0;


// UI status push
unsigned long lastStatusPush = 0;


// =====================================================================
// CHANNEL CHANGE
// =====================================================================

bool channelChangePending = false;

uint8_t pendingChannel = 1;

unsigned long channelChangeAt = 0;


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


// Only one phone controls motors at a time.
int controllerClient = -1;


// =====================================================================
// EEPROM FUNCTIONS
// =====================================================================

bool validWiFiChannel(uint8_t channel)
{
  return (
       channel == 1
    || channel == 6
    || channel == 11
  );
}


void loadWiFiChannel()
{
  EEPROM.begin(EEPROM_SIZE);

  uint8_t magic =
    EEPROM.read(
      EEPROM_MAGIC_ADDR
    );

  uint8_t stored =
    EEPROM.read(
      EEPROM_CH_ADDR
    );


  if (
    magic == EEPROM_MAGIC
    &&
    validWiFiChannel(stored)
  )
  {
    apChannel = stored;
  }
  else
  {
    apChannel = 1;
  }
}


void saveWiFiChannel(uint8_t channel)
{
  if (!validWiFiChannel(channel))
    return;


  // Avoid unnecessary flash writes.

  if (
    EEPROM.read(EEPROM_MAGIC_ADDR)
      == EEPROM_MAGIC
    &&
    EEPROM.read(EEPROM_CH_ADDR)
      == channel
  )
  {
    return;
  }


  EEPROM.write(
    EEPROM_MAGIC_ADDR,
    EEPROM_MAGIC
  );

  EEPROM.write(
    EEPROM_CH_ADDR,
    channel
  );

  EEPROM.commit();
}


// =====================================================================
// MOTOR HELPERS
// =====================================================================

int percentToPWM(int percent)
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
  int trimValue
)
{
  long result =
    ((long)pwm * trimValue)
    / 100;

  return constrain(
    result,
    -255,
    255
  );
}


// =====================================================================
// MOTOR OUTPUT
// =====================================================================

void setLeftMotor(int pwm)
{
  pwm =
    constrain(
      pwm,
      -255,
      255
    );


  if (pwm > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(
      ENA,
      pwm
    );
  }

  else if (pwm < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(
      ENA,
      -pwm
    );
  }

  else
  {
    analogWrite(ENA, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}


void setRightMotor(int pwm)
{
  pwm =
    constrain(
      pwm,
      -255,
      255
    );


  /*
      Right motor direction is electrically
      reversed because the motor is physically
      mounted opposite the left motor.
  */

  if (pwm > 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(
      ENB,
      pwm
    );
  }

  else if (pwm < 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(
      ENB,
      -pwm
    );
  }

  else
  {
    analogWrite(ENB, 0);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
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


  analogWrite(ENA, 0);
  analogWrite(ENB, 0);


  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// =====================================================================
// SAFE MOTOR RAMP
// =====================================================================

int rampMotor(
  int current,
  int target
)
{
  /*
      Never jump immediately:

          +200 -> -200

      Instead:

          +200
          +150
          +100
          +50
          0
          -50
          -100...

      This reduces:
          motor current spike
          battery voltage sag
          L298N stress
          ESP8266 reset possibility
  */


  int temporaryTarget =
    target;


  // Must reach zero before reversing

  if (
       (current > 0 && target < 0)
    || (current < 0 && target > 0)
  )
  {
    temporaryTarget = 0;
  }


  if (current < temporaryTarget)
  {
    current += rampStep;

    if (current > temporaryTarget)
      current = temporaryTarget;
  }

  else if (current > temporaryTarget)
  {
    current -= rampStep;

    if (current < temporaryTarget)
      current = temporaryTarget;
  }


  return current;
}


void updateMotorRamp()
{
  unsigned long now =
    millis();


  if (
    now - lastRampUpdate
    <
    RAMP_INTERVAL
  )
  {
    return;
  }


  lastRampUpdate = now;


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
// STEERING CORRECTION
// =====================================================================

char correctSteering(char command)
{
  if (!swapSteering)
    return command;


  switch (command)
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
// CALCULATE MOTOR TARGET
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


  // ECO motor speed limiter

  if (ecoMode)
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
    ((long)base *
      (100 - turnStrength))
    / 100;


  int pivot =
    ((long)base *
      turnStrength)
    / 100;


  /*
      Pivot turns draw considerably more current.

      Limit them when ECO is enabled.
  */

  if (ecoMode)
  {
    int maxPivot =
      percentToPWM(65);

    pivot =
      min(
        pivot,
        maxPivot
      );
  }


  int left = 0;
  int right = 0;


  switch (command)
  {

    // ---------------------------------------------------------
    // FORWARD
    // ---------------------------------------------------------

    case 'F':

      left = base;
      right = base;

      break;


    // ---------------------------------------------------------
    // BACKWARD
    // ---------------------------------------------------------

    case 'B':

      left = -base;
      right = -base;

      break;


    // ---------------------------------------------------------
    // LEFT
    // ---------------------------------------------------------

    case 'L':

      if (steeringMode == 0)
      {
        // Pivot

        left = -pivot;
        right = pivot;
      }

      else
      {
        // Smooth

        left = inside;
        right = base;
      }

      break;


    // ---------------------------------------------------------
    // RIGHT
    // ---------------------------------------------------------

    case 'R':

      if (steeringMode == 0)
      {
        left = pivot;
        right = -pivot;
      }

      else
      {
        left = base;
        right = inside;
      }

      break;


    // ---------------------------------------------------------
    // FORWARD LEFT
    // ---------------------------------------------------------

    case 'G':

      left = inside;
      right = base;

      break;


    // ---------------------------------------------------------
    // FORWARD RIGHT
    // ---------------------------------------------------------

    case 'H':

      left = base;
      right = inside;

      break;


    // ---------------------------------------------------------
    // BACK LEFT
    // ---------------------------------------------------------

    case 'I':

      left = -inside;
      right = -base;

      break;


    // ---------------------------------------------------------
    // BACK RIGHT
    // ---------------------------------------------------------

    case 'J':

      left = -base;
      right = -inside;

      break;


    default:

      left = 0;
      right = 0;

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
// COMMAND VALIDATION
// =====================================================================

bool validDriveCommand(char command)
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

void driveCommand(char command)
{
  if (!validDriveCommand(command))
  {
    stopCar();

    return;
  }


  lastControlPacket =
    millis();


  if (command == 'S')
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
// WIFI AP
// =====================================================================

bool createAccessPoint()
{
  /*
      Don't store WiFi configuration
      unnecessarily in ESP flash.
  */

  WiFi.persistent(false);


  WiFi.mode(
    WIFI_AP
  );


  /*
      Low latency mode.

      WiFi sleep intentionally disabled.
  */

  WiFi.setSleepMode(
    WIFI_NONE_SLEEP
  );


  /*
      Force 802.11g.

      More than enough throughput
      for our tiny WebSocket packets.
  */

  WiFi.setPhyMode(
    WIFI_PHY_MODE_11G
  );


  /*
      Adjustable from WebUI.
  */

  WiFi.setOutputPower(
    wifiTxPower
  );


  if (
    !WiFi.softAPConfig(
      AP_IP,
      AP_GATEWAY,
      AP_SUBNET
    )
  )
  {
    return false;
  }


  bool started =
    WiFi.softAP(
      AP_SSID,
      AP_PASS,

      apChannel,

      false,

      2
    );


  return started;
}


// =====================================================================
// CHANGE WIFI CHANNEL
// =====================================================================

void changeWiFiChannelNow()
{
  if (!channelChangePending)
    return;


  if (
    millis()
    <
    channelChangeAt
  )
  {
    return;
  }


  channelChangePending =
    false;


  /*
      SAFETY FIRST
  */

  stopCar();

  controllerClient = -1;


  uint8_t newChannel =
    pendingChannel;


  if (!validWiFiChannel(newChannel))
    return;


  saveWiFiChannel(
    newChannel
  );


  apChannel =
    newChannel;


  Serial.println();

  Serial.print(
    "Switching WiFi AP to channel "
  );

  Serial.println(
    apChannel
  );


  WiFi.softAPdisconnect(
    true
  );


  delay(50);


  bool ok =
    createAccessPoint();


  if (!ok)
  {
    Serial.println(
      "AP restart failed."
    );

    Serial.println(
      "Restarting ESP..."
    );

    delay(100);

    ESP.restart();
  }


  Serial.print(
    "RC_Car active on CH "
  );

  Serial.println(
    apChannel
  );
}


// =====================================================================
// ACCESS POINT WATCHDOG
// =====================================================================

void checkAccessPoint()
{
  unsigned long now =
    millis();


  if (
    now - lastAPCheck
    <
    AP_CHECK_INTERVAL
  )
  {
    return;
  }


  lastAPCheck = now;


  bool badMode =
    WiFi.getMode()
    !=
    WIFI_AP;


  bool badIP =
    WiFi.softAPIP()
    !=
    AP_IP;


  if (
    !badMode
    &&
    !badIP
  )
  {
    return;
  }


  Serial.println(
    "AP watchdog detected problem."
  );


  stopCar();

  controllerClient = -1;


  WiFi.softAPdisconnect(
    true
  );


  delay(30);


  if (!createAccessPoint())
  {
    Serial.println(
      "AP recovery failed. Rebooting."
    );

    delay(100);

    ESP.restart();
  }
}


// =====================================================================
// WEB PAGE
// =====================================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>

<head>

<meta charset="UTF-8">

<meta
 name="viewport"
 content="width=device-width,
 initial-scale=1,
 maximum-scale=1,
 user-scalable=no">

<meta
 name="theme-color"
 content="#07110e">

<title>RC Car Pro</title>


<style>

:root
{
    --accent:#21e28b;
    --background:#07110e;
    --panel:#101d18;
    --panel2:#172923;
    --text:#f3fff8;
    --muted:#81938a;
    --danger:#ff4057;
}


*
{
    box-sizing:border-box;

    user-select:none;

    -webkit-user-select:none;

    -webkit-tap-highlight-color:transparent;
}


html,
body
{
    width:100%;
    height:100%;

    margin:0;

    overflow:hidden;
}


body
{
    background:
        radial-gradient(
            circle at 50% -20%,
            #1d4939,
            var(--background) 45%
        );

    color:var(--text);

    font-family:
        system-ui,
        -apple-system,
        BlinkMacSystemFont,
        sans-serif;

    touch-action:none;
}


.app
{
    height:100%;

    max-width:720px;

    margin:auto;

    padding:12px;

    display:flex;

    flex-direction:column;
}


/* ===========================================================
   HEADER
=========================================================== */

header
{
    min-height:55px;

    display:flex;

    align-items:center;

    justify-content:space-between;
}


.title
{
    font-size:20px;

    font-weight:900;
}


.sub
{
    margin-top:3px;

    color:var(--muted);

    font-size:11px;
}


.dot
{
    display:inline-block;

    width:8px;

    height:8px;

    margin-right:5px;

    border-radius:50%;

    background:#777;
}


.dot.ok
{
    background:var(--accent);

    box-shadow:
        0 0 12px
        var(--accent);
}


.gear
{
    width:42px;

    height:42px;

    border:
        1px solid
        #ffffff15;

    border-radius:13px;

    background:var(--panel);

    color:white;

    font-size:20px;
}


/* ===========================================================
   STATUS
=========================================================== */

.stats
{
    display:grid;

    grid-template-columns:
        repeat(5,1fr);

    gap:6px;

    margin:
        7px 0 9px;
}


.stat
{
    padding:7px 3px;

    text-align:center;

    border:
        1px solid
        #ffffff0c;

    background:
        #ffffff07;

    border-radius:12px;
}


.stat b
{
    display:block;

    color:var(--accent);

    font-size:13px;
}


.stat span
{
    color:var(--muted);

    font-size:8px;

    text-transform:uppercase;
}


/* ===========================================================
   MAIN
=========================================================== */

main
{
    flex:1;

    min-height:0;

    display:flex;

    align-items:center;

    justify-content:space-around;

    gap:12px;
}


/* ===========================================================
   DIRECTION PAD
=========================================================== */

.pad
{
    display:grid;

    grid-template-columns:
        repeat(3,75px);

    grid-template-rows:
        repeat(3,75px);

    gap:8px;
}


.drive
{
    border:
        1px solid
        #ffffff15;

    border-radius:20px;

    background:
        linear-gradient(
            145deg,
            #182a23,
            #0b1511
        );

    color:white;

    font-size:26px;

    touch-action:none;

    box-shadow:
        0 7px 16px
        #0005;

    transition:
        transform .07s,
        background .07s;
}


.drive.on
{
    background:
        var(--accent);

    color:#00150a;

    transform:
        scale(.92);

    box-shadow:
        0 0 22px
        #21e28b55;
}


.stop
{
    background:#401019;

    color:#ff8090;
}


/* ===========================================================
   SIDE
=========================================================== */

.side
{
    width:180px;

    display:flex;

    flex-direction:column;

    gap:8px;
}


.card
{
    padding:13px;

    background:
        #ffffff08;

    border:
        1px solid
        #ffffff0e;

    border-radius:17px;
}


.speedrow
{
    display:flex;

    align-items:center;

    justify-content:space-between;
}


.speed
{
    color:var(--accent);

    font-size:28px;

    font-weight:900;
}


input[type=range]
{
    width:100%;

    accent-color:
        var(--accent);
}


.modebuttons
{
    display:grid;

    grid-template-columns:
        1fr 1fr;

    gap:6px;

    margin-top:8px;
}


.smallbtn
{
    padding:9px 4px;

    border:
        1px solid
        #ffffff13;

    border-radius:10px;

    background:
        var(--panel2);

    color:white;

    font-size:11px;

    font-weight:700;
}


.smallbtn.selected
{
    background:
        var(--accent);

    color:#00150a;
}


.estop
{
    height:52px;

    border:
        1px solid
        #ff5568;

    border-radius:15px;

    background:#3b1018;

    color:#ff8492;

    font-size:14px;

    font-weight:900;
}


/* ===========================================================
   SETTINGS
=========================================================== */

.overlay
{
    position:fixed;

    inset:0;

    z-index:20;

    display:none;

    align-items:flex-end;

    justify-content:center;

    background:#000b;

    backdrop-filter:
        blur(6px);
}


.overlay.show
{
    display:flex;
}


.settings
{
    width:
        min(600px,100%);

    max-height:90vh;

    overflow:auto;

    padding:18px;

    background:#0c1713;

    border-radius:
        24px 24px 0 0;
}


.settingHeader
{
    display:flex;

    justify-content:
        space-between;

    align-items:center;
}


.settingHeader button
{
    width:38px;

    height:38px;

    border:0;

    border-radius:11px;

    background:#ffffff10;

    color:white;
}


.setting
{
    padding:12px 0;

    border-bottom:
        1px solid
        #ffffff0c;
}


.settingtop
{
    display:flex;

    justify-content:
        space-between;

    align-items:center;

    gap:15px;
}


.setting label
{
    font-size:14px;

    font-weight:700;
}


.hint
{
    margin-top:3px;

    color:var(--muted);

    font-size:10px;
}


.value
{
    color:var(--accent);

    font-weight:800;
}


select
{
    padding:8px;

    border:
        1px solid
        #ffffff12;

    border-radius:9px;

    background:
        var(--panel2);

    color:white;
}


.apply
{
    width:100%;

    margin-top:10px;

    padding:11px;

    border:0;

    border-radius:11px;

    background:
        var(--accent);

    color:#00150a;

    font-weight:900;
}


.channelstatus
{
    margin-top:8px;

    text-align:center;

    color:var(--muted);

    font-size:11px;
}


/* ===========================================================
   RESPONSIVE
=========================================================== */

@media(max-width:520px)
{
    .pad
    {
        grid-template-columns:
            repeat(3,66px);

        grid-template-rows:
            repeat(3,66px);
    }


    .side
    {
        width:130px;
    }


    .stats
    {
        grid-template-columns:
            repeat(5,1fr);
    }
}


@media(max-height:520px)
{
    .stats
    {
        display:none;
    }


    header
    {
        min-height:40px;
    }


    .pad
    {
        grid-template-columns:
            repeat(3,60px);

        grid-template-rows:
            repeat(3,60px);
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


<div class="sub">

<span
 class="dot"
 id="dot">
</span>

<span id="connection">

Connecting...

</span>

</div>

</div>


<button
 class="gear"
 id="gear">

⚙

</button>


</header>



<div class="stats">


<div class="stat">

<b id="ping">
--
</b>

<span>
Latency
</span>

</div>


<div class="stat">

<b id="averagePing">
--
</b>

<span>
Average
</span>

</div>


<div class="stat">

<b id="clients">
--
</b>

<span>
Devices
</span>

</div>


<div class="stat">

<b id="uptime">
--
</b>

<span>
Uptime
</span>

</div>


<div class="stat">

<b id="channel">
--
</b>

<span>
Channel
</span>

</div>


</div>



<main>


<div class="pad">


<button
 class="drive"
 data-c="G">

↖

</button>


<button
 class="drive"
 data-c="F">

▲

</button>


<button
 class="drive"
 data-c="H">

↗

</button>


<button
 class="drive"
 data-c="L">

◀

</button>


<button
 class="drive stop"
 id="stop">

■

</button>


<button
 class="drive"
 data-c="R">

▶

</button>


<button
 class="drive"
 data-c="I">

↙

</button>


<button
 class="drive"
 data-c="B">

▼

</button>


<button
 class="drive"
 data-c="J">

↘

</button>


</div>



<div class="side">


<div class="card">


<div class="speedrow">

<span>
SPEED
</span>

<span
 class="speed"
 id="speedText">

70

</span>

</div>


<input
 id="speed"
 type="range"
 min="20"
 max="100"
 value="70">


<div class="modebuttons">


<button
 class="smallbtn selected"
 id="eco">

🌿 ECO

</button>


<button
 class="smallbtn"
 id="sport">

⚡ SPORT

</button>


</div>


</div>



<button
 class="estop"
 id="estop">

⛔ EMERGENCY STOP

</button>


</div>


</main>


</div>



<!-- =========================================================
     SETTINGS
========================================================= -->


<div
 class="overlay"
 id="overlay">


<div class="settings">


<div class="settingHeader">


<h2>

⚙ Car Settings

</h2>


<button id="close">

✕

</button>


</div>



<!-- STEERING -->


<div class="setting">


<div class="settingtop">


<div>

<label>
Steering Style
</label>

<div class="hint">

Smooth saves power.
Pivot turns on the spot.

</div>

</div>


<select id="steering">

<option value="1">
Smooth
</option>

<option value="0">
Pivot
</option>

</select>


</div>


</div>



<!-- TURN STRENGTH -->


<div class="setting">


<div class="settingtop">


<div>

<label>
Turn Strength
</label>

<div class="hint">

Higher value gives sharper turns.

</div>

</div>


<span
 class="value"
 id="turnText">

70%

</span>


</div>


<input
 id="turn"
 type="range"
 min="30"
 max="100"
 value="70">


</div>



<!-- ACCELERATION -->


<div class="setting">


<div class="settingtop">


<div>

<label>
Acceleration

</label>


<div class="hint">

Lower value reduces current spikes.

</div>


</div>


<span
 class="value"
 id="rampText">

10

</span>


</div>


<input
 id="ramp"
 type="range"
 min="4"
 max="30"
 value="10">


</div>



<!-- LEFT TRIM -->


<div class="setting">


<div class="settingtop">


<div>


<label>
Left Motor Trim
</label>


<div class="hint">

Adjust if car doesn't drive straight.

</div>


</div>


<span
 class="value"
 id="leftText">

100%

</span>


</div>


<input
 id="leftTrim"
 type="range"
 min="70"
 max="110"
 value="100">


</div>



<!-- RIGHT TRIM -->


<div class="setting">


<div class="settingtop">


<div>

<label>
Right Motor Trim
</label>

<div class="hint">

Adjust if car doesn't drive straight.

</div>

</div>


<span
 class="value"
 id="rightText">

100%

</span>


</div>


<input
 id="rightTrim"
 type="range"
 min="70"
 max="110"
 value="100">


</div>



<!-- ECO LIMIT -->


<div class="setting">


<div class="settingtop">


<div>

<label>
ECO Maximum Speed
</label>

<div class="hint">

Lower gives longer battery runtime.

</div>

</div>


<span
 class="value"
 id="ecoLimitText">

85%

</span>


</div>


<input
 id="ecoLimit"
 type="range"
 min="50"
 max="100"
 value="85">


</div>



<!-- SWAP LEFT / RIGHT -->


<div class="setting">


<div class="settingtop">


<div>

<label>
Swap Left / Right
</label>

<div class="hint">

Enabled for your current motor orientation.

</div>

</div>


<input
 id="swap"
 type="checkbox"
 checked>


</div>


</div>



<!-- WIFI CHANNEL -->


<div class="setting">


<div class="settingtop">


<div>


<label>
WiFi Channel
</label>


<div class="hint">

Test 1, 6 and 11 and keep the one with lowest stable latency.

</div>


</div>


<select id="wifiChannel">

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
 class="apply"
 id="applyChannel">

APPLY WIFI CHANNEL

</button>


<div
 class="channelstatus"
 id="currentChannel">

Current Channel: --

</div>


<div
 class="hint"
 style="text-align:center;">

Changing the channel temporarily disconnects WiFi.
RC_Car will restart automatically.

</div>


</div>



<!-- WIFI POWER -->


<div class="setting">


<div class="settingtop">


<div>

<label>
WiFi Transmit Power
</label>

<div class="hint">

Lower power saves energy.
Increase it if range or stability becomes poor.

</div>

</div>


<select id="wifiPower">

<option value="85">
8.5 dBm
</option>

<option value="110">
11 dBm
</option>

<option
 value="140"
 selected>

14 dBm

</option>

<option value="175">
17.5 dBm
</option>

<option value="205">
20.5 dBm
</option>

</select>


</div>


</div>


</div>

</div>



<script>

// =====================================================================
// CONFIGURATION
// =====================================================================

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

    ecoLimit:85,

    wifiPower:140
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
                "RC_CONFIG"
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
catch(e)
{
}


// =====================================================================
// WEBSOCKET
// =====================================================================

let ws = null;

let reconnectTimer = null;

let driveTimer = null;

let currentCommand = "S";

let activeButton = null;


// =====================================================================
// PING
// =====================================================================

let pingHistory = [];


// =====================================================================
// SAVE CONFIG
// =====================================================================

function save()
{
    localStorage.setItem(
        "RC_CONFIG",
        JSON.stringify(cfg)
    );
}


// =====================================================================
// WEBSOCKET SEND
// =====================================================================

function send(message)
{
    if(
        ws
        &&
        ws.readyState
        ===
        WebSocket.OPEN
    )
    {
        ws.send(message);

        return true;
    }


    return false;
}


// =====================================================================
// CONNECT
// =====================================================================

function connect()
{
    clearTimeout(
        reconnectTimer
    );


    ws =
        new WebSocket(
            "ws://"
            +
            location.hostname
            +
            ":81/"
        );


    ws.onopen =
        function()
        {
            document
                .getElementById(
                    "dot"
                )
                .classList
                .add(
                    "ok"
                );


            document
                .getElementById(
                    "connection"
                )
                .textContent =
                "LOW LATENCY CONNECTED";


            sendConfig();


            send(
                "S:"
                +
                cfg.speed
            );
        };


    ws.onclose =
        function()
        {
            stopLocal();


            document
                .getElementById(
                    "dot"
                )
                .classList
                .remove(
                    "ok"
                );


            document
                .getElementById(
                    "connection"
                )
                .textContent =
                "RECONNECTING...";


            reconnectTimer =
                setTimeout(
                    connect,
                    500
                );
        };


    ws.onerror =
        function()
        {
            try
            {
                ws.close();
            }
            catch(e)
            {
            }
        };


    ws.onmessage =
        function(event)
        {
            const msg =
                event.data;


            // -------------------------------------------------
            // PING RESPONSE
            // -------------------------------------------------

            if(
                msg.startsWith(
                    "P:"
                )
            )
            {
                const timestamp =
                    Number(
                        msg.substring(2)
                    );


                const rtt =
                    Date.now()
                    -
                    timestamp;


                document
                    .getElementById(
                        "ping"
                    )
                    .textContent =
                    rtt
                    +
                    "ms";


                pingHistory.push(
                    rtt
                );


                if(
                    pingHistory.length
                    >
                    10
                )
                {
                    pingHistory.shift();
                }


                const average =
                    Math.round(
                        pingHistory.reduce(
                            (a,b) =>
                                a+b,
                            0
                        )
                        /
                        pingHistory.length
                    );


                document
                    .getElementById(
                        "averagePing"
                    )
                    .textContent =
                    average
                    +
                    "ms";


                return;
            }


            // -------------------------------------------------
            // STATUS RESPONSE
            // -------------------------------------------------

            if(
                msg.startsWith(
                    "T:"
                )
            )
            {
                const p =
                    msg
                    .substring(2)
                    .split(",");


                document
                    .getElementById(
                        "clients"
                    )
                    .textContent =
                    p[0];


                document
                    .getElementById(
                        "uptime"
                    )
                    .textContent =
                    formatTime(
                        Number(
                            p[1]
                        )
                    );


                const currentChannel =
                    Number(
                        p[3]
                    );


                document
                    .getElementById(
                        "channel"
                    )
                    .textContent =
                    "CH"
                    +
                    currentChannel;


                document
                    .getElementById(
                        "currentChannel"
                    )
                    .textContent =
                    "Current Channel: "
                    +
                    currentChannel;


                document
                    .getElementById(
                        "wifiChannel"
                    )
                    .value =
                    currentChannel;


                return;
            }


            // -------------------------------------------------
            // CHANNEL SWITCH ACKNOWLEDGEMENT
            // -------------------------------------------------

            if(
                msg.startsWith(
                    "CHANGING:"
                )
            )
            {
                const channel =
                    msg.split(":")[1];


                document
                    .getElementById(
                        "connection"
                    )
                    .textContent =
                    "SWITCHING TO CH "
                    +
                    channel;


                document
                    .getElementById(
                        "currentChannel"
                    )
                    .textContent =
                    "Switching to Channel "
                    +
                    channel
                    +
                    "...";


                return;
            }


            // -------------------------------------------------
            // ALREADY CURRENT CHANNEL
            // -------------------------------------------------

            if(
                msg.startsWith(
                    "CH:"
                )
            )
            {
                const channel =
                    msg.split(":")[1];


                document
                    .getElementById(
                        "currentChannel"
                    )
                    .textContent =
                    "Current Channel: "
                    +
                    channel;


                return;
            }
        };
}


// =====================================================================
// CONFIG SEND
// =====================================================================

let configTimer = null;


function sendConfig()
{
    clearTimeout(
        configTimer
    );


    configTimer =
        setTimeout(
            function()
            {
                send(

                    "C:"
                    +
                    cfg.turn
                    +
                    ","
                    +
                    cfg.leftTrim
                    +
                    ","
                    +
                    cfg.rightTrim
                    +
                    ","
                    +
                    cfg.ramp
                    +
                    ","
                    +
                    cfg.steering
                    +
                    ","
                    +
                    (cfg.swap ? 1 : 0)
                    +
                    ","
                    +
                    (cfg.eco ? 1 : 0)
                    +
                    ","
                    +
                    cfg.ecoLimit
                    +
                    ","
                    +
                    cfg.wifiPower
                );
            },

            40
        );
}


// =====================================================================
// DRIVE
// =====================================================================

function drive(
    command,
    button
)
{
    clearInterval(
        driveTimer
    );


    currentCommand =
        command;


    if(activeButton)
    {
        activeButton
            .classList
            .remove(
                "on"
            );
    }


    activeButton =
        button
        ||
        null;


    if(activeButton)
    {
        activeButton
            .classList
            .add(
                "on"
            );
    }


    send(
        "D:"
        +
        command
    );


    /*
        WebSocket heartbeat only while moving.
    */

    driveTimer =
        setInterval(
            function()
            {
                send(
                    "D:"
                    +
                    currentCommand
                );
            },

            100
        );
}


function stopLocal()
{
    clearInterval(
        driveTimer
    );


    driveTimer =
        null;


    currentCommand =
        "S";


    if(activeButton)
    {
        activeButton
            .classList
            .remove(
                "on"
            );
    }


    activeButton =
        null;


    send(
        "D:S"
    );
}


// =====================================================================
// DIRECTION BUTTONS
// =====================================================================

document
    .querySelectorAll(
        ".drive[data-c]"
    )
    .forEach(
        function(button)
        {
            button
                .addEventListener(
                    "pointerdown",

                    function(event)
                    {
                        event.preventDefault();


                        drive(
                            button.dataset.c,
                            button
                        );
                    }
                );
        }
    );


window.addEventListener(
    "pointerup",
    stopLocal
);


window.addEventListener(
    "pointercancel",
    stopLocal
);


// =====================================================================
// STOP
// =====================================================================

document
    .getElementById(
        "stop"
    )
    .addEventListener(
        "pointerdown",

        function(event)
        {
            event.preventDefault();

            stopLocal();
        }
    );


// =====================================================================
// EMERGENCY STOP
// =====================================================================

document
    .getElementById(
        "estop"
    )
    .addEventListener(
        "pointerdown",

        function(event)
        {
            event.preventDefault();


            stopLocal();


            send(
                "X"
            );


            if(
                navigator.vibrate
            )
            {
                navigator.vibrate(
                    40
                );
            }
        }
    );


// =====================================================================
// SPEED
// =====================================================================

const speedSlider =
    document.getElementById(
        "speed"
    );


const speedText =
    document.getElementById(
        "speedText"
    );


speedSlider
    .addEventListener(
        "input",

        function()
        {
            cfg.speed =
                Number(
                    speedSlider.value
                );


            speedText.textContent =
                cfg.speed;


            save();


            send(
                "S:"
                +
                cfg.speed
            );
        }
    );


// =====================================================================
// ECO / SPORT
// =====================================================================

function updateModeButtons()
{
    document
        .getElementById(
            "eco"
        )
        .classList
        .toggle(
            "selected",
            cfg.eco
        );


    document
        .getElementById(
            "sport"
        )
        .classList
        .toggle(
            "selected",
            !cfg.eco
        );
}


document
    .getElementById(
        "eco"
    )
    .onclick =
        function()
        {
            cfg.eco =
                true;


            save();

            updateModeButtons();

            sendConfig();
        };


document
    .getElementById(
        "sport"
    )
    .onclick =
        function()
        {
            cfg.eco =
                false;


            save();

            updateModeButtons();

            sendConfig();
        };


// =====================================================================
// SETTINGS OPEN / CLOSE
// =====================================================================

const overlay =
    document.getElementById(
        "overlay"
    );


document
    .getElementById(
        "gear"
    )
    .onclick =
        function()
        {
            stopLocal();


            overlay
                .classList
                .add(
                    "show"
                );
        };


document
    .getElementById(
        "close"
    )
    .onclick =
        function()
        {
            overlay
                .classList
                .remove(
                    "show"
                );
        };


// =====================================================================
// UI SETTINGS
// =====================================================================

function updateUI()
{
    speedSlider.value =
        cfg.speed;


    speedText.textContent =
        cfg.speed;


    document
        .getElementById(
            "turn"
        )
        .value =
        cfg.turn;


    document
        .getElementById(
            "turnText"
        )
        .textContent =
        cfg.turn
        +
        "%";


    document
        .getElementById(
            "ramp"
        )
        .value =
        cfg.ramp;


    document
        .getElementById(
            "rampText"
        )
        .textContent =
        cfg.ramp;


    document
        .getElementById(
            "leftTrim"
        )
        .value =
        cfg.leftTrim;


    document
        .getElementById(
            "leftText"
        )
        .textContent =
        cfg.leftTrim
        +
        "%";


    document
        .getElementById(
            "rightTrim"
        )
        .value =
        cfg.rightTrim;


    document
        .getElementById(
            "rightText"
        )
        .textContent =
        cfg.rightTrim
        +
        "%";


    document
        .getElementById(
            "steering"
        )
        .value =
        cfg.steering;


    document
        .getElementById(
            "swap"
        )
        .checked =
        cfg.swap;


    document
        .getElementById(
            "ecoLimit"
        )
        .value =
        cfg.ecoLimit;


    document
        .getElementById(
            "ecoLimitText"
        )
        .textContent =
        cfg.ecoLimit
        +
        "%";


    document
        .getElementById(
            "wifiPower"
        )
        .value =
        cfg.wifiPower;


    updateModeButtons();
}


// =====================================================================
// TURN
// =====================================================================

document
    .getElementById(
        "turn"
    )
    .oninput =
        function(event)
        {
            cfg.turn =
                Number(
                    event.target.value
                );


            save();

            updateUI();

            sendConfig();
        };


// =====================================================================
// ACCELERATION
// =====================================================================

document
    .getElementById(
        "ramp"
    )
    .oninput =
        function(event)
        {
            cfg.ramp =
                Number(
                    event.target.value
                );


            save();

            updateUI();

            sendConfig();
        };


// =====================================================================
// LEFT TRIM
// =====================================================================

document
    .getElementById(
        "leftTrim"
    )
    .oninput =
        function(event)
        {
            cfg.leftTrim =
                Number(
                    event.target.value
                );


            save();

            updateUI();

            sendConfig();
        };


// =====================================================================
// RIGHT TRIM
// =====================================================================

document
    .getElementById(
        "rightTrim"
    )
    .oninput =
        function(event)
        {
            cfg.rightTrim =
                Number(
                    event.target.value
                );


            save();

            updateUI();

            sendConfig();
        };


// =====================================================================
// STEERING STYLE
// =====================================================================

document
    .getElementById(
        "steering"
    )
    .onchange =
        function(event)
        {
            stopLocal();


            cfg.steering =
                Number(
                    event.target.value
                );


            save();

            sendConfig();
        };


// =====================================================================
// SWAP LEFT / RIGHT
// =====================================================================

document
    .getElementById(
        "swap"
    )
    .onchange =
        function(event)
        {
            stopLocal();


            cfg.swap =
                event.target.checked;


            save();

            sendConfig();
        };


// =====================================================================
// ECO LIMIT
// =====================================================================

document
    .getElementById(
        "ecoLimit"
    )
    .oninput =
        function(event)
        {
            cfg.ecoLimit =
                Number(
                    event.target.value
                );


            save();

            updateUI();

            sendConfig();
        };


// =====================================================================
// WIFI TX POWER
// =====================================================================

document
    .getElementById(
        "wifiPower"
    )
    .onchange =
        function(event)
        {
            cfg.wifiPower =
                Number(
                    event.target.value
                );


            save();

            sendConfig();
        };


// =====================================================================
// WIFI CHANNEL
// =====================================================================

document
    .getElementById(
        "applyChannel"
    )
    .addEventListener(
        "click",

        function()
        {
            stopLocal();


            const requestedChannel =
                Number(
                    document
                        .getElementById(
                            "wifiChannel"
                        )
                        .value
                );


            if(
                   requestedChannel !== 1
                && requestedChannel !== 6
                && requestedChannel !== 11
            )
            {
                return;
            }


            document
                .getElementById(
                    "currentChannel"
                )
                .textContent =
                "Preparing Channel "
                +
                requestedChannel
                +
                "...";


            send(
                "W:"
                +
                requestedChannel
            );
        }
    );


// =====================================================================
// PING
// =====================================================================

setInterval(
    function()
    {
        if(
            ws
            &&
            ws.readyState
            ===
            WebSocket.OPEN
        )
        {
            send(
                "P:"
                +
                Date.now()
            );
        }
    },

    1500
);


// =====================================================================
// TIME FORMAT
// =====================================================================

function formatTime(seconds)
{
    if(seconds < 60)
    {
        return seconds
        +
        "s";
    }


    const minutes =
        Math.floor(
            seconds / 60
        );


    if(minutes < 60)
    {
        return minutes
        +
        "m";
    }


    return (
        Math.floor(
            minutes / 60
        )
        +
        "h"
    );
}


// =====================================================================
// SAFETY EVENTS
// =====================================================================

document.addEventListener(
    "visibilitychange",

    function()
    {
        if(document.hidden)
        {
            stopLocal();
        }
    }
);


window.addEventListener(
    "pagehide",
    stopLocal
);


window.addEventListener(
    "blur",
    stopLocal
);


window.addEventListener(
    "contextmenu",

    function(event)
    {
        event.preventDefault();
    }
);


// =====================================================================
// START
// =====================================================================

updateUI();

connect();

</script>

</body>

</html>
)HTML";


// =====================================================================
// HTTP ROOT
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
    INDEX_HTML
  );
}


// =====================================================================
// WEBSOCKET EVENT
// =====================================================================

void webSocketEvent(
  uint8_t client,
  WStype_t type,
  uint8_t* payload,
  size_t length
)
{

  // -------------------------------------------------------------------
  // CONNECTED
  // -------------------------------------------------------------------

  if (type == WStype_CONNECTED)
  {
    webSocket.sendTXT(
      client,
      "READY"
    );

    return;
  }


  // -------------------------------------------------------------------
  // DISCONNECTED
  // -------------------------------------------------------------------

  if (type == WStype_DISCONNECTED)
  {
    /*
        If controlling phone disappears,
        stop motors immediately.
    */

    if (
      controllerClient
      ==
      client
    )
    {
      stopCar();

      controllerClient = -1;
    }


    return;
  }


  // -------------------------------------------------------------------
  // TEXT ONLY
  // -------------------------------------------------------------------

  if (type != WStype_TEXT)
    return;


  if (length == 0)
    return;


  char message[120];


  size_t copyLength =
    min(
      length,
      sizeof(message) - 1
    );


  memcpy(
    message,
    payload,
    copyLength
  );


  message[copyLength] =
    '\0';


  // -------------------------------------------------------------------
  // DRIVE
  //
  // D:F
  // D:B
  // D:L
  // etc.
  // -------------------------------------------------------------------

  if (
    message[0] == 'D'
    &&
    message[1] == ':'
  )
  {
    char command =
      message[2];


    if (
      !validDriveCommand(
        command
      )
    )
    {
      stopCar();

      return;
    }


    /*
        First client to actually drive
        gets control ownership.
    */

    if (
      command != 'S'
      &&
      controllerClient < 0
    )
    {
      controllerClient =
        client;
    }


    /*
        Other phones cannot fight
        over motor control.
    */

    if (
      command != 'S'
      &&
      controllerClient
      !=
      client
    )
    {
      return;
    }


    /*
        STOP is always accepted.
    */

    if (command == 'S')
    {
      stopCar();

      controllerClient = -1;

      return;
    }


    driveCommand(
      command
    );


    return;
  }


  // -------------------------------------------------------------------
  // EMERGENCY STOP
  // -------------------------------------------------------------------

  if (
    message[0] == 'X'
  )
  {
    stopCar();

    controllerClient = -1;

    return;
  }


  // -------------------------------------------------------------------
  // SPEED
  //
  // S:70
  // -------------------------------------------------------------------

  if (
    message[0] == 'S'
    &&
    message[1] == ':'
  )
  {
    speedPercent =
      constrain(
        atoi(
          message + 2
        ),
        20,
        100
      );


    if (
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


  // -------------------------------------------------------------------
  // CONFIGURATION
  //
  // C:
  // turn,
  // leftTrim,
  // rightTrim,
  // ramp,
  // steering,
  // swap,
  // eco,
  // ecoLimit,
  // txPowerTenths
  // -------------------------------------------------------------------

  if (
    message[0] == 'C'
    &&
    message[1] == ':'
  )
  {
    int turn;
    int lt;
    int rt;
    int ramp;
    int steer;
    int swap;
    int eco;
    int ecoLimit;
    int txPower;


    int count =
      sscanf(

        message + 2,

        "%d,%d,%d,%d,%d,%d,%d,%d,%d",

        &turn,
        &lt,
        &rt,
        &ramp,
        &steer,
        &swap,
        &eco,
        &ecoLimit,
        &txPower
      );


    if (count == 9)
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
          steer,
          0,
          1
        );


      swapSteering =
        swap != 0;


      ecoMode =
        eco != 0;


      ecoSpeedLimit =
        constrain(
          ecoLimit,
          50,
          100
        );


      txPower =
        constrain(
          txPower,
          0,
          205
        );


      wifiTxPower =
        txPower /
        10.0f;


      /*
          Change radio output power
          without restarting AP.
      */

      WiFi.setOutputPower(
        wifiTxPower
      );


      if (
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


    return;
  }


  // -------------------------------------------------------------------
  // WIFI CHANNEL CHANGE
  //
  // W:1
  // W:6
  // W:11
  // -------------------------------------------------------------------

  if (
    message[0] == 'W'
    &&
    message[1] == ':'
  )
  {
    int requested =
      atoi(
        message + 2
      );


    if (
         requested != 1
      && requested != 6
      && requested != 11
    )
    {
      webSocket.sendTXT(
        client,
        "CHANNEL_ERROR"
      );

      return;
    }


    /*
        Already using this channel.
    */

    if (
      requested
      ==
      apChannel
    )
    {
      char reply[20];


      snprintf(
        reply,
        sizeof(reply),

        "CH:%d",

        apChannel
      );


      webSocket.sendTXT(
        client,
        reply
      );


      return;
    }


    /*
        Never change AP radio
        while motors are moving.
    */

    stopCar();

    controllerClient = -1;


    pendingChannel =
      requested;


    /*
        Wait 500ms before switching.

        This lets WebSocket acknowledgement
        reach the phone before AP disappears.
    */

    channelChangeAt =
      millis()
      +
      500;


    channelChangePending =
      true;


    char reply[30];


    snprintf(
      reply,
      sizeof(reply),

      "CHANGING:%d",

      requested
    );


    webSocket.sendTXT(
      client,
      reply
    );


    return;
  }


  // -------------------------------------------------------------------
  // PING
  //
  // P:<timestamp>
  // -------------------------------------------------------------------

  if (
    message[0] == 'P'
    &&
    message[1] == ':'
  )
  {
    /*
        Immediate echo.
    */

    webSocket.sendTXT(
      client,
      message
    );


    return;
  }
}


// =====================================================================
// STATUS PUSH
// =====================================================================

void pushStatus()
{
  unsigned long now =
    millis();


  if (
    now - lastStatusPush
    <
    2000
  )
  {
    return;
  }


  lastStatusPush =
    now;


  char message[100];


  snprintf(
    message,
    sizeof(message),

    "T:%u,%lu,%u,%u",

    WiFi.softAPgetStationNum(),

    now / 1000,

    ESP.getFreeHeap(),

    apChannel
  );


  webSocket.broadcastTXT(
    message
  );
}


// =====================================================================
// SETUP
// =====================================================================

void setup()
{
  Serial.begin(115200);


  delay(100);


  // -------------------------------------------------------------------
  // LOAD SAVED WIFI CHANNEL
  // -------------------------------------------------------------------

  loadWiFiChannel();


  // -------------------------------------------------------------------
  // MOTOR PINS
  // -------------------------------------------------------------------

  pinMode(IN1, OUTPUT);

  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);

  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);

  pinMode(ENB, OUTPUT);


  /*
      Explicit PWM range.

      0   = motors off
      255 = full PWM
  */

  analogWriteRange(
    255
  );


  /*
      1 kHz gives a good balance of:
        motor control
        CPU load
        switching losses
  */

  analogWriteFreq(
    1000
  );


  /*
      NEVER boot with motors active.
  */

  stopCar();


  // -------------------------------------------------------------------
  // LOWER TCP LATENCY
  // -------------------------------------------------------------------

  /*
      Disable Nagle.

      Small WebSocket commands should
      leave immediately.
  */

  WiFiClient::setDefaultNoDelay(
    true
  );


  // -------------------------------------------------------------------
  // CREATE AP
  // -------------------------------------------------------------------

  bool apStarted =
    false;


  for (
    int attempt = 0;
    attempt < 3;
    attempt++
  )
  {
    if (
      createAccessPoint()
    )
    {
      apStarted =
        true;

      break;
    }


    WiFi.softAPdisconnect(
      true
    );


    delay(50);
  }


  /*
      If WiFi stack is in a strange state,
      restarting lets setup() try again.
  */

  if (!apStarted)
  {
    delay(100);

    ESP.restart();
  }


  // -------------------------------------------------------------------
  // HTTP SERVER
  // -------------------------------------------------------------------

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );


  server.onNotFound(
    handleRoot
  );


  server.begin();


  // -------------------------------------------------------------------
  // WEBSOCKET
  // -------------------------------------------------------------------

  webSocket.begin();


  webSocket.onEvent(
    webSocketEvent
  );


  // -------------------------------------------------------------------
  // SERIAL INFORMATION
  // -------------------------------------------------------------------

  Serial.println();

  Serial.println(
    "======================================="
  );

  Serial.println(
    "       RC CAR PRO READY"
  );

  Serial.println(
    "======================================="
  );


  Serial.print(
    "SSID: "
  );

  Serial.println(
    AP_SSID
  );


  Serial.print(
    "IP: "
  );

  Serial.println(
    WiFi.softAPIP()
  );


  Serial.print(
    "WiFi Channel: "
  );

  Serial.println(
    apChannel
  );


  Serial.print(
    "WiFi TX power: "
  );

  Serial.print(
    wifiTxPower
  );

  Serial.println(
    " dBm"
  );


  Serial.println(
    "Controller:"
  );

  Serial.println(
    "http://192.168.4.1"
  );


  Serial.println(
    "======================================="
  );


  lastControlPacket =
    millis();
}


// =====================================================================
// LOOP
// =====================================================================

void loop()
{

  // -------------------------------------------------------------------
  // NETWORK
  // -------------------------------------------------------------------

  server.handleClient();

  webSocket.loop();


  // -------------------------------------------------------------------
  // MOTORS
  // -------------------------------------------------------------------

  updateMotorRamp();


  // -------------------------------------------------------------------
  // MOTOR COMMUNICATION FAILSAFE
  // -------------------------------------------------------------------

  if (
    activeCommand
    !=
    'S'
  )
  {
    if (
      millis()
      -
      lastControlPacket
      >
      CONTROL_TIMEOUT
    )
    {
      stopCar();

      controllerClient = -1;
    }
  }


  // -------------------------------------------------------------------
  // CHANGE CHANNEL IF REQUESTED
  // -------------------------------------------------------------------

  changeWiFiChannelNow();


  // -------------------------------------------------------------------
  // KEEP ACCESS POINT ALIVE
  // -------------------------------------------------------------------

  checkAccessPoint();


  // -------------------------------------------------------------------
  // STATUS
  // -------------------------------------------------------------------

  pushStatus();


  // -------------------------------------------------------------------
  // ESP8266 WIFI/WATCHDOG
  // -------------------------------------------------------------------

  yield();
}
