#define VERSION "\nDongleBuddy CMHASD V0.230307 by keith.rickard@hotmail.com"  // Started Jun 2022
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ESPWiFiServer.h>
const char ssid[] = "Meade 16 inch LX200";
const char pass[] = "CRAYFORD";

// PARAMETERS
#define   DEBUG_MODE        0
#define   LISTEN_MODE       0
#define   BUF_SIZE          50
#define   GUIDE_TEST_PERIOD 100
#define   EVENTDELAY        150     // Time to wait for break in last serial comms between PC and telescope
#define   ENQ_PERIOD        3000    // Time in ms between sending DB controller azimuth info
#define   SER_TIMEOUT       1000
#define   SER1_TIMEOUT      5000    // LX200 RS232 time out
#define   SER2_TIMEOUT      500     // Bluetooth device time out
#define   BAUD_BT           9600
#define   BAUD_DB           9600
#define   BAUD_LX           9600
#define   BAUD_PC           9600
#define   BAUD_WF           19200   // 115200 does not work to well with the onboard ESP wifi chip 
#define   BAUD_MX           9600
#define   BAUD_GPS          9600
 
#define   WEST          2       // Guide port W -RA
#define   SOUTH         4       // Guide port S -DEC
#define   NORTH         3       // Guide port N +DEC
#define   EAST          5       // Guide port E +RA
#define   LED_DB        7       // 
#define   LED_PC        8
#define   LED_BT        9
#define   LED_WF        10
#define   TXD_GPS       11      // GPS -> Arduino
#define   RXD_GPS       12      // GPS <- Arduino
#define   TXD_WF        14      // Serial3 - RX3 for WiFi
#define   RXD_WF        15      // Serial3 - TX3 for WiFi
#define   RXD_BT        16      // Serial2 - TX2 for Bluetooth (user device)
#define   TXD_BT        17      // Serial2 - RX2 for Bluetooth (user device)
#define   RXD_RS232     18      // Serial1 - TX1 For LX200 (through MAX3232 module)
#define   TXD_RS232     19      // Serial1 - RX1 For LX200 (through MAX3232 module)
#define   LED_E         22
#define   LED_S         24
#define   LED_W         26
#define   LED_N         34
#define   BTN_PC        46      // Select PC comms
#define   BTN_BT        48      // Select Bluetooth comms
#define   BTN_WF        50      // Select WiFi comms
#define   STAT_BT       A5
#define   TXD_DB        A11     // Serial4(!) BT <- Arduino
#define   RXD_DB        A12     // Serial4(!) BT -> Arduino
#define   STAT_DB       A13                                                       
#define   GUIDE_12V     A14
#define   RS232_12V     A15

#define   LED_DIM       1
#define   LED_BRIGHT    20
#define   LED_OFF       0

#define   MODE_NUL      0
#define   MODE_PC       5
#define   MODE_BT       6
#define   MODE_WF       7
#define   MODE_LX       8

//SoftwareSerial SerialDB (TXD_DB, RXD_DB);       // Arduino RX, Arduino TX 
//SoftwareSerial SerialGPS (TXD_GPS, RXD_GPS);    // Arduino RX, Arduino TX
SoftwareSerial SerialMix(TXD_GPS, RXD_DB);        // Receive data from GPS, send data to the Dome controller
Stream* _serial;
ESP8266Server server(4030);
ESP8266Client client;
 
unsigned long timerNS,                          // Timers
              timerEW,
              timerPause, 
              timerLED = millis() + 500,
              timerEvent,                       // Time of last PC <> Telescope comms 
              timerComms;
struct gpsData {
     float lat = 51.4140984;                    // Coords of Dick Chambers Observatory
     float lng = 0.2373936;
     int yr;
     byte mth, day, hr, min, sec;
     byte fix = B1111;
}    gpsData;
bool debug = DEBUG_MODE,                        // 1 = Debug Mode active
     listen = LISTEN_MODE,                      // Send PC & LX200 to Bluetooth module
     scopeConnected = 0,
     gettingGPS = 0,                            // GPS fix is being got
     ESPworking = 0,                            // 1 = ESP is talking to the Arduino
     movingNS = 0,                              // GuideBuddy: Flags to indicate is Dec or RA axis are moving
     movingEW = 0;
char lxSLEW = 'S',                              // 'S' = slewing,  'T' = tracking, 'L' = lost dongle connection
     buf[BUF_SIZE],
     lxCmd[4] = "###",
     bufDec[16];                                // +12*34:56 or +12*
byte lxAZM = 0,                                 // Telescope's calculated azimuth (0 to 255, North = 0)
     lastAZM = 0,
     gbReady = 0,                               // GuideBuddy ready
     mode = MODE_NUL,
     dataFlag = 0,
     led[] = {1, LED_N, LED_W, LED_S, LED_E, LED_PC, LED_BT, LED_WF, LED_DB};
int  mth, day, yr;

// --------------------------------------------------------------------------------------------------
void setup() {
  pinMode(WEST, INPUT_PULLUP);          // Make ST-4 lines high resistance to ensure no pulse are active (GuideBuddy)
  pinMode(EAST, INPUT_PULLUP);
  pinMode(SOUTH, INPUT_PULLUP);
  pinMode(NORTH, INPUT_PULLUP);
  pinMode(BTN_PC, INPUT_PULLUP);        // Initialise Button input pins
  pinMode(BTN_BT, INPUT_PULLUP);
  pinMode(BTN_WF, INPUT_PULLUP);
  for (byte i = 1; i <= 4; i++) {       // light up all LEDs
    pinMode(led[i], OUTPUT);
    digitalWrite(led[i], HIGH);
  }

  while (!digitalRead(BTN_PC)) {
    delay(20);                          // Enter DEBUG mode
    digitalWrite(LED_PC, LOW);
    delay(20);
    digitalWrite(LED_PC, HIGH);
    debug = 1;
  }
  while (!digitalRead(BTN_BT)) {
    delay(20);                          // Enter LISTEN mode
    digitalWrite(LED_BT, LOW);
    delay(20);
    digitalWrite(LED_BT, HIGH);
    listen = 1;
  }

  for (byte i = 5; i <= 8; i++) {
    analogWrite(led[i], LED_BRIGHT);
  }
  analogWrite(LED_DB, LED_DIM);
  SerialMix.begin(BAUD_MX);             // Data from GPS, data out DB BT        (pins 11 & A13)
  Serial.begin(BAUD_PC);                // USB Serial - for PC                  (pins 1 & 0)
  Serial.setTimeout(SER_TIMEOUT);
  Serial1.begin(BAUD_LX);               // LX200 via MAX3232                    (pins 18 & 19)
  Serial1.setTimeout(SER1_TIMEOUT);
  Serial2.begin(BAUD_BT);               // Bluetooth Module                     (pins 16 & 17)
  Serial2.setTimeout(SER2_TIMEOUT);
  Serial3.begin(BAUD_WF);               // Onboard ESP32 WiFi                   (pins 14 & 15)
  if (debug) Serial.println(VERSION);
  delay(1000);
  mode = MODE_NUL;
  if (listen) {
    Serial2.println(VERSION);
    Serial2.println("Listening mode activated");
  }

  // Initialise WiFi Access Point
  if (debug) Serial.println("Entering ESP setup...");
  if ((ESPworking = wifi.begin(Serial3, BAUD_WF))) {  // Binds Serial3 to ESP8266, AT+RST, AT, ATE0, AT+CIPMUX=1, AT+CWMODE=1, AT=CIPINFO=1, AT+CWATUOCONN=0
        if (debug) Serial.println("wifi.begin(Serial3, BAUD_WF)\nwifi.showInfo(false)");
      wifi.showInfo(false);                           // AT+CIPINFO=0         Suppress IPD info and only receive SkyFi bytes
       if (debug) Serial.println("wifi.setMode(WIFI_AP)");
      wifi.setMode(WIFI_AP);                          // AT+CWMODE=2          Select Soft Access Point
        if (debug) Serial.println("wifi.setMode(WIFI_AP)");
      wifi.softAP(ssid, pass);                        // AT+CWSAP...          Set WiFi name and password
        if (debug) Serial.println("wifi.softAP(ssid, pass)");
      wifi.softAPIP();                                // AT+CIPAP?            Get ESP's IP address, gateway and netmask 
        if (debug) Serial.println("wifi.softAPIP()");
      wifi.setTimeout(20);                            // AT+CIPSTO=20         Set timeout before closing an inactive connection
        if (debug) Serial.println("wifi.setTimeout(20)");
      wifi.setMux(false);                             // AT+CIPMUX=0          false = single connection (true = allow multiple connections)
        if (debug) Serial.println("wifi.setMux(false)");
      server.begin();                                 // AT+CIPSERVER=1,4030  Create server with port number 4030
        if (debug) Serial.println("server.begin()");
      server.setTimeout(20);                          // AT+CIPSTO=20         Set TCP server timeout
        if (debug) Serial.println("server.setTimeout(20)");
      client = server.available();
        if (debug) Serial.println("client = server.available()");
  }
  Serial3.println("AT+CIPAP_CUR=\"10.0.0.1\",\"10.0.0.1\",\"255.255.255.0\"");  // WiFi IP, Gateway and mask (same as SkyFi's)
  delay(100);
  while (Serial3.available()){
    Serial3.read();
    delay(10);
  }

  if (debug) ESPworking ? Serial.println("Wifi ready") : Serial.println("WiFi not responding");
  for (byte i = 1; i <= 7; i++) digitalWrite(led[i], LOW); // Turn off all LEDs except DomeBuddy LED
  //testGPSModulePC();
  //testWifiPC();
  //testESPpc();
  //testBluetoothPC();
  //testBluetoothLX200();
  //testWiFiLX200();
  //testWifiLX200raw();
  //testPcLX200();
  //rawPcLX200();
  //test12Vpins();
  //mode = MODE_WF;
  //startingGoto();
  //while(1);
}
// --------------------------------------------------------------------------------------------------
void loop() {
  if (debug) Serial.println("{loop}");
  checkScope();                         // See if telescope is connected
  serviceGuideBuddy();                  // Deal with any GuideBuddy pulses
  checkDevice();                        // Check status of connected device
  serviceComms();                       // Allow comms between a device and telescope for 5 seconds
  getAzimuth();                         // Get azimuth and slew status
  sendAZM();                            // Send azimuth info to DomeBuddy Controller
  serviceLEDs();                        // Illuminate LEDs as required
}
#pragma region COMMS STUFF
// ==================================================================================================
// SERVICE THE THROUGHPUT OF DATA BETWEEN THE PC AND TELESCOPE
// ==================================================================================================
void checkDevice() {
  if (debug) Serial.println("{checkDevice}");
  if (gettingGPS) getGPSdata();                                 // Try and get GPS data
  byte mode_prev = mode;
  if (!digitalRead(BTN_WF)) if(ESPworking) mode = MODE_WF;      // Select WiFi if its button is pressed
  if (!digitalRead(BTN_BT)) mode = MODE_BT;                     // Select Bluetooth if its button is pressed
  if (!digitalRead(BTN_PC)) mode = MODE_PC;                     // Select PC if its button is pressed
  if (!digitalRead(BTN_PC) || !digitalRead(BTN_BT) || !digitalRead(BTN_WF)) Serial1.print("#:Q#:Qn#:Qs#:Qe#:Qe#");
  // FIND WHICH DEVICE IS CHATTING TO THE DONGLE FIRST
  if (mode == MODE_NUL) {                                       // No connection to a device yet
    if (ESPworking) {                                           // If onboard ESP chip is working
      client = server.available();
      if (client.connected()) mode = MODE_WF;                   // WF mode if something has been received via WiFi
    }
    if (Serial2.available()) mode = MODE_BT;                    // BT mode if something has been received from Bluetooth
    if (Serial.available()) mode = MODE_PC;                     // PC mode if something has been received from via USB
    if (mode != MODE_NUL) {
      analogWrite(led[mode], 255);
      timerLED = millis() + 50;
      led[0] = 0;
    }
  }
  if (mode == MODE_BT) listen = 0;
  if (mode != mode_prev) {
    gettingGPS = 0;
    for (byte i = 5; i <= 7; i++) digitalWrite(led[i], i == mode);    // Light selected device LED
    if (debug) Serial.println(mode == MODE_PC ? "\nPC connected" : (mode == MODE_BT ? "\nBluetooth connected" : "\nWiFi connected"));
    while (Serial1.available()) Serial1.read();                       // Clear out LX200 buffer
  }
}
// --------------------------------------------------------------------------------------------------
void serviceComms() {
/* Unfortunately, the 16" Meade LX200 has only one serial port, unlike its smaller brothers, and therefore its port has
// has to be shared between a connecting device (e.g. a PC via USB, Android and Apple devices) and the Dome controller.
// This is a critical function.  The LX200 has commands which give different type of responses to one another, with one
// giving a variable response (SC - set calendar date).  Recognising how the LX200 will responding is important so that
// the LX200 can be asked for its azimuth and so minimising the flow of data between a device and the LX200.  To add 
// additional complexity, this function also handles commands for the Guide Buddy functionality so that guiding pulses,
// etc, can be delivered to the LX200 mount via its control panel's CCD port.                                        */

  if (debug) Serial.println("{serviceComms}");
  if (!scopeConnected){                           // Quit if connection with the telescope has not been established
    while (Serial.available()) Serial.read();     // PC
    while (Serial1.available()) Serial1.read();   // LX200
    while (Serial2.available()) Serial2.read();   // Bluetooth
    return;
  }

  unsigned long timer = millis() + ENQ_PERIOD;    // Send data to DB controller every 3 seconds
  timerEvent = millis();
  while (millis() < timer) {
    checkDevice();
    if (mode) {
      gettingGPS = 0;
      switch (mode) {
        case MODE_PC: pcComms(); break;
        //case MODE_PC: serialComms(&Serial);
        case MODE_BT: btComms(); break;
        //case MODE_WF: serialComms(&SerialMix); break; 
        case MODE_WF: wfComms(); break;
      }
      serviceGuideBuddy();
    }
    serviceLEDs();
    if(timerEvent > timer) timer = timerEvent;    // Extend period if an event took place
  }
}
// --------------------------------------------------------------------------------------------------
void wfComms() {
  if (debug) Serial.println("{wfComms}");
  //debug = 1;
  String wifiStr, cmdStr, respStr = "";
  byte type;
  char c1, c2;

  ESP8266Client client = server.available();
  while (client.connected()) {
    if (client.available()) {
      delay(10);
      cmdStr = "";                                                        // So starts the hard work dealing with WiFi comms...
      wifiStr = client.readStringUntil('#') + '#';                        // All LX200 commnands start with a ':' & end with a '#'
      if(wifiStr == "0,CLOSED\r\n#") {
        if (debug) Serial.println("Wifi connection has been closed");
        return;
      }
      for (byte i = wifiStr.length() - 1; i >= 0; i--) {                  // Search backwards from end of string for a colon
        cmdStr = wifiStr.charAt(i) + cmdStr;                              // Form command for the LX200
        if (wifiStr.charAt(i) == ':') if (!isDigit(wifiStr.charAt(i + 1))) break; // LX200 commands start with a ':' but never followed by a digit
      }
      if (cmdStr.charAt(0) == ':') {                                      // Has a command been properly received?
        if (debug) {Serial.print("[W>T]"); Serial.println(cmdStr);}
        Serial1.print(cmdStr);
        respStr = "";
        c1 = cmdStr.charAt(1);
        c2 = cmdStr.charAt(2);
        if (c1 == 'Q') if (lxSLEW == 'G') lxSLEW = 'T';
        type = 0;                                                         // Type = 0: no response, 1: 1 byte, 2: 2 strings, 3: 1 string ending '#'
        if (strchr("GCD", c1)) type = 4;                                  // These LX200 cmds have a '#' terminated string response
        if (strchr("S\x06", c1)) type = (c2 == 'C') ? 2 : 1;              // 'S' & 'ACK' cmds give a 1 byte response. 'SC' could have 2 more text strings
        if (c1 == 'M') {
          if (c2 == 'A') type = 1;                                        // The'MA' command has a 1 byte response
          if (c2 == 'S') type = 3;                                        // The'MS' command has a 1 byte response & maybe a string ending '#' for errors
        }
        if (c1 == 'h') if (c2 = '?') type = 1;                            // 'h?' has a 1 byte response 
        if (c1 == 'L') {                                                  // 'L' (library) commnands are very unlikely to be used
          if (strchr("fI", c2)) type = 4;                                 // These 'L' commands have a '#' terminated string response
          if (strchr("os", c2)) type = 1;                                 // These 'L' commands have a 1 byte response
        }
        if (type) {                                                       // Get the LX200 response and deal with it correctly
          timerComms = millis() + 1000;
          while(!Serial1.available() && (millis() < timerComms));
          if (type <= 3) respStr = char(Serial1.read());                  // Get a 1 byte response
          if (type == 2) if (respStr == "1") type = 4;                    // The LX200's date is being changed - continue if invalid date
          if (type == 3) if (respStr != "0") type = 4;                    // Slew command will give a message if repsonse not '0'
          if (type == 4) respStr += Serial1.readStringUntil('#') + '#';   // Get a '#' terminated string
          if (type == 3) startingGoto();                                  // If type is still 3 then scope is doing a go-to slew

          if (debug) Serial.print("[T>W]"); Serial.println(respStr);
          client.print(respStr);                                          // Send string to device if one was received
          
          if (respStr.charAt(1) == 'U') {
            respStr = "";
            timerComms = millis() + SER1_TIMEOUT;
            while (millis() < timerComms && client.connected()) {
              if (Serial1.available()) {
                respStr = Serial1.readStringUntil('#') + '#';
                if (debug) {Serial.print("[T>W]"); Serial.println(respStr);}
                client.print(respStr);
                break;
              }
            }
          }
        }
      }
    }
    while(Serial1.read() >= 0);
  }
  while(Serial1.read() >= 0);
  //debug = 0;
}
// --------------------------------------------------------------------------------------------------
bool serialComms(Stream* _serial) {
  if (debug) Serial.println("{serialComms}");
  if (_serial->available()) {                                       // PC command
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    timerEvent = millis() + EVENTDELAY;
    if (strchr("<>v^_|%?=~!@V", _serial->peek())) guideBuddy();      // Action any GuideBuddy commands
    else {
      lxCmd[0] = lxCmd[1]; lxCmd[1] = lxCmd[2];                     // If :SC command has been received to 
      lxCmd[2] = Serial.read();                                     // set the calendar date then allow 5 secs
      if (strcmp(lxCmd, "SC#") == 0) timerEvent = millis() + SER1_TIMEOUT;  // for LX200 to recalculate planet positions
      if (strcmp(lxCmd, "Mn#") == 0) pulseLED(LED_N);
      if (strcmp(lxCmd, "Ms#") == 0) pulseLED(LED_S);
      if (strcmp(lxCmd, "Me#") == 0) pulseLED(LED_E);
      if (strcmp(lxCmd, "Mw#") == 0) pulseLED(LED_W);
      if (strcmp(lxCmd, "Qn#") == 0) digitalWrite(LED_N, LOW);
      if (strcmp(lxCmd, "Qs#") == 0) digitalWrite(LED_S, LOW);  
      if (strcmp(lxCmd, "Qe#") == 0) digitalWrite(LED_E, LOW);
      if (strcmp(lxCmd, "Qw#") == 0) digitalWrite(LED_W, LOW);
      if (lxCmd[2] == 'Q') if (lxSLEW == 'G') lxSLEW = 'T';         // Stop any Dome go-to slewing
      if (lxCmd[2] == '\\') lxCmd[2] = '\x6';
      Serial1.write(lxCmd[2]);                                      // Send byte to LX200
      if (lxCmd[2] == '#') delay(10);
      if (strcmp(lxCmd, "MS#") == 0) startingGoto();
      timerLED = millis() + 20;
      led[0] = B00;
    }
  }
  if (Serial1.available()) {                                        // Telescope response
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    _serial->write(Serial1.read());
    timerEvent = millis() + EVENTDELAY;
    timerLED = millis() + 20;
    led[0] = B00;
  }
  return 1;                                                         // In PC mode
}
// --------------------------------------------------------------------------------------------------
bool pcComms() {
  if (debug) Serial.println("{pcComms}");
  if (Serial.available()) {                                         // PC command
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    timerEvent = millis() + EVENTDELAY;
    if (strchr("<>v^_|%?=~!@V", Serial.peek())) guideBuddy();        // Action any GuideBuddy commands
    else {
      lxCmd[0] = lxCmd[1]; lxCmd[1] = lxCmd[2];                     // If :SC command has been received to 
      lxCmd[2] = Serial.read();                                     // set the calendar date then allow 5 secs
      if (strcmp(lxCmd, "SC#") == 0) timerEvent = millis() + SER1_TIMEOUT;  // for LX200 to recalculate planet positions
      if (strcmp(lxCmd, "Mn#") == 0) pulseLED(LED_N);
      if (strcmp(lxCmd, "Ms#") == 0) pulseLED(LED_S);
      if (strcmp(lxCmd, "Me#") == 0) pulseLED(LED_E);
      if (strcmp(lxCmd, "Mw#") == 0) pulseLED(LED_W);
      if (strcmp(lxCmd, "Qn#") == 0) digitalWrite(LED_N, LOW);
      if (strcmp(lxCmd, "Qs#") == 0) digitalWrite(LED_S, LOW);
      if (strcmp(lxCmd, "Qe#") == 0) digitalWrite(LED_E, LOW);
      if (strcmp(lxCmd, "Qw#") == 0) digitalWrite(LED_W, LOW);
      if (lxCmd[2] == 'Q') if (lxSLEW == 'G') lxSLEW = 'T';         // Stop any Dome go-to slewing
      if (lxCmd[2] == '\\') lxCmd[2] = '\x6';
      Serial1.write(lxCmd[2]);                                      // Send byte to LX200
      if (listen) {
        Serial2.write(lxCmd[2]);                                    // Send this to Bluetooth
        if (lxCmd[2] == '#') Serial2.write('\n');
      }
      if (lxCmd[2] == '#') delay(10);
      if (strcmp(lxCmd, "MS#") == 0) startingGoto();
      timerLED = millis() + 20;
      led[0] = B00;
    }
  }
  while (Serial1.available()) {                                     // Telescope response
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    if (listen) {
      Serial2.write(Serial1.peek());                                // Send this to Bluetooth
      if (Serial1.peek() == '#') Serial2.write('\n');
    }
    Serial.write(Serial1.read());
    timerEvent = millis() + EVENTDELAY;
    timerLED = millis() + 20;
    led[0] = B00;
  }
  return 1;                                                         // In PC mode
}
// --------------------------------------------------------------------------------------------------
bool btComms() {
  if (debug) Serial.println("{btComms}");
  while(Serial2.available()) {                                      // PC command
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    timerEvent = millis() + EVENTDELAY;
    if (strchr("<>v^_|%?=~!@V", Serial2.peek())) guideBuddy();       // Action any GuideBuddy commands
    else {
      lxCmd[0] = lxCmd[1]; lxCmd[1] = lxCmd[2];                     // If :SC command has been received to 
      lxCmd[2] = Serial2.read();                                    // set the calendar date then allow 5 secs
      if (strcmp(lxCmd, "SC#") == 0) timerEvent = millis() + SER1_TIMEOUT;  // for LX200 to recalculate planet positions
      if (strcmp(lxCmd, "Mn#") == 0) pulseLED(LED_N);
      if (strcmp(lxCmd, "Ms#") == 0) pulseLED(LED_S);
      if (strcmp(lxCmd, "Me#") == 0) pulseLED(LED_E);
      if (strcmp(lxCmd, "Mw#") == 0) pulseLED(LED_W);
      if (strcmp(lxCmd, "Qn#") == 0) digitalWrite(LED_N, LOW);
      if (strcmp(lxCmd, "Qs#") == 0) digitalWrite(LED_S, LOW);
      if (strcmp(lxCmd, "Qe#") == 0) digitalWrite(LED_E, LOW);
      if (strcmp(lxCmd, "Qw#") == 0) digitalWrite(LED_W, LOW);
      if (lxCmd[2] == 'Q') if (lxSLEW == 'G') lxSLEW = 'T';         // Stop any Dome go-to slewing
      if (lxCmd[2] == '\\') lxCmd[2] = '\x6';
      Serial1.write(lxCmd[2]);                                      // Send byte to LX200
      if (lxCmd[2] == '#') delay(10);
      if (strcmp(lxCmd, "MS#") == 0) startingGoto();
      timerLED = millis() + 20;
      led[0] = B00;
    }
  }
  while (Serial1.available()) {                                     // Telescope response
    analogWrite(led[mode], LED_OFF);                                // Comms in progress
    Serial2.write(Serial1.read());
    timerEvent = millis() + EVENTDELAY;
    timerLED = millis() + 20;
    led[0] = B00;
  }
  return 1;                                                         // In BT mode
}
// --------------------------------------------------------------------------------------------------
void serviceLEDs() {
// If looking for telescope, the device light(s) will be lit (all if device not yet selected).
// If trying to get a GPS fix, the directional lights will light in turn clockwise.
// If no link with the dome controller, the DomeBuddy light will flash brightly and rapidly.
// If a link with the dome controller exists, the DomeBuddy light is lit dimly.
// If comms between telescope and device occur then the device LED will briefly flash.
// If comms between telescope and DomeBuddy occur then the DomeBuddy LED will briefly flash.
// Applicable directional LED(s) will light if a guide pulse occurs and turn off when done.
// If all directional LEDs flash at same time then GuideBuddy is not working
  if (debug) Serial.println("{serviceLEDs}");
//  return;

  if ((analogRead(RS232_12V) < 100) || (analogRead(GUIDE_12V) >100)) {                    // Wrong plug in wrong socket?
    if (debug) {
      if (analogRead(RS232_12V) < 100) Serial.println("Not connected to the telescope");
      if (analogRead(GUIDE_12V) > 100) Serial.println("CCD cable in RS232 socket");
    }
    for (byte i = WEST; i <= EAST; i++) pinMode(i, INPUT_PULLUP);                         // Deacitivate GuideBuddy pulses
    led[0] = 0;
    while((analogRead(RS232_12V) < 100) || (analogRead(GUIDE_12V) > 100)) {               // Restart when plugs changed
      if (millis() > timerLED) {
        timerLED = millis() + 100;
        led[0] ^=1;
        for (byte i = 5; i <= 8; i++) analogWrite(led[i], led[0] * LED_BRIGHT);
        if (!ESPworking) digitalWrite(led[MODE_WF], LOW);  
        if (dbCxn()) analogWrite(LED_DB, LED_DIM);
      }
    }
    {asm volatile ("jmp 0");}
  };

  if (debug) {
    Serial.print("ESP:            "); Serial.println(ESPworking);
    Serial.print("gettingGPS:     "); Serial.println(gettingGPS);
    Serial.print("gpsData.fix:    "); Serial.println(gpsData.fix);
    Serial.print("gbReady:        "); Serial.println(gbReady);
    Serial.print("scopeConnected: "); Serial.println(scopeConnected);
    Serial.print("mode:           "); Serial.println(mode);
  }
  if (millis() > timerLED) {
    timerLED = millis() + 25;
    ++led[0] &= B1111;
    for (byte i = 5; i <= 7; i++) analogWrite(led[mode ? mode : i], scopeConnected ? 64 : ((led[0]/8) & B01 ? LED_BRIGHT : 0)); // Device LED(s)
    if (!ESPworking) digitalWrite(led[MODE_WF], LOW);                                                              // ESP not working?
    if (gettingGPS) for (byte i = 1; i <= 4; i++) digitalWrite(led[i], i == led[0] / 4 + 1);                       // Dir LEDs circling - GPS fixing
    else if (!gbReady) for (byte i = 1; i <=4; i++) digitalWrite(led[i], (led[0] == 0) && mode == MODE_PC);        // Dir LEDs flashing - GuideBuddy not working
    analogWrite(LED_DB, dbCxn() ? LED_DIM : ((led[0]/4) & B01 ? LED_DIM : LED_OFF));                               // DomeBuddy LED
  }
  if (!gettingGPS && gbReady) if (!movingEW && !movingNS && gbReady != 2) for (byte i = 1; i <= 4; i++) digitalWrite(led[i], LOW); // Direction LEDS
}
// --------------------------------------------------------------------------------------------------
bool dbCxn() {
  //if (gbReady == 1) return 1;
  return digitalRead(STAT_DB);
}
#pragma endregion COMMS STUFF
// --------------------------------------------------------------------------------------------------
#pragma region GUIDEBUDDY
// ==================================================================================================
// GUIDEBUDDY
// ==================================================================================================
void serviceGuideBuddy() {
  if (debug) Serial.println("{serviceGuideBuddy}");
  if (timerPause) {                                 // If in Pause mode, a timer for blinking the LED is active
    if(millis() > timerPause){
      timerPause = millis() + 500;                  // Blink the LED every 0.5 sec to indicate in Pause mode
      digitalWrite(LED_E, 1 - digitalRead(LED_E));
      digitalWrite(LED_W, 1 - digitalRead(LED_W));
      digitalWrite(LED_N, 1 - digitalRead(LED_N));
      digitalWrite(LED_S, 1 - digitalRead(LED_S));
    }
  } else{
    if (movingEW) if (timerEW) if(millis() > timerEW) stopEW();   // If the EW timer has expired, turn off East/West pulse
    if (movingNS) if (timerNS) if(millis() > timerNS) stopNS();   // If the NS timer has expired, turn off North/South pulse
  }
}
// --------------------------------------------------------------------------------------------------
void checkGuideBuddy() {
  if (debug) Serial.println("{checkGuideBuddy}");
  if (mode != MODE_NUL) return;
  char bufDec[16], alignMode[2];
  for (byte i = 0; i <= 15; i++) bufDec[i] = 0;
  Serial1.setTimeout(SER1_TIMEOUT);
  while(Serial1.available()) Serial1.read();

  do {
    gbReady = 0;                                                // Assume GuideBuddy not working
    Serial1.print("#:Q#:Qn#:Qs#:Qe#:Qw#:AP#:\6#");              // Stop any motion, put into POLAR mode & get Dec
    Serial1.readBytes(alignMode, 1);
    if (debug) Serial.println(alignMode);
    Serial1.print(":AP#");
    if (alignMode[0] == 'L') Serial1.print(":RG#:Me#");         // If was LAND mode then pause RA tracking
    Serial1.print(":GD#");
    pinMode(NORTH, INPUT_PULLUP);
    pinMode(SOUTH, INPUT_PULLUP);
    pinMode(EAST, INPUT_PULLUP);
    pinMode(WEST, INPUT_PULLUP);

    if (waitSerial1(SER1_TIMEOUT)) break;
    bufDec[Serial1.readBytesUntil('#', bufDec, 15)] = '\0';     // +12*34:56 or +12*34
    if (strlen(bufDec) < 9) {                                   // Ensure Dec is in HiPrecision mode
      Serial1.print("#:U#:GD#");
      if (waitSerial1(SER1_TIMEOUT)) break;
      bufDec[Serial1.readBytesUntil('#', bufDec, 15)] = '\0';
    }
    if (debug) Serial.println(bufDec);
    pinMode(NORTH, OUTPUT); digitalWrite(NORTH, LOW);           // Send a North guide pulse
    delay(GUIDE_TEST_PERIOD);
    pinMode(NORTH, INPUT_PULLUP);
    Serial1.println("#:GD#");                                   // Get the Dec again

    if (waitSerial1(SER1_TIMEOUT)) break;
    buf[Serial1.readBytesUntil('#', buf, 15)] = '\0';
    if (debug) Serial.println(buf);
    if (strcmp(buf, bufDec)) {                                  // Has Dec changed?
      gbReady = 1;                                              // It has - GuideBuddy is working
      pinMode(SOUTH, OUTPUT); digitalWrite(SOUTH, LOW);         // Send a South guide pulse to return to the beginng.
      delay(GUIDE_TEST_PERIOD);
      pinMode(SOUTH, INPUT_PULLUP);
    }
  } while(0);
  Serial1.print(":Qe#:RG#:A"); Serial1.print(alignMode); Serial1.print('#'); // Stop any RA move and put back in original mode
  if (debug) Serial.println(gbReady ? "GuideBuddy ready" : "GuideBuddy not working");
}
// --------------------------------------------------------------------------------------------------
bool waitSerial1(long ms) {               // Wait for data to be received
  timerComms = millis() + ms;
  while(millis() < timerComms) {
    if (Serial1.available()) return 0;
  }
  return 1;
}
// --------------------------------------------------------------------------------------------------
void guideBuddy() {
  if (debug) Serial.println("{guideBuddy}");
  clearBuf();
  if (Serial.readBytes(buf, 1)) {
    if (listen) Serial1.println(buf);
    switch (buf[0]) {
      case '<': moveEW(WEST);             break; // Send pulse to move mount west for given number of ms
      case '>': moveEW(EAST);             break; // Send pulse to move mount east for given number of ms
      case 'v': moveNS(SOUTH);            break; // Send pulse to move mount south for given number of ms
      case '^': moveNS(NORTH);            break; // Send pulse to move mount north for given number of ms
      case '_': stopEW();                 break; // Stop any east or west guide pulses
      case '|': stopNS();                 break; // Stop any north or south guide pulse
      case '%': stopAll();                break; // Stop any guide pulses
      case '!': isMoving();               break; // Reports if any guiding pulses are still active ('1' = active)
      case '=': pause();                  break; // Stop mount tracking
      case '~': debug ^= 1;               break; // Start/stop debugging messages being output
      case '?': buf[0] = gbReady + '0';   break; // Return '1' to indicate GuideBuddy is responding, or 'O' if not
      case '@': buf[0] = 0; domeAZ();     break; // Return dome azimuth
      case 'V': Serial.println(VERSION); 
                if (listen) Serial1.println(VERSION);
                buf[0] = 0; break;
      default: buf[0] = 0;                break; // What has been received is bad - ignore it.
    }
    if(buf[0]) {
      if (buf[0] == '~') {
        mode == MODE_NUL;
        if (debug) Serial.println(VERSION);
      }
      Serial.print(buf[0]);                  // Send a repsonse if it exists.
      Serial.print("#");
      if (listen) {
        Serial1.print(buf[0]);
        Serial1.println('#');
      }
    }
  }
}
// --------------------------------------------------------------------------------------------------
void moveEW(byte direction){          // Direction is EAST or WEST
  if (debug) Serial.println("{moveEW}");
  if (!gbReady) return;
  if(timerPause) stopNS();            // If parked, rest NORTH & SOUTH lines
  stopEW();                           // Stop any current EAST or WEST pulse
  timerEW = getDuration();            // See if there is a duration in the command for pulse
  pinMode(direction, OUTPUT);         // Activate EAST or WEST line
  digitalWrite(direction, LOW);       // Ground the line to send a pulse
  digitalWrite((direction == EAST ? LED_E : LED_W), LOW);
  digitalWrite((direction == EAST ? LED_W : LED_E), HIGH);
  movingEW = 1;                       // If there is no error then the mount is now moving East or West
}
// --------------------------------------------------------------------------------------------------
void moveNS(byte direction){          // Direction is NORTH or SOUTH
  if (debug) Serial.println("{moveNS}");
  if (!gbReady) return;
  if(timerPause) stopEW();            // If parked, reset EAST & WEST lines
  stopNS();                           // Stop any current NORTH or SOUTH pulse
  timerNS = getDuration();            // See if there is a duration in the command of pulse
  pinMode(direction, OUTPUT);         // Activate NORTH or SOUTH line
  digitalWrite(direction, LOW);       // Ground the line to send a pulse
  pulseLED(direction == NORTH ? LED_N : LED_S);
  movingNS = 1;                       // It there is no error then the mount is now moving North or South
}
// --------------------------------------------------------------------------------------------------
void pulseLED(byte dirLED) {
  if (debug) Serial.println("{pulseLED}");
  if (!gbReady) {
    for (byte i = 1; i <= 4; i++) digitalWrite(led[i], LOW);
    gbReady = 2;
  }
  switch (dirLED) {
    case LED_S: digitalWrite(LED_S, LOW); break;
    case LED_N: digitalWrite(LED_N, LOW); break;
    case LED_E: digitalWrite(LED_W, LOW); break;
    case LED_W: digitalWrite(LED_E, LOW); break;
  }
  digitalWrite(dirLED, HIGH);
}
// --------------------------------------------------------------------------------------------------
void stopEW(){
  if (debug) Serial.println("{stopEW}");
  pinMode(EAST, INPUT_PULLUP);        // Deactivate lines using the Arduino pull-up resistors 
  pinMode(WEST, INPUT_PULLUP);
  digitalWrite(LED_E, LOW);
  digitalWrite(LED_W, LOW);
  movingEW = timerPause = 0;          // Mount is no longer moving or paused
  timerEW = 1;                        // Stop the pulse timer
}
// --------------------------------------------------------------------------------------------------
void stopNS(){
  if (debug) Serial.println("{stopNS}");
  pinMode(NORTH, INPUT_PULLUP);       // Deactivate lines using the Arduino's pull-up resistors
  pinMode(SOUTH, INPUT_PULLUP);
  digitalWrite(LED_N, LOW);
  digitalWrite(LED_S, LOW);
  movingNS = timerPause = 0;          // Mount is no longer moving or paused
  timerNS = 1;                        // Stop the pulse timer
}
// --------------------------------------------------------------------------------------------------
void pause(){                         // This pauses the telescope mount
  if (debug) Serial.println("{pause}");
  timerEW = timerNS = 1;              // Stop the pulse timers
  pinMode(EAST, OUTPUT);              // Pausing is done by grounding all of the pulse lines to make the active
  pinMode(WEST, OUTPUT);
  pinMode(NORTH, OUTPUT);
  pinMode(SOUTH, OUTPUT);
  digitalWrite(LED_E, HIGH);
  digitalWrite(LED_W, HIGH);
  digitalWrite(LED_N, HIGH);
  digitalWrite(LED_S, HIGH);
  timerPause = 1;                     // Pause LED blink timer is made active
  movingNS = movingEW = 0;            // The mount is now classed as not moving
}
// --------------------------------------------------------------------------------------------------
void stopAll(){
  if (debug) Serial.println("{stopAll}");
  stopEW();                           // Stop all pulses and allow the mount to track normally
  stopNS();
}
// --------------------------------------------------------------------------------------------------
void isMoving(){                            // Reports if a pulse is in progress
  if (debug) Serial.println("{isMoving}");
  buf[0] = '0';                             // Default resp is '0' meaning no guiding pulses are active
  if(movingEW || movingNS) buf[0] = '1';    // '1' means one or more guiding pulses are active
  if(timerPause) buf[0] = '2';              // '2' means the mount is in Pause mode
}
// --------------------------------------------------------------------------------------------------
unsigned long getDuration(){
 if (debug) Serial.println("{getDuration}");
 unsigned long value = timerPause = 0;      // Cancel pause LED blink timer as a request to start a guiding pulse has been received
  value = Serial.parseInt();
  if (value) return value + millis();
  return 0;
}
// --------------------------------------------------------------------------------------------------
void domeAZ() {
  if (debug) Serial.println("{domeAZ}");
  Serial.print(lxAZM);
  Serial.print('#');
  buf[0] = 0;
}
#pragma endregion GUIDEBUDDY
// --------------------------------------------------------------------------------------------------
#pragma region DOMEBUDDY STUFF
// ==================================================================================================
// INITIALISE CONNECTION WITH THE LX200
// ==================================================================================================
void checkScope() {
  if (debug) Serial.println("{checkScope}");
  if (scopeConnected) return;

// LOOK FOR THE MEADE LX200
  if (debug) Serial.println("Looking for LX200");
  //if (testLXlink()) return;                           // Return if already connected
  Serial1.flush();                                    // Clear transmit buffer
  while (Serial1.available()) Serial1.read();         // Clear receive buffer

  clearBuf();
  Serial1.setTimeout(200);
  Serial1.print("#:GC#");                             // Get date
  scopeConnected = bool(Serial1.readBytes(buf, 9) != 0);
  Serial1.setTimeout(SER1_TIMEOUT);
  if (debug) Serial.println(buf);

  if (scopeConnected) {
    if (buf[8] == '#') {
      sscanf(buf, "%02d/%02d/%02d", &mth, &day, &yr); // Grab date info (for comparing against GPS later)
      gpsData.day = day;
      gpsData.mth = mth;
      gpsData.yr = yr + 2000;
      gettingGPS = 1;                                 // Update LX200's clock with GPS data 
      Serial1.print("#:Q#:Qn#:#Qs#:Qe#:Qw#:Sw4#");    // Max slew rate is set to 4 degrees per second
      waitSerial1(SER1_TIMEOUT); Serial1.read();
      //scopeConnected = 1;                             // If successful then we have a connection
      //if (!atStartPosition()) return;                 // Tell telescope it is horizonal and pointing due south
      if (debug) Serial.println("\nLX200 found");
      checkGuideBuddy();                              // See if telescope is responding to GuideBuddy
      return;
    }
  }  
  if (debug) Serial.println("No telescope found");
};
// --------------------------------------------------------------------------------------------------
bool atStartPosition() {// THIS ROUNTINE IS CURRENTLY REDUNDANT
  if (debug) Serial.println("atStartPosition()");
  scopeConnected = 0;
  clearBuf();
  Serial1.print("#:\06#");
  if (!Serial1.readBytes(buf, 1)) return 0;
  if (debug) Serial.println(buf);
  if (buf[0] == 'L') {
    if (debug) Serial.println("LX200 is parked - no action taken");
    return scopeConnected = 1;                              // LX200 is parked
  }
  clearBuf();
  Serial1.print("#:Gl#");
  if (!Serial1.readBytesUntil('#', buf, 5)) return 0;       // LX200 not repsonding
  if (debug) Serial.println(buf);
  if (buf[0] == '1') {
    if (debug) Serial.println("Already synched LX200 as horizontal and pointing due south");
    return scopeConnected = 1;                              // LX200 has already been setup
  }

  clearBuf();                                               // This routine tells telescope it is pointing
  Serial1.print("#:GR#");                                   // due south and is horizontal.
  if (!Serial1.readBytesUntil('#', buf, 9)) return 0;
  if (buf[5] == '.') {                                      // Put into hi-precision mode
    Serial1.print("#:U#:GR#");
    if (!Serial1.readBytesUntil('#', buf, 9)) return 0;
  }
  Serial1.print("#:Sr"); Serial1.print(buf); Serial1.print('#'); if (!Serial1.readBytes(buf, 1)) return 0;
  //Serial1.print("#:Sd-38*24:51#");  if (!Serial1.readBytes(buf, 1)) return 0;
  Serial1.print("#:Sd-38*00:00#");  if (!Serial1.readBytes(buf, 1)) return 0;
  Serial1.print("#:CM#");           if (!Serial1.readBytesUntil('#', buf, 33)) return 0;
  Serial1.print("#:Sl100#");        if (!Serial1.readBytes(buf, 1)) return 0; // Mark LX200 so we know it has been setup
  if(debug) Serial.println("LX200 now synched as horizontal and pointing due south");
  return scopeConnected = 1;
}
// --------------------------------------------------------------------------------------------------
bool testLXlink () {
  if (debug) Serial.println("{textLXlink}");
  clearBuf();
  Serial1.print("#:\x6#");                                // Send a short command for a 1 byte response for speed to test cxn
  Serial1.setTimeout(200);
  scopeConnected = (Serial1.readBytes(buf, 1) != 0);
  if (debug) Serial.println(buf);
  Serial1.setTimeout(SER1_TIMEOUT);
  if (!scopeConnected) {
    stopAll();                                            // Stop all guide pulses
    timerLED = millis() - 1;
    led[0] = B11;
    if (debug) Serial.println("LX200 not found");
  }
  else if (debug) Serial.println("LX200 connected");
  return scopeConnected;
}
// ==================================================================================================
// GET AZIMUTH FROM TELESCOPE
// ==================================================================================================
void getAzimuth() {
  byte tmpAZM;
  //byte gtSLEW, gtAZM;
  if (debug) Serial.println("{getAzimuth}");
  if (!scopeConnected) return;                                    // No point getting data if telescope is not connected
  if (!dbCxn()) return;                                           // No point getting data if no connection with Dome Controller

  digitalWrite(LED_DB, LED_OFF);
  //gtSLEW = lxSLEW; gtAZM = lxAZM;                                 // Remember any active go-to action
  do {
    lxSLEW = 'L';                                                 // Assume no connection with the telesope ('L' = lost connection)
    scopeConnected = 0;
    Serial1.print("#:GZ#");                                       // Get LX200's azimuth - either ddd*mm or ddd*mmm:ss
    if (Serial1.readBytesUntil('#', buf, 10) < 6) break;          // Invalid response - assume connection is lost

    float a = (buf[0] - '0') * 6000 + (buf[1] - '0') * 600 + (buf[2] - '0') * 60 + (buf[4] - '0') * 10 + (buf[5] - '0');
    scopeConnected = 1;                                           // Telescope is still connected
    a = 256.0 * a / 21600 + 128;                                  // 128 adjusts meridian (N is 0 degs);
    tmpAZM = (byte)a;                                             // Azimuth is reduced to a value 0 to 255
    if (debug) {Serial.print(lastAZM); Serial.print(" v "); Serial.println(tmpAZM);}
    lxSLEW = ((tmpAZM == --lxAZM) || (tmpAZM == ++lxAZM) || tmpAZM == ++lxAZM) ? 'T' : 'S';
    
    lxAZM = tmpAZM;
    if (debug) {
      buf[3] = '*';
      Serial.println(buf);
      if (a < 0) Serial.print('0');
      Serial.print(lxAZM, HEX); Serial.print(' '); Serial.println(lxAZM);
    }
  } while(0);
  /*if (gtSLEW == 'G' && lxSLEW == 'S') {                           // Keep sending go-to info if scope is still slewing
    lxSLEW = 'G';
    lxAZM = gtAZM;
  }
  else gtSLEW = lxSLEW; */                                          // Go-to not happening - deactivate gtSLEW with 'T' or 'S'
  led[0] = 0;
  timerLED = millis() + 20;
}
// ==================================================================================================
// SEND AZIMUTH CODE TO DOME CONTROLLER
// ==================================================================================================
void sendAZM(){
  if (debug) Serial.println("{sendAZM}");
  if (!scopeConnected) return;
  if (!dbCxn()) return;
// Sends ':', 'S' or 'G' or 'T', Xh, Xl, ~Xh, ~Xl, '#', e.g. for lxAZM  = 0, ":T00FF#" is sent
// lxSLEW is 'S', G, or 'T' and lxAZM is the scope's azimuth 0-255 (0 is due north)

  byte negAZM = lxAZM ^ 0xFF;                       // Calculate 1's complement of scopeAZM
  if (debug) {
    Serial.print("Transmitted azimuth data \n:");   // Azimuth is being sent, 0 degrees is due north
    Serial.print(lxSLEW);                           // Send slewing info ('S' = slewing, 'T' = tracking, 'G' = go-to)
    if (lxAZM < 16) Serial.print('0');              // Send Azimuth as two hexdigit number
    Serial.print(lxAZM, HEX);                       // Azimuth has been mapped between 0-255
    if (negAZM < 16) Serial.print('0');             // Send negative Azimuth as two hexdigit number
    Serial.print(negAZM, HEX);                      // Sending this ensures azimuth is sent correctly
    Serial.print("# ");                             // Send terminator
    Serial.println(lxAZM);
  }

  // SEND AZIMUTH TO DOME BUDDY CONTROLLER
  SerialMix.begin(BAUD_DB);
  SerialMix.print(':');                             // Azimuth is being sent, 0 degrees is due north
  SerialMix.print(lxSLEW);                          // Send slewing info ('S' = slewing, 'T' = tracking, 'G' = go-to)
  if (lxAZM < 16) SerialMix.print('0');             // Send Azimuth as two hexdigit number
  SerialMix.print(lxAZM, HEX);                      // Azimuth has been mapped between 0-255
  if (negAZM < 16) SerialMix.print('0');            // Send negative Azimuth as two hexdigit number
  SerialMix.print(negAZM, HEX);                     // Sending this ensures azimuth is sent correctly
  SerialMix.print('#');                             // Send terminator
  SerialMix.end();
  timerLED = millis() + 12;
  lastAZM = lxAZM;
}
// ==================================================================================================
// TELESCOPE IS PEFORMING A GO-TO OPERATION (AN 'MS' COMMNAD WAS SENT TO LX200)
// ==================================================================================================
void startingGoto() {
  if (debug) Serial.print("{startingGoto}");
  //if (lxSLEW == 'S') return;
  if (mode != MODE_WF) {                                // Get indicator byte from MS command if not using WiFi
    Serial1.readBytes(buf, 1);
    if (listen) Serial2.write(buf[0]);
    mode == MODE_PC ? Serial.write(buf[0]) : Serial2.write(buf[0]);
    if (buf[0] != '0') return;                          // '0' means valid GO-TO is starting
  }

  //Serial1.print(":Sr23:09:17#"); delay(100);Serial1.read();
  Serial1.print(":Gr#");                                // Get target RA 12:34[:56]
  if (Serial1.readBytesUntil('#', buf, 10) < 6) return;
  //Serial.print("RA:  "); Serial.println(buf);
  float ra = (buf[0] - '0') * 600 + (buf[1] - '0') * 60 + (buf[3] - '0') * 10 + (buf[4] - '0');   // Seconds are ignored
  ra /= 4.0;
  //Serial.print("ra: "); Serial.println(ra, 8);

  //Serial1.print(":Sd-06*43:12#"); delay(100); Serial1.read();
  Serial1.print(":Gd#");                                // Get target DEC +12*34[:56]
  if (Serial1.readBytesUntil('#', buf, 10) < 6) return;
  //Serial.print("DEC: "); Serial.println(buf);
  float dec = (buf[1] - '0') * 600 + (buf[2] - '0') * 60 + (buf[4] - '0') * 10 + (buf[5] - '0');  // Arcseconds are ignored
  if (buf[0] == '-') dec = -dec;
  dec /= 60.0;
  //Serial.print("dec: "); Serial.println(dec, 8);

  //Serial1.print(":SS03:26:41#"); delay(100); Serial1.read();
  Serial1.print(":GS#");                                // Get sidereal time 12:34:56
  if (Serial1.readBytesUntil('#', buf, 10) < 8) return;
  //Serial.print("SID: "); Serial.println(buf);
  float sid = 36000.0 *(buf[0] - '0') + (buf[1] - '0') * 3600 + (buf[3] - '0') * 600 + (buf[4] - '0') * 60+ (buf[6] - '0') * 10 + (buf[7] - '0');
  sid /= 240.0;                                        // 24*60*60/(15*24)=240 - current RA at meridian
  //Serial.print("sid: "); Serial.println(sid, 8);

  /* CALCULATE AZIMUTH! Astronomical Algorithms V2 page 93 (13.5 & 13.6) - LX200 can't give the Azimuth of target
      tan A = (sin H)/(cos H sin l - tan d cos l) */
  float H = normF(sid - ra, 360.0); 			                  // Hour Angle (increasing W from S)
  //Serial.print("H: "); Serial.println(H, 8);
  float l = gpsData.lat;                                    // Latitude
  //l = 38.0 + 55.0 / 60.0 + 17 / 3600.0;
  //Serial.print("lat: "); Serial.println(l, 8);
  float A = normF(atan2D(sinD(H), cosD(H) * sinD(l) - tanD(dec) * cosD(l)), 360.0);
  //Serial.print("A: "); Serial.println(A, 8);
  A = 256.0 * A / 360 + 128;                                // Reduce azimuth to 0 - 255 (North is 0)
  lxAZM = byte(A);
  //Serial.println(lxAZM);
  lxSLEW = 'G';
  sendAZM();                                                // Send the controller the telescope's target azimuth and get the Dome moving
  delay(100);
  sendAZM();
}
//---------------------------------------------------------------------------------------------------
float atan2D(float a, float b) {
  return atan2(a, b) * 57.29577951;
}
//---------------------------------------------------------------------------------------------------
float sinD(float a) {
  return sin(a / 57.29577951);
}
//---------------------------------------------------------------------------------------------------
float cosD(float a) {
  return cos(a / 57.29577951);
}
//---------------------------------------------------------------------------------------------------
float tanD(float a) {
  return tan(a / 57.29577951);
}
//---------------------------------------------------------------------------------------------------
float normF(float v, float r) {
  return v - int (v / r) * r + (v < 0) * r;
}
#pragma endregion DOMEBUDDY STUFF
// --------------------------------------------------------------------------------------------------
#pragma region GPS STUFF
// ==================================================================================================
// UPDATE TELESCOPE SETTINGS WITH GPS DATA
// ==================================================================================================
void getGPSdata() {
// This routine waits for a $GNRMC sentence to be received and grabs the time, date & global coords
  String gpsTime, gpsDate, gpsLng, gpsLat, line = "";
  if (debug) Serial.println("{getGPSdata}");
  if (!gettingGPS || !gpsData.fix) return;
  if (!testLXlink()) return;                                    // Quit if telescope link lost
  SerialMix.begin(BAUD_DB);

  if (SerialMix.read() == '$') {
    line = SerialMix.readStringUntil('\n') + "\n";
    if (line.startsWith("GNRMC")) {
      if (debug) Serial.print(line);

      if (gpsData.fix & B1100 == B0000) sendLoc();
      if (gpsData.fix & B0001){                                 // Get GPS time
        gpsTime = parseGPS(line, 1);
        if (gpsTime != "") {
          gpsData.hr  = gpsTime.substring(0,2).toInt();
          gpsData.min = gpsTime.substring(2,4).toInt();
          gpsData.sec = gpsTime.substring(4,6).toInt();
          gpsData.fix &= B1110;
          sendTime();
          sendSidTime();
        }
      }
      if (gpsData.fix & B0010) {                                // Get GPS date
        gpsDate = parseGPS(line, 9);
        if (gpsDate != "") {
          gpsData.day = gpsDate.substring(0,2).toInt();
          gpsData.mth = gpsDate.substring(2,4).toInt();
          gpsData.yr  = gpsDate.substring(4,6).toInt() + 2000;
          gpsData.fix &= B1101;
          sendDate();
        }
      }
      if (gpsData.fix & B0100) {                                // Get GPS latitude
        gpsLat  = parseGPS(line, 3);
        if (gpsLat != "") {
          gpsData.lat = gpsLat.toFloat() / 100;
          gpsData.fix &= B1011;
          if (parseGPS(line,4) == "S") gpsData.lat = -gpsData.lat;
        }
      }
      if (gpsData.fix & B1000) {                                // Get GPS longitude
        gpsLng  = parseGPS(line, 5);
        if (gpsLng != "") {
          gpsData.lng = gpsLng.toFloat() / 100;
          gpsData.fix &= B0111;
          if (parseGPS(line,6) == "W") gpsData.lng = -gpsData.lng;
        }
      }
      if (gpsData.fix & B1100 == 0) sendLoc();
      if (debug) {
        Serial.print("Time: "); Serial.println(gpsTime);
        Serial.println(gpsData.hr); Serial.println(gpsData.min); Serial.println(gpsData.sec);
        Serial.print("Lat:  "); Serial.println(gpsLat);
        Serial.println(gpsData.lat);
        Serial.print("Lng:  "); Serial.println(gpsLng);
        Serial.println(gpsData.lng);
        Serial.print("Date: "); Serial.println(gpsDate);
        Serial.println(gpsData.day); Serial.println(gpsData.mth); Serial.println(gpsData.yr);
      }
    }
  }
  if (!gpsData.fix) {
    gettingGPS = 0;
    if (debug) Serial.println("GPS fix obtained");
  }
}
// --------------------------------------------------------------------------------------------------
String parseGPS (String sentence, int n) {        // Get argument 'n' from the GPS sentence
  byte start = 0, count = 0;
  for (byte i = 0; i < sentence.length(); i++){
    if (sentence.charAt(i) == ',')
      if (++count == n) {
        start = i + 1; 
        break;
      }
  }
  if (sentence.charAt(start) == ',') return "";
  for (byte i = start; i < sentence.length(); i++){
    if (sentence.charAt(i) == ',') return sentence.substring(start, i);
  }
  return "";
}
// --------------------------------------------------------------------------------------------------
void sendTime() {
  if (debug) Serial.println("{sendTime}");
  sprintf(buf, "#:SL%02d:%02d:%02d#", gpsData.hr, gpsData.min, gpsData.sec);    // Form command
  if (debug) Serial.println(buf);
  Serial1.print(buf);                                                           // Update the LX200's clock to local time
  clearBuf();
  Serial1.readBytes(buf,1);                                                     // Read in a '0' or '1'
  if (debug) Serial.println(buf);
}
// --------------------------------------------------------------------------------------------------
void sendDate() {
  if (debug) Serial.println("{sendDate}");
  if (mth == gpsData.mth && day == gpsData.day && yr == (gpsData.yr % 100)) {
    if (debug) Serial.println("GPS date is the same as LX200's.");
    return;
  }
  day = gpsData.day;
  mth = gpsData.day;
  yr = gpsData.yr + 2000;
  gpsData.fix |= B0001;                   // Get GPS time again
  sprintf(buf, "#:SC%02d/%02d/%02d#", gpsData.mth, gpsData.day, (gpsData.yr % 100));
  Serial1.print(buf);                     // Send date to LX200
  if (debug) Serial.println(buf);
  clearBuf();
  Serial1.readBytes(buf,1);               // Read in a '0' or '1'
  if (debug) Serial.println(buf);
  if (buf[0] == '0') return;              // Quit if date is invalid
  clearBuf();
  Serial1.readBytesUntil('#', buf, 33);   // Get "Updating planetary data" response
  if (debug) Serial.println(buf);
  waitSerial1(SER1_TIMEOUT);              // Wait for the LX200 to recalculate
  clearBuf();
  Serial1.readBytesUntil('#', buf, 33);   // Get blank line response
  if (debug) Serial.println(buf);
}
// --------------------------------------------------------------------------------------------------
void sendLoc() {
  if (debug) Serial.println("{sendLoc}");
  gpsData.fix |= B0001;                                     // Get GPS time again
  Serial1.print("#:W1#:SMGPS#");                            // Set LX200's site location 1 name as "GPS" and select it
  clearBuf();
  waitSerial1(SER1_TIMEOUT);
  Serial1.readBytes(buf,1);                                 // Read in a '0' or '1'

  int deg, min;
  float lat = gpsData.lat,
        lng = gpsData.lng;
  char sign = lat >= 0 ? '+' : '-';                         // Sort out latitude hemisphere

  lat = abs(lat) + 0.008333333;                             // Add 0.5 arcmin to account for rounding
  deg = int(lat);
  min = int((lat - deg) * 60);
  sprintf(buf, "#:St+%02d*%02d#", deg, min);                // Form command string
  buf[4] = sign;                                            // Fix hemisphere sign

  Serial1.print(buf);                                       // Send Latitude to telescope
  if (debug) Serial.println(buf);
  clearBuf();
  Serial1.readBytes(buf,1);                                 // Read in a '0' or '1'
  if (debug) Serial.println(buf);

// Longitude for the LX200 is from 0 deg 0 min to 359 deg 59 min, increasing west from Greenwich.
// E.g. 10 degs east is 350 and 10 degs west is 10 degs for the LX200.
// The GPS data longitude is negative west of Greenwich (0 to -180) and postive east (0 to 180)

  lng = (lng > 0.00 ? 360.0 - lng : -lng) + 0.008333333;    // Rounding adj of 0.5 arcmin

  deg = int(lng);
  min = int((lng - deg) * 60.0);
  sprintf(buf, "#:Sg%03d*%02d#", deg, min);                 // Form command string
  Serial1.print(buf);                                       // Send longitude to telescope
  if (debug) Serial.println(buf);
  clearBuf();
  Serial1.readBytes(buf,1);                                 // Read in a '0' or '1'
  if (debug) Serial.println(buf);
}
// --------------------------------------------------------------------------------------------------
void sendSidTime() {
  if (debug) Serial.println("{sendSidTime}");
// Calculate Julian Day x 10 (Astronomical Algorithms page 59)
  int y = gpsData.yr, 
      m = gpsData.mth;
  if(m < 3){
    y--;
    m+= 12;
  }
  int a = int(y / 100),
      b = 2 - a + int(a / 4);
  long i = (float)(365.25 * (y + 4716)),
       j = 30.6001 * (m + 1);
  unsigned long jdx10 = (i + j + gpsData.day + b) * 10 - 15245L;

// Calculate Sidereal time at Greenwich (Astronomical Algorithms page 83)
  long  jd = (jdx10 - 24515450);
  float T = float(jd) / 365250.0,
        Th0 = norm((100.46061837 + 36000.770053608 * T + 0.000387933 * T * T - (T * T * T) / 38710000) / 15, 24.0);
        Th0 += 1.00273790935 * ((float)gpsData.hr * 3600.0 + (float)gpsData.min * 60.0 + (float)gpsData.sec + 0.25) / 3600.0;

// Calulate local sidereal time by taking account of longitude
  Th0 += (gpsData.lng / 15.0);                                  // W of Greenwich is -ve, E is +ve
  Th0 = norm(Th0, 24.0);
  int sidHr  = int(Th0),
      sidMin = int((Th0 - sidHr) * 60.0),
      sidSec = int((Th0 - sidHr - sidMin / 60.0) * 3600.0);

  sprintf(buf, "#:SS%02d:%02d:%02d#", sidHr, sidMin, sidSec);   // Form command string

  Serial1.print(buf);                                           // Update LX200's sidereal clock
  if (debug) Serial.println(buf);
  clearBuf();
  Serial1.readBytes(buf,1);                                     // Read in a '0' or '1'
  if (debug) Serial.println(buf);
}
// --------------------------------------------------------------------------------------------------
void clearBuf() {
//  if (debug) Serial.println("{clearBuf}");
  for(byte i = 0; i <= BUF_SIZE; i++) buf[i] = 0;
}
// --------------------------------------------------------------------------------------------------
float norm(float a, float b){       //Normalises a between 0 and b
//  if (debug) Serial.println("{norm}");
  return a - int(a / b) * b + (a < 0) * b;
}
#pragma endregion GPS STUFF
// --------------------------------------------------------------------------------------------------
#pragma region TEST STUFF
// --------------------------------------------------------------------------------------------------
void testGPSModulePC() {
  Serial.println("testGPSmodulePC");
  while (true) {
    if (SerialMix.available()) Serial.write(SerialMix.read());
    if (Serial.available()) SerialMix.write(Serial.read());
  }
}
// --------------------------------------------------------------------------------------------------
void testBluetoothPC() {
  Serial.println("testBluetoothPC");
  while(1) {
    if (Serial.available()) Serial2.write(Serial.read());    // Send any PC data to Bluetooth
    if (Serial2.available()) Serial.write(Serial2.read());    // Send any Bluetooth data to the PC
  }
}
// --------------------------------------------------------------------------------------------------
void testBluetoothLX200() {
  Serial.println("testBluetoothLX200");
  while(1) {
    if (Serial1.available()) {
      if (dataFlag != 1) Serial.print("\n[T=>B]");
      dataFlag = 1;
      Serial.write(Serial1.peek());
      Serial2.write(Serial1.read());                      // Send any Telescope data to Bluetooth
    }
    if (Serial2.available()) {
      if (dataFlag != 2) Serial.print("\n[B=>T]");
      dataFlag = 2;
      Serial.write(Serial2.peek());
      Serial1.write(Serial2.read());                      // Send any WiFi data to Bluetooth
    }
  }
}
// --------------------------------------------------------------------------------------------------
void testWiFiLX200() {
  Serial.println("testWiFiLX200");
  while(1) {
    ESP8266Client client = server.available();
    while (client.connected()) {
      if (Serial1.available()) {
        if (dataFlag != 1) Serial.print("\n[T=>W]");
        dataFlag = 1;
        Serial.write(Serial1.peek());
        client.print(Serial1.read());                      // Send any Telescope data to WiFi
      }
      if (client.available()) {
        if (dataFlag != 2) Serial.print("\n[W=>T]");
        dataFlag = 2;
        Serial.write(client.peek());
        Serial1.write(client.read());                      // Send any WiFi data to the LX200
      }
    }
  }
}
// --------------------------------------------------------------------------------------------------
void testESPpc() {
  Serial.println("testESPpc");

  while (1) {
    while(Serial.available()) {
      if (dataFlag == 2) Serial.print("\n[P=>W]");
      dataFlag = 1;
      Serial.write(Serial.peek());
      Serial3.write(Serial.read());                      // Send any Telescope data to WiFi
  }
    while (Serial3.available()) {
      if (dataFlag == 1) Serial.print("\n[W=>P]");
      dataFlag = 2;
      Serial.write(Serial3.read());                      // Send any WiFi data to the LX200
    }
  }
}
// --------------------------------------------------------------------------------------------------
void testWifiPC() {
  Serial.println("testWiFiPC.");

  while(1) {
    ESP8266Client client = server.available();
    while (client.connected()) {
      while (Serial.available()) {
        if (dataFlag != 1) Serial.print("\n[P=>W]");
        dataFlag = 1;
        Serial.write(Serial.peek());
        client.print(Serial.read());                      // Send any Telescope data to WiFi
      }
      while (client.available()) {
        if (dataFlag != 2) Serial.print("\n[W=>P]");
        dataFlag = 2;
        Serial.write(client.read());                      // Send any WiFi data to the LX200
      }
    }
  }
}
// --------------------------------------------------------------------------------------------------
void testWifiLX200raw() {
  Serial.println("testWiFiPCraw");
  while(1) {
    ESP8266Client client = server.available();
    while (client.connected()) {
      if (Serial1.available()) {client.write(Serial1.read()); delay(10);}   // Send any PC data to WiFi
      if (client.available()) Serial1.write(client.read());    // Send any WiFi data to the PC
    }
  }
}
// --------------------------------------------------------------------------------------------------
void testPcLX200() {
  Serial.println("testPcLX200");
  while(1) {
    ESP8266Client client = server.available();
    if (Serial1.available()) {
      if (dataFlag != 1) Serial.print("\n[T=>P]");
      dataFlag = 1;
      Serial.write(Serial1.read());                      // Send any Telescope data to PC
    }
    if (Serial.available()) {
      if (dataFlag != 2) Serial.print("\n[P=>T]");
      dataFlag = 2;
      Serial.write(Serial.peek());
      Serial1.write(Serial.read());                      // Send any PC data to the LX200
    }
  }
}
// --------------------------------------------------------------------------------------------------
void rawPcLX200() {
  digitalWrite(LED_N, HIGH);
  while(1) {
    while(Serial1.available()) Serial.write(Serial1.read());                   // Send any Telescope data to PC
    while(Serial.available()) Serial1.write(Serial.read());                    // Send any PC data to the LX200
  }
}
// --------------------------------------------------------------------------------------------------
void test12Vpins () {
  while(1) {
    Serial.print(analogRead(GUIDE_12V)>100);
    Serial.print("\t");
    Serial.println(analogRead(RS232_12V)>100);
  }
}
// --------------------------------------------------------------------------------------------------
#pragma endregion TEST STUFF
// --------------------------------------------------------------------------------------------------
#pragma region NOTES
/*DIP Switches for ESP module on Arduino Mega 2560 WiFi board:
------------------------------------------------------------
* ESP  <-> Serial (USB): 0000110X  Switch: RX0/TX0 - ESP Sketch/Firmware Upload
* ESP  <-> Serial (USB): 0000111X  Switch: RX0/TX0 - ESP Comms
* Mega <-> Serial (USB): 1111000X  Switch: RX3/TX3 - Full Arduino Mega Access with ESP on Serial3 Baud 115200
* Mega <-> Serial (USB): 0011000X  Switch: RX0/TX0 - Full Arduino Mega Access (Comms/Sketch Upload), ESP not connected
  Mega <-> ESP only    : 1100000X  Switch: N/A     - Comms between Mega & ESP only
  Nothing connected    : 0000000X  Switch: N/A     - Both independent

  After firmware upload, issue "AT+UART_DEF=19200,8,1,0,0" to change Baud rate from 115200 to 19200.  115200 does not work well.
  DomeBuddy Dongle sits between a PC's USB port and the Meade 16" LX200 SCT Classic telescope.
  It presents itself as a COM port serial device on the PC with the setting 19200 baud, 8 data bits, 1 stop bit, no parity.
  It allows the PC to communicate with the telescope unhindered.  If the dongle has a Bluetooth connection with the
  DomeBuddy Controller, then every 5 seconds it will wait for the comms between the PC and telescope to be quiet before 
  it asks for information from the telescope.  The information will allow the dongle to determine its slew status and azimuth.
  The dongle will then transmit the azimuth, reduced to a value between 0 and 255, and slew status in encoded form.
  There are two buttons which when both pressed will reset the dongle.
*/
#pragma endregion NOTES
// --------------------------------------------------------------------------------------------------
#pragma region JUNKYARD
void startingGoto2() {
  return;
  if (lxSLEW == 'G') return;
  if (mode != MODE_WF) {                                // Get indicator byte from MS command if not using WiFi
    Serial1.readBytes(buf, 1);
    if (listen) Serial2.write(buf[0]);
    mode == MODE_PC ? Serial.write(buf[0]) : Serial2.write(buf[0]);
    if (buf[0] != '0') return;                          // '0' means valid GO-TO is starting
  }

  char telRA[15];
  char telDEC[15];
  char objRA[15];
  char objDEC[15];

  Serial1.print(":Q#:GR#"); telRA[Serial1.readBytesUntil('#', telRA, 10)] = 0;
    Serial.print("\ntelRA:"); Serial.println(telRA);
  Serial1.print(":GD#"); telDEC[Serial1.readBytesUntil('#', telDEC, 10)] = 0;
    Serial.print("telDEC:"); Serial.println(telDEC);
  Serial1.print(":GA#"); buf[Serial1.readBytesUntil('#', buf, 10)] = 0;
    Serial.print("telGA:"); Serial.println(buf);
  Serial1.print(":Gr#"); objRA[Serial1.readBytesUntil('#', objRA, 10)] = 0;
    Serial.print("objRA:"); Serial.println(objRA);
  Serial1.print(":Gd#"); objDEC[Serial1.readBytesUntil('#', objDEC, 10)] = 0;
    Serial.print("objDEC:"); Serial.println(objDEC);
  Serial1.print(":CM#"); Serial1.readBytesUntil('#', buf, 33);
  Serial1.print(":Q#:GR#"); buf[Serial1.readBytesUntil('#', buf, 10)] = 0;
    Serial.print("telRA:"); Serial.println(buf);
  Serial1.print(":GD#"); buf[Serial1.readBytesUntil('#', buf, 10)] = 0;
    Serial.print("telDEC:"); Serial.println(buf);
  Serial1.print(":GA#"); buf[Serial1.readBytesUntil('#', buf, 10)] = 0;
    Serial.print("objGA:"); Serial.println(buf);
  Serial1.print(":Sr"); Serial1.print(telRA); Serial1.print('#');
  Serial1.print(":Sd"); Serial1.print(telDEC); Serial1.print('#');
  Serial1.print(":CM#"); Serial1.readBytesUntil('#', buf, 33);
  Serial1.print(":Sr"); Serial1.print(objRA); Serial1.print('#');
  Serial1.print(":Sd"); Serial1.print(objDEC); Serial1.print('#');
  Serial1.print(":MS#"); Serial1.read();

  float a = (buf[0] - '0') * 6000 + (buf[1] - '0') * 600 + (buf[2] - '0') * 60 + (buf[4] - '0') * 10 + (buf[5] - '0');
  a = 256.0 * a / 21600 + 128;                                  // 128 adjusts meridian (N is 0 degs);
  lxAZM = (byte)a;
  lxSLEW = 'G';
  sendAZM();
  delay(100);
  sendAZM();
}
//---------------------------------------------------------------------------------------------------

#pragma endregion JUNKYARD
// -------------------------------------------------------------------------------------------------- 