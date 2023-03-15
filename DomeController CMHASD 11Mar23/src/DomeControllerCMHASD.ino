#define VERSION "\nDomeBuddy Controller CMHASD V2.220311 by keithrickard@hotmail.com"
#define LCD_VERS "$$Version 2.230311$Nby Keith Rickard"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
// For Arduino Mega 2560
#define TEST          1
#define RESET_EEPROM  1

#pragma region DEFINITIONS
// PARAMETERS
#define ENC_DIR       1       // 0 = test rig, 1 = dome rig
#define MIN_RES       10000L  // Minimum acceptable dome rotation resolution
#define MTR_ALL       0
#define MTR_DIR1      1       // Ordinarily, 1 is CW > and 0 is CCW <
#define MTR_DIR2      1
#define MTR_DIR3      1
#define PERIOD_DEB    80
#define PERIOD_HOLD   500
#define PERIOD_CXN    5000   // Timeout period for not receiving comms from the dongle
#define BTBAUD        38400

#define ENC_ChA       21  // W  (SCL pin)
#define ENC_ChB       20  // Bk (SDA pin)
#define INDEX_SW      19  // Gy Homeswitch
#define BTSER_PIN1    18  // Y
#define BTSER_PIN2    17  // Gn1m
#define DIR1          8   // Motor 1
#define PWM1          9
#define DIR2          13   // Motor 2
#define PWM2          11
#define DIR3          12  // Motor 3
#define PWM3          10
#define ENGAGED       29  // Circuit for engagement of motors - 0 = means all are engaged
#define ENGAGED_GND   31
#define BTTXD         17  // RX1 pin  (BTRXD is not used as nothing needs to be transmitted)
#define BTSTAT        18
#define BUZZER        A1
#define BUZZER_GND    A0

#define LCD_VCC       5V  // R LCD pins (this definition is not used in sketch)
#define LCD_RS        23  // O
#define LCD_EN        25  // Y
#define LCD_D4        45  // Gn
#define LCD_D5        47  // Be
#define LCD_D6        49  // Pe
#define LCD_D7        51  // Gy
#define LCD_BL_A      53  // W
#define LCD_BL_K      GND // Bk VSS(1) = VE(3) = RW(5) = GND (this definition is not used in sketch)

#define BTN_L         A8  // W
#define BTN_L_GND     A9  // Bk
#define BTN_O         A10 // Pe
#define BTN_O_GND     A11 // Gy
#define BTN_E         A12 // R
#define BTN_E_GND     A13 // Bn
#define BTN_R         A14 // Be
#define BTN_R_GND     A15 // Gn

#define PRESSED       1
#define RELEASED      0

#define btnL    B01000
#define btnO    B00100
#define btnE    B00010
#define btnR    B00001
#define btnH    B10000
#define BLINK   B0001
#define CURSOR  B0010
#define DISP    B0100
#define LIGHT   B1000

volatile long currEnc, indexEnc[2];
volatile byte indexPtr;
long  prevEnc,
      currEncTest,
      prevMillis,
      enc180,
      encOne,
      targEnc,
      distEnc,
      stepDist,
      tempEnc,
      lastEnc,
      midPoint;
int   brakEnc[256];
bool  domeSafe = 0;
byte  testMode = TEST,
      domeAZM,
      targAZM,
      lastTargAZM,
      scopeAZM,           // 0 = N, 64 = E, 128 = S, 192 = W.
      dir,
      lastDir,
      flag, 
      pwm = 0,
      slaved = 0,
      parking = 0,
      btStat = 0,
      newData = 0,
      calibrating = 0,
      dispC = 0,
      dispR = 0,
      dispStatus = DISP,  //Bit 0 = cursor on/off, Bit 1 = blink on/off, Bit 2 = LCD on/off
      dispStatusBuf,
      btnsPressed = 0,
      btnsLast = 0, 
      btnsCurr = 0, 
      btnsDeb = 0, 
      scanStep = 0,
      encMoving = 0;
byte startPWM1, startPWM2, startPWM3, stopPWM1, stopPWM2, stopPWM3, maxPWM1, maxPWM2, maxPWM3;
char  buf[10],
      btnDBC = 0,
      disp[2][17],
      dispBuf[2][17];
unsigned long timerBtns,
              lcdTimer,
              cxnTimer;
struct ee{
  bool bzBtns = 1;
  bool bzAll = 1;
  bool bzMoves = 1;
  bool bzAlert = 1;
  bool bzComms =1;
  bool encDir = 0;               // 0 = clockwise, 1 = counter clockwise (test-rig)
  byte indexSw = 0;
  byte azmAdj = 0;               // 0 = azimuth is measured from N, 128 = from S
  byte dir1cw = MTR_DIR1;
  byte dir2cw = MTR_DIR2;
  byte dir3cw = MTR_DIR3;
  byte maxPWM1cw = 175;
  byte maxPWM2cw = 175;
  byte maxPWM3cw = 175;
  byte startPWM1cw = 90;
  byte startPWM2cw = 90;
  byte startPWM3cw = 90;
  byte stopPWM1cw = 90;
  byte stopPWM2cw = 90;
  byte stopPWM3cw = 90;
  byte maxPWM1ccw = 175;
  byte maxPWM2ccw = 175;
  byte maxPWM3ccw = 175;
  byte startPWM1ccw = 90;
  byte startPWM2ccw = 90;
  byte startPWM3ccw = 90;
  byte stopPWM1ccw = 90;
  byte stopPWM2ccw = 90;
  byte stopPWM3ccw = 90;
  byte rampSpeed = 15;
  byte stopSpeed = 1;
  int stuckTime = 4000;
  char password[7] = {"\0\0\0\0\0\0"};
  long lcdTimeout = 300000;             // 5 minutes time out (300 seconds)
  long enc360 = 46000;
  byte homeAZM = 96;
  int  addr = sizeof(ee);
} ee;

//SoftwareSerial btSerial(btTXD, btRXD);  // Bluetooth Module is in Master mode & bound to DomeBuddy Dongle's address
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#pragma endregion DEFINITIONS
//==============================================================================================================
#pragma region SETUP
void setup() {
  pinMode(ENC_ChA, INPUT_PULLUP);
  pinMode(ENC_ChB, INPUT_PULLUP);
  pinMode(INDEX_SW, INPUT_PULLUP);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUZZER_GND, OUTPUT);
  pinMode(BTN_R, INPUT_PULLUP);
  pinMode(BTN_R_GND,OUTPUT);
  pinMode(BTN_O, INPUT_PULLUP);
  pinMode(BTN_O_GND, OUTPUT);
  pinMode(BTN_E, INPUT_PULLUP);
  pinMode(BTN_E_GND, OUTPUT);
  pinMode(BTN_L, INPUT_PULLUP);
  pinMode(BTN_L_GND, OUTPUT);
  pinMode(ENGAGED, INPUT_PULLUP);
  pinMode(ENGAGED_GND, OUTPUT);
  pinMode(BTSTAT, INPUT);
  pinMode(LCD_BL_A, OUTPUT);

  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(BTN_R_GND, LOW);
  digitalWrite(BTN_O_GND, LOW);
  digitalWrite(BTN_E_GND, LOW);
  digitalWrite(BTN_L_GND, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(BUZZER_GND, LOW);
  digitalWrite(LCD_BL_A, HIGH);

  lcd.begin(16, 2);
  Serial.begin(230400);
  Serial.println(VERSION);
  Serial.println("Send ? for help");
  Serial2.begin(BTBAUD);                            // Open connection to Bluetooth module
  Serial2.setTimeout(500);
  ee.addr = sizeof(ee);
  #if RESET_EEPROM
    EEPROM.put(0, ee);
    EEPROM.put(ee.addr, 46000L / 2);
  #endif
  EEPROM.get(0, ee);
  EEPROM.get(ee.addr, prevEnc);                     // Get last known dome encoder position
  currEnc = prevEnc;
  setupISR();                                       // Activate encoders
  enc180 = ee.enc360 >> 1;                          // Value for 180 degrees (divide by 2)
  encOne = ee.enc360 >> 8;                          // Number of encoder counts for one azimuth reference point (divide by 256)
  scopeAZM = domeAZM = lastTargAZM = enc2azm(currEnc);
  lcdUDGs();
  do {
    LCD("$$ DICK CHAMBERS$N  OBSERVATORY"); sendDisp();
    beep(50, 1); delay(50); beep(50, 1); delay(50); beep(50, 1);
    lcdTimer = millis() + ee.lcdTimeout;
    waitBtns(2250);
    if (btnsLast & btnL && btnsLast & btnR) {
        for (byte i = 0; i <= 5; i++) ee.password[i] = 0;
        EEPROM.put(0, ee);
        LCD("$$PASSWORD RESET$NPress OK"); sendDisp();
        beep(1000, 1);
        waitOKesc(0);
    }
    if (btnsLast) break;
    LCD("$$   DOME BUDDY$N     CMHASD"); sendDisp();
    if (waitBtns(1500)) break;
    LCD(LCD_VERS); sendDisp();
    if (waitBtns(2000)) break;
    LCD("$$Credits:$NMartin Crow"); sendDisp();
    if (waitBtns(1500)) break;
    LCD("$$Martin Crow$NSteve Floodgate"); sendDisp();
    if (waitBtns(750)) break;
    LCD("$$Steve Floodgate$NAndy Barber"); sendDisp();
    if (waitBtns(750)) break;
    LCD("$$Andy Barber"); sendDisp();
    if (waitBtns(750)) break;
  } while (0);
}
#pragma endregion SETUP
//==============================================================================================================
#pragma region MAIN MENU
//--------------------------------------------------------------------------------------------------------------
void loop() {
  char *items[] = {"Track telescope",
                   "Move dome",
                   "Park dome",
                   "Home position",
                   "Dome safe: NO ",
                   "Sync to scope",
                   "Settings"};
  byte selection = 1;
  do{
    if (domeSafe) {
      *(items[4] + 11) = 'Y';
      *(items[4] + 12) = 'E';
      *(items[4] + 13) = 'S';
    } else {
      *(items[4] + 11) = 'N';
      *(items[4] + 12) = 'O';
      *(items[4] + 13) = ' ';
    }
    slaved = parking = calibrating = 0;
    selection =  menu(items, 7, selection);
    switch(selection & 15){
      case 1: aTrackTelescope();    break;
      case 2: manualMove(MTR_ALL);  break;
      case 3: aParkDome();          break;
      case 4: aSetHomePosn();       break;
      case 5: aConfirmSafety();     break;
      case 6: aSync2scope();        break;
      case 7: bSettings();          break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void aTrackTelescope() {
  if (!safe()) return;
  while (Serial2.available()) Serial2.read();             // Clear Bluetooth serial buffer (DongleBuddy)
  while (Serial.available()) Serial.read();               // Clear USB serial buffer (PC)
  scopeAZM = domeAZM;
  cxnTimer = millis() + PERIOD_CXN;
  slaved = 1;

  while (slaved == 1) {
    getScopeAZM();
    btCxn() ? LCD("$HAwaiting azmth ^") : LCD("$HConnecting...  _");
    if (disp[1][0] != 'A') LCD("$NAz:     Pt:    .");
    LCDintAt(1, 3, (360UL * byte(scopeAZM + ee.azmAdj) / 256.0), 3); LCD("` ");  // Azimuth in degrees
    LCDintAt(1, 11, domeAZM, 3); sendDisp();
    if (cxnTimer < millis()) {
        beep(100, ee.bzComms); if (!btCxn()) {delay(100); beep(100, ee.bzComms);}
        cxnTimer = millis() + PERIOD_CXN;
    }

    if (manualMove(MTR_ALL) & btnE) return;               // See if dome has been moved manually or if Esc has been pressed
    domeAZM = enc2azm(normEnc());                         // Normalise currEnc then map domeAZM from value 0 - 255
    if (pressedBtns() & btnO) {
      LCD("$HEncoder:       _");
      while (pressedBtns()) LCDintAt(0, 9, normEnc(), 5);
    }
    moveDome(scopeAZM, 1);
  }
}
//--------------------------------------------------------------------------------------------------------------
void aParkDome() {
  if (!safe()) return;
  if (domeAZM != ee.homeAZM) {
    LCD("$$Park Dome?$NPress OK or Esc"); sendDisp();
    if (waitOKesc (0) & btnE) return;
    slaved = parking = 1;
    while (parking && moveDome(ee.homeAZM, 0));
    if (!parking) return;
  }
  LCD ("$$DOME PARKED$");
  LCD("$NPress OK$F"); sendDisp();
  beep(1000, ee.bzAlert);
  waitOKesc(0);
}
//--------------------------------------------------------------------------------------------------------------
void aSetHomePosn() {
  LCD("$$Set as home$Nposition?");
  sendDisp();
  if (waitOKesc(0) & btnE) return;
  ee.homeAZM = enc2azm(normEnc());
  updateEEPROM(currEnc);
  LCD("$$Home position$Nhas been set. OK");
  sendDisp();
  while(waitOKesc(0) != btnO);
}
//--------------------------------------------------------------------------------------------------------------
void aConfirmSafety() {
  domeSafe = 0;
  LCD("$$Are all straps  $Nundone?       OK"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  LCD("$$Are all wedges  $Nremoved?      OK"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  LCD("$$Is the battery  $Ndisconnected? OK"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  LCD("$$All obstructions$Nremoved?      OK"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  LCD("$$Are all motors  $Nengaged?      OK"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  beep(1000, 1);
  domeSafe = 1;
}
//--------------------------------------------------------------------------------------------------------------
bool safe(){
  if (domeSafe) return 1;
  for (byte i = 0; i <= 3; i++) {
    beep(100, 1);
    delay(100);
  }
  LCD("$$DOME NOT SAFE TO$NTURN.   Press OK"); sendDisp();
  waitOKesc(0);
  return 0;
}
//--------------------------------------------------------------------------------------------------------------
void aSync2scope() {
  char temp[2][17];
  strcpy(temp[0], disp[0]);
  strcpy(temp[1], disp[1]);
  if (!btCxn) {
    LCD("$$Can't sync dome.$NScope not found."); sendDisp();
  } 
  else {
    while(scanBtns());
    beep(250, ee.bzBtns);
    LCD("$$Sync dome to$Nscope?    OK/Esc");
    if (waitOKesc(0) & btnO) {
      currEnc = azm2enc(scopeAZM);
      domeAZM = scopeAZM;
      LCD("$$DOME SYNCHED$NTO TELESCOPE"); sendDisp();
    }
  }
  beep(3000, ee.bzAlert);
  LCD("$$"); LCD(temp[0]); LCD("$N"); LCD(temp[1]); sendDisp();
}
#pragma endregion MAIN MENU
//==============================================================================================================
#pragma region SETTINGS MENU
//--------------------------------------------------------------------------------------------------------------
void bSettings() {
  if (checkPassword()) return;  
  char *items[] = {"Calibrate dome",
                   "Motor setup",
                   "Buzzer",
                   "LCD timeout",
                   "Enc. direction",
                   "Index switch",
                   "Azimuth mode  ",
                   "Check hardware",
                   "Set password",
                   "Factory reset",
                   "Exit",
                   "About"};
  byte selection = 1;
  do{

    selection =  menu(items, 12, selection);
    switch(selection & 15){
      case 1: fCalibrateDome();     break;
      case 2: cMotorSetup();        break;
      case 3: eBuzzer();            break;
      case 4: bLCDTimeout();        break;
      case 5: bEncoderDirection();  break;
      case 6: bIndexSwitch();       break;
      case 7: bAzimuthMode();       break;
      case 8: gCheckHardware();     break;
      case 9: bSetPassword();       break;
      case 10: bFactoryReset();     break;
      case 11: return;              break;
      case 12: bShowVars();         break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void bLCDTimeout(){
	LCD("$$Enter seconds$Nfor timeout:");
	ee.lcdTimeout = getValue(ee.lcdTimeout / 1000, 999, 0, 1, 15) * 1000;
  EEPROM.put(0, ee);
	lcdTimer = millis() + ee.lcdTimeout;
}
//--------------------------------------------------------------------------------------------------------------
void bEncoderDirection() {
  bool mode = ee.encDir;
  while (1) {
    LCD("$$Enc. direction$NUse <> : ");
    mode ? LCD("CW  >") : LCD("CCW <");
    sendDisp();
    switch (waitBtns(0) & 15) {
      case btnL:  mode = 0; break;
      case btnR:  mode = 1; break;
      case btnE:  return;   break;
      case btnO:  ee.encDir = mode;
                  EEPROM.put(0, ee);
                  setupISR();
                  return;
                  break;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void bIndexSwitch() {
  bool onOff = ee.indexSw;
  while (1) {
    LCD("$$Index sw active?$NUse <> : ");
    onOff ? LCD("Yes") : LCD("No");
    sendDisp();
    switch (waitBtns(0) & 15) {
      case btnL:  onOff^=1;  break;
      case btnR:  onOff^=1;  break;
      case btnE:  return;    break;
      case btnO:  ee.indexSw = onOff;
                  EEPROM.put(0, ee);
                  return;
                  break;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void bAzimuthMode() {
  byte adj = ee.azmAdj;
  while (1) {
    LCD("$$Measure azmimuth$Nfrom ");
    adj == 128 ? LCD("South (<>)") : LCD("North (<>)");
    sendDisp();
    switch(waitBtns(0) & 15) {
      case btnL:  adj = 0; break;
      case btnR:  adj = 128; break;
      case btnE:  return;   break;
      case btnO:  ee.azmAdj = adj;
                  EEPROM.put(0, ee);
                  setupISR();
                  return;
                  break;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void bSetPassword() {
  char pswd[] = "\0\0\0\0\0\0";
  LCD("$$Enter password:$N$B$C"); sendDisp();
  for (byte i = 0; i <= 5; i++) {
    switch (waitBtns(0)) {
      case btnL: pswd[i] = 'L'; break;
      case btnO: pswd[i] = 'O'; break;
      case btnE: pswd[i] = 'E'; break;
      case btnR: pswd[i] = 'R'; break;
      case btnE | btnH: return; break;
      default: break;
    }
    if (btnsLast == (btnO | btnH)) break;
    LCD("$N"); LCD(pswd); sendDisp();
  }
  strcpy(ee.password, pswd);
  if (!ee.password[0]) LCD("$$No password set.$NPress OK");
  else {
    LCD("$$$b$cPassword set to:$N"); LCD(ee.password); LCD(".");
  }
  sendDisp();
  EEPROM.put(0, ee);
  beep(500, ee.bzAlert);
  waitOKesc(3000);
}
//--------------------------------------------------------------------------------------------------------------
bool checkPassword () {
  if (ee.password[0] == '\0') return 0;                     // If no set password then return immediately
  LCD("$$Enter password:$N$B"); sendDisp();
  char pswd[] = "\0\0\0\0\0\0";
  byte len = strlen(ee.password);
  Serial.println(len);
  for (byte i = 0; i <= 5; i++)  {
    switch (waitBtns(0) & 15) {
      case btnL: pswd[i] = 'L'; break;
      case btnO: pswd[i] = 'O'; break;
      case btnE: pswd[i] = 'E'; break;
      case btnR: pswd[i] = 'R'; break;
    }
    if (btnsLast == (btnE | btnH)) return 1;
    LCD("$N"); LCD(pswd); sendDisp();
    if (i == len - 1) {
      if (strcmp(ee.password, pswd) == 0) return 0;       // Password is correct
    }
  };
  LCD("$H$bWrong password! "); sendDisp();
  waitOKesc(0);
  return 1;
}
//--------------------------------------------------------------------------------------------------------------
void bFactoryReset() {
  LCD("$$Factory reset$NAre you sure?");
  sendDisp();
  if (waitOKesc(0) == btnE) return;
  LCD("$$Press <, OK, Esc$Nthen > to reset.");
  sendDisp();
  if (waitBtns(0) != btnL) return;
  if (waitBtns(0) != btnO) return;
  if (waitBtns(0) != btnE) return;
  if (waitBtns(0) != btnR) return;
  LCD("$$RESETTING...");
  sendDisp();
  ee.bzBtns = 1;
  ee.bzAll = 1;
  ee.bzMoves = 1;
  ee.bzAlert = 1;
  ee.bzComms = 1;
  ee.encDir = 0;
  ee.indexSw = 0;
  ee.azmAdj = 0;
  ee.dir1cw = MTR_DIR1;
  ee.dir2cw = MTR_DIR2;
  ee.dir3cw = MTR_DIR3;
  ee.maxPWM1cw = 175;
  ee.maxPWM2cw = 175;
  ee.maxPWM3cw = 175;
  ee.startPWM1cw = 90;
  ee.startPWM2cw = 90;
  ee.startPWM3cw = 90;
  ee.stopPWM1cw = 90;
  ee.stopPWM2cw = 90;
  ee.stopPWM3cw = 90;
  ee.maxPWM1ccw = 175;
  ee.maxPWM2ccw = 175;
  ee.maxPWM3ccw = 175;
  ee.startPWM1ccw = 90;
  ee.startPWM2ccw = 90;
  ee.startPWM3ccw = 90;
  ee.stopPWM1ccw = 90;
  ee.stopPWM2ccw = 90;
  ee.stopPWM3ccw = 90;
  ee.password[0] = 0;
  ee.rampSpeed = 20;
  ee.stopSpeed = 1;
  ee.stuckTime = 4000;
  ee.lcdTimeout = 300000;
  ee.enc360 = 46000 * 20 /16;
  ee.homeAZM = 96;
  ee.addr = sizeof(ee);
  EEPROM.put(0, ee);
  updateEEPROM(17100L);
  currEnc = 0;
  beep(1000, 1);
  LCD("$$Factory reset is$Ncomplete.     OK");
  sendDisp();
  waitOKesc(0);
}
//--------------------------------------------------------------------------------------------------------------
void bShowVars() {
  char *items[] =  {"enc360         ",
                    "enc180         ",
                    "encOne         ",
                    "currEnc        ",
                    "ee.addr        ",
                    "ee.enc         ",
                    "homeAZM        ",
                    "domeAZM        ",
                    "scopeAZM       ",
                    "lcdTimeOut     ",
                    "lcdTimer       ",
                    "BTSTAT         ",
                    "btCxn()        ",
                    "indexSw        ",
                    "More...        ",
                    "Exit           "};
  byte selection = 1;
  do {
    embedValue(items[0], ee.enc360);
    embedValue(items[1], enc180);
    embedValue(items[2], encOne);
    getEnc();
    embedValue(items[3], currEnc);
    embedValue(items[4], ee.addr);
    long eeEnc;
    EEPROM.get(ee.addr, eeEnc);
    embedValue(items[5], eeEnc);
    embedValue(items[6], ee.homeAZM);
    embedValue(items[7], domeAZM);
    embedValue(items[8], scopeAZM);
    embedValue(items[9], ee.lcdTimeout / 1000);
    embedValue(items[10], (lcdTimer - millis()) / 1000);
    embedValue(items[11], digitalRead(BTSTAT));
    embedValue(items[12], btCxn());
    embedValue(items[13], ee.indexSw);
    selection = menu(items, 16, selection);
    switch(selection & 15){
      case 15: bShowVars2();  break;
      case 0: return;        break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void bShowVars2() {
  char *items[] =  {"startMtr1cw    ",
                    "startMtr2cw    ",
                    "startMtr3cw    ",
                    "startMtr1ccw   ",
                    "startMtr2ccw   ",
                    "startMtr3ccw   ",
                    "stopMtr1cw     ",
                    "stopMtr2cw     ",
                    "stopMtr3cw     ",
                    "stopMtr1ccw    ",
                    "stopMtr2ccw    ",
                    "stopMtr3ccw    ",
                    "testMode   LIVE",
                    "More...        ",
                    "Exit           "};
  byte selection = 1;
  do {
    embedValue(items[0], ee.startPWM1cw);
    embedValue(items[1], ee.startPWM2cw);
    embedValue(items[2], ee.startPWM3cw);
    embedValue(items[3], ee.startPWM1ccw);
    embedValue(items[4], ee.startPWM2ccw);
    embedValue(items[5], ee.startPWM3ccw);
    embedValue(items[6], ee.stopPWM1cw);
    embedValue(items[7], ee.stopPWM2cw);
    embedValue(items[8], ee.stopPWM3cw);
    embedValue(items[9], ee.stopPWM1ccw);
    embedValue(items[10], ee.stopPWM2ccw);
    embedValue(items[11], ee.stopPWM3ccw);
    if (testMode) strcpy(items[12], "testMode  TEST");
    selection = menu(items, 15, selection);
    switch(selection & 15){
      case 14: bShowVars3();  break;
      case 15: return;        break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void bShowVars3() {
  char *items[] =  {"maxMtr1cw     ",
                    "maxMtr2cw     ",
                    "maxMtr3cw     ",
                    "maxMtr1ccw    ",
                    "maxMtr2ccw    ",
                    "maxMtr3ccw    ",
                    "dir1cw        ",
                    "dir2cw        ",
                    "dir3cw        ",
                    "rampSpeed     ",
                    "stopSpeed     ",
                    "stuckTime     ",
                    "ee.encDir     ",
                    "ee.azmAdj     ",
                    "pswd    ROLLER",
                    "Exit          "};
  byte selection = 1;
  do {
    embedValue(items[0], ee.maxPWM1cw);
    embedValue(items[1], ee.maxPWM2cw);
    embedValue(items[2], ee.maxPWM3cw);
    embedValue(items[3], ee.maxPWM1ccw);
    embedValue(items[4], ee.maxPWM2ccw);
    embedValue(items[5], ee.maxPWM3ccw);
    embedValue(items[6], ee.dir1cw);
    embedValue(items[7], ee.dir2cw);
    embedValue(items[8], ee.dir3cw);
    embedValue(items[9], ee.rampSpeed);
    embedValue(items[10], ee.stopSpeed);
    embedValue(items[11], ee.stuckTime);
    items[12][12] = ee.encDir + '0';
    items[12][13] = ee.encDir ? '>' : '<';
    embedValue(items[13], ee.azmAdj);
    for(byte i =0; i <=5; i++) items[14][i + 8] = ee.password[i];
    selection = menu(items, 16, selection);
    switch(selection & 15){
      case 0: return; break;
    }
  } while (selection & 15);
}
#pragma endregion SETUP MENU
//==============================================================================================================
#pragma region MOTOR SETUP MENU
//--------------------------------------------------------------------------------------------------------------
void cMotorSetup() {
  char *items[] = {"Motor 1       ",
                   "Motor 2       ",
                   "Motor 3       ",
                   "Speed up rate ",
                   "Slow down rate",
                   "Stuck time    ",
                   "Start PWMs    ",
                   "Stop PWMs     ",
                   "Max PWMs      ",
                   "Exit          "};
  byte selection = 1;
  do{
    embedValue(items[5], ee.rampSpeed);
    embedValue(items[6], ee.stopSpeed);
    embedValue(items[7], ee.stuckTime / 1000);
    selection = menu(items, 10, selection);
    switch(selection & 15){
      case 1: cMotorMenu(1);    break;
      case 2: cMotorMenu(2);    break;
      case 3: cMotorMenu(3);    break;
      case 4: cSpeedUpRate();   break;
      case 5: cSlowDownRate();  break;
      case 6: cStuckTime();     break;
      case 7: dStartPWMs();     break;
      case 8: dStopPWMs();      break;
      case 9: dMaxPWMs();       break;
      case 10: return;          break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void cMotorMenu(byte mtr) {
  char *items[] = {"MOTOR 1 MENU  ",
                   "Turn motor    ",
                   "Align Motor   ",
                   "Direction    1",
                   "PWMS SPEEDS:  ",
                   "Start >       ",
                   "Stop  >       ",
                   "Max   >       ",
                   "Start <       ",
                   "Stop  <       ",
                   "Max   <       ",
                   "Exit          "};
  byte selection = 1;
  do {
    items[0][6] = '0' + mtr;
    switch (mtr) {
      case 1: items[3][13] =  ee.dir1cw + '0';
              embedValue(items[5], ee.startPWM1cw);
              embedValue(items[6], ee.stopPWM1cw);
              embedValue(items[7], ee.maxPWM1cw);
              embedValue(items[8], ee.startPWM1ccw);
              embedValue(items[9], ee.stopPWM1ccw);
              embedValue(items[10], ee.maxPWM1ccw);   break;
      case 2: items[3][13] =  ee.dir2cw + '0';
              embedValue(items[5], ee.startPWM2cw);
              embedValue(items[6], ee.stopPWM2cw);
              embedValue(items[7], ee.maxPWM2cw);
              embedValue(items[8], ee.startPWM2ccw);
              embedValue(items[9], ee.stopPWM2ccw);
              embedValue(items[10], ee.maxPWM2ccw);   break;
      case 3: items[3][13] =  ee.dir3cw + '0';
              embedValue(items[5], ee.startPWM3cw);
              embedValue(items[6], ee.stopPWM3cw);
              embedValue(items[7], ee.maxPWM3cw);
              embedValue(items[8], ee.startPWM3ccw);
              embedValue(items[9], ee.stopPWM3ccw);
              embedValue(items[10], ee.maxPWM3ccw);   break;
    }
    selection =  menu(items, 12, selection);
    switch (mtr) {
      case 1 :switch (selection & 15) {
                case 2: gMotor(PWM1, DIR1, 1);  break;
                case 3: cAlignMotor(1);         break;
                case 4: cDirMotor(1);           break;
                case 6: ee.startPWM1cw = setPWM(3, 1, ee.startPWM1cw, 0, ee.maxPWM1cw);     break;
                case 7: ee.stopPWM1cw = setPWM(5, 1, ee.stopPWM1cw, 0, ee.maxPWM1cw);       break;
                case 8: ee.maxPWM1cw = setPWM(1, 1, ee.maxPWM1cw, (ee.startPWM1cw > ee.stopPWM1cw) ? ee.startPWM1cw : ee.stopPWM1cw, 255); break;
                case 9: ee.startPWM1ccw = setPWM(4, 1, ee.startPWM1ccw, 0, ee.maxPWM1ccw);  break;
                case 10: ee.stopPWM1ccw = setPWM(6, 1, ee.stopPWM1ccw, 0, ee.maxPWM1ccw);    break;
                case 11: ee.maxPWM1ccw = setPWM(2, 1, ee.maxPWM1ccw, (ee.startPWM1ccw > ee.stopPWM1ccw) ? ee.startPWM1ccw : ee.stopPWM1ccw, 255); break;
                case 12: return; break;
              } break;
      case 2 :switch (selection & 15) {
                case 2: cAlignMotor(2);         break;
                case 3: gMotor(PWM2, DIR2, 2);  break;
                case 4: cDirMotor(2);           break;
                case 6: ee.startPWM2cw = setPWM(3, 2, ee.startPWM2cw, 0, ee.maxPWM2cw);     break;
                case 7: ee.stopPWM2cw = setPWM(5, 2, ee.stopPWM2cw, 0, ee.maxPWM2cw);       break;
                case 8: ee.maxPWM2cw = setPWM(1, 2, ee.maxPWM2cw, (ee.startPWM2cw > ee.stopPWM2cw) ? ee.startPWM2cw : ee.stopPWM2cw, 255); break;
                case 9: ee.startPWM2ccw = setPWM(4, 2, ee.startPWM2ccw, 0, ee.maxPWM2ccw);  break;
                case 10: ee.stopPWM2ccw = setPWM(6, 2, ee.stopPWM2ccw, 0, ee.maxPWM2ccw);    break;
                case 11: ee.maxPWM2ccw = setPWM(2, 2, ee.maxPWM2ccw, (ee.startPWM2ccw > ee.stopPWM2ccw) ? ee.startPWM2ccw : ee.stopPWM2ccw, 255); break;
                case 12: return; break;
              } break;
      case 3 :switch (selection & 15) {
                case 2: cAlignMotor(3);         break;
                case 3: gMotor(PWM3, DIR3, 3);  break;
                case 4: cDirMotor(3);           break;
                case 6: ee.startPWM3cw = setPWM(3, 3, ee.startPWM3cw, 0, ee.maxPWM3cw);     break;
                case 7: ee.stopPWM3cw = setPWM(5, 3, ee.stopPWM3cw, 0, ee.maxPWM3cw);       break;
                case 8: ee.maxPWM3cw = setPWM(1, 3, ee.maxPWM3cw, (ee.startPWM3cw > ee.stopPWM3cw) ? ee.startPWM3cw : ee.stopPWM3cw, 255); break;
                case 9: ee.startPWM3ccw = setPWM(4, 3, ee.startPWM3ccw, 0, ee.maxPWM3ccw);  break;
                case 10: ee.stopPWM3ccw = setPWM(6, 3, ee.stopPWM3ccw, 0, ee.maxPWM3ccw);    break;
                case 11: ee.maxPWM3ccw = setPWM(2, 3, ee.maxPWM3ccw, (ee.startPWM3ccw > ee.stopPWM3ccw) ? ee.startPWM3ccw : ee.stopPWM3ccw, 255); break;
                case 12: return; break;
              } break;
    }
    EEPROM.put(0, ee);
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void cSpeedUpRate() {
  LCD("$$Speed up rate:");
  ee.rampSpeed = getValue(ee.rampSpeed, 100, 0, 1, 15);
  EEPROM.put(0, ee);
}
//--------------------------------------------------------------------------------------------------------------
void cSlowDownRate() {
  LCD("$$Slow down rate:");
  ee.stopSpeed = getValue(ee.stopSpeed, 100, 0, 1, 15);
  EEPROM.put(0, ee);
}
//--------------------------------------------------------------------------------------------------------------
void cStuckTime() {
  LCD("$$Enter stuck time");
  ee.stuckTime = getValue(ee.stuckTime / 1000, 30, 0, 1, 15) * 1000;
  EEPROM.put(0, ee);
}
//--------------------------------------------------------------------------------------------------------------
void cAlignMotor(byte mtr) {
  if (!safe()) return;
  LCD("$$IMPORTANT!$NRelease motor ");
  LCDint(mtr);
  sendDisp();
  if (waitOKesc(0) == btnE) return;
  if ((mtr - 1) >= 3) return;
  LCD("$$Press < or > to$Nto turn MOTOR ");
  LCDint(mtr);
  sendDisp();
  manualMove(mtr); // Turn motor until OK or ESC has been pressed
}
//--------------------------------------------------------------------------------------------------------------
void cDirMotor(byte mtr) {
  byte dirCW;                           // Ordinarily, 1 is CCW and 0 is CW
  switch (mtr) {
    case 1: dirCW = ee.dir1cw; break;
    case 2: dirCW = ee.dir2cw; break;
    case 3: dirCW = ee.dir3cw; break;
  }
  while (1) {
    LCD("$$CW dir input for$Nmotor ");
    LCDint(mtr);
    LCD(": "); LCDint(dirCW);
    LCD("    <>");
    sendDisp();
    switch(waitBtns(0) & 15) {
      case btnL:  dirCW = 0; break;
      case btnR:  dirCW = 1; break;
      case btnE:  return;    break;
      case btnO:  switch (mtr) {
                    case 1: ee.dir1cw = dirCW; break;
                    case 2: ee.dir2cw = dirCW; break;
                    case 3: ee.dir3cw = dirCW; break;
                  }
                  EEPROM.put(0, ee);
                  return;    break;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void cMotorSetupOLD() {
  char *items[] = {"Start PWMs    ",
                   "Stop PWMs     ",
                   "Max PWMs      ",
                   "Speed up rate ",
                   "Slow down rate",
                   "Stuck time    ",
                   "Dir motor 1:  ",
                   "Dir motor 2:  ",
                   "Dir motor 3:  ",
                   "Align motor 1 ",
                   "Align motor 2 ",
                   "Align motor 3 ",
                   "Exit"};
  byte selection = 1;
  do{
    embedValue(items[6], ee.rampSpeed);
    embedValue(items[7], ee.stopSpeed);
    embedValue(items[8], ee.stuckTime / 1000);
    items[6][13] =  ee.dir1cw + '0';
    items[7][13] = ee.dir2cw + '0';
    items[8][13] = ee.dir3cw + '0';
    selection = menu(items, 13, selection);
    switch(selection & 15){
      case 1: dStartPWMs();     break;
      case 2: dStopPWMs();      break;
      case 3: dMaxPWMs();       break;
      case 4: cSpeedUpRate();   break;
      case 5: cSlowDownRate();  break;
      case 6: cStuckTime();     break;
      case 7: cDirMotor(1);     break;
      case 8: cDirMotor(2);     break;
      case 9: cDirMotor(3);     break;
      case 10: cAlignMotor(1);  break;
      case 11: cAlignMotor(2);  break;
      case 12: cAlignMotor(3);  break;
      case 13: return;          break;
    }
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
#pragma endregion MOTOR SETUP MENU
//==============================================================================================================
#pragma region MOTOR PWMS MENUS
//--------------------------------------------------------------------------------------------------------------
void dMaxPWMs() {
  char *items[] = {"MAX PWMS      ",
                   "Turn Motor 1  ",
                   "Motor 1 >     ",
                   "Motor 1 <     ",
                   "Turn Motor 2  ",
                   "Motor 2 >     ",
                   "Motor 2 <     ",
                   "Turn Motor 3  ",
                   "Motor 3 >     ",
                   "Motor 3 <     ",
                   "Exit"};
  byte selection = 1;
  do {
    embedValue(items[2], ee.maxPWM1cw);
    embedValue(items[3], ee.maxPWM1ccw);
    embedValue(items[5], ee.maxPWM2cw);
    embedValue(items[6], ee.maxPWM2ccw);
    embedValue(items[8], ee.maxPWM3cw);
    embedValue(items[9], ee.maxPWM3ccw);
    selection =  menu(items, 11, selection);
    switch (selection & 15) {
      case 2: gMotor(PWM1, DIR1, 1);  break;
      case 3: ee.maxPWM1cw = setPWM(1, 1, ee.maxPWM1cw, (ee.startPWM1cw > ee.stopPWM1cw) ? ee.startPWM1cw : ee.stopPWM1cw, 255); break;
      case 4: ee.maxPWM1ccw = setPWM(2, 1, ee.maxPWM1ccw, (ee.startPWM1ccw > ee.stopPWM1ccw) ? ee.startPWM1ccw : ee.stopPWM1ccw, 255); break;
      case 5: gMotor(PWM1, DIR2, 2);  break;
      case 6: ee.maxPWM2cw = setPWM(1, 2, ee.maxPWM2cw, (ee.startPWM2cw > ee.stopPWM2cw) ? ee.startPWM2cw : ee.stopPWM2cw, 255); break;
      case 7: ee.maxPWM2ccw = setPWM(2, 2, ee.maxPWM2ccw, (ee.startPWM2ccw > ee.stopPWM2ccw) ? ee.startPWM2ccw : ee.stopPWM2ccw, 255); break;
      case 8: gMotor(PWM1, DIR3, 3);  break;
      case 9: ee.maxPWM3cw = setPWM(1, 3, ee.maxPWM3cw, (ee.startPWM3cw > ee.stopPWM3cw) ? ee.startPWM3cw : ee.stopPWM3cw, 255); break;
      case 10: ee.maxPWM3ccw = setPWM(2, 3, ee.maxPWM3ccw, (ee.startPWM3ccw > ee.stopPWM3ccw) ? ee.startPWM3ccw : ee.stopPWM3ccw, 255); break;
      case 11: return; break;
    }
    EEPROM.put(0,ee);
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void dStartPWMs() {
  char *items[] = {"START PWMS    ",
                   "Turn Motor 1  ",
                   "Motor 1 >     ",
                   "Motor 1 <     ",
                   "Turn Motor 2  ",
                   "Motor 2 >     ",
                   "Motor 2 <     ",
                   "Turn Motor 3  ",
                   "Motor 3 >     ",
                   "Motor 3 <     ",
                   "Exit"};
  byte selection = 1;
  do {
    embedValue(items[2], ee.startPWM1cw);
    embedValue(items[3], ee.startPWM1ccw);
    embedValue(items[5], ee.startPWM2cw);
    embedValue(items[6], ee.startPWM2ccw);
    embedValue(items[8], ee.startPWM3cw);
    embedValue(items[9], ee.startPWM3ccw);
    selection =  menu(items, 11, selection);
    switch (selection & 15) {
      case 2: gMotor(PWM1, DIR1, 1);  break;
      case 3: ee.startPWM1cw = setPWM(3, 1, ee.startPWM1cw, 0, ee.maxPWM1cw); break;
      case 4: ee.startPWM1ccw = setPWM(4, 1, ee.startPWM1ccw, 0, ee.maxPWM1ccw); break;
      case 5: gMotor(PWM2, DIR2, 2);  break;
      case 6: ee.startPWM2cw = setPWM(3, 2, ee.startPWM2cw, 0, ee.maxPWM2cw); break;
      case 7: ee.startPWM2ccw = setPWM(4, 2, ee.startPWM2ccw, 0, ee.maxPWM2ccw); break;
      case 8: gMotor(PWM3, DIR3, 3);  break;
      case 9: ee.startPWM3cw = setPWM(3, 3, ee.startPWM3cw, 0, ee.maxPWM3cw); break;
      case 10: ee.startPWM3ccw = setPWM(4, 3, ee.startPWM3ccw, 0, ee.maxPWM3ccw); break;
      case 11: return; break;
    }
    EEPROM.put(0,ee);
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void dStopPWMs() {
  char *items[] = {"STOP PWMS     ",
                   "Turn Motor 1  ",
                   "Motor 1 >     ",
                   "Motor 1 <     ",
                   "Turn Motor 2  ",
                   "Motor 2 >     ",
                   "Motor 2 <     ",
                   "Turn Motor 3  ",
                   "Motor 3 >     ",
                   "Motor 3 <     ",
                   "Exit"};
  byte selection = 1;
  do {
    embedValue(items[2], ee.stopPWM1cw);
    embedValue(items[3], ee.stopPWM1ccw);
    embedValue(items[5], ee.stopPWM2cw);
    embedValue(items[6], ee.stopPWM2ccw);
    embedValue(items[8], ee.stopPWM3cw);
    embedValue(items[9], ee.stopPWM3ccw);
    selection =  menu(items, 11, selection);
    switch (selection & 15) {
      case 2: gMotor(PWM1, DIR1, 1);  break;
      case 3: ee.stopPWM1cw = setPWM(5, 1, ee.stopPWM1cw, 0, ee.maxPWM1cw); break;
      case 4: ee.stopPWM1ccw = setPWM(6, 1, ee.stopPWM1ccw, 0, ee.maxPWM1ccw); break;
      case 5: gMotor(PWM1, DIR2, 2);  break;
      case 6: ee.stopPWM2cw = setPWM(5, 2, ee.stopPWM2cw, 0, ee.maxPWM2cw); break;
      case 7: ee.stopPWM2ccw = setPWM(6, 2, ee.stopPWM2ccw, 0, ee.maxPWM2ccw); break;
      case 8: gMotor(PWM1, DIR3, 2);  break;
      case 9: ee.stopPWM3cw = setPWM(5, 3, ee.stopPWM3cw, 0, ee.maxPWM3cw); break;
      case 10: ee.stopPWM3ccw = setPWM(6, 3, ee.stopPWM3ccw, 0, ee.maxPWM3ccw); break;
      case 11: return; break;
    }
    EEPROM.put(0,ee);
  } while (selection & 15);
}
//--------------------------------------------------------------------------------------------------------------
void insertNo(byte i, char *item[], int v) {
  *(item[i] + 10) = char(int(v / 100) + 48);
  v = v - 100 * int(v / 100);
  *(item[i] + 11) = char(int(v / 10) + 48);
  v = v - 10 * int(v / 10);
  *(item[i] + 12) = char(int(v) + 48);
}
//--------------------------------------------------------------------------------------------------------------
byte setPWM(byte msg, byte motor, byte value, byte lower, byte upper) {
  switch (msg) {
    case 1: LCD("$$Enter max PWM  >$Nfor motor "); break;
    case 2: LCD("$$Enter max PWM  <$Nfor motor "); break;
    case 3: LCD("$$Enter start PWM>$Nfor motor "); break;
    case 4: LCD("$$Enter start PWM<$Nfor motor "); break;
    case 5: LCD("$$Enter stop PWM >$Nfor motor "); break;
    case 6: LCD("$$Enter stop PWM <$Nfor motor "); break;
  }
  LCDint(motor);
  LCD(":");
	return getValue(value, upper, lower, 1, 15);
}
#pragma endregion MOTOR PWMS MENU
//==============================================================================================================
#pragma region BUZZER MENU
//--------------------------------------------------------------------------------------------------------------
void eBuzzer() {
  char *items[] = {"All sounds     ",
                   "Buttons        ",
                   "Dome moves     ",
                   "Alerts         ",
                   "Comms warning  ",
                   "Exit           "};
  byte selection = 1;
  do{
 		*(items[0] + 13) = tick(ee.bzAll);
		*(items[1] + 13) = tick(ee.bzBtns);
		*(items[2] + 13) = tick(ee.bzMoves);
		*(items[3] + 13) = tick(ee.bzAlert);
		*(items[4] + 13) = tick(ee.bzComms);
    selection =  menu(items, 6, selection);
    switch(selection & 15){
		  case 1:	ee.bzAll   = 1^ee.bzAll   ;		break;
  		case 2:	ee.bzBtns  = 1^ee.bzBtns  ;		break;
      case 3: ee.bzMoves = 1^ee.bzMoves ;   break;
	 		case 4:	ee.bzAlert= 1^ee.bzAlert;		  break;
	  	case 5:	ee.bzComms = 1^ee.bzComms ;		break;
      case 6: return;                       break;
	  }
  } while (selection & 15);
  EEPROM.put(0, ee);
}
//--------------------------------------------------------------------------------------------------------------
char tick(bool t) {
	return t ? '\x01' : '\x1F';
}
#pragma endregion BUZZER MENU
//==============================================================================================================
#pragma region CALIBRATION MENU
//--------------------------------------------------------------------------------------------------------------
void fCalibrateDome() {
  char *items[] = {"Manual",
                   "Set resolution",
                   "Auto",
                   "Exit"};
  byte selection = 1;
  do{
    calibrating = 0;
    selection =  menu(items, 4, selection);
    switch(selection & 15){
      case 1: fManualCalib();     break;
      case 2: fSetResolution();   break;
      case 3: fAutoCalib();       break;
      case 4: return;             break;
    }
    if (calibrating) {
      calibrating = 0;                            // Finish off the calibration work
      enc180 = ee.enc360 >> 1;                    // Reference points for 180 degrees
      encOne = ee.enc360 >> 8;                    // Reference points for 1 degree
      ee.homeAZM = 0;                             // Home position is lost so set it to 0
      currEnc = 0;
      domeAZM = 0;
      normEnc();
      EEPROM.put(0, ee);                          // Save the new value of the dome resolution
      currEnc = enc180;                           // Set currEnc to mid point
      updateEEPROM(currEnc);
      LCD("$$Calibration done$NResolution:");
      LCDint(ee.enc360);
      sendDisp();
      while(scanBtns());
      waitOKesc(2000);
      LCD("$$Dome now needs$Nto be resynched.");
      sendDisp();
      waitOKesc(2000);
    }
  } while (selection & 15);  
}
//--------------------------------------------------------------------------------------------------------------
void fManualCalib() {
  if (!safe()) return;
  LCD("$$CALIBRATING DOME$NAre you sure?"); sendDisp();
  if (waitOKesc(0) & btnE) return;
  LCD("$$CALIBRATION MODE$NACTIVATED"); sendDisp();
  for (byte i = 1; i <= 10; i++) {                          // Give 10 quick beeps
    delay(50); beep(50, ee.bzAlert);
  }
  long enc360old = ee.enc360;
  getEnc();
  prevEnc = currEnc;
  do {
    calibrating = 1;
    currEnc = 0;
    if (manualMove(MTR_ALL) == btnE) break;
    ee.enc360 = abs(currEnc);                               // Calculate the new resolution
    if (ee.enc360 > MIN_RES && prevEnc != currEnc) return;  // A reasonable resoluton has been got
    ee.enc360 = enc360old;                                  // Restore enc360 with original value
    LCD("$$Bad resolution.$NRetry?    OK/Esc"); sendDisp();
  } while(!(waitOKesc(0) & btnE));                          // Loop if Esc button not pressed

  calibrating = 0;                                          // Calibration has been aborted
  getEnc();
  currEnc = prevEnc + currEnc;                              // Restore currEnc
  updateEEPROM(normEnc());
  ee.enc360 = enc360old;                                    // Restore original resolution
  LCD("$$CALIBRATION$NABORTED.      OK"); sendDisp();
  beep(1000, ee.bzAlert);
  waitOKesc(0);
  return;
}
//--------------------------------------------------------------------------------------------------------------
void fSetResolution() {
  LCD("$$Enter resolution");
  calibrating = 1;
  long value = getValue(ee.enc360, 99999L, MIN_RES, 1, 15); 
  if (value == ee.enc360) return;
  ee.enc360 = value;
}
//--------------------------------------------------------------------------------------------------------------
void fAutoCalib() {
  if (!ee.indexSw) {
    LCD("$$Index switch is$Nnot active.   OK");
    sendDisp();
    waitOKesc(0);
    return;
  }
  if (!safe()) return;
  LCD("$$Auto calibration$NPress OK or Esc");
  sendDisp();
  if (waitOKesc(0) & 15 == BTN_E) return;

  long prevEnc360 = ee.enc360;
  indexEnc[0] = indexEnc[1] = currEnc = 0;
  ee.enc360 = 99999;
  enc180 = ee.enc360 >> 1;                    // Reference points for 180 degrees
  encOne = ee.enc360 >> 8;                    // Reference points for 1 degree
  indexPtr = domeAZM = 0;
  calibrating = 1;
  attachInterrupt(digitalPinToInterrupt(INDEX_SW), indexISR, FALLING);
  moveDome(255, 0);
  if (indexPtr > 1) ee.enc360 = abs(indexEnc[1] - indexEnc[0]);
  else ee.enc360 = prevEnc360;                // Calibration aborted - nothing's changed
  detachInterrupt(digitalPinToInterrupt(INDEX_SW));
}
#pragma endregion CALIBRATION MENU
//==============================================================================================================
#pragma region CHECK HARDWARE
//--------------------------------------------------------------------------------------------------------------
void gCheckHardware() {
  char *items[] = {"Dongle test",
                   "Encoder test",
                   "Motor 1 test",
                   "Motor 2 test",
                   "Motor 3 test",
                   "All motors",
                   "Index switch",
                   "Buttons test",
                   "Exit"};
  byte selection = 1;
  do{
    selection =  menu(items, 9, selection);
    switch(selection & 15){
      case 1: gDongle();              break;
      case 2: gEncoder();             break;
      case 3: gMotor(PWM1, DIR1, 1);  break;
      case 4: gMotor(PWM2, DIR2, 2);  break;
      case 5: gMotor(PWM3, DIR3, 3);  break;
      case 6: gMotor(0, 0, 0);        break;
      case 7: gIndexSwitch();         break;
      case 8: gButtons();             break;
      case 9: return;                 break;
    }
  } while (selection & 15);  
}
//--------------------------------------------------------------------------------------------------------------
void gDongle() {
  byte  csr = 0,
        flag = 2;
  char c = 0;
  while (Serial2.available()) Serial2.read();           // Clear Bluetooth serial buffer (DongleBuddy)
  LCD("$$Connecting...");
  do {
    if (btCxn()) {
      LCD("$HReceiving:     ^$N");
      while (Serial2.available()) {
        for (byte i = 0; i <= 14; i++) disp[1][i] = disp[1][i + 1];
        disp[1][15] = Serial2.read();
        LCD("$N");
        LCD(disp[1]);
        delay(100);
        flag = 1;
      }
      if (flag == 1) {
        sendDisp();
        flag = 2;
      }
    }
    else if (flag == 2) {
      LCD("$HDongle not found");
      flag = 0;
      sendDisp();
    }
    scanBtns();
  } while (!(btnsLast & btnO) && !(btnsLast & btnE));
}
//--------------------------------------------------------------------------------------------------------------
void gEncoder() {
  long oldEnc = -1;
  LCD("$$");
  LCD(  "Encoder:");
  LCD("$NMax value:");
  LCDintAt(1, 11, ee.enc360 - 1, 5);
  do {
    LCDat(0, 8);
    PIND & B10 ? LCD("A") : LCD("a");
    PIND & B01 ? LCD("B") : LCD("b");
    if (oldEnc != normEnc()) {
      oldEnc = normEnc();
      LCDintAt(0, 11, oldEnc, 5);
      sendDisp();
    }
    scanBtns();
  } while (!(btnsLast & btnO) && !(btnsLast & btnE));
}
//--------------------------------------------------------------------------------------------------------------
void gMotor(int pwmPin, int dirPin, byte mtr) {
  if (!safe()) return;
  int   pwm = 0,
        oldPwm = 1;                               // 1 force the display to be sent on first pass
  bool  flag = 0;
  long  oldEnc = 0;
  LCD("$$IMPORTANT!");
  if(mtr) {
    LCD("$NRelease motor ");
    LCDint(mtr);
  }
  else LCD("$NRelease motors");
  sendDisp();
  beep(500, ee.bzAlert);
  if (waitOKesc(0) & btnE) return;
  LCD("$$Encoder:");
  LCD("$NDir: -  PWM:");
  LCDintAt(0, 11, normEnc(), 5);
  LCDintAt(1, 13, pwm, 3);
  sendDisp();
  do {
    setDirection(pwm < 0);
    pwm ? startEnc() : stopEnc();
    if (mtr) {
      analogWrite(pwmPin, abs(pwm));
    }
    else {
      analogWrite(PWM1, abs(pwm));
      analogWrite(PWM2, abs(pwm));
      analogWrite(PWM3, abs(pwm));
    }
    flag = 0;
    if (oldPwm != pwm || oldEnc != normEnc()) {
      oldPwm = pwm; oldEnc = normEnc();
      LCDintAt(0, 11, oldEnc, 5);
      LCDintAt(1, 13, abs(oldPwm), 3);
      LCDat(1, 5);
      if (!pwm) LCD(".");
      else (pwm > 0) ? LCD("<") : LCD(">");
      flag = 1;
    }
    if (flag) sendDisp();                       // Only send display to PC if finished changing
    btnsLast = pressedBtns();
    switch (btnsLast & B1111) {
      case btnL:
        if (pwm < 255) pwm++;
        delay(5);
        break;
      case btnR:
        if (pwm > -255) pwm--;
        delay(5);
        break;
      case btnO:
        pwm = abs(pwm);
        while (pwm) {
          pwm--;
          if (mtr) analogWrite(pwmPin, pwm);
          else {
            analogWrite(PWM1, pwm);
            analogWrite(PWM2, pwm);
            analogWrite(PWM3, pwm);
          }
          LCDintAt(0, 11, normEnc(), 5);
          LCDintAt(1, 13, abs(pwm), 3);
          delay(ee.stopSpeed);
        }
        break;
    }
  } while (!(btnsLast & btnE));
  pwm = abs(pwm);
  while (pwm) {
    pwm--;
    if (mtr) analogWrite(pwmPin, pwm);
    else {
      analogWrite(PWM1, pwm);
      analogWrite(PWM2, pwm);
      analogWrite(PWM3, pwm);
    }
    LCDintAt(0, 11, normEnc(), 5);
    LCDintAt(1, 13, abs(pwm), 3);
    delay(ee.stopSpeed);
  }
  beep(20, ee.bzAlert);
}
//--------------------------------------------------------------------------------------------------------------
void gIndexSwitch() {
  int lastIndexSw = 2;                    // '2' forces display to be sent to PC on first pass
  LCD("$$Index swtch test");
  LCD("$NStatus: ");
  do {
    if (lastIndexSw != digitalRead(INDEX_SW)) {
      lastIndexSw = digitalRead(INDEX_SW);
      LCDintAt(1, 8, digitalRead(INDEX_SW), 0);
      lastIndexSw ? LCD(" OPEN  ") : LCD(" CLOSED ");
      sendDisp();
    }
    scanBtns();
  } while (!(btnsLast & btnO) && !(btnsLast & btnE));
}
//--------------------------------------------------------------------------------------------------------------
void gButtons() {
  bool flag = 1;
  LCD("$$  Test buttons");
  LCD("$N   . .. ... .");
  sendDisp();
  while(1) {
    scanBtns();
    LCDat(1, 6);
    if (digitalRead(BTN_L)) {LCD("$N   < .. ... ."); flag = 1;}
    if (!digitalRead(BTN_O)) {LCD("$N   . OK ... ."); flag = 1;}
    if (!digitalRead(BTN_E)) {LCD("$N   . .. Esc ."); flag = 1;}
    if (digitalRead(BTN_R)) {LCD("$N   . .. ... >"); flag = 1;}
    if (!digitalRead(BTN_O) && !digitalRead(BTN_E)) return;
    if (flag) {sendDisp(); flag = 0;}
  }
}
#pragma endregion CHECK HARDWARE
//==============================================================================================================
#pragma region MENU HANDLING
//--------------------------------------------------------------------------------------------------------------
byte menu(char *item[], byte n, byte focus) {
//*item[] = list of menu items, n = no. of items,
//focus => bits 3-0: item no. in focus (to be arrowed) bits 7-4: item to show on top of LCD display
  byte  topLine = focus / 16,
        flag = 0;
  --focus &= 15;
  do{
    LCD("$$$b$c");                                                                    // Clear screen, no blink, no cursor
    for(byte i = 0; i < 2; i++){
      LCDat(i, 0);
      if ((i + topLine) == focus) LCD(">");
      else LCD(" ");
      LCD(*(item + i + topLine));
    };
    do {   
      LCDat(1, 15); btCxn() ? LCD("^") : LCD(" ");                                    // Wait for a button press
      sendDisp();
    } while (!scanBtns());
    beepBtns();
    switch (btnsLast){
      case btnL:      if(focus > 0) if(--focus<topLine) topLine = focus;              // Move up menu
                       break;
      case btnR:      if(focus < n-1) if(++focus > topLine + 1) topLine = focus - 1;  // Move down menu
                       break;
      case btnO:      return ++focus + topLine * 16;                                  // Return item no. selected
                      break;
      case btnO|btnH: return ++focus + topLine * 16;                                  // Return item no. selected
                      break;
    }
  } while(!(btnsLast & btnE));                                                    // Leave once OK or Esc pressed
  return 0;                                                                       // 0 means Esc has been pressed
}
//--------------------------------------------------------------------------------------------------------------
long getValue(long oldVal, long u, long l, byte r, byte c) {          // Value, upper limit, lower limit, row, column of last digit
  char buf[2][17];
	char val[6] = "00000";
	byte d = (u > 9) + (u > 99) + (u > 999) + (u > 9999) + (u > 99999);
	val[d + 1] = 0;							                                        // Reduce 'val; string length to the number of decimal digits of the upper limit
 	long v = oldVal;
	byte csr = 0;
	for(byte i = 0; i <= d; i++){			                                  // Convert Decimal > string of characters
		val[d - i] = v % 10 + '0';
		v /= 10;
	}
	
	LCD("$B");								                                          // Blinking cursor!
	while(true){
		if(btnsLast){
			LCDat(r, c-d);
			LCD(val);
			LCDat(r, c-d+csr);
			LCD("$C");
		}
    sendDisp();
    while(!scanBtns());
		switch (btnsLast){
			case btnE:		val[csr] -= (val[csr] >'0')-9*(val[csr] =='0');		// Increment digit
							sendDisp(); break;
			case btnO:		val[csr] += (val[csr] <'9')-9*(val[csr] =='9');		// Decrement digit
							sendDisp(); break;
			case btnR:		csr += (csr < d);                                 // Move cursor right
							sendDisp(); break;
			case btnL:		csr -= (csr > 0);                                 // Move cursor left
							sendDisp(); break;
			case btnE|btnH: return oldVal;										              // Escape and keep original value
			case btnO|btnH: v = 0;												                  // Try and accept value
							for(byte i = 0; i <=d; i++) v = v*10 +(val[i] - '0');
							if(v <= u && v >= l){
								LCD("$$Value has been$Nset to: ");
								LCDint(v);
                sendDisp();
								waitBtns(2000);
								return v;										                          // Return value if within limit
							}
              strcpy(buf[0], disp[0]);
              strcpy(buf[1], disp[1]);
              LCD("$$$cOut of range of$N");
              LCDint(l); LCD(" to "); LCDint(u);
              sendDisp();
							beep(200, ee.bzAlert);
							waitBtns(1000);
              LCD("$$");
              LCD(buf[0]);
              LCD("$N");
              LCD(buf[1]);
         			LCDat(r, c-d+csr);
              LCD("$C$B");
              sendDisp();
							break;
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
void embedValue(char line[], long v) {
  char buf[10];
  byte ptr = 0;
  for (byte i = 13; i >= 0; i--) {
    if(isdigit(*(line + i))) {
      *(line + i) = ' ';
    }
    else break;
  }
  //Serial.println("embedValue()");
  //Serial.println(line);
  if (v < 0) {
    v = - v;
    buf[ptr++] = '-';
  }
  byte len = byte(log10(v));
  long div = 1;
  for (byte i = 1; i <= len; i++) div *= 10;
  for (byte i = 0; i <= len; i++) {
    buf[ptr++] = long(v / div) + '0';
    v -= int(v / div) * div;
    div /= 10;
  }
  for (byte i = 0; i < ptr; i++) {
    *(line + 14 - ptr + i) = buf[i];
  }
  //Serial.println(line);
}
#pragma endregion MENU HANDLING
//==============================================================================================================
#pragma region BUTTONS
byte scanBtns() {
  btnsLast = 0;
  byte btnsPressed = pressedBtns();
  switch(scanStep){
    case 0:
      if(btnsPressed){
        btnsDeb = btnsPressed;
        timerBtns = millis() + PERIOD_DEB;      //Eliminate Debounce
        scanStep = 1;
      }
      break;
    case 1:
      if(btnsPressed == btnsDeb){               //Check for no Debounce
        if(millis() > timerBtns){
          btnsCurr = btnsDeb;
          timerBtns = millis() + PERIOD_HOLD;
          scanStep = 2;                         // Now see if held
        }
      }
      else scanStep = 0;
      break;
    case 2:
      if(btnsPressed == btnsCurr){              // Check for Hold
        if(millis() > timerBtns){
          btnsCurr = btnsCurr|btnH;             // Buttons have been held long enough
          beep(100, ee.bzBtns);
          scanStep = 3;                         // Wait for release debounce
        }
      }
      else{                                     // Pressed buttons have changed
        if(btnsPressed) scanStep = 0;           // Any buttons pressed? Yes - start over
        else{                                   // Buttons are no longer pressed
          timerBtns = 0;
          scanStep = 3;                         // Wait for release debounce
        }
      }
      break;
    case 3:
      if(millis() > timerBtns){                 // Wait for release debounce
        if(btnsPressed) timerBtns = millis() + PERIOD_DEB;
        else{
          btnsLast = btnsCurr;
          scanStep = btnsCurr = 0;
          beepBtns();
        }
      }
      break;
    case 4:
      scanStep = 0;
      beepBtns();
  }
  return btnsLast;
}
//--------------------------------------------------------------------------------------------------------------
byte pressedBtns() {
  btnsPressed = 0;
  if (!scanStep) if (Serial.available()) {     // Handle PC "key" presses
    getScopeAZM();
    btnsPressed = btnsLast;
  }
  if (scanStep < 4) {
    if (testMode) {
      btnsPressed  = !digitalRead(BTN_O) * btnO; // Push-to-make button
      btnsPressed += !digitalRead(BTN_E) * btnE; // Push-to-make button
      btnsPressed += !digitalRead(BTN_R) * btnR; // Push-to-break button!
      btnsPressed += !digitalRead(BTN_L) * btnL; // Push-to-break button!
    } else {
      btnsPressed  = !digitalRead(BTN_O) * btnO; // Push-to-make button
      btnsPressed += !digitalRead(BTN_E) * btnE; // Push-to-make button
      btnsPressed +=  digitalRead(BTN_R) * btnR; // Push-to-break button!
      btnsPressed +=  digitalRead(BTN_L) * btnL; // Push-to-break button!
    }
  }
  if (btnsPressed == B1111) {
    LCD("$l");
    beep(1000, 1);
    asm volatile ("jmp 0");
    return;
  }

  if (btnsPressed) {
    LCD("$L");
    if (ee.lcdTimeout > 0) lcdTimer = millis() + ee.lcdTimeout; // A button has been pressed - reset display timeout
  }
  else if (millis() > lcdTimer && ee.lcdTimeout) LCD("$l");
  return btnsPressed; 
}
//--------------------------------------------------------------------------------------------------------------
byte waitBtns(unsigned long period) {
	if(!period) period = 0xFFFFFFFF;  // Wait for period in milliseconds, but indefinitely if period is 0 (or at most 49 days!).
  else period+= millis();
	while((millis() < period) && !scanBtns());
	return btnsLast;
}
//--------------------------------------------------------------------------------------------------------------
byte waitOKesc(unsigned long period){
	while((waitBtns(period) != btnO) && (btnsLast != btnE));
	if((btnsLast == btnO) | (btnsLast == btnE)) return btnsLast;
	return 0;
}
//--------------------------------------------------------------------------------------------------------------
void beepBtns(){
  beep(3, ee.bzBtns);
}
//--------------------------------------------------------------------------------------------------------------
void beep(int t, bool noise){
  if (ee.bzAll && noise) digitalWrite(BUZZER, HIGH);
  delay(t);
  digitalWrite(BUZZER, LOW);
}
#pragma endregion BUTTONS
//==============================================================================================================
#pragma region LCD STUFF
void LCD(char *p){
// Motor back EMF can crash the LCD display.  This function keeps a record of what has been printed to it and the
// cursor status so that the display can be restored.
  char c, e;
  for(byte i = 0; *(p + i) > 0; i++){
    c = *(p + i);
    if(c == '$'){                                                             // Handle LCD escape codes
      switch(*(p + ++i)) {
        case 'B': dispStatus |=  BLINK;  lcd.blink(); break;                  // Switch on blinking cursor
        case 'b': dispStatus &= !BLINK;  lcd.noBlink(); break;                // Switch off blinking cursor
        case 'C': dispStatus |=  CURSOR; lcd.cursor();  break;                // Switch on underscore cursor
        case 'c': dispStatus &= !CURSOR; lcd.noCursor(); break;               // Switch off underscore cursor
        case 'O': dispStatus |=  DISP;   lcd.display();  break;               // Switch on display
        case 'o': dispStatus &= !DISP;   lcd.noDisplay(); break;              // Switch off display
        case 'L': dispStatus |=  LIGHT;  digitalWrite(LCD_BL_A, HIGH); break; // Switch on backlight
        case 'l': dispStatus &= !LIGHT;  digitalWrite(LCD_BL_A, LOW); break;  // Switch off backlight
        case 'H': dispC = dispR = 0; lcd.home(); break;                       // Place cursor in top left corner
        case 'N': dispR = 1; dispC = 0; lcd.setCursor(dispC, dispR); break;   // Next line (CR & LF!)
        case 'F': for(byte j = dispC; j<16; j++){                             // Fill row with spaces to its end
                    lcd.print(disp[dispR][dispC++] = ' ');
                  };
                  break;
        case '$': strcpy(disp[0], "                ");                        // Clear screen
                  strcpy(disp[1], disp[0]);
                  dispC = dispR = 0;
                  dispStatus &= B0100;
                  lcd.clear();
                  lcd.noBlink();
                  lcd.noCursor();
                  break;
      }
    }
    else if((dispC < 16) & (dispC >=0)){
      switch (c) {
        case '}':  e = (*(p + ++i) - '0') * 16;
                   e += *(p + ++i) - '0';
                   c = char(e);
                  break;
        case '>': c = '\x7E';   break;  // Right arrow
        case '<': c = '\x7F';   break;  // Left arrow
        case '`': c = '\xDF';   break;  // Degree symbol
        case '^': c = '\x02';   break;  // Bluetooth Symbol
        case '\x1F': c = '\xEB'; break;
        default: break;
      }
      dispStatus &= B1111;
      disp[dispR][dispC++]= c;        //Place the next character into the display buffer
      lcd.write(byte(c));             //and then print it.
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void LCDat(byte r, byte c){           //Postion cursor correctly on the display buffer
  dispC = c & 15;
  dispR = r & 1;
  lcd.setCursor(dispC, dispR);
}
//--------------------------------------------------------------------------------------------------------------
void LCDint(long v){                   //Write a number to the display buffer
  char buf1[10], buf2[10];
  if (v < 0) {LCD("-"); v = -v;}
  if (v > 9999) {
    sprintf(buf1, "%04d", v % 10000);
    sprintf(buf2, "%d", long(v / 10000));
    LCD(buf2);
    LCD(buf1);
  }
  else {
    sprintf(buf1, "%d", v);
    LCD(buf1);
  }
  return;

  sprintf(buf, "%d", long(v));
  LCD(buf);
}
//--------------------------------------------------------------------------------------------------------------
void LCDintAt(byte r, byte c, long v, byte d) {
  long value = v;
  byte len = 1;
  if (v < 0) {
    len++;
    value = -v;
  }
  if (v) len += log10(value);
  LCDat(r, c);
  for (byte i = 0; i <= d - len - 1; i++) LCD(" ");
  LCDint(v);
}
//--------------------------------------------------------------------------------------------------------------
void sendDisp() {
  if (strcmp(dispBuf[0], disp[0]) != 0 || strcmp(dispBuf[1], disp[1]) != 0 || dispStatusBuf != dispStatus) {
    strcpy(dispBuf[0], disp[0]);
    strcpy(dispBuf[1], disp[1]);
    dispStatusBuf = dispStatus;
  } else return;                                // Return if LCD has not been updated

  char v;
  char buf[] = "\n+------------------+";
  Serial.print(buf);
  for (byte r = 0; r <= 1; r++) {
    Serial.print("\n| ");
    for (byte c = 0; c <= 15; c++) {
      v = disp[r][c];
      switch (v) {
        case '\x7E': v = '>';     break;
        case '\x7F': v = '<';     break;
        case '\xDF': v = '*';     break;
        case '\xEB': v = 'x';     break;
        case '\xFF': v = '\x01';  break;
        case '\x01': v = '/';     break;
        case '\x02': v = 'B';     break;
        case '\x03': v = '^';     break;
        case '\x04': v = 'v';     break;
        case '\x05': v = '-';     break;
      }
      Serial.write(v);
    }
    Serial.print(" |");
  }
  if (dispStatus & CURSOR) buf[dispC + 3] = '^';
  Serial.println(buf);
}
//--------------------------------------------------------------------------------------------------------------
void LCDrestore(){                    //Restore the LCD with the contents of the display buffer
  lcd.begin(16, 2);
  lcdUDGs();
  lcd.clear();
  lcd.print(disp[0]);
  lcd.setCursor(0, 1);
  lcd.print(disp[1]);
  lcd.setCursor(dispC, dispR);
  if(dispStatus & B0001) lcd.blink();
  if(dispStatus & B0010) lcd.cursor();
  if((!dispStatus) & B0100) lcd.noDisplay();
  digitalWrite(LCD_BL_A, dispStatus > B0111);   // Switch on (or off) LCD backlight
}
//--------------------------------------------------------------------------------------------------------------
void lcdUDGs(){
  byte data1[8] = {   // Tick
    B00000,
    B00001,
    B00010,
    B10100,
    B01000,
    B00000,
    B00000,
    B00000};
  byte data2[8] = {   // Bluetooth symbol
    B00100,
    B10110,
    B01101,
    B00110,
    B01101,
    B10110,
    B00100,
    B00000};
  byte data3[8] = {   // Up arrow
    B00100,
    B01110,
    B10101,
    B00100,
    B00100,
    B00100,
    B00100,
    B00000};
  byte data4[8] = {   // Down arrow
    B00100,
    B00100,
    B00100,
    B00100,
    B10101,
    B01110,
    B00100,
    B00000};
  byte data5[8] = {   // Top line
    B11111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};
  lcd.createChar(1,data1);  // Tick
  lcd.createChar(2,data2);  // Bluetooth symbol
  lcd.createChar(3,data3);  // Up arrow
  lcd.createChar(4,data4);  // Down arrow
  lcd.createChar(5,data5);  // Top line
}
#pragma endregion CONSOLE
//==============================================================================================================
#pragma region DOME MOTION
//--------------------------------------------------------------------------------------------------------------
byte moveDome(byte azm, byte tol) {             // Move dome to target azimuth (tol = tolerence)
  if (!safe()) return 0;
  if (domeAZM == azm || lastTargAZM == azm)  return 0; // Return if target is same as current dome azimuth
  targAZM = azm;
  // INITIALISE ------------------------------------------------------------------------------------------------
  targEnc = azm2enc(targAZM);                   // Convert target (0 - 255) into encoder reference point
  getEnc();
  distEnc = targEnc - currEnc;                  // Get distance to travel

  if (abs(distEnc) > enc180) {                  // Is the distance more than 180 degs?
    if (currEnc < enc180) currEnc += ee.enc360; // If currEnc is less than 180 degs then 360 degs needs to be added
    else targEnc += ee.enc360;                  // Otherwise 360 degs needs to be added to the targEnc
    distEnc = targEnc - currEnc;                // Recalculate distance (now <= 180 degs)
  }
  dir = (distEnc >= 0);                         // Get direction (TRUE = right (CW), FALSE = left (CCW) )
  prevEnc = currEnc;

//  if (tol) if (dir == lastDir) if (targAZM == lastTargAZM) return 0;  // Return if in tolerance of last targAZM 

  midPoint = targEnc - distEnc / 2;             // Calc mid point reference point between start and end positions
  for (int i = 0; i <= 255; i++) brakEnc[i] = 0;
  sendDisp();
  for (byte i; i <= 3; i++) {                   // Give audible warning - move about to take place
    beep(50, ee.bzMoves);
    delay(50);
  }
  unsigned long moveTimer = millis() + ee.stuckTime;
  lastEnc = prevEnc;
  pwm = 1;
  setDirection(dir);
  sendInfo("MOVE DOME - INITIALISE");

  LCD("$$T:    0 Tg:  0 _$NE:    0 Pt:  0 .");              // T = targEnc, TZ targAZM / E = normEnc(), AZ = domeAZM 
  LCDintAt(0, 2, norm(targEnc), 5);                         // Display Target Enc
  LCDintAt(0, 11, targAZM, 3); LCD(" }03");                 // Display targAZm + Ramp up arrow 
  LCDintAt(1, 2, norm(prevEnc) , 5);                        // Display currEnc
  LCDintAt(1, 11, domeAZM, 3); dir ? LCD(" >"): LCD(" <");  // Display domeAZM + Direction arrow
  sendDisp();

  // RAMP-UP ---------------------------------------------------------------------------------------------------
  do {                                          // Ramp-up the motors
    delay(ee.rampSpeed);                        // Delay creates a rate of ramping-up
    if (indexPtr > 2) {slaved = 0; break;}      // Auto calibration is complete
    if (pressedBtns()) {
      Serial.println("KEY PRESSED - MOVE ABORTED");
      slaved = 2; break;                        // Stop moving and slaving immediately if a button is pressed
    }
    startEnc();
    tempEnc = currEnc;                          // Grab currEnc
    if (dir)  if (tempEnc > midPoint) break;    // Mid point check for RIGHT (CW) - break out if reached
    if (!dir) if (tempEnc < midPoint )break;    // Mid point check for LEFT (CCW) - break out if reached
    stepDist = tempEnc - prevEnc;               // See how far the dome has moved
    prevEnc = tempEnc;                          // Remember current encoder value for next time

    if (lastEnc != prevEnc) {                   // See if the dome is stuck
      moveTimer = millis() + ee.stuckTime;      // Reset timer if not
      lastEnc = prevEnc;                        // Remember for next time
    }
    if((millis() > moveTimer)) {slaved = 0; break;}           // Dome is stuck - stop moving & slaving immediately
    targEnc -= brakEnc[pwm] = int(stepDist);                  // Update tragEnc - point for when to slow down
    analogWrite(PWM1, map(pwm, 0, 255, startPWM1, maxPWM1));  // Increase the motors' speed
    analogWrite(PWM2, map(pwm, 0, 255, startPWM2, maxPWM2));  // The 'map' function helps to account for diffs
    analogWrite(PWM3, map(pwm, 0, 255, startPWM3, maxPWM3));  // in speeds of the motors
    LCDintAt(1, 2, norm(currEnc), 5);                         // Display currEnc
    LCDintAt(1, 11, enc2azm(currEnc), 3);                     // Display domeAZM
  } while (++pwm);                                            // Loop if maximum speed not reached

  // SLEW ------------------------------------------------------------------------------------------------------
  if (!dir) targEnc += (encOne / 2);
  sendInfo("MOVE DOME - SLEWING");
  LCDat(0, 15); LCD("}05");                                               // Top line character - top speed
  sendDisp();
  //  if (abs(distEnc) > encOne * 2) targEnc += (dir ? encOne : -encOne) / 2; // (CW : CCW) - adj for moves > 2 ref points
  prevEnc = targEnc;

  // SLOW DOWN -------------------------------------------------------------------------------------------------
  getEnc();
  lastEnc = currEnc;
  moveTimer = millis() + ee.stuckTime;
  byte firstPass = 1;
  while (--pwm) {                                                     // Slow down the motors
    if (indexPtr > 2) slaved = 0;
    if (slaved != 1) {
      delay(slaved == 2 ?  1 :ee.stopSpeed);                          // If slew aborted (slaved = 0) then just slow down (2 = keypressed)
      getEnc();
      LCDintAt(1, 2, norm(currEnc), 5);
      LCDintAt(1, 11, enc2azm(currEnc), 3);
    }
    else {                                                            // Else do a controlled slow down
      while (!pressedBtns()) {                                        // Wait for encoder to reach the value for this PWM step
        getEnc();
        if (dir)  if (currEnc > targEnc) break;                       // RIGHT (CW) - slew complete when targEnc reached
        if (!dir) if (currEnc < targEnc) break;                       // LEFT (CCW) - slew complete when targEnc reached
        if (currEnc != lastEnc) moveTimer = millis() + ee.stuckTime;  // If encoder is moving then moveTimer is reset
        if (!(slaved = (millis() < moveTimer))) break;                // If no motion detected within STUCK_TIME then abort
        lastEnc = currEnc;                                            // Remember current encoder value for next time round
        LCDintAt(1, 2, norm(currEnc), 5);
        LCDintAt(1, 11, enc2azm(currEnc), 3);
      }
      if (btnsPressed) slaved = 2;
    }
    if (firstPass) {
      sendInfo("MOVE DOME - SLOW DOWN");
      LCDat(0, 15); LCD("}04");                                     // Ramping down arrow
      sendDisp();
      firstPass = 0;
    }

    targEnc += brakEnc[pwm];                                        // Upate targEnc
    analogWrite(PWM1, map(pwm, 0, 255, stopPWM1, maxPWM1));         // Slow down the motors
    analogWrite(PWM2, map(pwm, 0, 255, stopPWM2, maxPWM2));
    analogWrite(PWM3, map(pwm, 0, 255, stopPWM3, maxPWM3));
  };                                                                // When pwm is 0 then the motors are turned off
  
  // FINISH-UP -------------------------------------------------------------------------------------------------
  analogWrite(PWM1, 0);                             // Switch off the motors completely
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  stopEnc();
  domeAZM = enc2azm(normEnc());                     // Normalise currEnc and update domeAZM
  LCDintAt(1, 2, currEnc, 5);
  LCDintAt(1, 11, enc2azm(currEnc), 3);

  updateEEPROM(currEnc);                            // Update EEPROM with current Dome Azimuth
  LCDat(0, 15); LCD("_");                           // Dome finished moving
  LCDat(1, 15); LCD(".");
  sendInfo("MOVE DOME - FINISH UP");

  if (!slaved) {
    if (indexPtr > 1) LCD("$CALIBRATION DONE");
    else LCD("$HDOME IS STUCK$F");
  }
  if (slaved == 2) parking ? LCD("$HPARKING ABORTED") : LCD("$HTRACKING ABORTED");
  if (slaved != 1) {
    LCD("$NPress OK or ESC ");
    sendDisp();
    beep(1000, ee.bzAlert);
    if (!slaved) {
      while (!waitOKesc(1)) {
        beep(1000, ee.bzAlert);            // Dome is stuck - beep continuously
      }
    }
    else waitOKesc(0);
    beep(1, ee.bzAlert);
    parking = slaved = 0;                           // Stop reacting to received azimuth from scope
  }
  cxnTimer = millis() + PERIOD_CXN;
  lastDir = dir;
  lastTargAZM = targAZM;
  
  if (slaved == 2) return slaved;                   // A button was pressed so stop slaving
  return !((byte(domeAZM - targAZM) <= tol) || (byte(targAZM - domeAZM) <= tol)); // 0 if within tolerance, 1 if not

  //return domeAZM != targAZM;                        // Return 0 if domeAZM is same as target azimuth, 1 if not
}
//--------------------------------------------------------------------------------------------------------------
void sendInfo(char msg[]) {
  Serial.println(msg);
  Serial.print("domeAZM:  "); Serial.println(domeAZM);
  Serial.print("targAZM:  "); Serial.println(targAZM);
  Serial.print("lastTarg: "); Serial.println(lastTargAZM);
  Serial.print("prevEnc:  "); Serial.println(prevEnc);
  Serial.print("lastEnc:  "); Serial.println(lastEnc);
  getEnc();
  Serial.print("currEnc:  "); Serial.println(currEnc);
  Serial.print("targEnc:  "); Serial.println(targEnc);
  Serial.print("distEnc:  "); Serial.println(distEnc);
  Serial.print("midPoint: "); Serial.println(midPoint);
  Serial.print("dir:      "); Serial.print(dir); Serial.print(" "); Serial.println(dir ? "> CW" : "< CCW");
  Serial.print("pwm:      "); Serial.println(pwm);
  Serial.print("slaved:   "); Serial.println(slaved);
}
//--------------------------------------------------------------------------------------------------------------
byte manualMove(byte mtr) { // return
  if (!safe()) return btnE;
  byte button, pwm;
  if (slaved) {
    button = pressedBtns();
    if (!(button & (btnL | btnR))) return button;     // Return if Left nor Right buttons pressed
    LCD("$HResynching dome "); sendDisp();
  } else {
    LCD("$$Press < or > to$Nrotate the dome"); sendDisp();

    while(scanBtns());                                // Wait for all buttons to be released.
  }
  while(1) {                                          // Main loop
    btnsCurr = btnsDeb = btnsLast = scanStep = 0;
    pwm = 1;
    do {
      button = pressedBtns();
      if (button) beep(50, ee.bzBtns);
      if (button & btnE || button & btnO) {           // Esc button pressed?
        while(scanBtns());
        delay(500);
        while(scanBtns());
        return btnE;                                  // Exit
      }
      //if ((button & btnO) && slaved) return btnO;
      if (slaved) if (!(button & (btnR | btnL))) return btnO;
    } while (!(button & (btnR | btnL)));
    getEnc();
    LCD("$N$F");
//  "PWM 123 DZ 123 _" 
//   0123456789ABCDEF
    if (disp[1][0] != 'P') LCD("$NPWM:    ");
    if (!slaved) {
      if (disp[0][0] != 'E') LCD("$HEncoder:$F");
      if (disp[1][8] != 'P') if (!calibrating) LCDat(1, 8); LCD("Pt:");
      LCDintAt(0, 9, calibrating ? currEnc :normEnc(), 5);
      if (!calibrating) LCDintAt(1, 11, enc2azm(norm(currEnc)), 3);
    }
    if (!slaved) {LCDat(0, 15); LCD("}03"); sendDisp();}      // Ramping up arrow
    LCDat(1, 14); (button & btnR) ? LCD(" >") : LCD(" <");    // Direction arrow.
    sendDisp();
    setDirection(button & btnR);                              // CW is '>' if pressed, CCW if  '<'

    // START-UP ------------------------------------------------------------------------------------------------
    do {                                                      // Ramp-up the motor(s)
      delay(ee.rampSpeed);
      if (pressedBtns() != button) break;                     // Break out if a button is no longer pressed
      LCDintAt(1, 4, pwm, 3);
      startEnc();
      if (!slaved) {
        LCDintAt(0, 9, calibrating ? currEnc :normEnc(), 5);
        if (!calibrating) LCDintAt(1, 11, enc2azm(norm(currEnc)), 3);
      }            
      if (mtr == 1 || !mtr) analogWrite(PWM1, map(pwm, 0, 255, startPWM1, mtr ? startPWM1 : maxPWM1));  // mtr > 0 = align mtr
      if (mtr == 2 || !mtr) analogWrite(PWM2, map(pwm, 0, 255, startPWM2, mtr ? startPWM2 : maxPWM2));
      if (mtr == 3 || !mtr) analogWrite(PWM3, map(pwm, 0, 255, startPWM3, mtr ? startPWM3 : maxPWM3));
    } while(++pwm);                                           // Loop if maximum speed not reached

    // CONTINUE --------------------------------------------------------------------------------------------------
    if (!slaved) {LCDat(0, 15); LCD("}05"); sendDisp();}                // Top line
    while (pressedBtns() == button) {
      if (!slaved) {
        getEnc();
        LCDintAt(0, 9, calibrating ? currEnc :normEnc(), 5);            // Keep the dome turning while button is being pressed
        if (!calibrating) LCDintAt(1, 11, enc2azm(norm(currEnc)), 3);
      }
    }     
    sendDisp();
    beepBtns();

    // SLOW DOWN -------------------------------------------------------------------------------------------------
    if (!slaved) {LCDat(0, 15); LCD("}04");}                            // Down arrow
    do {
      pwm--;
      if (mtr == 1 || !mtr) analogWrite(PWM1, map(pwm, 0, 255, stopPWM1, mtr ? stopPWM1 : maxPWM1));  // mtr > 0 = align mtr
      if (mtr == 2 || !mtr) analogWrite(PWM2, map(pwm, 0, 255, stopPWM2, mtr ? stopPWM2 : maxPWM2));
      if (mtr == 3 || !mtr) analogWrite(PWM3, map(pwm, 0, 255, stopPWM3, mtr ? stopPWM3 : maxPWM3));
      delay(ee.stopSpeed);
      LCDintAt(1, 4, pwm, 3);
      if (!slaved) {
        getEnc();
        LCDintAt(0, 9, calibrating ? currEnc :normEnc(), 5);            // Keep the dome turning while button is being pressed
        if (!calibrating) LCDintAt(1, 11, enc2azm(norm(currEnc)), 3);
      }
    } while(pwm);

    // FINISH-UP --------------------------------------------------------------------------------------------------
    pwm = 0;
    analogWrite(PWM1, 0);                                   // Switch off the motors completely
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    stopEnc();
    if (!slaved) {LCDat(0, 15); LCD("_");}
    LCDat(1, 15); LCD("."); sendDisp();
    if (!calibrating) {                                     // Do stuff if not calibrating
      if (!slaved) domeAZM = enc2azm(normEnc());            // If not slaving, normalise currEnc
      else currEnc = azm2enc(domeAZM);                      // Make currEnc be encoder value for domeAZM
      updateEEPROM(currEnc);
    }
  }
}
//--------------------------------------------------------------------------------------------------------------
void setDirection(byte setDir) {
  dir = setDir;
  digitalWrite(DIR1, ee.dir1cw ? dir^1 : dir);   // Select direction on the motor control board
  digitalWrite(DIR2, ee.dir2cw ? dir^1 : dir);   // Select direction on the motor control board
  digitalWrite(DIR3, ee.dir3cw ? dir^1 : dir);   // Select direction on the motor control board
  if (dir) {                                      // Is dome to move CW (right) ?
    startPWM1 = ee.startPWM1cw;
    startPWM2 = ee.startPWM2cw;
    startPWM3 = ee.startPWM3cw;
    stopPWM1 = ee.stopPWM1cw;
    stopPWM2 = ee.stopPWM2cw;
    stopPWM3 = ee.stopPWM3cw;
    maxPWM1 = ee.maxPWM1cw;
    maxPWM2 = ee.maxPWM2cw;
    maxPWM3 = ee.maxPWM3cw;
  } else {                                        // Or is dome to move CCW (left)?
    startPWM1 = ee.startPWM1ccw;
    startPWM2 = ee.startPWM2ccw;
    startPWM3 = ee.startPWM3ccw;
    stopPWM1 = ee.stopPWM1ccw;
    stopPWM2 = ee.stopPWM2ccw;
    stopPWM3 = ee.stopPWM3ccw;
    maxPWM1 = ee.maxPWM1ccw;
    maxPWM2 = ee.maxPWM2ccw;
    maxPWM3 = ee.maxPWM3ccw;
  }
}
#pragma endregion DOME MOTION
//==============================================================================================================
#pragma region DOME BUDDY STUFF
//--------------------------------------------------------------------------------------------------------------
bool getScopeAZM() {                                            // Get and interpret scope's azimuth from DomeBuddy Dongle via Bluetooth
  newData = 0;
  for (byte i = 0; i <= 9; i++) buf[i] = 0;
  if (getFromUSB()) return newData = 0;                         // Return if a "button" has been pressed
  if (!buf[0]) if (!getFromBT()) return newData = 0;            // No valid data received
  if (parking || !slaved) return;

  LCD("$H$F$H");
  if (buf[1] ==  'T' || buf[1] == 'G') {
    byte value, chksum;                                           // Telescope is tracking or doing a GO TO
    newData = 0;
    value = buf[2] - ((buf[2] >= 'A') ? 55 : 48);                 // Get telescope azimuth into value (from hex)
    value = value * 16 + buf[3] - ((buf[3] >= 'A') ? 55 : 48);
    chksum = buf[4] - ((buf[4] >= 'A') ? 55 : 48);                // Calculate the checksum (from hex)
    chksum = chksum * 16 + buf[5] - ((buf[5] >= 'A') ? 55 : 48);
    if (value == byte(~chksum)) {                                 // Is the data valid?
      scopeAZM = value;                                           // Azimuth has been received correctly
      LCD("Go to azmth:"); LCDint((360UL * byte(scopeAZM + ee.azmAdj) / 256.0)); LCD("`");
      newData = 1;
    }
    else LCD("BAD CHECK SUM!");
  }
  else {  
    switch (buf[1]) {
      case 'S' : LCD("Scope is moving.");   newData = 1; break;   // Telescope is slewing
      case 'L' : LCD("Scope not on?");      newData = 0; break;   // Lost connection with telescope - last known position given
      default  : LCD("DATA NOT VALID!");    newData = 0; break;
    }
  }
  sendDisp();
  delay(1000);
  cxnTimer = millis() + PERIOD_CXN;                            // Reset connection timer for comms timeout
  return newData;                                              // Valid data has been received
}
//--------------------------------------------------------------------------------------------------------------
bool getFromUSB() {                               // Returns 1 for "key pressed", 0 if not
  if (!Serial.available()) return 0;
  char peek = Serial.peek();
  if (peek == '\n' || peek == '\r') {
    Serial.read();
    return 0;
  }
  if (peek == ':' || isDigit(peek)) {
    Serial.readBytesUntil('#', buf, 7);           // Read in simulated dongle data
    if (peek == ':') return 0;
    int azm = 0;                                  // Else form and return :T00FF# or similar
    for (byte i = 0; i <= 2; i++) {               // A number between 0 and 255 has been given
      if (!isdigit(buf[i])) break;
      azm = azm * 10 + (buf[i] - '0');
    }
    if (azm > 255) return buf[0] = 0;
    sprintf(buf, ":T%02X%02X#", azm, ~azm & 255);
    return 0;                                     // "Scope" azimuth received. 0 means no "key pressed"
  }

  switch (Serial.read()) {
    case '\n': return 0; break;
    case '\r': return 0; break;
    case '?': help();     return 0; break;
    case 'D': sendDisp(); return 0; break;
    case 'd': sendDisp(); return 0; break;
    case 'V': sendVars(); return 0; break;
    case 'v': sendVars(); return 0; break;
    case 'A': sendBrakEnc(); return 0; break;
    case 'C': sendEEPROM(); return 0; break;
    case '<': btnsLast = btnL; scanStep = 4; return 1; break;
    case '>': btnsLast = btnR; scanStep = 4; return 1; break;
    case ',': btnsLast = btnL; scanStep = 4; return 1; break;
    case '.': btnsLast = btnR; scanStep = 4; return 1; break;
    case 'O': btnsLast = btnO; scanStep = 4; return 1; break;
    case 'E': btnsLast = btnE; scanStep = 4; return 1; break;
    case 'o': btnsLast = btnO|btnH; scanStep = 4; return 1; break;
    case 'e': btnsLast = btnE|btnH; scanStep = 4; return 1; break;
    case 'X': btStat = 0; Serial.println("Live BT mode"); return 0; break;
    case 'x': btStat = 0; Serial.println("Live BT mode"); return 0; break;
    case 'B': btStat = 2; Serial.println("Simulate BT connected"); return 0; break;
    case 'b': btStat = 1; Serial.println("Simulate BT not connected");return 0; break;
    case 'T': testMode = 1; Serial.println("TEST MODE ACTIVE"); return 0; break;
    case 't': testMode = 0; Serial.println("LIVE MODE ACTIVE"); return 0; break;
  }
}
//--------------------------------------------------------------------------------------------------------------
bool getFromBT() {
  if (!btCxn()) return 0;                                       // Return immediately if no Bluetooth connection
  if (btStat) return 0;                                         // Return immediately if in simulated mode
  for (byte i = 0; i <= 9; i++) buf[i] = 0;
  do {
    while (Serial2.peek() >= 0){                                // Find the sentence start marker ':'
      if (Serial2.peek() == ':') break;                         // Found the marker so break out
      if (Serial2.peek() < 0) return;                           // Return if timed out
      Serial2.read();                                           // Dump the invalid character in the receive buffer
    }                                                           // Loop back and try again
    Serial2.readBytesUntil('#', buf, 7);                        // Get azmiuth info (7 bytes)
  } while (Serial2.peek() >= 0);
  return buf[0] > 0;
}
//--------------------------------------------------------------------------------------------------------------
long normEnc() {
  getEnc();
  if (calibrating) return currEnc;
  long value = currEnc;                                       // Normalises currEnc between 0 and enc360 - 1
  return currEnc = value % ee.enc360 + ((value < 0) ? ee.enc360 : 0);
}
//--------------------------------------------------------------------------------------------------------------
long norm(long value) {
  return value % ee.enc360 + ((value < 0) ? ee.enc360 : 0); // Normalises currEnc between 0 and enc360 - 1
}
//--------------------------------------------------------------------------------------------------------------
byte enc2azm (long enc){
  return ((enc + encOne / 2) * 256) / ee.enc360;
}
//--------------------------------------------------------------------------------------------------------------
long azm2enc (long azm){
  return (azm * ee.enc360) / 256;
}
//--------------------------------------------------------------------------------------------------------------
bool btCxn() {
  if (!btStat) return digitalRead(BTSTAT);                      // If not in simulation mode, return real BTSTAT
  return btStat - 1;                                            // Return simulated value (1: 0, 2: 1)
}
//--------------------------------------------------------------------------------------------------------------
void help() {
  Serial.println("\nCOMMANDS");
  Serial.println("--------");
  Serial.println("<\t Left button pressed (or ,)");
  Serial.println(">\t Right button pressed (or .)");
  Serial.println("O\t OK button pressed");
  Serial.println("E\t Esc button pressed");
  Serial.println("o\t OK button held then released");
  Serial.println("e\t ESC button held then released");
  Serial.println("D\t Get the LCD display");
  Serial.println("B\t Simulate Bluetooth connected");
  Serial.println("b\t Simulate Bluetooth not connected");
  Serial.println("X\t Stop Bluetooth simulation");
  Serial.println("T\t Enter test mode");
  Serial.println("t\t Enter live mode");
  Serial.println("S\t Pretend telescope is slewing");
  Serial.println("V\t See key variables");
  Serial.println("A\t See brakEnc[] array");
  Serial.println("C\t See contents of EPROM");
  Serial.println("L\t Pretend connection lost with telescope");
  Serial.println("N\t Go to azimuth point where N is a number 0 to 255");
  Serial.println(":T00FF#  Go to azimuth point Hex 00 (FF is inverse of 00)");
}
//--------------------------------------------------------------------------------------------------------------
void sendVars() {
  Serial.print("\nmaxPWM1cw  \t"); Serial.println(ee.maxPWM1cw);
  Serial.print("maxPWM2cw    \t"); Serial.println(ee.maxPWM2cw);
  Serial.print("maxPWM3cw    \t"); Serial.println(ee.maxPWM3cw);
  Serial.print("maxPWM1ccw   \t"); Serial.println(ee.maxPWM1ccw);
  Serial.print("maxPWM2ccw   \t"); Serial.println(ee.maxPWM2ccw);
  Serial.print("maxPWM3ccw   \t"); Serial.println(ee.maxPWM3ccw);
  Serial.print("startPWM1cw  \t"); Serial.println(ee.startPWM1cw);
  Serial.print("startPWM2cw  \t"); Serial.println(ee.startPWM2cw);
  Serial.print("startPWM3cw  \t"); Serial.println(ee.startPWM3cw);
  Serial.print("startPWM1ccw \t"); Serial.println(ee.startPWM1ccw);
  Serial.print("startPWM2ccw \t"); Serial.println(ee.startPWM2ccw);
  Serial.print("startPWM3ccw \t"); Serial.println(ee.startPWM3ccw);
  Serial.print("stopPWM1cw   \t"); Serial.println(ee.stopPWM1cw);
  Serial.print("stopPWM2cw   \t"); Serial.println(ee.stopPWM2cw);
  Serial.print("stopPWM3cw   \t"); Serial.println(ee.stopPWM3cw);
  Serial.print("stopPWM1ccw  \t"); Serial.println(ee.stopPWM1ccw);
  Serial.print("stopPWM2ccw  \t"); Serial.println(ee.stopPWM2ccw);
  Serial.print("stopPWM3ccw  \t"); Serial.println(ee.stopPWM3ccw);
  Serial.print("ee.dir1cw    \t"); Serial.println(ee.dir1cw);
  Serial.print("ee.dir2cw    \t"); Serial.println(ee.dir2cw);
  Serial.print("ee.dir3cw    \t"); Serial.println(ee.dir3cw);
  Serial.print("ee.rampSpeed \t"); Serial.println(ee.rampSpeed);
  Serial.print("ee.stopSpeed \t"); Serial.println(ee.stopSpeed);
  Serial.print("ee.stuckTime \t"); Serial.println(ee.stuckTime);
  Serial.print("enc360       \t"); Serial.println(ee.enc360);
  Serial.print("enc180       \t"); Serial.println(enc180);
  Serial.print("encOne       \t"); Serial.println(encOne);
  Serial.print("azmAdj       \t"); Serial.println(ee.azmAdj);
  Serial.print("ee.indexSw   \t"); Serial.println(ee.indexSw);
  getEnc();
  Serial.print("currEnc      \t"); Serial.println(currEnc);
  Serial.print("homeAZM      \t"); Serial.println(ee.homeAZM);
  Serial.print("domeAZM      \t"); Serial.println(domeAZM);
  Serial.print("scopeAZM     \t"); Serial.println(scopeAZM);
  Serial.print("ee.addr      \t"); Serial.println(ee.addr);
  Serial.print("ee.enc       \t"); Serial.println(long(EEPROM.read(ee.addr) + EEPROM.read(ee.addr + 1) *256 + EEPROM.read(ee.addr + 2) * 65536));
  Serial.print("domeSafe     \t"); Serial.println(domeSafe);
  Serial.print("ee.password  \t"); Serial.print(ee.password); Serial.println(".");
  Serial.print("ee.encDir    \t"); Serial.print(ee.encDir); Serial.println(ee.encDir ? " >CW" :  " <CCW");
  Serial.print("BTSTAT       \t"); Serial.println(digitalRead(BTSTAT));
  Serial.print("btStat       \t"); Serial.println(btStat);
  Serial.print("btCxn()      \t"); Serial.println(btCxn());
  Serial.print("newData      \t"); Serial.println(newData);
  Serial.print("slaved       \t"); Serial.println(slaved);
  Serial.print("prevEnc      \t"); Serial.println(prevEnc);
  Serial.print("targEnc      \t"); Serial.println(targEnc);
  Serial.print("distEnc      \t"); Serial.println(distEnc);
  Serial.print("stepDist     \t"); Serial.println(stepDist);
  Serial.print("tempEnc      \t"); Serial.println(tempEnc);
  Serial.print("lastEnc      \t"); Serial.println(lastEnc);
  Serial.print("midPoint     \t"); Serial.println(midPoint);
  Serial.print("dir          \t"); Serial.println(dir);
  Serial.print("pwm          \t"); Serial.println(pwm);
  Serial.print("testMode     \t"); Serial.println(testMode ? "TEST" :  "LIVE");
}
//--------------------------------------------------------------------------------------------------------------
void sendBrakEnc() {
  Serial.println("brakEnc[256]:");
  for (int i = 0; i <= 240; i+= 16) {
    printHex3(i);
    Serial.print(':');
    for (byte j = 0; j <= 15; j++) {
      Serial.print(' ');
      printHex3(brakEnc[i + j]);
    }
    Serial.println();
    if (i == 240) break;
  }
}
//--------------------------------------------------------------------------------------------------------------
void sendEEPROM() {
  Serial.println("EEPROM:");
  for (int i = 0; i <= 4064; i+= 32) {
    printHex3(i);
    Serial.print(':');
    for (byte j = 0; j <= 31; j++) {
      Serial.print(' ');
      if (EEPROM.read(i + j) < 16) Serial.print('0');
      Serial.print(EEPROM.read(i + j), HEX);
    }
    Serial.println();
  }
}
//--------------------------------------------------------------------------------------------------------------
void printHex3(long v) {
  if (v < 256) Serial.print('0');
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}
//-------------------------- ------------------------------------------------------------------------------------
void updateEEPROM(long enc) {
  long eeEnc;

// Store at EEPROM address ee.addr.  If it cannot store at this address, the next address is tried.
  ee.addr--;
  do {
    ee.addr++;
    if (ee.addr > (4096 - 4)) {
      LCD("$$EEPROM write $Nfailed: ");
      LCDint(ee.addr);
      beep(3000, ee.bzAlert);
      return;
    }
    EEPROM.put(ee.addr, enc);
    EEPROM.get(ee.addr, eeEnc);
  } while (eeEnc != enc);
  EEPROM.put(0, ee);                // Update addr pointer as necessary
}
//--------------------------------------------------------------------------------------------------------------
#pragma endregion DOME BUDDY STUFF
//==============================================================================================================
#pragma region ISR STUFF
//--------------------------------------------------------------------------------------------------------------
void setupISR() {   // Mode = 0 live, Mode = 1 test rig
  detachInterrupt(digitalPinToInterrupt(ENC_ChA));
  if (testMode) return;
  if (ee.encDir) attachInterrupt(digitalPinToInterrupt(ENC_ChA), encISRccw, CHANGE);
  else attachInterrupt(digitalPinToInterrupt(ENC_ChA), encISRcw, CHANGE);
}
//--------------------------------------------------------------------------------------------------------------
void encISRcw() {                   // Clockwise
  switch (PIND & B11) {             // PIND Bit 1 is ChA, PIND Bit 0 is ChB
    case B00: currEnc++; return;
    case B01: currEnc--; return;
    case B10: currEnc--; return;
    case B11: currEnc++; return;
  }
}
//--------------------------------------------------------------------------------------------------------------
void encISRccw() {                  // Counter clockwise
  switch (PIND & B11) {             // PIND Bit 1 is ChA, PIND Bit 0 is ChB
    case B00: currEnc--; return;
    case B01: currEnc++; return;
    case B10: currEnc++; return;
    case B11: currEnc--; return;
  }
}
//--------------------------------------------------------------------------------------------------------------
void getEnc() {
  if (testMode) {
    if (!encMoving) return;
    long offset  = millis() - prevMillis;
    prevMillis = millis();
    if (!dir) offset = -offset;
    currEnc += offset;
  }
}
//--------------------------------------------------------------------------------------------------------------
void startEnc() {
  if (testMode) {
    if (!encMoving) {
      prevMillis = millis();
      encMoving = 1;
    }
    getEnc();
  }
}
//--------------------------------------------------------------------------------------------------------------
void stopEnc() {
  if (testMode) {
    getEnc();
    encMoving = 0;
  }
}
//--------------------------------------------------------------------------------------------------------------
void indexISR() {
  if (currEnc - indexEnc[indexPtr] < 1000) return;    // Index switch triggered - get rid off debounce
  indexEnc[indexPtr++] = currEnc;
  if (indexPtr < 2) return;
  detachInterrupt(digitalPinToInterrupt(INDEX_SW));   // Triggered twice (one rotation) - stop interrupt
}
#pragma endregion ISR STUFF
//==============================================================================================================