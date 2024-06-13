#include <Arduino.h>
#include <SPI.h>

#include <WiFi.h>
#include <WiThrottleProtocol.h>

// WiFi Network Setup
const char* ssid = "MySSID";
const char* password =  "MyPWD";

// WiThrottle Server and Port
IPAddress serverAddress(192,168,1,1);
int serverPort = 12090;

const char version[] = "0.2a";

//Helper Functions
#define ARRAY_SIZE(array) ((sizeof(array))/(sizeof(array[0])))

#define BLINK_INTERVAL 400 //interval in milliseconds to blink leds when needed
#define SHORT_DCC_ADDRESS_LIMIT 127
#define MAX_THROTTLES 10

//Pins
const int  dataPinsLength    = 8;
const byte dataPins[dataPinsLength]       PROGMEM = {27, 26, 25, 32, 22, 19, 5, 17};
const int  addressPinsLength = 4;
const byte addressPins[addressPinsLength] PROGMEM = {23, 13, 15, 14};
const int  specialPinsLength = 5;
const byte specialPins[specialPinsLength] PROGMEM = {18, 33, 16, 4, 21}; //35 ReadButton; 34 WriteLamp; 36 Convert; 37 ReadAnalog; 39 DisplayClock


const int lampAndButtonArrayLength = 10;
// lamp arrays                         //  1, 2, 3, 4, 5, 6, 7, 8, 9, 0
const int previewRowLamp[lampAndButtonArrayLength] PROGMEM  = {38,36,34,32, 1, 3, 5, 7, 6, 4};
const int programRowLamp[lampAndButtonArrayLength] PROGMEM  = {33,35,37,39,13,15,14,12, 0, 2};
const int  keybusRowLamp[lampAndButtonArrayLength] PROGMEM  = {30,28,26,24, 9, 8,11,10,51,53};
const int patternRowLamp[lampAndButtonArrayLength] PROGMEM  = {70,69,65,79,75,68,70,67,78,77};

// button arrays
const int previewRowButton[lampAndButtonArrayLength] PROGMEM = {24,25,26,27,28,29,30,31,36,37};
const int programRowButton[lampAndButtonArrayLength] PROGMEM = {16,17,18,19,20,21,22,23,32,33};
const int  keybusRowButton[lampAndButtonArrayLength] PROGMEM = { 8, 9,10,11,12,13,14,15,34,35};
const int patternRowButton[lampAndButtonArrayLength] PROGMEM = {68,69,66,65,64,76,77,74,73,72};


//Input
const int BtBffSize = 80;
bool buttonBuffer[BtBffSize];
bool oldButtonBuffer[BtBffSize];
byte onButtonBuffer[BtBffSize];
byte offButtonBuffer[BtBffSize];

//Blinking Function
bool blinkState = 0;
unsigned long previousMillis = 0;
bool lampBlink[100];

const int analogBufferLength = 16;
unsigned int analogBuffer[analogBufferLength];
unsigned int oldAnalogBuffer[analogBufferLength];
unsigned int sendAnalogBuffer[32];

//Output
const int lampBufferLength = 100;
bool lampBuffer[100];
const int displayBufferLength = 7;
byte displayBuffer[7];

// LOCO SETUP
WiFiClient client;
WiThrottleProtocol Throttle;

int currentSpeed[MAX_THROTTLES];   // set to maximum possible (6)
Direction currentDirection[MAX_THROTTLES];   // set to maximum possible (6)

TrackPower trackPower = PowerUnknown;

// function states
boolean functionStates[MAX_THROTTLES][MAX_FUNCTIONS];   // set to maximum possible (6 throttles)

// function labels
String functionLabels[MAX_THROTTLES][MAX_FUNCTIONS];   // set to maximum possible (6 throttles)

// consist function follow
int functionFollow[MAX_THROTTLES][MAX_FUNCTIONS];   // set to maximum possible (6 throttles)

// speedstep
int currentSpeedStep[MAX_THROTTLES];   // set to maximum possible (6 throttles)

// throttle
int currentThrottleIndex = 0;
char currentThrottleIndexChar = '0';
int maxThrottles = MAX_THROTTLES;

int heartBeatPeriod = 10; // default to 10 seconds
long lastServerResponseTime;  // seconds since start of Arduino
boolean heartbeatCheckEnabled = true;

// used to stop speed bounces
long lastSpeedSentTime = 0;
int lastSpeedSent = 0;
// int lastDirectionSent = -1;
int lastSpeedThrottleIndex = 0;

String inputLocoAddress;

// Always return a positive integer by multiplying by minus one if less than zero
int abs2(int input) {
  return input < 0 ? -input : input;
}

void setup() {
  initPanel();

  inputLocoAddress.reserve(32);
  
  Serial.begin(115200); //Debug
  Serial.println("");
  Serial.println("GVG WiThrottleProtocol Client");
  Serial.print("Version: ");
  Serial.println(version);
  Serial.println("");

  // Connect to WiFi network
  Serial.println("Connecting to WiFi.."); 
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) delay(1000);  
  Serial.print("Connected with IP: "); Serial.println(WiFi.localIP());

  // Connect to the server
  Serial.println("Connecting to the server...");
  if (!client.connect(serverAddress, serverPort)) {
    Serial.println("connection failed");
    while(1) delay(1000);
  }
  Serial.println("Connected to the server");

  Throttle.connect(&client);
  Serial.println("WiThrottle connected");
  Throttle.setDeviceName("GVG110"); 

}

void loop() {
  // READ PANEL STATE
  readButtons(); //Read buttons
  readAnalog(); //Read analog
  Throttle.check(); // parse incoming wiThrottle messages

  // Compare old and new buttonstate
  byte onButtonCounter = 0;
  byte offButtonCounter = 0;
  for (byte i = 0; i < BtBffSize; i++) {
    if (buttonBuffer[i] != oldButtonBuffer[i]) {
      if (!buttonBuffer[i]) {
        onButtonBuffer[onButtonCounter] = i;
        onButtonCounter++;
      } else {
        offButtonBuffer[offButtonCounter] = i;
        offButtonCounter++;
      }
    }
    oldButtonBuffer[i] = buttonBuffer[i];
  }

  // Compares analog values
  byte potHasChangedCounter = 0;
  for (byte i=0; i < 16; i++) {
    if (abs2(analogBuffer[i] - oldAnalogBuffer[i]) > 2) {
      sendAnalogBuffer[potHasChangedCounter] = i;
      sendAnalogBuffer[potHasChangedCounter + 1] = analogBuffer[i];
      potHasChangedCounter += 2;
    }
    oldAnalogBuffer[i] = analogBuffer[i];
  }

// Process Surface Interaction
  if (potHasChangedCounter || onButtonCounter || offButtonCounter) { 
    
    // If analog changed, process action
    if (potHasChangedCounter) { 
      Serial.println("Analog input has changed");
      for (byte i=0; i < potHasChangedCounter; i+=2) {
        Serial.print("pot ID: ");
        Serial.print(sendAnalogBuffer[i]);
        Serial.print(" value: ");
        Serial.println(sendAnalogBuffer[i+1]);
        doAnalogAction(sendAnalogBuffer[i],sendAnalogBuffer[i+1]);
      }
    }

    // If buttons changed to on, process action
    if (onButtonCounter) { 
      Serial.println("Button pressed");
      for (byte i=0; i < onButtonCounter; i++) {
        Serial.print("button ID: ");
        Serial.println(onButtonBuffer[i]);
        doButtonAction(onButtonBuffer[i]);
      }
    }

    if (offButtonCounter) { // If buttons changed to off, then send
      Serial.println("Button released");
      for (byte i = 0; i < offButtonCounter; i++) {
        Serial.print("button ID: ");
        Serial.println(offButtonBuffer[i]);
      }
    }
  }

  updateDirectionLampState();

  blinkLEDs();
  writeLamps();
  writeDisplay();
}

void updateDirectionLampState(){
  Direction leadLocoCurrentDirection;
  String leadLoco = Throttle.getLeadLocomotive(currentThrottleIndexChar);
  leadLocoCurrentDirection = Throttle.getDirection(currentThrottleIndexChar, leadLoco);
  if( leadLocoCurrentDirection == Forward ) {
    //setLamp(, false); // button 52
    setLamp(46, false); // down arrow
    setLamp(44, true); // button 53
    setLamp(47, true); // up arrow
  } else {
    //setLamp(, true); // button 52
    setLamp(46, true); // down arrow
    setLamp(44, false); // button 53
    setLamp(47, false); // up arrow
  }
}

bool getLamp(int lampId) {
  return bool(lampBuffer[lampId]);
}

int isPreviewRowButton(int buttonId) {
  for (int i=0; i < lampAndButtonArrayLength; i++) {
    if(buttonId == previewRowButton[i]){
      return i;
    }
  }
  return -1;
}
int isProgramRowButton(int buttonId) {
  for (int i=0; i < lampAndButtonArrayLength; i++) {
    if(buttonId == programRowButton[i]){
      return i;
    }
  }
  return -1;
}
int isKeybusRowButton(int buttonId) {
  for (int i=0; i < lampAndButtonArrayLength; i++) {
    if(buttonId == keybusRowButton[i]){
      return i;
    }
  }
  return -1;
}
int isPatternRowButton(int buttonId) {
  for (int i=0; i < lampAndButtonArrayLength; i++) {
    if(buttonId == patternRowButton[i]){
      return i;
    }
  }
  return -1;
}

void setLamp(int lampId, bool onOff) {
  if(lampId > 80) return;
  lampBuffer[lampId] = int(onOff);
}

void setAllLamps(bool onOff){
  for (byte i=0; i < 100; i++) {
    lampBuffer[i] = int(onOff);
  }
}

void setDisplayBlank(){
  for(int i=0; i<sizeof displayBuffer/sizeof displayBuffer[0]; i++){
    displayBuffer[i] = 15;
  }
}

void setDisplayInt(int clockDisplay) {
  // Blank current screen display
  setDisplayBlank();
  if(clockDisplay == 0){
    displayBuffer[3] = 0;
    return;
  }
  byte leng = (byte)floor(log10(clockDisplay) + 1);
  char displayArray[4];
  itoa(clockDisplay, displayArray, 10);

  byte counter = 0;
  byte displayWidth = 4;
  byte offset = displayWidth - leng;
  do {
    displayBuffer[counter + offset] = int(displayArray[counter]);
    counter++;
  } while (counter < leng);
}

void writeDisplay() {
  dataLinesAsOutput();
  delayMicroseconds(30);
  byte i = 0;
  while (i < 8) {
    setDisplayAddressBus(i);
    delayMicroseconds(10);
    digitalWrite(dataPins[3], (displayBuffer[i] & 1));
    digitalWrite(dataPins[4], (displayBuffer[i] & 2) >> 1);
    digitalWrite(dataPins[5], (displayBuffer[i] & 4) >> 2);
    digitalWrite(dataPins[6], (displayBuffer[i] & 8) >> 3);
    i++;
    beginSpecialFunction(4);
    delayMicroseconds(10);
    endSpecialfunction();
  }
}

void writeLamps() {
  dataLinesAsOutput();
  byte tmp1_counter = 0;
  delayMicroseconds(50);
  byte i = 0;
  while (i < 10) {
    setAddressBus(i);
    byte g = 0;
    while (g < 8) {
      digitalWrite(dataPins[g], lampBuffer[tmp1_counter + g]);
      g++;
      delayMicroseconds(3);
    }
    beginSpecialFunction(1);
    tmp1_counter += 8;
    delayMicroseconds(30);
    endSpecialfunction();
    i++;
  }
  endSpecialfunction();
  delayMicroseconds(300);
}

void blinkLEDs() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= BLINK_INTERVAL){
    for(int i = 0; i < ARRAY_SIZE(lampBlink); i++){
      if(lampBlink[i]){
        setLamp(i,blinkState);
      }
    }
    blinkState = (blinkState == 0) ? 1 : 0;
    previousMillis = currentMillis;
  }
}

void readButtons() {
  delayMicroseconds(50);
  dataLinesAsInput2();
  delayMicroseconds(50);
  byte tmp1_counter = 0;
  byte i = 0;
  while (i < 10) {
    setAddressBus(i);
    beginSpecialFunction(0);
    delayMicroseconds(30);
    byte g = 0;
    while (g < 8) {
      buttonBuffer[tmp1_counter + g] = digitalRead(dataPins[g]);
      g++;
      delayMicroseconds(10);
    }
    endSpecialfunction();
    tmp1_counter += 8;
    i++;
  }
}

void readAnalog() {
  dataLinesAsInput();
  byte i = 0;
  while (i < 15) {
    setAddressBus(i);
    delayMicroseconds(30);
    beginSpecialFunction(2);
    delayMicroseconds(30);
    endSpecialfunction();
    delayMicroseconds(30);
    beginSpecialFunction(3);
    delayMicroseconds(30);
    byte a = 7;
    byte b = 9;
    analogBuffer[i] = 0;
    while (a != 255) {
      analogBuffer[i] |= (digitalRead(dataPins[a]) << b);
      a--;
      b--;
    }
    endSpecialfunction();
    delayMicroseconds(30);
    beginSpecialFunction(3);
    delayMicroseconds(30);
    a = 7;
    while (a > 5) {
      analogBuffer[i] |= (digitalRead(dataPins[a]) << b);
      a--;
      b--;
    }
    endSpecialfunction();
    i++;
  }
}

void beginSpecialFunction(byte pin) {  //0 ReadButton; 1 WriteLamp; 2 Convert; 3 readAnalog; 4 Displayclock
  byte i = 0;
  while (i < specialPinsLength) {
    digitalWrite(specialPins[i], HIGH);
    i++;
  }
  digitalWrite(specialPins[pin], LOW);
  delayMicroseconds(30);
}

void endSpecialfunction() {
  byte i = 0;
  while (i < specialPinsLength) {
    digitalWrite(specialPins[i], HIGH);
    i++;
    delayMicroseconds(30);
  }
}

void setDisplayAddressBus(byte number) {
  digitalWrite(dataPins[0], (number & 1));
  digitalWrite(dataPins[1], (number & 2) >> 1);
  digitalWrite(dataPins[2], (number & 4) >> 2);
}

void setAddressBus(byte number) {
  digitalWrite(addressPins[0], (number & 1));
  digitalWrite(addressPins[1], (number & 2) >> 1);
  digitalWrite(addressPins[2], (number & 4) >> 2);
  digitalWrite(addressPins[3], (number & 8) >> 3);
  delayMicroseconds(10);
}

void dataLinesAsInput() {
  byte i = 0;
  while (i < dataPinsLength) {
    pinMode(dataPins[i], INPUT);
    i++;
    delayMicroseconds(10);
  }
}

void dataLinesAsInput2() {
  byte i = 0;
  while (i < dataPinsLength) {
    pinMode(dataPins[i], INPUT_PULLUP);
    i++;
    delayMicroseconds(10);
  }
}

void dataLinesAsOutput() {
  byte i = 0;
  while (i < dataPinsLength) {
    pinMode(dataPins[i], OUTPUT);
    i++;
    delayMicroseconds(10);
  }
}

void allDatalinesLow() {
  byte i = 0;
  while (i < dataPinsLength) {
    digitalWrite(dataPins[i], LOW);
    i++;
    delayMicroseconds(10);
  }
}

void initPanel(){
  byte i = 0;
  while (i < addressPinsLength) {
    pinMode(addressPins[i], OUTPUT);
    i++;
  }
  i = 0;
  while (i < specialPinsLength) {
    pinMode(specialPins[i], OUTPUT);
    i++;
  }

  i = 0;
  while (i < analogBufferLength) {
    analogBuffer[i] = 0;
    i++;
  }
  i = 0;
  while (i < lampBufferLength) {
    displayBuffer[i] = 15;
    i++;
  }
  i = 0;
  while (i < lampBufferLength) {
    lampBuffer[i] = 0;
    if (i == 42) {
      lampBuffer[i] = 1;
    }
    i++;
  }
  i = 0;
  while (i < BtBffSize) {
    buttonBuffer[i] = 1;
    oldButtonBuffer[i] = 1;
    i++;
  }
  writeLamps();
  writeDisplay();
}

void setTrackPower(TrackPower powerState) {
  Throttle.setTrackPower(powerState);
  trackPower = powerState;
}

void toggleTrackPower(){
    if (trackPower==PowerOn) {
        setTrackPower(PowerOff);
    }else{
        setTrackPower(PowerOn);
    }
}

int getMultiThrottleIndex(char multiThrottle) {
    int mThrottle = multiThrottle - '0';
    if ((mThrottle >= 0) && (mThrottle<=5)) {
        return mThrottle;
    } else {
        return 0;
    }
}

char getMultiThrottleChar(int multiThrottleIndex) {
  return '0' + multiThrottleIndex;
}

void doAnalogAction(unsigned int potId, unsigned int value){
  // 2 is the T-Bar
  if( potId == 2 ){
    setDisplayInt(value);
    setSpeed(currentThrottleIndex, value);
  }
  return;
}

void doButtonAction(byte buttonId) {
  // If is edit mode, do edit mode things then return.
  // If Edit button is pressed again exit edit mode.
  if(isEditMode()){
    switch (buttonId){
      case 79: {
        toggleEditMode();
        break;
      }
      default: {
        editMode(buttonId);
        break;
      }
    }
    return;
  }
  // If a button on preview row is pressed, set that throttle to active.
  if (int buttonValue = isPreviewRowButton(buttonId)){
    changeActiveThrottle(buttonValue);
    return;
  }
  if (int buttonValue = isKeybusRowButton(buttonId)){
    doFunction(currentThrottleIndex, buttonId, true);
  }
  switch (buttonId) {
      // EDIT MODE
      case 79: {
        toggleEditMode();
        break;
      }
      case 53: {
        changeDirection(currentThrottleIndex, Forward);
        break;
      }
      case 52: {
        changeDirection(currentThrottleIndex, Reverse);
        break;
      }
      case 50: {
        toggleDirection(currentThrottleIndex);
        break; 
      }
      case 39: {
        setSpeed(currentThrottleIndex, 0); // STOP CURRENT LOCO
        break;
      }
      case 54: {
        speedEmergStop();
        break;
      }
      case 38: {
        speedEmergStopCurrentLoco();
        break;
      }
      case 7: {
        toggleTrackPower();
        break;
      }
      case 51: {
        stopThenToggleDirection();
        break;
      }
  }
}

// TOGGLE EDIT MODE
void toggleEditMode() {
  if(!getLamp(76)){  // If lamp 76 is off: turn on and enable edit mode.
    Serial.println("Entered Address EDIT Mode");
    setLamp(76,true);
    // START flahsing Preview buttons for selection
    for (int lamp : previewRowLamp) {
      lampBlink[lamp] = 1;
      setLamp(lamp, true);
    }
    inputLocoAddress = "";
  } else {
    Serial.println("Exited Address EDIT Mode");
    // turn off and disable edit mode
    setLamp(76,false);
    // STOP flashing Preview buttons
    for (int lamp : previewRowLamp) {
      lampBlink[lamp] = 0;
      setLamp(lamp, false);
    }
    // Clear display
    setDisplayBlank();
  }
}

bool isEditMode() {
  return getLamp(76); // Lamp 76 is the edit mode lamp
}

void editMode(byte buttonId) {
  if (int buttonValue = isPreviewRowButton(buttonId)){
    // STOP Flashing Preview buttons after selection
    for (int lamp : previewRowLamp) {
      lampBlink[lamp] = 0;
      setLamp(lamp, false);
    }
    setLamp(previewRowLamp[buttonValue], true);
    currentThrottleIndex = buttonValue;
    currentThrottleIndexChar = getMultiThrottleChar(currentThrottleIndex);
  }
  if(currentThrottleIndex != -1){
    // Use Program Buttons to input DCC Address
    if (int buttonValue = isProgramRowButton(buttonId)){
      inputLocoAddress += (buttonValue % 10);
    }
    if(buttonId == 70){ // set Entered Address as Loco ID at selected locoThrottleIndex
      String loco = getLocoWithLength(inputLocoAddress);
      currentThrottleIndexChar = getMultiThrottleChar(currentThrottleIndex);
      Throttle.addLocomotive(currentThrottleIndexChar, loco);
      Throttle.getDirection(currentThrottleIndexChar, loco);
      Throttle.getSpeed(currentThrottleIndexChar);
      resetFunctionStates(currentThrottleIndex);
      setLamp(76,false);
    }
    if(buttonId == 78 || buttonId == 63) {
      inputLocoAddress.remove(inputLocoAddress.length()-1);
    }
    setDisplayInt(inputLocoAddress.toInt());
  }

}

void changeActiveThrottle(int buttonValue){
  int wasThrottle = currentThrottleIndex;
  for (int lamp : previewRowLamp) {
    setLamp(lamp, false);
  }
  setLamp(previewRowLamp[buttonValue], true);

  currentThrottleIndex = buttonValue;
  if (currentThrottleIndex >= maxThrottles) {
    currentThrottleIndex = 0;
  }
  currentThrottleIndexChar = getMultiThrottleChar(currentThrottleIndex);
}

void toggleDirection(int multiThrottleIndex) {
  if (Throttle.getNumberOfLocomotives(getMultiThrottleChar(multiThrottleIndex)) > 0) {
    changeDirection(multiThrottleIndex, (currentDirection[multiThrottleIndex] == Forward) ? Reverse : Forward );
  }
}

void changeDirection(int multiThrottleIndex, Direction direction) {
  String loco; String leadLoco;
  Direction leadLocoCurrentDirection;
  char multiThrottleChar = getMultiThrottleChar(multiThrottleIndex);
  int locoCount = Throttle.getNumberOfLocomotives(multiThrottleChar);

  if (locoCount > 0) {
    currentDirection[multiThrottleIndex] = direction;

    if (locoCount == 1) {
      Throttle.setDirection(multiThrottleChar, direction);  // change all

    } else {
      leadLoco = Throttle.getLeadLocomotive(multiThrottleChar);
      leadLocoCurrentDirection = Throttle.getDirection(multiThrottleChar, leadLoco);

      for (int i=1; i<locoCount; i++) {
        loco = Throttle.getLocomotiveAtPosition(multiThrottleChar, i);
        if (Throttle.getDirection(multiThrottleChar, loco) == leadLocoCurrentDirection) {
          Throttle.setDirection(multiThrottleChar, loco, direction);
        } else {
          if (Throttle.getDirection(multiThrottleChar, loco) == Reverse) {
            Throttle.setDirection(multiThrottleChar, loco, Forward);
          } else {
            Throttle.setDirection(multiThrottleChar, loco, Reverse);
          }
        }
      }
      Throttle.setDirection(multiThrottleChar, leadLoco, direction);
    } 
  }
}

void speedEmergStop() {
  for (int i=0; i<maxThrottles; i++) {
    Throttle.emergencyStop(getMultiThrottleChar(i));
    currentSpeed[i] = 0;
  }
}

void speedEmergStopCurrentLoco() {
  Throttle.emergencyStop(currentThrottleIndexChar);
  currentSpeed[currentThrottleIndex] = 0;
}

void setSpeed(int multiThrottleIndex, int amt) {
  char multiThrottleIndexChar = getMultiThrottleChar(multiThrottleIndex);
  if (Throttle.getNumberOfLocomotives(multiThrottleIndexChar) > 0) {
    int newSpeed = amt;
    if (newSpeed >126) { newSpeed = 126; }
    if (newSpeed <0) { newSpeed = 0; }
    Throttle.setSpeed(multiThrottleIndexChar, newSpeed);
    currentSpeed[multiThrottleIndex] = newSpeed;

    // used to avoid bounce
    lastSpeedSentTime = millis();
    lastSpeedSent = newSpeed;
    // lastDirectionSent = -1;
    lastSpeedThrottleIndex = multiThrottleIndex;
  }
}

void stopThenToggleDirection() {
  if (Throttle.getNumberOfLocomotives(currentThrottleIndexChar) > 0) {
    if (currentSpeed[currentThrottleIndex] != 0) {
      setSpeed(currentThrottleIndex,0);
    } else {
      toggleDirection(currentThrottleIndex);
    }
    currentSpeed[currentThrottleIndex] = 0;
  }
}

String getLocoWithLength(String loco) {
  int locoNo = loco.toInt();
  String locoWithLength = "";
  if ( (locoNo > SHORT_DCC_ADDRESS_LIMIT) 
  //|| ( (locoNo <= SHORT_DCC_ADDRESS_LIMIT) && (loco.charAt(0)=='0') && (!serverType.equals("DCC-EX" ) ) ) 
  ) {
    locoWithLength = "L" + loco;
  } else {
    locoWithLength = "S" + loco;
  }
  return locoWithLength;
}

void doFunction(int multiThrottleIndex, int functionNumber, boolean pressed) {
  char multiThrottleIndexChar = getMultiThrottleChar(multiThrottleIndex);
  Throttle.setFunction(multiThrottleIndexChar, functionNumber, pressed);
}

void resetFunctionStates(int multiThrottleIndex) {
  for (int i=0; i<MAX_FUNCTIONS; i++) {
    functionStates[multiThrottleIndex][i] = false;
  }
}