#include <TimeLib.h>
#include <DS3232RTC.h>
#include <Wire.h>
#include <Timezone.h>
#include <TM1637Display.h>
#include <Adafruit_LEDBackpack.h>
#include <SoftwareSerial.h>
#include <CapacitiveSensor.h>

CapacitiveSensor touchControl = CapacitiveSensor(17,16);
//Daylight saving time via "Timezone.h"
//creating the statements
TimeChangeRule myDST = {"MESZ", Last, Sun, Mar, 2, +120};
TimeChangeRule mySDT = {"MEZ", Last, Sun, Oct, 3, +60};
//initilizing an object of class "Timezone", with the attributes from above
Timezone DE(myDST, mySDT);
//giving the pointer to tcr to TimeChangeRule
TimeChangeRule *tcr;
//initilizing utc and local time of type time_t
time_t utc, local;

//defining the pin for the alarm flip switch
#define FLIP_SWITCH 2
//defining the pins for the alarm display
#define ALARM_DISPLAY_CLK 14
#define ALARM_DISPLAY_DIO 15
//defining the pins for the date display
#define DATE_DISPLAY_CLK 9
#define DATE_DISPLAY_DIO 8
//defining the pins for the rotary encoders 1 and 2
#define RT1_ENCODER_CLK 6
#define RT1_ENCODER_DT 7
#define RT2_ENCODER_CLK 4
#define RT2_ENCODER_DT 5
//defining the pins for the DF Player
#define DFPLAYER_RX 10
#define DFPLAYER_TX 11

#define SONG_LIMIT 400

//initializing variables for getting the rotary encoders position
int ec1Last;
int ec1Val;
int ec2Last;
int ec2Val;
//initializing variable for the alarm hour and alarm minute
int ahour;
int amin;
//initializing variable for time refrence
unsigned long encoderMillis;
//initializing boolean for alarm
boolean playing = false;
boolean displayON = true;

//filling the DF-Player array with commands
//Start-byte; Version; amount of bytes following; Command; Feedback; Paramenter1; Parameter2; Checksum1; Checksum2; End-byte
uint8_t dfCMD[10] {0x7E, 0xFF, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF};

//initialize dfPlayer
SoftwareSerial dfPlayer(DFPLAYER_RX, DFPLAYER_TX);
//initialize the alarm display
TM1637Display alarmDisplay(ALARM_DISPLAY_CLK, ALARM_DISPLAY_DIO);
//initialize the date display
TM1637Display dateDisplay(DATE_DISPLAY_CLK, DATE_DISPLAY_DIO);
//Initializing a variable for the clock display
Adafruit_7segment clockDisplay = Adafruit_7segment();

void setup() {
  //begin serial communictaion with PC (Baudrate 115200Hz)
  Serial.begin(115200);
  Serial.setTimeout(0);

  //setting up an unused analog pin for random seeding for random playback
  randomSeed(analogRead(5));
  //begin serial communictaion with the DFPlayer
  dfPlayer.begin(9600);
  if (dfPlayer.isListening() == true) {
    Serial.println("DFPlayer is online!");
  }

  //print indicator for touch-sensed values
  Serial.println("Touch-sensed values:");
  Serial.print("raw");
  Serial.print("\t");
  Serial.print("auto");
  Serial.print("\t");
  Serial.print("cycle");
  Serial.print("\t");
  Serial.print("sum");
  Serial.print("\t");
  Serial.print("avg");
  Serial.println();

  //begin serial communictaion with the clock display on I²C Bus with Hardware ID
  clockDisplay.begin(0x70);

  //setting the brightness
  alarmDisplay.setBrightness(3, true); //up to 7
  dateDisplay.setBrightness(3, true); //up to 7
  clockDisplay.setBrightness(16); //up to 16

  //setting the pin configuration of the flip switch with Pullup resistor
  pinMode(FLIP_SWITCH, INPUT_PULLUP);

  //setting the pin configurations of the rotary encoders
  pinMode(RT1_ENCODER_CLK, INPUT);
  pinMode(RT1_ENCODER_DT, INPUT);
  pinMode(RT2_ENCODER_CLK, INPUT);
  pinMode(RT2_ENCODER_DT, INPUT);
  ec1Last = digitalRead(RT1_ENCODER_DT);
  ec2Last = digitalRead(RT1_ENCODER_DT);

  //Sync the internal clock
  setSyncProvider(RTC.get);
  setSyncInterval(3600); //sync every hour
  //print the alarm and date display
  displayPrintAlarm();
  displayPrintDate();

  DFsendCommand(0x09, 0x00, 0x02); //playback source TF-Card
  DFsendCommand(0x0F, 0x00, 0x01); //select folder 1

  touchControl.set_CS_AutocaL_Millis(10000); //autocalibration every 10s
  touchControl.set_CS_Timeout_Millis(4000); //wait max 4s
}

void loop() {
  //setting the RTC time from the serial port
  setRTCTime();
  //sensedValueing, if the encoders turned
    testRotaryEncoders();

  static unsigned long syncMillis; //refrence for calling the subroutine below every 5s
  if ((millis() - syncMillis) >= 5000) {
    utc = now(); //set utc time
    local = DE.toLocal(utc, &tcr); //convert utc to local
    static unsigned long alarmStartMillis;
    //print time
    clockDisplayPrint();
    if (playing == false) {
      alarmStartMillis = testForAlarm(); //set time refrence to start of alarm
    }

    //increase volume by one every 0,2s after start of alarm
    static int j = 1;
    if (playing == true) {
      if (j < 30) {
        if (millis() - alarmStartMillis >= (j * 200)) {
          DFsendCommand(0x04, 0x00, 0x00); //increase volume by one
          j++;
        }
      }
      else {
        playing = false; //set playing to false after timeout
      }
    }
    else {
      j = 1; //set j back to 1
    }

    syncMillis = millis(); //time refrence for calling this subroutine
    displayPrintDate(); //print date
  }
  //test for touch every 125ms if the encoders were not in use the last 2,5s
  static unsigned long touchMillis;
  if ((millis() - touchMillis) >= 125 && millis() - encoderMillis >= 2500) {
    testForTouching(); //test if device got touched
    touchMillis = millis(); //time refrence
  }
}

void testForTouching() {
  //get touch sensor value
  unsigned long sensedValue = touchControl.capacitiveSensor(100);

  //stop playback if being touched
  if (sensedValue > 3750 && playing == true) {
    DFsendCommand(0x0E, 0x00, 0x00); //stop Playback
    playing = false;
  }

  touchCalibration(sensedValue); //calibrate touch values

  //higher the brightness at night when touched
  if (hour(local) >= 21 || hour(local) <= 10) {
    if (sensedValue >= 4000 && playing == false && displayON == false) {
      alarmDisplay.setBrightness(2, true);
      dateDisplay.setBrightness(2, true);
      clockDisplay.setBrightness(12);
      displayON = true;
      encoderMillis = millis();
    }
    //lower the brightness at night
    else if (displayON == true && playing == false) {
      alarmDisplay.setBrightness(2, false);
      dateDisplay.setBrightness(2, false);
      clockDisplay.setBrightness(0);
      displayON = false;
    }
  }
  //higher the brightness at day
  else if (displayON == false) {
    alarmDisplay.setBrightness(3, true); //up to 7
    dateDisplay.setBrightness(3, true); //up to 7
    clockDisplay.setBrightness(16); //up to 16
    displayON = true;
  }

  //print time, date and alarm
  clockDisplayPrint();
  displayPrintDate();
  displayPrintAlarm();
}

//method for calibrating touch values
void touchCalibration(unsigned long sensedValue) {

  //initializing variables for touch sensor data smoothing and calibration
  static const int arrayLength = 20;
  static int index = 0;
  static unsigned long touchArray[arrayLength];
  static unsigned long touchAverage = 0;
  static unsigned long touchTotal = 0;

  touchTotal = touchTotal - touchArray[index];
  touchArray[index] = sensedValue;
  touchTotal = touchTotal + touchArray[index];
  touchAverage = touchTotal / arrayLength;

  //printing different values for debugging
  Serial.print(touchControl.capacitiveSensorRaw(100));
  Serial.print("\t");
  Serial.print(sensedValue);
  Serial.print("\t");
  Serial.print(index);
  Serial.print("\t");
  Serial.print(touchTotal);
  Serial.print("\t");
  Serial.println(touchAverage);


  index++; //increase array index by one
  if (index >= arrayLength)
    index = 0; //reset to zero
  //recalibrate if average value is too high, max every 20s
  static unsigned long calibrationMillis;
  if (touchAverage > 3500 && millis() - calibrationMillis >= 20000) {
    touchControl.reset_CS_AutoCal(); // calibrate via library
    calibrationMillis = millis();
  }
}

//testing for rotary encoder movement
void testRotaryEncoders() {
  ec1Val = digitalRead(RT1_ENCODER_DT);
  if (ec1Val != ec1Last) {
    //the encoder is rotating
    if (digitalRead(RT1_ENCODER_CLK) == ec1Val) //clockwise, because A changed first
      amin++;
    else
      amin--;
    ec1Last = ec1Val;
    encoderMillis = millis();
    //print alarm time and set high brightness
    alarmDisplay.setBrightness(3, true);
    clockDisplay.setBrightness(16);
    displayON = true;
    displayPrintAlarm();
  }

  ec2Val = digitalRead(RT2_ENCODER_DT);
  // TODO: sensedValue while loop, instead of if
  if (ec2Val != ec2Last) {
    //the encoder is rotating
    static short i = 0;

    if (digitalRead(RT2_ENCODER_CLK) == ec2Val) //clockwise, because A changed first
      i++;
    else //counter clockwise, because B changed first
      i--;
    ec2Last = ec2Val;
    encoderMillis = millis();

    //increasing hour only every two steps
    if (i%2 == 0) {
      ahour = ahour + (i/2);
      i = 0;
    }
    //print alarm time and set high brightness
    alarmDisplay.setBrightness(3, true);
    clockDisplay.setBrightness(16);
    displayON = true;
    displayPrintAlarm();
  }
}

//display the time
void clockDisplayPrint() {
  //make one variable out of hours and minutes
  int timeValue = hour(local)*100 + minute(local);
  //display the given time with dots
  clockDisplay.print(timeValue, DEC);
  //Serial.println(timeValue);
  //print zeros when given hour is <10 || <1
  if (timeValue < 1000)
    clockDisplay.writeDigitNum(0, 0);
  if (timeValue < 100)
    clockDisplay.writeDigitNum(1, 0);
  if (timeValue < 10)
    clockDisplay.writeDigitNum(3, 0);
  clockDisplay.drawColon(true); //print with colon
  clockDisplay.writeDisplay(); //write to I²C Bus
}

/*  code to process time sync messages from the serial port
on linux, type: stty -F /dev/ttyACM0 cs8 115200 ignbrk -brkint -imaxbel -opost -onlcr -isig
 -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts && date +T%s > /dev/ttyACM0 */
#define TIME_HEADER  'T' // Header tag from the message
//setting the RTC from the serial port
void setRTCTime() {
  if (Serial.available()) { //test if serial communication is established
    if (Serial.find(TIME_HEADER)) { //look for the "T"
      unsigned long pcTime = Serial.parseInt(); //parse the integer
      RTC.set(pcTime); //set RTC with UNIX time
      setTime(pcTime); //set local time with UNIX time
      Serial.println(pcTime);
    }
  }
}

//prevent hour and minute variable from having impossible values
void alarmConversion() {
  if (ahour >= 24)
    ahour = 0;
  if (ahour <= -1)
    ahour = 23;

  if (amin >= 60) {
    amin = 0;
    ahour++;
  }
  if (amin <= -1) {
    amin = 59;
    ahour--;
  }
}

//check if the alarm should ring
unsigned long testForAlarm() {
  if (ahour == hour(local) && amin == minute(local) && (millis() - encoderMillis) >= 5000 && digitalRead(FLIP_SWITCH) == LOW) {
    Serial.println("ALARM!");
    DFsendCommand(0x03, 0x00, random(0, SONG_LIMIT)); //selecting random song
    DFsendCommand(0x01, 0x00, 0x00); //play song
    DFsendCommand(0x06, 0x00, 0x01); //specify volume to level 1

    playing = true;
    alarmDisplay.setBrightness(3, true); //up to 7
    dateDisplay.setBrightness(3, true); //up to 7
    clockDisplay.setBrightness(16); //up to 16
    displayON = true;

    //print time, date and alarm
    clockDisplayPrint();
    displayPrintDate();
    displayPrintAlarm();

    return millis(); //return for time refrence
  }
}

void displayPrintDate() {
  int date = day(local) * 100 + month(local); //make one variable out of day and month
  dateDisplay.showNumberDecEx(date, 0x40, true, 4, 0); //print with dot
}
//display the alarm time
void displayPrintAlarm() {
  alarmConversion();
  alarmDisplay.showNumberDecEx(ahour, 0x10, true, 2, 0); //print alarm hours with dot
  alarmDisplay.showNumberDecEx(amin, 0x10, true, 2, 2); //print alarm minutes with dot
}

//method for calculating the checksum
void DFchecksum() {
  uint16_t sum = 0;
  for (int j = 1; j <= 6; j++) {
    sum += dfCMD[j]; //adding all bytes, except start and end byte
  }
  sum = (0-sum); //further processing the sum, like in the original library

  //splitting the 16-bit number into two 8-bit
  dfCMD[7] = (uint8_t) (sum >> 8);
  dfCMD[8] = (uint8_t) (sum);
}

//method for sending commands to the DFPlayer
void DFsendCommand(uint8_t cmd, uint8_t para1, uint8_t para2) {
  dfCMD[3] = cmd;
  dfCMD[5] = para1;
  dfCMD[6] = para2;
  DFchecksum();
  dfPlayer.write(dfCMD, 10); //write on serial bus
  delay(50); //delay for 50ms
}
