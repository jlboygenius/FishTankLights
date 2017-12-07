///////////////////////////////////////////////////////////////////
// Current Satellite LED+ Controller  V4.1                       //
//   Indychus...Dahammer...mistergreen @ plantedtank.net         //
//   This code is public domain.  Pass it on.                    //
//   Confirmed on Arduino UNO 1.0.5                              //
//   Req. Time, TimeAlarms, RTClib, IRremote                     //
///////////////////////////////////////////////////////////////////
//
// This version uses Ken Shirriff's IRremote library to Rx/Tx the IR codes
// http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html
// 
// This code does NOT use PIN 13 on the Uno, as do previous versions
// Instead PIN 3, which is a PWM pin, is used. So you'll need to connect
// your LED to PIN 3 instead of PIN 13 for it to work.
//
// You can test the IR commands via the Arduino software's serial monitor
// by sending in a value from 1 - 32. Values follow the remote control, 
// left to right, top to bottom (e.g 1 = Orange, 2 = Blue, 21 = Moon1, etc)
//
// Install LCD per instructions at http://learn.adafruit.com/character-lcds/overview
//
// Added support for 2 Thermometers but it hasn't been tested
//
#include <Wire.h>
#include <RTClib.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

RTC_DS1307 RTC;
IRsend irsend;

#define RANDPIN 0            // This needs to be set to an unused Analog pin, Used by RandomStorm()
static FILE uartout = {0};   // UART FILE structure
static FILE lcdout = {0} ;   // LCD FILE structure

//---------- LCD SETUP
#define LCD_COLS 20      // Number of columns on the LCD (e.g. 16, 20, etc)
#define LCD_ROWS 4       // Number of rows on the LCD (e.g. 2, 4, etc)
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);  // Arduino pins that the LCD is connected too

//---------- THERMOMETER SETUP
#define ONE_WIRE_BUS 2                // Ardunio pin for thermometer; comment out if you don't have a thermometer
#ifdef ONE_WIRE_BUS
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress thermometer_1 = { 0x28, 0xF9, 0x14, 0x65, 0x04, 0x00, 0x00, 0x16 }; // You'll need to change to match your sensor
//DeviceAddress thermometer_2 = { 0x28, 0xA6, 0x33, 0xA8, 0x04, 0x00, 0x00, 0x11 }; // You'll need to change to match your sensor
#endif


// Current Satellite+ IR Codes (NEC Protocol)
unsigned long codeHeader = 0x20DF; // Always the same
// Remote buttons listed left to right, top to bottom
PROGMEM const unsigned int arrCodes[32] = {0x3AC5,  // 1 -  Orange
                                     0xBA45,  // 2 -  Blue
                                     0x827D,  // 3 -  Rose
                                     0x02FD,  // 4 -  Power On/Off
                                     0x1AE5,  // 5 -  White
                                     0x9A65,  // 6 -  FullSpec
                                     0xA25D,  // 7 -  Purple
                                     0x22DD,  // 8 -  Play/Pause
                                     0x2AD5,  // 9 -  Red Up
                                     0xAA55,  // 10 - Green Up
                                     0x926D,  // 11 - Blue Up
                                     0x12ED,  // 12 - White Up
                                     0x0AF5,  // 13 - Red Down
                                     0x8A75,  // 14 - Green Down
                                     0xB24D,  // 15 - Blue Down
                                     0x32CD,  // 16 - White Down
                                     0x38C7,  // 17 - M1 Custom
                                     0xB847,  // 18 - M2 Custom
                                     0x7887,  // 19 - M3 Custom
                                     0xF807,  // 20 - M4 Custom
                                     0x18E7,  // 21 - Moon 1
                                     0x9867,  // 22 - Moon 2
                                     0x58A7,  // 23 - Moon 3
                                     0xD827,  // 24 - Dawn/Dusk
                                     0x28D7,  // 25 - Cloud 1
                                     0xA857,  // 26 - Cloud 2
                                     0x6897,  // 27 - Cloud 3
                                     0xE817,  // 28 - Cloud 4
                                     0x08F7,  // 29 - Storm 1
                                     0x8877,  // 30 - Storm 2
                                     0x48B7,  // 31 - Storm 3
                                     0xC837}; // 32 - Storm 4

// These are the messages that print on the serial monitor & lcd when each IR code is sent
#define MAX_MSG_LEN 13  // Maximum length of the arrMSG messages
const char PROGMEM arrMSG[][MAX_MSG_LEN+1] = {"Orange",        "Blue",          "Rose",          "On/Off",
                                             "White",         "Full Spec",     "Purple",        "Play/Pause",
                                             "Red Up",        "Green Up",      "Blue Up",       "White Up",
                                             "Red Down",      "Green Down",    "Blue Down",     "White Down",
                                             "Custom 1",      "Custom 2",      "Custom 3",      "Custom 4",
                                             "Moon 1",        "Moon 2",        "Moon 3",        "Dawn/Dusk",
                                             "Cloud 1",       "Cloud 2",       "Cloud 3",       "Cloud 4",
                                             "Storm 1",       "Storm 2",       "Storm 3",       "Storm 4"};

void SetAlarms()
{
  // Set up your desired alarms here
  // The default value of dtNBR_ALARMS is 6 in TimeAlarms.h.
  // This code sets 9 alarms by default, so you'll need to change dtNBR_ALARMS to 9 or more
  // Changes the times to suit yourself. Add as many alarms as you like, just stay within dtNBR_ALARMS
  Alarm.alarmRepeat(9,00,0, DawnDusk);
  //Alarm.alarmRepeat(7,30,0, Cloud2);     // (HR,MIN,SEC,FUNCTION)
  Alarm.alarmRepeat(10,00,0, FullSpec);
  //Alarm.alarmRepeat(18,00,0, Cloud2);
  Alarm.alarmRepeat(17,00,0, DawnDusk);
  Alarm.alarmRepeat(19,00,0, Moon2);
  
  // Comment these out if you don't want the chance of a random storm each day
  Alarm.alarmRepeat(12,00,00, RandomStorm);
  RandomStorm();  // Sets up intial storm so we don't have wait until alarm time

#ifdef ONE_WIRE_BUS  
  Alarm.timerRepeat(600, PrintTemp);  // Print temps every 30 minutes
  PrintTemp();                         // Display initial temps
#endif
}

void setup()
{
  Wire.begin();
  RTC.begin();
  Serial.begin(9600);
  lcd.begin(LCD_COLS, LCD_ROWS);

#ifdef ONE_WIRE_BUS
  sensors.begin();   // Start temp sensors
  sensors.setResolution(thermometer_1, 9);  // Set them to 9 bit mode
//  sensors.setResolution(thermometer_2, 9);
#endif

  // fill in the UART file descriptor with pointer to writer.
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;  // Output stdout to UART
   
  // fill in lcd file descriptor (we'll use fprintf to output to it)
  fdev_setup_stream (&lcdout, lcd_putchar, NULL, _FDEV_SETUP_WRITE);
  
  if (! RTC.isrunning()) {
    // If no RTC is installed, set time to compile time at each reset
    printf_P(PSTR("RTC is NOT running!\n"));  // Store this string in PROGMEM
    RTC.adjust(DateTime(__DATE__, __TIME__));
    }
  
  setSyncProvider(syncProvider);     // reference our syncProvider function instead of RTC_DS1307::get()
  
  printf_P(PSTR("Time: %02d:%02d:%02d\n"), hour(), minute(), second());  // Print the time
  SetAlarms();  // Set up above alarms
  
  // Print available SRAM for debugging, comment out if you want
  printf_P(PSTR("SRAM: %d\n"), freeRam());
  
  printf_P(PSTR("To test IR codes, send 1 - 32\n"));
}

void loop()
{
  if (Serial.available() > 0) {
    delay(5); //Wait for transmission to finish
    TestCodes(SerialReadInt());  // Go grab IR code and send it
  }
  Alarm.delay(100);   // Service alarms & wait (msec)
  lcd.setCursor(0,0);
  fprintf(&lcdout, "%02d:%02d", hour(), minute());  // Print the time HH:MM to the lcd
}

time_t syncProvider()
{
  //this does the same thing as RTC_DS1307::get()
  return RTC.now().unixtime();
}

void RandomStorm ()
{ 
  // Schedules a storm between 1 & 8 in the evening
  // It sets Storm2, followed by Cloud2 or DawnDusk or Moon2, depending on when the storm is over
  randomSeed(analogRead(RANDPIN));        // Generate random seed on unused pin
  byte RH = random(23);                   // Randomizer for RandomStorm
  byte RM = random(59);
  byte RS = random(59);
  byte TSDurationH = random(2);
  byte TSDurationM = random(59);
  
  lcd.setCursor(0,1);
  if (RH >= 13 && RH <= 21)
    { // If 1:00PM - 8:00PM schedule a storm
      Alarm.alarmOnce(RH,RM,RS,Storm2);
      
      // Store strings in PROGMEM
      printf_P(PSTR("Scheduled Storm: %02d:%02d:%02d Duration: %02d:%02d\n"), RH, RM, RS, TSDurationH, TSDurationM);
      fprintf_P(&lcdout, PSTR("Sch Storm: %02d:%02d"), RH, RM);
          
      if ((RH + TSDurationH) < 19)   // Switch to Cloud2 if storm ends between 1:00-6:59pm
        {
          Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,Cloud2);
        }
      else if ((RH + TSDurationH) < 21)  // Switch to DawnDusk if storm ends between 7:00-8:59pm
        {
          Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,DawnDusk);
        }
      else                               // Otherwise, switch to Night2
        {
          Alarm.alarmOnce((RH + TSDurationH),(RM + TSDurationM),RS,Moon2);
        }
    }
    else
    { // Don't really need this, but it can stay till we need the space
      printf_P(PSTR("No storm today\n"));
      fprintf_P(&lcdout, PSTR("No storm today  "));
    }
}

int SerialReadInt()
{
  // Reads first 2 bytes from serial monitor; anything more is tossed
  byte i;
  char inBytes[3];
  char * inBytesPtr = &inBytes[0];  // Pointer to first element
    
    for (i=0; i<2; i++)             // Only want first 2 bytes
      inBytes[i] = Serial.read();
    inBytes[i] =  '\0';             // Put NULL character at the end
    while (Serial.read() >= 0)      // If anything else is there, throw it away
      ; // do nothing      
    return atoi(inBytesPtr);        // Convert to decimal
}

void TestCodes (int cmd)
{
  // Handles commands sent in from the serial monitor
  if (cmd >= 1 && cmd <= 32)
    { 
      // cmd must be 1 - 32
      SendCode(cmd-1, 1);
    }
#ifdef ONE_WIRE_BUS
    else if (cmd == 33)
    {
      PrintTemp();  // Send 33 to test temp sensors
    }
#endif
    else { printf_P(PSTR("Invalid Choice\n")); }
}

void SendCode ( int cmd, byte numTimes )
{ // cmd = the element of the arrCode[] array that holds the IR code to be sent
  // numTimes = number of times to emmit the command
  // Shift header 16 bits to left, fetch code from PROGMEM & add it with bitwise or
  unsigned long irCode = (codeHeader << 16) | pgm_read_word_near(arrCodes + cmd);
  for( byte i = 0; i < numTimes; i++)
  {
    irsend.sendNEC(irCode,32); // Send/emmit code
    delay(100);
  }
  // Print the string associated with the IR code & the time
  //printf("%S: 0x%lx %02d:%02d:%02d\n", arrMSG[cmd], irCode, hour(), minute(), second());
  printf("%S: %02d:%02d:%02d\n", arrMSG[cmd], hour(), minute(), second());

  lcd.setCursor(6,0);
  fprintf(&lcdout, "%-10S", arrMSG[cmd]);
}

int freeRam ()
{
  // Returns available SRAM
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// Output function for stdout redirect
static int uart_putchar (char c, FILE *stream)
{
  // Serial write fumction
  Serial.write(c);
  return 0;
}

// Output function for lcd output
static int lcd_putchar(char ch, FILE* stream)
{
  // lcd write function
  lcd.write(ch);
  return (0);
}

#ifdef ONE_WIRE_BUS
void PrintTemp()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(thermometer_1);
  lcd.setCursor(0,2);
  lcd.print("Tank 1: ");
  lcd.print(DallasTemperature::toFahrenheit(tempC), 1);
  lcd.print((char)223);
  
//  tempC = sensors.getTempC(thermometer_2);
//  lcd.setCursor(0,3);
//  lcd.print("Tank 2: ");
//  lcd.print(DallasTemperature::toFahrenheit(tempC), 1);
//  lcd.print((char)223);
}
#endif

// IR Code functions, called by alarms
void Orange() {SendCode(0,2);}
void Blue() {SendCode(1,2);}
void Rose() {SendCode(2,2);}
void PowerOnOff() {SendCode(3,1);}
void White() {SendCode(4,2);}
void FullSpec() {SendCode(5,2);}
void Purple() {SendCode(6,2);}
void Play() {SendCode(7,1);}
void RedUp() {SendCode(8,1);}
void GreenUp() {SendCode(9,1);}
void BlueUp() {SendCode(10,1);}
void WhiteUp() {SendCode(11,1);}
void RedDown() {SendCode(12,1);}
void GreenDown() {SendCode(13,1);}
void BlueDown() {SendCode(14,1);}
void WhiteDown() {SendCode(15,1);}
void M1Custom() {SendCode(16,2);}
void M2Custom() {SendCode(17,2);}
void M3Custom() {SendCode(18,2);}
void M4Custom() {SendCode(19,2);}
void Moon1() {SendCode(20,2);}
void Moon2() {SendCode(21,2);}
void Moon3() {SendCode(22,2);}
void DawnDusk() {SendCode(23,2);}
void Cloud1() {SendCode(24,2);}
void Cloud2() {SendCode(25,2);}
void Cloud3() {SendCode(26,2);}
void Cloud4() {SendCode(27,2);}
void Storm1() {SendCode(28,2);}
void Storm2() {SendCode(29,2);}
void Storm3() {SendCode(30,2);}
void Storm4() {SendCode(31,2);}
