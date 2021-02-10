#include <ResponsiveAnalogRead.h>
#include <Bounce.h>

// OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins) - pin 17-18 (A3-A4)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// the MIDI channel number to send messages
const int channel = 1;
const int cchigh = 127;
const int cclow = 0;

int isRec = 0;
int isDub = 0;

ResponsiveAnalogRead a0(A0, true);
ResponsiveAnalogRead a1(A1, true);
ResponsiveAnalogRead a2(A2, true);
ResponsiveAnalogRead a3(A3, true);
ResponsiveAnalogRead a6(A6, true);
ResponsiveAnalogRead a7(A7, true);
ResponsiveAnalogRead a8(A8, true);
ResponsiveAnalogRead a9(A9, true);
ResponsiveAnalogRead a14(A14, true);
int previousA0 = -1;
int previousA1 = -1;
int previousA2 = -1;
int previousA3 = -1;
int previousA6 = -1;
int previousA7 = -1;
int previousA8 = -1;
int previousA9 = -1;
int previousA14 = -1;


// Create Bounce objects for each button.  The Bounce object
// automatically deals with contact chatter or "bounce", and
// it makes detecting changes very simple.
Bounce button1 = Bounce(1, 5);  // 5 = 5 ms debounce time
Bounce button2 = Bounce(2, 5);  // which is appropriate for good
Bounce button3 = Bounce(3, 5);  // quality mechanical pushbuttons
Bounce button4 = Bounce(4, 5);
Bounce button5 = Bounce(5, 5);  // if a button is too "sensitive"
Bounce button6 = Bounce(6, 5);  // to rapid touch, you can
Bounce button7 = Bounce(7, 5);  // increase this time.
Bounce button8 = Bounce(8, 5);
Bounce button9 = Bounce(9, 5);
Bounce button10 = Bounce(10, 5);
Bounce button12 = Bounce(12, 5);

// countdown
#define COUNTDOWN_TIME  48000ul
//#define COUNTDOWN_TIME  100ul

// n14
String line1;
String line2;

char
    szString[20];
byte
    mins, secs;
       
unsigned long
    timeTemp,
    timeNow,
    timeStart,
    timeElapsed,
    timeLeft;

bool countDownDone = 0;

void setup() {

  //OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    for(;;); // Don't proceed, loop forever
  }
  
  pinMode(0, OUTPUT);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  digitalWrite(0, LOW);

  // countdown
  timeStart = millis();
  mins = 1;
  secs = 1;
  
  jasSplash();    // Draw 'stylized' characters
}

void jasSplash(void) {
  display.clearDisplay();

  display.setTextSize(6);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("P1"));
  display.display();
  delay(2); // set as long as soft is not ready (20000)?
}

void loop() {
  
  // ANALOG
  a0.update();
  a1.update();
  a2.update();
  a3.update();
  a6.update();
  a7.update();
  a8.update();
  a9.update();
  a14.update();
  int n0 = a0.getValue() / 8;
  if (n0 != previousA0) {
    usbMIDI.sendControlChange(20, n0, channel);
    previousA0 = n0;
  }
  int n1 = a1.getValue() / 8;
  if (n1 != previousA1) {
    usbMIDI.sendControlChange(21, n1, channel);
    previousA1 = n1;
  }
  int n2 = a2.getValue() / 8;
  if (n2 != previousA2) {
    usbMIDI.sendControlChange(22, n2, channel);
    previousA2 = n2;
  }
  int n3 = a3.getValue() / 8;
  if (n3 != previousA3) {
    usbMIDI.sendControlChange(25, n3, channel);
    previousA3 = n3;
  }
  int n6 = a6.getValue() / 8;
  if (n6 != previousA6) {
    usbMIDI.sendControlChange(26, n6, channel);
    previousA6 = n6;
  }
  int n7 = a7.getValue() / 8;
  if (n7 != previousA7) {
    usbMIDI.sendControlChange(27, n7, channel);
    previousA7 = n7;
  }
  int n8 = a8.getValue() / 8;
  if (n8 != previousA8) {
    usbMIDI.sendControlChange(28, n8, channel);
    previousA8 = n8;
  }
  int n9 = a9.getValue() / 8;
  if (n9 != previousA9) {
    usbMIDI.sendControlChange(29, n9, channel);
    previousA9 = n9;
  }
  int n14 = a14.getValue() / 8;
  if (n14 != previousA14) {
    usbMIDI.sendControlChange(23, n14, channel);
    previousA14 = n14;
  }

  //DIGITAL
  button1.update();
  button2.update();
  button3.update();
  button4.update();
  button5.update();
  button6.update();
  button7.update();
  button8.update();
  button9.update();
  button10.update();
  button12.update();

  if (button1.fallingEdge()) {
    usbMIDI.sendControlChange(1, cclow, channel);
  }
  if (button2.fallingEdge()) {
    usbMIDI.sendControlChange(2, cclow, channel);
  }
  if (button3.fallingEdge()) {
    usbMIDI.sendControlChange(3, cclow, channel);
  }
  if (button4.fallingEdge()) {
    usbMIDI.sendControlChange(4, cclow, channel);
  }
  if (button5.fallingEdge()) {
    usbMIDI.sendControlChange(5, cclow, channel);
  }
  if (button6.fallingEdge()) {
    usbMIDI.sendControlChange(6, cclow, channel);
  }
  if (button7.fallingEdge()) {
    usbMIDI.sendControlChange(14, cclow, channel);
    if(isDub == 0) {
      isDub = 1;
      digitalWrite(0, HIGH);
    } else {
      isDub = 0;
      digitalWrite(0, LOW);
    }
  }
  if (button8.fallingEdge()) {
    usbMIDI.sendControlChange(8, cclow, channel);
  }
  if (button9.fallingEdge()) {
    usbMIDI.sendControlChange(9, cclow, channel);
  }
  if (button10.fallingEdge()) {
    usbMIDI.sendControlChange(10, cclow, channel);
    if(isRec == 0) {
      isRec = 1;
      digitalWrite(0, HIGH);
    } else {
      isRec = 0;
      digitalWrite(0, LOW);
    }
  }
  if (button12.fallingEdge()) {
    usbMIDI.sendControlChange(12, cclow, channel);
  }

  // Check each button for "rising" edge
  if (button1.risingEdge()) {
    usbMIDI.sendControlChange(1, cchigh, channel);
  }
  if (button2.risingEdge()) {
    usbMIDI.sendControlChange(2, cchigh, channel);
  }
  if (button3.risingEdge()) {
    usbMIDI.sendControlChange(3, cchigh, channel);
  }
  if (button4.risingEdge()) {
    usbMIDI.sendControlChange(4, cchigh, channel);
  }
  if (button5.risingEdge()) {
    usbMIDI.sendControlChange(5, cchigh, channel);
  }
  if (button6.risingEdge()) {
    usbMIDI.sendControlChange(6, cchigh, channel);
  }
  if (button7.risingEdge()) {
    usbMIDI.sendControlChange(14, cchigh, channel);
  }
  if (button8.risingEdge()) {
    usbMIDI.sendControlChange(8, cchigh, channel);
  }
  if (button9.risingEdge()) {
    usbMIDI.sendControlChange(9, cchigh, channel);
  }
  if (button10.risingEdge()) {
    usbMIDI.sendControlChange(10, cchigh, channel);
  }
  if (button12.risingEdge()) {
    usbMIDI.sendControlChange(12, cchigh, channel);
  }


  if(!countDownDone) {
    DoCountdown();
  } else {
    if(isRec) {
        line1 = "*REC*";
        line2 = "*****";
    } else if(isDub) {
        line1 = "*DUB*";
        line2 = "*****";
    } else {
      if(n14 >= 0 && n14 <= 10) {
        line1 = "Loop";
        line2 = "1";
      } else if(n14 >= 11 && n14 <= 20) {
        line1 = "Loop";
        line2 = "2";
      } else if(n14 >= 21 && n14 <= 30) {
        line1 = "Loop";
        line2 = "3";
      } else if(n14 >= 31 && n14 <= 40) {
        line1 = "Loop";
        line2 = "4";
      } else if(n14 >= 41 && n14 <= 50) {
        line1 = "FX1";
        line2 = "Synth";
      } else if(n14 >= 51 && n14 <= 60) {
        line1 = "FX2";
        line2 = "Synth";
      } else if(n14 >= 61 && n14 <= 70) {
        line1 = "FX1";
        line2 = "ADC";
      } else if(n14 >= 71 && n14 <= 80) {
        line1 = "FX2";
        line2 = "ADC";
      } else if(n14 >= 81 && n14 <= 90) {
        line1 = "Generator";
        line2 = "";
      } else if(n14 >= 91 && n14 <= 100) {
        line1 = "Utils";
        line2 = "";
      } else if(n14 >= 101 && n14 <= 110) {
        line1 = "Samp-EQ";
        line2 = "";
      } else if(n14 >= 111 && n14 <= 120) {
        line1 = "Arp";
        line2 = "Seq";
      } else if(n14 >= 121 && n14 <= 127) {
        line1 = "Mixer";
        line2 = "";
      }
    }
    display.clearDisplay();
    display.setTextSize(4);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.println(line1);
    display.setCursor(0,36);             // Start at top-left corner
    display.println(line2);
    display.display();  
  }
  

  // MIDI Controllers should discard incoming MIDI messages.
  // http://forum.pjrc.com/threads/24179-Teensy-3-Ableton-Analog-CC-causes-midi-crash
  while (usbMIDI.read()) {
    // ignore incoming messages
  }
}


void DoCountdown()
{
    display.clearDisplay();
    static unsigned long
        lastTimeNow = 0;
    static byte
        lastsecs = 1;
       
    timeNow = millis();
    timeElapsed = timeNow - timeStart;
   
    if( mins == 0 && secs == 0 )
        return;
       
    timeLeft = COUNTDOWN_TIME - timeElapsed;

    mins = (byte)(timeLeft / 60000ul);
    timeTemp = timeLeft - (mins * 60000);
    secs = (byte)(timeTemp / 1000ul);
    timeTemp = timeTemp - (secs * 1000ul);

    if( mins == 0 && secs == 0 ) {
        display.setTextSize(6);             // Normal 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.println(F("JAS"));
        display.display();
        countDownDone = 1;
    } else if( secs != lastsecs ) {
        lastsecs = secs;
        display.setTextSize(4);
        display.setCursor(0,0);
        sprintf( szString, "%02d:%02d", mins, secs );
        //display.setCursor( 50, 30 );
        display.print( szString );
        display.setCursor(0,36);             // Start at top-left corner
        display.println("<-OFF");

        display.display();
    }
}
