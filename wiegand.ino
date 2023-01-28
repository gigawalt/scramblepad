/*
 * Use Scramblepad as an authentication device
 * Require reading of an RFID card + PIN
 * Known bugs: 
 *    - a card whose serial ends with 0xB cannot be distinguished from a PIN that contains the other bytes, so we do not allow these
 *    - a PIN with even number of digits can also be entered with a trailing '0'
 * Walter Belgers <walter@belge.rs>
 * 20230128
 */

#include <Wiegand.h>

// User changeable settings
int opentime = 3; // door open time in seconds

// User definition (ugly coding, can probably be done better)
// names<uid> : username (only used for logging)
// card<uid>[]: 8-byte HEX card id, in 2-byte parts
// code<uid>[]: PIN code, in 2-byte parts, add 0xB as last character
//              if PIN is even number of bytes, add leading 0
//              example: PIN 1234 would be 0x01, 0x23, 0x4B
//              PIN length minimum 1 character, no maximum
//              lower left button can be included as 0xA 
int numusers = 2;
String   names[] = {"(not used)", "Walter", "gigawalt"};
uint8_t  card1[] = { 0x01, 0x3E, 0x82, 0xA8}; // Walter
uint8_t  card2[] = { 0xCA, 0x7D, 0x03, 0xB1}; // gigawalt
uint8_t *cards[] = { NULL, &card1[0], &card2[0] };
uint8_t  code1[] = { 0x01, 0x23, 0x4B}; // Walter PIN 123
uint8_t  code2[] = { 0x01, 0x23, 0x4B}; // gigawalt PIN 1234
uint8_t *codes[] = { NULL, &code1[0], &code2[0] };
// End user changeable settings


// These are the pins connected to the Wiegand D0 and D1 signals.
// Ensure your board supports external Interruptions on these pins
#define PIN_D0 2
#define PIN_D1 3

// global variable keeping track of whose card is read
int card;
// global variable keeping state of door
bool dooropen;

// timeticker to keep track of time on when to close the lock
int timeticker = 0;

// The object that handles the wiegand protocol
Wiegand wiegand;

// Initialize Wiegand reader
void setup() {
  Serial.begin(9600);

  //Install listeners and initialize Wiegand reader
  wiegand.onReceive(receivedData, ""); // not used for Scamblepad
  wiegand.onReceiveError(receivedDataError, "");
  wiegand.onStateChange(stateChanged, "State changed: ");
  wiegand.begin(Wiegand::LENGTH_ANY, true);

  //initialize pins as INPUT and attaches interruptions
  pinMode(PIN_D0, INPUT);
  pinMode(PIN_D1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0), pinStateChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1), pinStateChanged, CHANGE);

  //Sends the initial pin state to the Wiegand library
  pinStateChanged();

  // initialize digital pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // start in state that no valid card was swiped
  card = 0;         // no card seen
  dooropen = false; // door closed
  closeDoor();
}

// Every few milliseconds, check for pending messages on the wiegand reader
// This executes with interruptions disabled, since the Wiegand library is not thread-safe
void loop() {
  timeticker++;
  if (timeticker == 10*opentime) { // 3 seconds
    closeDoor();
    timeticker = 0;
  }
  noInterrupts();
  wiegand.flush();
  interrupts();
  //Sleep a little -- this doesn't have to run very often.
  delay(100);
}

// When any of the pins have changed, update the state of the wiegand library
void pinStateChanged() {
  wiegand.setPin0State(digitalRead(PIN_D0));
  wiegand.setPin1State(digitalRead(PIN_D1));
}

// Notifies when a reader has been connected or disconnected.
// Instead of a message, the seconds parameter can be anything you want -- Whatever you specify on `wiegand.onStateChange()`
void stateChanged(bool plugged, const char* message) {
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
    if (!plugged) {
      Serial.println("Lost Scramblepad - shutting down (tamper alert)");
      detachInterrupt(digitalPinToInterrupt(PIN_D0));
      detachInterrupt(digitalPinToInterrupt(PIN_D1));
    }
}

// Notifies when a card was read.
// Instead of a message, the seconds parameter can be anything you want -- Whatever you specify on `wiegand.onReceive()`
void receivedData(uint8_t* data, uint8_t bits, const char* message) {
    Serial.print("This should not happen: ");
    Serial.print(message);
    Serial.print(bits);
    Serial.print("bits / ");
    //Print value in HEX
    uint8_t bytes = (bits+7)/8;
    for (int i=0; i<bytes; i++) {
        Serial.print(data[i] >> 4, HEX);
        Serial.print(data[i] & 0xF, HEX);
    }
    Serial.println();
}

// All messages from the Scramblepad are flagged as DataError, process them here
void receivedDataError(Wiegand::DataError error, uint8_t* rawData, uint8_t rawBits, const char* message) {
    // determine type (card or PIN), for card length should be 32 bits and cannot end in 0xB (the "ENTER" for a PIN)
    if ( (rawBits == 32) && ((rawData[3] & 0xF) != 0xB) ) {
      Serial.print("Card presented with id ");
      for (int i=0; i<4; i++) {
          Serial.print(rawData[i] >> 4, HEX);
          Serial.print(rawData[i] & 0xF, HEX);
      }
      card=0; // forget possible earlier card
      // check whose card it is
      for (int i=1; i<=numusers; i++) { // start with i==1, because 0 is reserved for "no valid card read"
        if ( *rawData == *cards[i] ) {
          card=i;
        }
      }
      if (card==0) { Serial.println(" - unknown card ignored"); } else { Serial.print(" - user "); Serial.println(names[card]); }
    } else {
      // PIN presented (or card with last byte 0xB, which will not work)
      Serial.print("PIN entered: ");
      uint8_t bytes = (rawBits+7)/8; 
      for (int i=0; i<bytes; i++) {
          Serial.print(rawData[i] >> 4, HEX);
          Serial.print(rawData[i] & 0xF, HEX);
      }
      if (card > 0 && *rawData == *codes[card]) {
        Serial.print(" - user "); Serial.println(names[card]);
        // use LED for the moment to signal correct credentials
       openDoor();
      } else {
        Serial.println(" - unknown PIN or correct card not seen");
      }
      card = 0; // either way, reset state
    } 
}

void openDoor() {
  // TODO actually open the door
  Serial.println("Door is opened");
  digitalWrite(LED_BUILTIN, HIGH);
  timeticker = 0;
  dooropen = true;
}

void closeDoor() {
  // TODO actually close the door
  if (dooropen == true) { Serial.println("Door is closed"); }
  digitalWrite(LED_BUILTIN, LOW);
  dooropen = false;
}
