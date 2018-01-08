#include <SPI.h>
#include <RH_RF69.h>
#include <ArduinoJson.h>
#include <QList.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

#define NS 8
// int16_t packetnum = 0;  // packet counter, we increment per xmission
int myId = 0;
QList<int> neighbors;
int8_t mode = 0;


StaticJsonBuffer<400> jsonBuffer;

JsonObject& outMsg0 = jsonBuffer.createObject();
JsonArray& outNodes0 = outMsg0.createNestedArray("nodes");

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  delay(10000);

  randomSeed(analogRead(0));
  myId = (int)random(1, 251);
  neighbors.clear();
  neighbors.push_front((int)77);
  neighbors.push_front((int)myId);
  neighbors.push_front((int)88);
  neighbors.clear();
  neighbors.push_front((int)myId);
  outNodes0.add((int)myId);

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
  Blink(LED, 50, 2);
  delay(200);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  delay(2000);
}

void loop() {
  delay(100);  // Wait 0.1 second between transmits, could also 'sleep' here!

  if (mode == 0) {
    sendNodesMsg();
    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = RH_RF69_MAX_MESSAGE_LEN; // sizeof(buf);
  
    if (rf69.waitAvailableTimeout(500))  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
        buf[len] = '\0';
        Serial.print("Got a reply len ");
        Serial.println(len);
        Serial.print("Got a reply: ");
        Serial.println((char*)buf);
        int other = getFirstNode(buf);

        // merge the first of the others into our list of neighbors.
        int added_count = 0;
        if (other > 0) {
          if (neighbors.indexOf(other) == -1) {
            Serial.print("adding other ");
            Serial.println(other);
            Blink(LED, 50, 8); //blink LED 8 times, 50ms between blinks
            Blink(LED, 500, 1); //blink LED 1 times, 500ms between blinks
            delay(200);
            neighbors.push_back((int)other);
            outNodes0.add((int)other);
            added_count++;
          }
          else {
            Serial.print("already have other ");
            Serial.println(other);
          }
        } 
        else {
          Serial.print("bad value of other ");
          Serial.println(other);
        }
        
        if (added_count) {
          Blink(LED, 500, 2); //blink LED 1 times, 50ms between blinks
          delay(20);
          // Send a return messagereturn!
          sendNodesMsg();  
        }
      } else {
          Blink(LED, 200, 5);
          delay(200);
        Serial.println("Receive failed");
      }
    } else {
      Blink(LED, 200, 8);
      delay(200);
      Serial.println("No reply, is another RFM69 listening?");
    }
  }
}

void sendNodesMsg() {
  char radiopacket[80];
  int bytes_written = outMsg0.printTo((char *)radiopacket, sizeof(radiopacket));
  if (bytes_written > 4 ) {
    Serial.print("First "); Serial.println(neighbors[0]);
    Serial.print("Sending "); Serial.println(radiopacket);
    // Send a message!
    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf69.waitPacketSent();
  }
  else {
    Serial.print("error too few bytes written "); Serial.print(radiopacket); Serial.print(" "); Serial.println(bytes_written);
  }
}

int getFirstNode(uint8_t* buf) {
  StaticJsonBuffer<1600> jsonInBuffer;
  JsonObject& inMsg0 = jsonInBuffer.parseObject(buf);
  JsonArray& inNodes0 = inMsg0["nodes"];
  if (!inMsg0.success()) {
    Serial.println("parseObject failed");
    return 0;
  }
  if (!inNodes0.success()) {
    Serial.println("parseObject nodes Array failed");
    return 0;
  }
  if (!(inNodes0[0] > 0)) {
    Serial.println("first value <= 0");
    return 0;
  }
  return inNodes0[0];
}


int copyAll(QList<int> l, int* d, int ds) {
  int s = min(ds, l.length());
  for (int i = 0; i < s; i++) {
    d[i] = (int)l[i];
  }
  return s;
}

boolean scanFor(uint8_t needle, QList<unsigned char> haystack) {
  
  for(int i = 0; i < haystack.length(); i++) {
    if (needle == haystack[i]) {
      return true;
    }
  }
  return false;
}

void Blink(byte PIN, byte delay_ms, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(delay_ms);
    digitalWrite(PIN,LOW);
    delay(delay_ms);
  }
}

