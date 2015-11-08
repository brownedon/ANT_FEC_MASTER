#include <SoftwareSerial.h>

double RIDER_WEIGHT = 80; //kg's
//
//master transmits data at intervals
//
// From antmessage.h
#define UCHAR unsigned char
#define MESG_TX_SYNC                      ((UCHAR)0xA4)
#define MESG_ASSIGN_CHANNEL_ID            ((UCHAR)0x42)
#define MESG_CHANNEL_MESG_PERIOD_ID       ((UCHAR)0x43)
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID    ((UCHAR)0x44)
#define MESG_CHANNEL_RADIO_FREQ_ID        ((UCHAR)0x45)
#define MESG_NETWORK_KEY_ID               ((UCHAR)0x46)
#define MESG_SYSTEM_RESET_ID              ((UCHAR)0x4A)
#define MESG_OPEN_CHANNEL_ID              ((UCHAR)0x4B)
#define MESG_CHANNEL_ID_ID                ((UCHAR)0x51)
#define MESG_RESPONSE_EVENT_ID            ((UCHAR)0x40)
#define MESG_BROADCAST_DATA_ID            ((UCHAR)0x4E)
#define MESG_ACKNOWLEDGE_DATA_ID          ((UCHAR)0x4F)
#define MESG_CAPABILITIES_ID              ((UCHAR)0x54)

#define DEBUG 0

#define ANT_CHAN           0
#define ANT_NET            0    // public network
#define ANT_TIMEOUT       256   // 12 * 2.5 = 30 seconds
#define ANT_DEVICETYPE    17  //0x11  
#define ANT_FREQ          57    // Garmin radio frequency  0x39
#define ANT_PERIOD        8192    // Garmin search period
#define ANT_NETWORKKEY {0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45}

//fe-c
//#define ANT_SPORT_FITNESS_EQUIPMENT_PERIOD 8192
#define ANT_SPORT_FITNESS_EQUIPMENT_TYPE 0x11
//#define ANT_FITNESS_EQUIPMENT_FREQUENCY 57
//+// ant+ fitness equipment profile data pages
#define FITNESS_EQUIPMENT_GENERAL_PAGE 0x10
#define FITNESS_EQUIPMENT_TRAINER_SPECIFIC_PAGE 0x19
#define ACKNOWLEDGED_DATA_0x4F 0x4F

// For Software Serial
//RFDUINO
#define ANT_CTS              12
#define ANT_TXD              8
#define ANT_RXD              7
#define ANT_BAUD             4800 // Baud for ANT chip

#define PACKETREADTIMEOUT   100
#define MAXPACKETLEN        1000

// Global Variables
double slope = 0;
//double speed = 0;
int rxBufCnt = 0;
unsigned char rxBuf[MAXPACKETLEN];
unsigned char antNetKey[] = ANT_NETWORKKEY;

SoftwareSerial mySerial(ANT_RXD, ANT_TXD);

long packetCount = 0;
double accumPower = 0;
long last_0x10 = 0;
long time_start;
uint8_t event_counter = 0;
int send_gen = 0;
int reg_count = 0;
long last_time = 0;
uint8_t distance = 0;

enum {
  errDefault,
  errPacketSizeExceeded,
  errChecksumError,
  errMissingSync
};

void errorHandler(int errIn) {
#ifdef DEBUG
  Serial.println();
  Serial.print("Error: ");
  Serial.println(errIn);
#endif
}

unsigned char writeByte(unsigned char out, unsigned char chksum) {
#ifdef DEBUG
  Serial.print(out, HEX);
  Serial.print(" ");
#endif
  mySerial.write(out);
  chksum ^= out;
  return chksum;
}

void sendPacket(unsigned msgId, unsigned char argCnt, ...) {
  va_list arg;
  va_start (arg, argCnt);
  unsigned char byteOut;
  unsigned char chksum = 0;
  int cnt = 0;

#ifdef DEBUG
  Serial.print("TX: ");
#endif

  chksum = writeByte(MESG_TX_SYNC, chksum); // send sync
  chksum = writeByte(argCnt, chksum);       // send length
  chksum = writeByte(msgId, chksum);        // send message id

  // send data
  for (cnt = 1; cnt <= argCnt; cnt++) {
    byteOut = va_arg(arg, unsigned int);
    chksum = writeByte(byteOut, chksum);
  }
  va_end(arg);

  writeByte(chksum, chksum);                // send checksum
#ifdef DEBUG
  Serial.println();
#endif
}

void printPacket(unsigned char * packet) {
  int cnt = 0;
  while (cnt < packet[1] + 4) {
    Serial.print(packet[cnt++], HEX);
    Serial.print  (" ");
  }
  Serial.println();
}

int readPacket(unsigned char *packet, int packetSize, int readTimeout) {
  unsigned char byteIn;
  unsigned char chksum = 0;

  long timeoutExit = millis() + readTimeout;

  while (timeoutExit > millis()) {

    while (mySerial.available() > 0) {
      byteIn = mySerial.read();
      // timeoutExit = millis() + readTimeout;
      if ((byteIn == MESG_TX_SYNC) && (rxBufCnt == 0)) {
        rxBuf[rxBufCnt++] = byteIn;
        chksum = byteIn;
      } else if ((rxBufCnt == 0) && (byteIn != MESG_TX_SYNC)) {
        Serial.println("Missing sync");
        Serial.println(byteIn, HEX);
        errorHandler(errMissingSync);
        return -1;
      } else if (rxBufCnt == 1) {
        rxBuf[rxBufCnt++] = byteIn;       // second byte will be size
        chksum ^= byteIn;
      } else if (rxBufCnt < rxBuf[1] + 3) { // read rest of data taking into account sync, size, and checksum that are each 1 byte
        rxBuf[rxBufCnt++] = byteIn;
        chksum ^= byteIn;
      } else {
        rxBuf[rxBufCnt++] = byteIn;
        if (rxBufCnt > packetSize) {
          Serial.println("Packet Size Exceeded");
          errorHandler(errPacketSizeExceeded);
          return -1;
        } else {
          memcpy(packet, &rxBuf, rxBufCnt); // should be a complete packet. copy data to packet variable, check checksum and return
          packetCount++;
          if (chksum != packet[rxBufCnt - 1]) {
            Serial.println("Checksum error");
            errorHandler(errChecksumError);
            rxBufCnt = 0;
            return -1;
          } else {
            rxBufCnt = 0;
            return 1;
          }
        }
      }
    }
  }
  return 0;
}


void txMessage(uint8_t*  message, uint8_t  messageSize)
{
  uint8_t i;
  uint8_t  txBuffer[32];

  int txBufferPos   = 0;                 // set position to 0
  int txBufferSize  = messageSize + 3;   // message plus sync, size and checksum
  txBuffer[0]   = 0xa4;              // sync byte
  txBuffer[1]   = (uint8_t) messageSize - 1;    // message size - command size (1)

  for (i = 0; i < messageSize; i++)
    txBuffer[2 + i] = message[i];

  // calculate the checksum
  for (i = 0; i < txBufferSize - 1; ++i)
    txBuffer[txBufferSize - 1] = txBuffer[txBufferSize - 1] ^ txBuffer[i];

  // now send via UART
  for (i = 0; i < txBufferSize; i++) {
    mySerial.write(txBuffer[i]);
    Serial.print(txBuffer[i], HEX);
    Serial.print(":");
  }
  Serial.println("");
}

void printHeader(const char * title) {
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(packetCount);
  Serial.print(" - ");
  Serial.print(title);
}

int checkReturn() {
  byte packet[MAXPACKETLEN];
  int packetsRead;

  packetsRead = readPacket(packet, MAXPACKETLEN, PACKETREADTIMEOUT);

  // Data <sync> <len> <msg id> <channel> <msg id being responded to> <msg code> <chksum>
  // <sync> always 0xa4
  // <msg id> always 0x40 denoting a channel response / event
  // <msg code? success is 0.  See page 84 of ANT MPaU for other codes
#ifdef DEBUG
  if (packetsRead > 0) {
    Serial.print("RX: ");
    printPacket(packet);
  }
#endif

  return packetsRead;
}

//General FE Data
void send_0x10(double speed) {
  Serial.println("Send 0x10");
  //PAGE
  //0x10, 0x19, 0x02, 0x06, 0x8d, 0x20, 0x00, 0x30
  //speed  1 unit = 0.001 m/s,  hardcoded to 30 km/h
  //208d = 8333*0.001 8.3=m/s = 30 km/hr
  //distance is in meters
  long current_time = millis() - time_start;
  uint8_t elapsed = (current_time * .001 * 4) - ((round((current_time * .001 * 4) / 64)) * 64);
  speed = 8.3;  //m/s
  distance = distance + (((millis() - last_0x10) * .001) * speed);
  uint8_t dist_trav = distance - ((round(distance / 256)) * 256);

  uint8_t speedMSB =  (byte) (((int)((uint16_t)(speed * 1000)) >> 8) & 0xFF);
  uint8_t speedLSB =  (byte) ((int)((uint16_t)(speed * 1000)) & 0xFF);

  uint8_t setup_10[] = {0x4e, 0x00, FITNESS_EQUIPMENT_GENERAL_PAGE, 0x19, elapsed, dist_trav, speedLSB, speedMSB, 0x00, 0x30};
  txMessage(setup_10, 10);
  last_0x10 = millis();
}

//Specific Trainer Data
void send_0x19(double slope, double speed) {
  Serial.println("Send 0x19");
  // see http://www.gribble.org/cycling/power_v_speed.html
  //   Plegs (watts) = (1-(Lossdt/100))-1 · ( ( 9.8067 (m/s2) · W (kg) · ( sin(arctan(G/100)) + Crr · cos(arctan(G/100)) ) ) + ( 0.5 · Cd · A (m2) · Rho (kg/m3) · (V (m/s))2 ) ) · V (m/s)

  double power;
  double Lossdt = 3;
  double Crr = 0.005;
  double Rho = 1.226;
  double frontalArea = 0.509;
  double Cd = 0.63; //drag coefficient
  power = pow(1 - (Lossdt / 100), -1) * ( ( 9.8067  * RIDER_WEIGHT * ( sin(atan(slope / 100)) + Crr * cos(atan(slope / 100)) ) ) + ( 0.5 * Cd * frontalArea * Rho * (pow(speed, 2) ))) * speed;

  accumPower = accumPower + power;

  uint8_t powerMSB =  (byte) (((int)((uint16_t)power) >> 8) & 0xFF);
  uint8_t powerLSB =  (byte) ((int)((uint16_t)power) & 0xFF);

  double rollAccumPower = accumPower - ((round(accumPower / 65536)) * 65536);
  uint8_t accumPowerMSB =  (byte) (((int)((uint16_t)rollAccumPower) >> 8) & 0xFF);
  uint8_t accumPowerLSB =  (byte) ((int)((uint16_t)rollAccumPower) & 0xFF);

  uint8_t setup_19[] = {0x4e, 0x00, 0x19, event_counter, 0x50, accumPowerLSB, accumPowerMSB, powerLSB, powerMSB, 0x30};
  txMessage(setup_19, 10);
}

//FE Capabilities
void send_0x36() {
#ifdef DEBUG
  Serial.println("Send 0x36");
#endif
  //0x80=support basic resistance
  uint8_t setup_36[] = {0x4e, 0x00, 0x36, 0xff, 0xff, 0xff, 0xff, 0xbe, 0xef, 0x80};
  txMessage(setup_36, 10);
}

//Manufacturer’s Identification
void send_0x50() {
#ifdef DEBUG
  Serial.println("Send 0x50");
#endif
  uint8_t setup_50[] = {0x4e, 0x00, 0x50, 0xff, 0xff, 0x01, 0x0f, 0x00, 0x85, 0x83};
  txMessage(setup_50, 10);
}

//Product Information
void send_0x51() {
#ifdef DEBUG
  Serial.println("Send 0x51");
#endif
  uint8_t setup_51[] = {0x4e, 0x00, 0x51, 0xff, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00};
  txMessage(setup_51, 10);
}

void readPackets() {
  byte packet[MAXPACKETLEN];
  int packetsRead;
  unsigned char msgId, msgSize;
  unsigned char *msgData;

  packetsRead = readPacket(packet, MAXPACKETLEN, PACKETREADTIMEOUT);
  if (packetsRead > 0) {
    msgId = packet[2];
    msgSize = packet[1];
    msgData = &packet[3];

    switch (msgId) {

      case MESG_RESPONSE_EVENT_ID:
        //ignore success messages
        if (packet[3] != 0) {
          Serial.print("MESG_RESPONSE_EVENT_ID: ");
          printPacket(packet);
          Serial.println(packet[3], HEX);
        }
        break;

      case MESG_CAPABILITIES_ID:
        printHeader("MESG_CAPABILITIES_ID: ");
        printPacket(packet);
        break;

      case  MESG_ACKNOWLEDGE_DATA_ID:
        if (packet[4] == 0x30) {
          Serial.print("RECEIVE:Resistance:");
          Serial.println(packet[11], DEC);
        }
        if (packet[4] == 0x33) {
          word grade = ( (packet[10] << 8)
                         + (packet[9] ) );
          //NOTE Zwift needs to be doubled this may not be the case with others!!!
          slope = (((grade * 0.01) - 200) * 2);
          Serial.print("RECEIVE:Grade ");
          Serial.println(slope);

        }

        if (packet[4] == 0x46) {
          Serial.print("RECEIVE:request for page ");
          Serial.println(packet[10], HEX);
          if (packet[10] == 0x50) {
            send_0x50();
          }

          if (packet[10] == 0x36) {
            send_0x36();
          }
        }

        if (packet[4] != 0x33 && packet[4] != 0x30 && packet[4] != 0x46) {
          printPacket(packet);
        }
        break;
      case MESG_BROADCAST_DATA_ID:
        printPacket(packet);
        //note: these rollover at 256
        //6  = elapsed time  in 1/4 seconds
        //7  = distance in meters
        //10 = heart rate

        if (packet[4] == FITNESS_EQUIPMENT_TRAINER_SPECIFIC_PAGE ) {
          Serial.print("Event count:");
          Serial.println(packet[5], DEC);
        }
        if (packet[4] == FITNESS_EQUIPMENT_GENERAL_PAGE) {
          Serial.print("Distance:");
          Serial.println(packet[7], DEC);
          Serial.print("Speed:");
          word speed = ( (packet[9] << 8)
                         + (packet[8] ) );
          Serial.println(speed, DEC);
        }

        if (packet[4] == ANT_SPORT_FITNESS_EQUIPMENT_TYPE ) {
          Serial.print("Incline Percent:");
          word incline_percent = ( (packet[9] << 8)
                                   + (packet[8] ) );
          Serial.println(incline_percent, DEC);
          Serial.print("Resistance:");
          Serial.println(packet[10], DEC);
        }
        break;
      default:
        printHeader("MESG_ID_UKNOWN: ");
        printPacket(packet);
        break;
    }
  }
}


void ant_config() {
  Serial.println("Config Starting");
  delay(100);
  Serial.println("Reset");
  sendPacket(MESG_SYSTEM_RESET_ID, 1, 0);
  delay(600);

  // Flush read buffer
  while (mySerial.available() > 0) {
    mySerial.read();
  }

  // Assign Channel
  //   Channel: 0
  //   Channel Type: for Receive Channel
  //   Network Number: 0 for Public Network
  Serial.println("Assign Channel");
  while (checkReturn() == 0) {
    sendPacket(MESG_ASSIGN_CHANNEL_ID, 3, ANT_CHAN, 0x10, ANT_NET);
    delay(10);
  }

  Serial.println("Set channel id");
  // Set Channel ID
  //   Channel Number: 0
  //   Device Number LSB: 0 for a slave to match any device
  //   Device Number MSB: 0 for a slave to match any device
  //   Device Type: bit 7 0 for pairing request bit 6..0 for device type
  //   Transmission Type: 0 to match any transmission type
  while (checkReturn() == 0) {
    sendPacket(MESG_CHANNEL_ID_ID, 5, ANT_CHAN, 0, 0, ANT_DEVICETYPE, 0x05);
    delay(10);
  }

  Serial.println("Set network key");
  // Set Network Key
  //   Network Number
  //   Key
  while (checkReturn() == 0) {
    sendPacket(MESG_NETWORK_KEY_ID, 9, ANT_NET, antNetKey[0], antNetKey[1], antNetKey[2], antNetKey[3], antNetKey[4], antNetKey[5], antNetKey[6], antNetKey[7]);
    delay(10);
  }

  Serial.println("Channel Freq");
  //ANT_send(1+2, MESG_CHANNEL_RADIO_FREQ_ID, CHAN0, FREQ);
  // Set Channel RF Frequency
  //   Channel
  //   Frequency = 2400 MHz + (FREQ * 1 MHz) (See page 59 of ANT MPaU) 0x39 = 2457 MHz
  while (checkReturn() == 0) {
    sendPacket(MESG_CHANNEL_RADIO_FREQ_ID, 2, ANT_CHAN, ANT_FREQ);
    delay(10);
  }

  Serial.println("Channel Period");
  // Set Channel Period
  while (checkReturn() == 0) {
    sendPacket(MESG_CHANNEL_MESG_PERIOD_ID, 3, ANT_CHAN, (ANT_PERIOD & 0x00FF), ((ANT_PERIOD & 0xFF00) >> 8));
    delay(10);
  }

  Serial.println("Open Channel");
  //Open Channel
  while (checkReturn() == 0) {
    sendPacket(MESG_OPEN_CHANNEL_ID, 1, ANT_CHAN);
    delay(10);
  }

  delay(1000);
  Serial.println("Config Done");
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  pinMode(ANT_RXD, INPUT);
  pinMode(ANT_TXD, OUTPUT);

  mySerial.begin(ANT_BAUD);
  ant_config();
  time_start = millis();
  last_0x10 = time_start;
}

void loop() {
  int skip = 0;
  double speed = 8.3;

  //packet goes out every 250 ms, whether we send it or not
  //so even though we try to send 0x10..0x10..0x19..0x19,  THAT's not what we get
  if (millis() - last_time > 200) {
    last_time = millis();
    send_gen++;

    if (send_gen == 60 || send_gen == 61) {
      send_0x50();
      skip = 1;
    }

    if (send_gen == 126 || send_gen == 127) {
      send_0x51();
      skip = 1;
      if (send_gen >= 127) {
        send_gen = 0;
      }
    }

    if (skip == 0) {
      reg_count++;
      event_counter++;
      if (reg_count == 1 || reg_count == 2) {
        send_0x19(slope, speed);
      }

      if (reg_count == 3 || reg_count == 4) {
        send_0x10(speed);
      }

      if (reg_count >= 4)
        reg_count = 0;
    }
  }

  readPackets();


}
