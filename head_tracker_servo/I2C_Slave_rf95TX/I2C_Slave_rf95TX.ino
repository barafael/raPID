#include"Wire.h"

union int_bytes {
  uint8_t b[4];
  struct yp {
    uint16_t y;
    uint16_t p;
  } yp;
} int_bytes;

#define DEBUG_OUT

// LoRa 9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example LoRa9x_RX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup()
{
  while (!Serial);
  Serial.begin(9600);

  // Start the I2C Bus as Slave on address 9
  Wire.begin(12);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  delay(100);

  Serial.println("Arduino LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  // Send a message to rf95_server

#ifdef DEBUG_OUT
  Serial.print(int_bytes.yp.p);
  Serial.print('\t');
  Serial.println(int_bytes.yp.y);
#endif

  rf95.send((uint8_t *)int_bytes.b, 4);

  rf95.waitPacketSent();
}

void receiveEvent(int bytes) {
  for (int i = 0; i < 4; i++) {
    int_bytes.b[i] = Wire.read();
  }
}

