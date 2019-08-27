
#include "usb_dev.h"
#include "usb_desc.h"
#include "core_pins.h"
#include <string.h>

#define TX_PACKET_LIMIT 3

struct Encoder
{
  uint8_t pinA;
  uint8_t pinB;
  int pinAPrevious;
  int pinBPrevious;
  float pos;
  uint8_t direction;
};
struct Encoder panWheel;


static uint8_t transmit_previous_timeout=0;

// When the PC isn't listening, how long do we wait before discarding data?
#define TX_TIMEOUT_MSEC 30

#if F_CPU == 240000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1600)
#elif F_CPU == 216000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1440)
#elif F_CPU == 192000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1280)
#elif F_CPU == 180000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1200)
#elif F_CPU == 168000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1100)
#elif F_CPU == 144000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 932)
#elif F_CPU == 120000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 764)
#elif F_CPU == 96000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 596)
#elif F_CPU == 72000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 512)
#elif F_CPU == 48000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 428)
#elif F_CPU == 24000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 262)
#endif

uint8_t usb_ifc_data[IFC_TX_SIZE];


static usb_packet_t *rx_packet=NULL;

void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
  encoder->pinA = pinA;
  encoder->pinB = pinB;
  encoder->pos = 0;
  encoder->direction = direction;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  encoder->pinAPrevious = digitalRead(pinA);
  encoder->pinBPrevious = digitalRead(pinB);
}

int8_t updateEncoder(struct Encoder* encoder)
{
  int8_t encoderMotion = 0;
  int pinACurrent = digitalRead(encoder->pinA);
  int pinBCurrent = digitalRead(encoder->pinB);

  // has the encoder moved at all?
  if (encoder->pinAPrevious != pinACurrent)
  {
    // Since it has moved, we must determine if the encoder has moved forwards or backwards
    encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

    // If we are in reverse mode, flip the direction of the encoder motion
    if (encoder->direction == 1)
      encoderMotion = -encoderMotion;
  }
  encoder->pinAPrevious = pinACurrent;
  encoder->pinBPrevious = pinBCurrent;

  return encoderMotion;
}


void setup() {
  initEncoder(&panWheel, 31, 32, 0);
}

void loop() {

    while(usb_ifc_available()) {
      usb_ifc_read_message();
    }

    int32_t panMotion = updateEncoder(&panWheel);
    
    if (panMotion != 0)
    {
      eos_encoder_nudge_send(0, (panMotion > 0));
    }
}

static int encoder_locations[4] = {8,7,6,5};

void eos_encoder_nudge_send(int encoder_num, bool up)
{
    usb_ifc_data[0] = 0x03;
    usb_ifc_data[2] = 0x01;
    usb_ifc_data[4] = 0x04;
    
    for (int i=0; i<4; i++) {
      usb_ifc_data[encoder_locations[i]] = 0x00;
    }

    usb_ifc_data[encoder_locations[encoder_num]] = up ? 0x01 : 0xff;
    
    uint32_t wait_count=0;
    usb_packet_t *tx_packet;

    while (1) {
            if (!usb_configuration) {
                    return;
            }
            if (usb_tx_packet_count(IFC_TX_ENDPOINT) < TX_PACKET_LIMIT) {
                    tx_packet = usb_malloc();
                    if (tx_packet) break;
            }
            if (++wait_count > TX_TIMEOUT || transmit_previous_timeout) {
                    transmit_previous_timeout = 1;
                    return;
            }
            yield();
        }
  transmit_previous_timeout = 0;
  memcpy(tx_packet->buf, usb_ifc_data, IFC_TX_SIZE);
  tx_packet->len = IFC_TX_SIZE;
  usb_tx(IFC_TX_ENDPOINT, tx_packet);
}




uint32_t usb_ifc_available(void)
{
  uint32_t index;

  if (!rx_packet) {
    if (!usb_configuration) return 0;
    rx_packet = usb_rx(IFC_RX_ENDPOINT);
    if (!rx_packet) return 0;
    if (rx_packet->len == 0) {
      usb_free(rx_packet);
      rx_packet = NULL;
      return 0;
    }
  }
  index = rx_packet->index;
  return rx_packet->len - index;
}

uint32_t usb_ifc_read_message(void)
{
  uint32_t n, index;

  if (!rx_packet) {
    if (!usb_configuration) return 0;
    rx_packet = usb_rx(IFC_RX_ENDPOINT);
    if (!rx_packet) return 0;
    if (rx_packet->len == 0) {
      usb_free(rx_packet);
      rx_packet = NULL;
      return 0;
    }
  }
  index = rx_packet->index;
  n = ((uint32_t *)rx_packet->buf)[index/4];
  index += 4;
  if (index < rx_packet->len) {
    rx_packet->index = index;
  } else {
    usb_free(rx_packet);
    rx_packet = usb_rx(IFC_RX_ENDPOINT);
  }
  return n;
}

