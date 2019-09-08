/* d-diot BT-Pcb-328P-firmware - MySensors Configuration */

#define MY_BAUD_RATE (9600ul)

//#define MY_SIGNING_SOFT
//#define MY_SIGNING_REQUEST_SIGNATURES
#ifdef MY_SIGNING_ATSHA204_PIN
#define MY_SIGNING_ATSHA204
#endif

#define MY_RADIO_RFM69
#ifdef MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_IS_RFM69HW
#define MY_RFM69_FREQUENCY RFM69_868MHZ
#define MY_RFM69_RST_PIN 9
#endif

#ifndef MY_RADIO_RFM69
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_IRQ_PIN 2
#endif

#define MY_DEFAULT_ERR_LED_PIN 14
#define MY_DEFAULT_TX_LED_PIN 15
#define MY_DEFAULT_RX_LED_PIN 16
// Timeout before starting loop without gateway connection
#define MY_TRANSPORT_WAIT_READY_MS 10000
#define MY_SPLASH_SCREEN_DISABLED
static const bool ack = false;