/*      Last edit: Jan 11, 2014
 *
 * BBQduino Maverick 732 BBQ Wireless Thermometer Sniffer v0.1x
 *     Also verified to work properly with Ivation Model #IVAWLTHERM BBQ Thermometer
 *
 *    (c) 2014 B. Tod Cox, John Cox
 *    (c) 2020 Martin Koerner
 *
 * BBQduino is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BBQduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 *
 * Receives temperature data from a Maverick 732 (and clones) Wireless BBQ Thermometer
 * and outputs to an Adafruit HT1632 LED matrix as well as providing temps via a web server.
 *
 * Manchester decoding and interrupt handler based in part on code and ideas from  
 *         http://www.practicalarduino.com/projects/weather-station-receiver
 *         http://kayno.net/2010/01/15/arduino-weather-station-receiver-shield/
 *         https://forums.adafruit.com/viewtopic.php?f=8&t=25414&start=0
 *         http://wiki.openpicus.com/index.php?title=Wifi_bbq
 *     Web server & HTML/Canvas gauges sources:
 *         http://startingelectronics.com/tutorials/arduino/ethernet-shield-web-server-tutorial/  
 *         https://github.com/Mikhus/canv-gauge
 *
 *      Licenses of above works are include by reference.
 */

#include <Arduino.h>

#define RX_STATE_RESET()                  \
  {                                       \
    short_count = packet_bit_pointer = 0; \
    long_count = 0;                       \
    rxState = RX_STATE_IDLE;              \
    current_bit = BIT_ZERO;               \
  }

#define TIMER_PERIOD_US 1            // Timer resolution
#define BBQ_PACKET_BIT_LENGTH 13 * 8 //13 bytes with 2 quaternary encoded nibbles each

// pulse widths. short pulses ~500us, long pulses ~1000us. 50us tolerance
#define SHORT_PULSE_MIN_WIDTH 170
#define SHORT_PULSE_MAX_WIDTH 350
#define START_LONG_MIN_WIDTH 4000 // Long low with short 5.2ms
#define START_LONG_MAX_WIDTH 5500
#define LONG_PULSE_MIN_WIDTH 380
#define LONG_PULSE_MAX_WIDTH 650

// number of long/shorts in a row before the stream is treated as valid
#define SHORT_COUNT_SYNC_MIN 8

// states the receiver can be
#define RX_STATE_IDLE 0            // waiting for incoming stream
#define RX_STATE_READY 1           // "preamble" almost complete, next rising edge is start of data
#define RX_STATE_RECEIVING 2       // receiving valid stream
#define RX_STATE_PACKET_RECEIVED 3 // valid stream received

#define BIT_ZERO 0
#define BIT_ONE 1

#define INACTIVEVALUE 999

uint8_t rxState = RX_STATE_IDLE;
int temperatures[2] = {INACTIVEVALUE, INACTIVEVALUE};

uint tmp_probe1, tmp_probe2;
uint32_t check_data;
uint16_t chksum_data, chksum_sent, chk_xor, chk_xor_expected = 0;
uint captured_time;
uint current_bit;
uint packet_bit_pointer;
uint short_count;
uint long_count;

boolean previous_period_was_short = false;

byte BBQ_packet[(BBQ_PACKET_BIT_LENGTH / 8)];
byte BBQ_packet_process[(BBQ_PACKET_BIT_LENGTH / 8)];
byte probe1_array[6], probe2_array[6];
byte last_BBQ_packet[(BBQ_PACKET_BIT_LENGTH / 8)];

static void handleStateIdle(uint16_t delta);
static void handleStateReady(uint16_t delta);
static void handleStateReceive(uint16_t delta);
static boolean validatePacket();
static uint8_t quart(uint8_t param);
static uint16_t calculate_checksum(uint32_t data);

void bbq433Init()
{
  RX_STATE_RESET();
}

boolean bbq433CheckData(uint16_t delta)
{
  boolean validDataReceived = false;

  switch (rxState)
  {
  case RX_STATE_IDLE:
    handleStateIdle(delta);
    break;
  case RX_STATE_READY:
    handleStateReady(delta);
    break;
  case RX_STATE_RECEIVING:
    handleStateReceive(delta);
    break;
  }

  // check to see if a full packet has been received
  if (packet_bit_pointer >= BBQ_PACKET_BIT_LENGTH)
  {

    // full packet received, switch state to RX_STATE_PACKET_RECEIVED
    for (uint8_t i = 0; i < 13; i++)
    {
      BBQ_packet_process[i] = BBQ_packet[i];
    }
    validDataReceived = validatePacket();
    RX_STATE_RESET();
  }

  return validDataReceived;
}

int bbq433GetTemperature(uint8_t index)
{
  int requestedTemperature = INACTIVEVALUE;

  if(index < sizeof(temperatures))
  {
    requestedTemperature = temperatures[index];
  }

  return requestedTemperature;
}

static void handleStateIdle(uint16_t delta)
{
  if (((delta >= SHORT_PULSE_MIN_WIDTH) && (delta <= SHORT_PULSE_MAX_WIDTH)))
  {
    // short pulse, continue counting short pulses
    short_count++;
    if (short_count != long_count)
    { //should only have long (~5 ms) low followed by (~250 usec) high; if not, clear count
      short_count = 0;
      long_count = 0;
    }
    if (short_count == SHORT_COUNT_SYNC_MIN)
    { //8 long/short start pulses in row; packet with info is about to start!
      rxState = RX_STATE_READY;
    }
  }
  else if (((delta >= START_LONG_MIN_WIDTH) && (delta <= START_LONG_MAX_WIDTH)))
  {
    // long pulse. if there has been enough short pulses beforehand, we have a valid bit stream, else reset and start again
    long_count++;
  }
  else
  {
    RX_STATE_RESET();
  }
}

static void handleStateReady(uint16_t delta)
{
  // this rising edge (assuming a ~4.8msec low) is start of data
  if (((delta >= START_LONG_MIN_WIDTH) && (delta <= START_LONG_MAX_WIDTH)))
  {
    // start of data packet
    // long pulse
    // swap the currrent_bit
    current_bit = !current_bit;
    BBQ_packet[packet_bit_pointer >> 3] |= (0x80 >> (packet_bit_pointer & 0x07)); //store the bit
    packet_bit_pointer++;
    previous_period_was_short = false;
    rxState = RX_STATE_RECEIVING;
  }
  else
  {
    RX_STATE_RESET();
  }
}

static void handleStateReceive(uint16_t delta)
{
  // incoming pulses are a valid bit stream, manchester encoded. starting with a zero bit, the next bit will be the same as the
  // previous bit if there are two short pulses, or the bit will swap if the pulse is long
  if (((delta >= SHORT_PULSE_MIN_WIDTH) && (delta <= SHORT_PULSE_MAX_WIDTH)))
  {
    // short pulse
    if (previous_period_was_short)
    {
      // previous bit was short, add the current_bit value to the stream and continue to next incoming bit
      if (current_bit == BIT_ONE)
      {
        BBQ_packet[packet_bit_pointer >> 3] |= (0x80 >> (packet_bit_pointer & 0x07));
      }
      else if (current_bit == BIT_ZERO)
      {
        BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer & 0x07));
      }

      packet_bit_pointer++;

      previous_period_was_short = false;
    }
    else
    {
      // previous bit was long, remember that and continue to next incoming bit
      previous_period_was_short = true;
    }
  }
  else if (((delta >= LONG_PULSE_MIN_WIDTH) && (delta <= LONG_PULSE_MAX_WIDTH)))
  {
    current_bit = !current_bit;

    // add current_bit value to the stream and continue to next incoming bit
    if (current_bit == BIT_ONE)
    {
      BBQ_packet[packet_bit_pointer >> 3] |= (0x80 >> (packet_bit_pointer & 0x07));
    }
    else if (current_bit == BIT_ZERO)
    {
      BBQ_packet[packet_bit_pointer >> 3] &= ~(0x80 >> (packet_bit_pointer & 0x07));
    }
    packet_bit_pointer++;
  }
}

static boolean validatePacket()
{
  boolean validDataReceived = false;
  uint8_t i;

  /*for (i = 0; i < ((BBQ_PACKET_BIT_LENGTH / 8)); i++)
  {
    Serial.print(BBQ_packet_process[i], HEX);
    Serial.print(" ");
  }
  Serial.println();*/

  if ((BBQ_packet_process[0] == 0xAA) &&
      (BBQ_packet_process[1] == 0x99) &&
      (BBQ_packet_process[2] == 0x95) &&
      ((BBQ_packet_process[3] == 0x59) || //regular data packet
       (BBQ_packet_process[3] == 0x6A))   //update transmitter chk_xor_expected--still contains temp info!!!
  )
  {
    tmp_probe2 = tmp_probe1 = 0;

    // convert temp packet from quaternary encoding
    probe2_array[0] = quart(BBQ_packet_process[8] & 0x0F);
    probe2_array[1] = quart(BBQ_packet_process[8] >> 4);
    probe2_array[2] = quart(BBQ_packet_process[7] & 0x0F);
    probe2_array[3] = quart(BBQ_packet_process[7] >> 4);
    probe2_array[4] = quart(BBQ_packet_process[6] & 0x0F);

    probe1_array[0] = quart(BBQ_packet_process[6] >> 4);
    probe1_array[1] = quart(BBQ_packet_process[5] & 0x0F);
    probe1_array[2] = quart(BBQ_packet_process[5] >> 4);
    probe1_array[3] = quart(BBQ_packet_process[4] & 0x0F);
    probe1_array[4] = quart(BBQ_packet_process[4] >> 4);

    for (i = 0; i <= 4; i++)
    {
      tmp_probe2 += probe2_array[i] * (1 << (2 * i));
    }

    for (i = 0; i <= 4; i++)
    {
      tmp_probe1 += probe1_array[i] * (1 << (2 * i));
    }

    //calc checksum and XOR with sent checksum to see if we got good data from correct transmitter
    //checksum calculation needs nibbles 6-17; see adafruit link for info.
    check_data = (uint32_t)quart(BBQ_packet_process[3] >> 4) << 22;
    check_data |= (uint32_t)quart(BBQ_packet_process[3] & 0x0F) << 20;
    check_data |= (uint32_t)tmp_probe1 << 10;
    check_data |= (uint32_t)tmp_probe2;

    chksum_data = calculate_checksum(check_data);

    // nibbles 18-21 have checksum info from sender
    // convert sent checksum nibbles from quaternary encoding
    chksum_sent = (uint16_t)quart(BBQ_packet_process[9] >> 4) << 14;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[9] & 0x0F) << 12;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[10] >> 4) << 10;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[10] & 0x0F) << 8;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[11] >> 4) << 6;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[11] & 0x0F) << 4;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[12] >> 4) << 2;
    chksum_sent |= (uint16_t)quart(BBQ_packet_process[12] & 0x0F);

    // if packet is valid and from correct transmitter, chk_xor is constant
    // chk_xor will be different for each transmitter and will only change when
    //    a) transmitter is powered off/on
    //    b) sync/reset button is pressed on transmitter
    // Maverick wireless BBQ thermometers only allow the receiver to update the
    // transmitter chk_xor ONCE.  any new 6A packets are ignored by receiver until
    // receiver is power cylced or reset.

    chk_xor = chksum_data ^ chksum_sent;

    //check if we need to update chk_xor_expected
    if (BBQ_packet_process[3] == 0x6A)
    {
      chk_xor_expected = chk_xor;
    }

    //Serial.printf("chk_xor: %x, chk_xor_expected: %x\n\n", chk_xor, chk_xor_expected);

    // finish up probe temp calculations to yield celcius temps
    // if the chk_xor is good for current packet
    // and update temps/display if all is good
    if (chk_xor == chk_xor_expected)
    {
      validDataReceived = true;

      if (tmp_probe1 != 0)
      { //check for unplugged temp probe
        temperatures[0u] = tmp_probe1 - 532;
        //Serial.printf("Temperature 0: %i\n", temperatures[0u]);
      }
      else
      {
        temperatures[0u] = INACTIVEVALUE;
      }
      if (tmp_probe2 != 0)
      { //check for unplugged temp probe
        temperatures[1u] = tmp_probe2 - 532;
        //Serial.printf("Temperature 1: %i\n", temperatures[1u]);
      }
      else
      {
        temperatures[1u] = INACTIVEVALUE;
      }
    }
  }

  return validDataReceived;
}

// make the quarternary convertion
static uint8_t quart(uint8_t param)
{
  param &= 0x0F;
  if (param == 0x05)
    return (0);
  if (param == 0x06)
    return (1);
  if (param == 0x09)
    return (2);
  if (param == 0x0A)
    return (3);

  return 0;
}

static uint16_t shiftreg(uint16_t currentValue)
{
  uint8_t msb = (currentValue >> 15) & 1;
  currentValue <<= 1;
  if (msb == 1)
  {
    // Toggle pattern for feedback bits
    // Toggle, if MSB is 1
    currentValue ^= 0x1021;
  }
  return currentValue;
}

static uint16_t calculate_checksum(uint32_t data)
{
  uint16_t mask = 0x3331; //initial value of linear feedback shift register
  uint16_t csum = 0x0;
  int i = 0;
  for (i = 0; i < 24; ++i)
  {
    if ((data >> i) & 0x01)
    {
      //data bit at current position is "1"
      //do XOR with mask
      csum ^= mask;
    }
    mask = shiftreg(mask);
  }
  return csum;
}
