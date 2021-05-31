/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-rf
 * @{
 *
 * \file
 * Implementation of the cc2538 RF driver
 */
#include "contiki.h"
#include "dev/radio.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "net/netstack.h"
#include "net/mac/tsch/tsch.h"
#include "sys/energest.h"
#include "dev/lan8720.h"


#include <string.h>
/*---------------------------------------------------------------------------*/
#define CHECKSUM_LEN 2

/* uDMA channel control persistent flags */
#define UDMA_TX_FLAGS (UDMA_CHCTL_ARBSIZE_128 | UDMA_CHCTL_XFERMODE_AUTO \
    | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8 \
    | UDMA_CHCTL_SRCINC_8 | UDMA_CHCTL_DSTINC_NONE)

#define UDMA_RX_FLAGS (UDMA_CHCTL_ARBSIZE_128 | UDMA_CHCTL_XFERMODE_AUTO \
    | UDMA_CHCTL_SRCSIZE_8 | UDMA_CHCTL_DSTSIZE_8 \
    | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_DSTINC_8)

/*
 * uDMA transfer threshold. DMA will only be used to read an incoming frame
 * if its size is above this threshold
 */
#define UDMA_RX_SIZE_THRESHOLD 3
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "cc2538-rf"
#define LOG_LEVEL LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
/* Local RF Flags */
#define RX_ACTIVE     0x80
#define RF_MUST_RESET 0x40
#define RF_ON         0x01

/* Bit Masks for the last byte in the RX FIFO */
#define CRC_BIT_MASK 0x80
#define LQI_BIT_MASK 0x7F
/* RSSI Offset */
#define RSSI_OFFSET    73
#define RSSI_INVALID -128

/* 192 usec off -> on interval (RX Callib -> SFD Wait). We wait a bit more */
#define ONOFF_TIME                    RTIMER_ARCH_SECOND / 3125
/*---------------------------------------------------------------------------*/
#ifdef CC2538_RF_CONF_AUTOACK
#define CC2538_RF_AUTOACK CC2538_RF_CONF_AUTOACK
#else
#define CC2538_RF_AUTOACK 1
#endif
/*---------------------------------------------------------------------------
 * MAC timer
 *---------------------------------------------------------------------------*/
/* Timer conversion */
#define RADIO_TO_RTIMER(X) ((uint32_t)((uint64_t)(X) * RTIMER_ARCH_SECOND / SYS_CTRL_32MHZ))

#define CLOCK_STABLE() do {															\
			while ( !(REG(SYS_CTRL_CLOCK_STA) & (SYS_CTRL_CLOCK_STA_XOSC_STB)));	\
		} while(0)
/*---------------------------------------------------------------------------*/
/* Are we currently in poll mode? Disabled by default */
static uint8_t volatile poll_mode = 0;
/* Do we perform a CCA before sending? Enabled by default. */
static uint8_t send_on_cca = 1;
static int8_t rssi;
static uint8_t crc_corr;
/*---------------------------------------------------------------------------*/
static uint8_t rf_flags;
static uint8_t rf_channel = IEEE802154_DEFAULT_CHANNEL;

static int on(void);
static int off(void);
/*---------------------------------------------------------------------------*/
/* TX Power dBm lookup table. Values from SmartRF Studio v1.16.0 */
typedef struct output_config {
  radio_value_t power;
  uint8_t txpower_val;
} output_config_t;

static const output_config_t output_power[] = {
  {  7, 0xFF },
  {  5, 0xED },
  {  3, 0xD5 },
  {  1, 0xC5 },
  {  0, 0xB6 },
  { -1, 0xB0 },
  { -3, 0xA1 },
  { -5, 0x91 },
  { -7, 0x88 },
  { -9, 0x72 },
  {-11, 0x62 },
  {-13, 0x58 },
  {-15, 0x42 },
  {-24, 0x00 },
};

static radio_result_t get_value(radio_param_t param, radio_value_t *value);

#define OUTPUT_CONFIG_COUNT (sizeof(output_power) / sizeof(output_config_t))

/* Max and Min Output Power in dBm */
#define OUTPUT_POWER_MIN    (output_power[OUTPUT_CONFIG_COUNT - 1].power)
#define OUTPUT_POWER_MAX    (output_power[0].power)
/*---------------------------------------------------------------------------*/
/*
 * The maximum number of bytes this driver can accept from the MAC layer for
 * transmission or will deliver to the MAC layer after reception. Includes
 * the MAC header and payload, but not the FCS.
 */
#define MAX_PAYLOAD_LEN (CC2538_RF_MAX_PACKET_LEN - CHECKSUM_LEN)
/*---------------------------------------------------------------------------*/
PROCESS(cc2538_rf_process, "cc2538 RF driver");
/*---------------------------------------------------------------------------*/
/**
 * \brief Get the current operating channel
 * \return Returns a value in [11,26] representing the current channel
 */
static uint8_t
get_channel()
{
  return rf_channel;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set the current operating channel
 * \param channel The desired channel as a value in [11,26]
 */
static void
set_channel(uint8_t channel)
{
  uint8_t was_on = 0;

  LOG_INFO("Set Channel\n");

  /* Changes to FREQCTRL take effect after the next recalibration */

  /* If we are off, save state, otherwise switch off and save state */
  if((REG(RFCORE_XREG_FSMSTAT0) & RFCORE_XREG_FSMSTAT0_FSM_FFCTRL_STATE) != 0) {
    was_on = 1;
    off();
  }
  REG(RFCORE_XREG_FREQCTRL) = CC2538_RF_CHANNEL_MIN +
    (channel - CC2538_RF_CHANNEL_MIN) * CC2538_RF_CHANNEL_SPACING;

  /* switch radio back on only if radio was on before - otherwise will turn on radio foor sleepy nodes */
  if(was_on) {
    on();
  }

  rf_channel = channel;
}
/*---------------------------------------------------------------------------*/
static radio_value_t
get_pan_id(void)
{
  return (radio_value_t)(REG(RFCORE_FFSM_PAN_ID1) << 8 | REG(RFCORE_FFSM_PAN_ID0));
}
/*---------------------------------------------------------------------------*/
static void
set_pan_id(uint16_t pan)
{
  REG(RFCORE_FFSM_PAN_ID0) = pan & 0xFF;
  REG(RFCORE_FFSM_PAN_ID1) = pan >> 8;
}
/*---------------------------------------------------------------------------*/
static radio_value_t
get_short_addr(void)
{
  return (radio_value_t)(REG(RFCORE_FFSM_SHORT_ADDR1) << 8 | REG(RFCORE_FFSM_SHORT_ADDR0));
}
/*---------------------------------------------------------------------------*/
static void
set_short_addr(uint16_t addr)
{
  REG(RFCORE_FFSM_SHORT_ADDR0) = addr & 0xFF;
  REG(RFCORE_FFSM_SHORT_ADDR1) = addr >> 8;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Reads the current signal strength (RSSI)
 * \return The current RSSI in dBm
 *
 * This function reads the current RSSI on the currently configured
 * channel.
 */
static radio_value_t
get_rssi(void)
{
  int8_t rssi;
  uint8_t was_off = 0;

  /* If we are off, turn on first */
  if((REG(RFCORE_XREG_FSMSTAT0) & RFCORE_XREG_FSMSTAT0_FSM_FFCTRL_STATE) == 0) {
    was_off = 1;
    on();
  }

  /* Wait for a valid RSSI reading */
  do {
    rssi = REG(RFCORE_XREG_RSSI);
  } while(rssi == RSSI_INVALID);
  rssi -= RSSI_OFFSET;

  /* If we were off, turn back off */
  if(was_off) {
    off();
  }

  return rssi;
}
/*---------------------------------------------------------------------------*/
/* Returns the current CCA threshold in dBm */
static radio_value_t
get_cca_threshold(void)
{
  return (int8_t)(REG(RFCORE_XREG_CCACTRL0) & RFCORE_XREG_CCACTRL0_CCA_THR) - RSSI_OFFSET;
}
/*---------------------------------------------------------------------------*/
/* Sets the CCA threshold in dBm */
static void
set_cca_threshold(radio_value_t value)
{
  REG(RFCORE_XREG_CCACTRL0) = (value & 0xFF) + RSSI_OFFSET;
}
/*---------------------------------------------------------------------------*/
/* Returns the current TX power in dBm */
static radio_value_t
get_tx_power(void)
{
  int i;
  uint8_t reg_val = REG(RFCORE_XREG_TXPOWER) & 0xFF;

  /*
   * Find the TXPOWER value in the lookup table
   * If the value has been written with set_tx_power, we should be able to
   * find the exact value. However, in case the register has been written in
   * a different fashion, we return the immediately lower value of the lookup
   */
  for(i = 0; i < OUTPUT_CONFIG_COUNT; i++) {
    if(reg_val >= output_power[i].txpower_val) {
      return output_power[i].power;
    }
  }
  return OUTPUT_POWER_MIN;
}
/*---------------------------------------------------------------------------*/
/*
 * Set TX power to 'at least' power dBm
 * This works with a lookup table. If the value of 'power' does not exist in
 * the lookup table, TXPOWER will be set to the immediately higher available
 * value
 */
static void
set_tx_power(radio_value_t power)
{
  int i;

  for(i = OUTPUT_CONFIG_COUNT - 1; i >= 0; --i) {
    if(power <= output_power[i].power) {
      REG(RFCORE_XREG_TXPOWER) = output_power[i].txpower_val;
      return;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_frame_filtering(uint8_t enable)
{
  if(enable) {
    REG(RFCORE_XREG_FRMFILT0) |= RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;
  } else {
    REG(RFCORE_XREG_FRMFILT0) &= ~RFCORE_XREG_FRMFILT0_FRAME_FILTER_EN;
  }
}
/*---------------------------------------------------------------------------*/
static void
set_shr_search(int enable)
{
  if(enable) {
    REG(RFCORE_XREG_FRMCTRL0) &= ~RFCORE_XREG_FRMCTRL0_RX_MODE;
  } else {
    REG(RFCORE_XREG_FRMCTRL0) |= RFCORE_XREG_FRMCTRL0_RX_MODE;
  }
}
/*---------------------------------------------------------------------------*/
static void
mac_timer_init(void)
{
  CLOCK_STABLE();
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_SYNC;
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_RUN;
  while(!(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE));
  REG(RFCORE_SFR_MTCTRL) &= ~RFCORE_SFR_MTCTRL_RUN;
  while(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE);
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_SYNC;
  REG(RFCORE_SFR_MTCTRL) |= (RFCORE_SFR_MTCTRL_RUN);
  while(!(REG(RFCORE_SFR_MTCTRL) & RFCORE_SFR_MTCTRL_STATE));
}
/*---------------------------------------------------------------------------*/
static void
set_poll_mode(uint8_t enable)
{
  poll_mode = enable;

  if(enable) {
    mac_timer_init();
    REG(RFCORE_XREG_RFIRQM0) &= ~RFCORE_XREG_RFIRQM0_FIFOP; /* mask out FIFOP interrupt source */
    REG(RFCORE_SFR_RFIRQF0) &= ~RFCORE_SFR_RFIRQF0_FIFOP;   /* clear pending FIFOP interrupt */
    NVIC_DisableIRQ(RF_TX_RX_IRQn);                         /* disable RF interrupts */
  } else {
    REG(RFCORE_XREG_RFIRQM0) |= RFCORE_XREG_RFIRQM0_FIFOP;  /* enable FIFOP interrupt source */
    NVIC_EnableIRQ(RF_TX_RX_IRQn);                          /* enable RF interrupts */
  }
}
/*---------------------------------------------------------------------------*/
static void
set_send_on_cca(uint8_t enable)
{
  send_on_cca = enable;
}
/*---------------------------------------------------------------------------*/
static void
set_auto_ack(uint8_t enable)
{
  if(enable) {
    REG(RFCORE_XREG_FRMCTRL0) |= RFCORE_XREG_FRMCTRL0_AUTOACK;
  } else {
    REG(RFCORE_XREG_FRMCTRL0) &= ~RFCORE_XREG_FRMCTRL0_AUTOACK;
  }
}
/*---------------------------------------------------------------------------*/
static uint32_t
get_sfd_timestamp(void)
{
  uint64_t sfd, timer_val, buffer;

  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000000;
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_LATCH_MODE;
  timer_val = REG(RFCORE_SFR_MTM0) & RFCORE_SFR_MTM0_MTM0;
  timer_val |= ((REG(RFCORE_SFR_MTM1) & RFCORE_SFR_MTM1_MTM1) << 8);
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000000;
  timer_val |= ((REG(RFCORE_SFR_MTMOVF0) & RFCORE_SFR_MTMOVF0_MTMOVF0) << 16);
  timer_val |= ((REG(RFCORE_SFR_MTMOVF1) & RFCORE_SFR_MTMOVF1_MTMOVF1) << 24);
  buffer = REG(RFCORE_SFR_MTMOVF2) & RFCORE_SFR_MTMOVF2_MTMOVF2;
  timer_val |= (buffer << 32);

  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000001;
  REG(RFCORE_SFR_MTCTRL) |= RFCORE_SFR_MTCTRL_LATCH_MODE;
  sfd = REG(RFCORE_SFR_MTM0) & RFCORE_SFR_MTM0_MTM0;
  sfd |= ((REG(RFCORE_SFR_MTM1) & RFCORE_SFR_MTM1_MTM1) << 8);
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000010;
  sfd |= ((REG(RFCORE_SFR_MTMOVF0) & RFCORE_SFR_MTMOVF0_MTMOVF0) << 16);
  sfd |= ((REG(RFCORE_SFR_MTMOVF1) & RFCORE_SFR_MTMOVF1_MTMOVF1) << 24);
  buffer = REG(RFCORE_SFR_MTMOVF2) & RFCORE_SFR_MTMOVF2_MTMOVF2;
  sfd |= (buffer << 32);

  return RTIMER_NOW() - RADIO_TO_RTIMER(timer_val - sfd);
}
/*---------------------------------------------------------------------------*/
/* Enable or disable radio test mode emmiting modulated or unmodulated
 * (carrier) signal. See User's Guide pages 719 and 741.
 */
static uint32_t prev_FRMCTRL0, prev_MDMTEST1;
static uint8_t was_on;

static void
set_test_mode(uint8_t enable, uint8_t modulated)
{
  radio_value_t mode;
  get_value(RADIO_PARAM_POWER_MODE, &mode);

  if(enable) {
    if(mode == RADIO_POWER_MODE_CARRIER_ON) {
      return;
    }
    was_on = (mode == RADIO_POWER_MODE_ON);
    off();
    prev_FRMCTRL0 = REG(RFCORE_XREG_FRMCTRL0);
    /* This constantly transmits random data */
    REG(RFCORE_XREG_FRMCTRL0) = 0x00000042;
    if(!modulated) {
      prev_MDMTEST1 = REG(RFCORE_XREG_MDMTEST1);
      /* ...adding this we send an unmodulated carrier instead */
      REG(RFCORE_XREG_MDMTEST1) = 0x00000018;
    }
    CC2538_RF_CSP_ISTXON();
  } else {
    if(mode != RADIO_POWER_MODE_CARRIER_ON) {
      return;
    }
    CC2538_RF_CSP_ISRFOFF();
    REG(RFCORE_XREG_FRMCTRL0) = prev_FRMCTRL0;
    if(!modulated) {
      REG(RFCORE_XREG_MDMTEST1) = prev_MDMTEST1;
    }
    if(was_on) {
      on();
    }
  }
}
/*---------------------------------------------------------------------------*/
/* Netstack API radio driver functions */
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  int cca;
  uint8_t was_off = 0;

  LOG_INFO("CCA\n");

  /* If we are off, turn on first */
  if((REG(RFCORE_XREG_FSMSTAT0) & RFCORE_XREG_FSMSTAT0_FSM_FFCTRL_STATE) == 0) {
    was_off = 1;
    on();
  }

  /* Wait on RSSI_VALID */
  while((REG(RFCORE_XREG_RSSISTAT) & RFCORE_XREG_RSSISTAT_RSSI_VALID) == 0);

  if(REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_CCA) {
    cca = CC2538_RF_CCA_CLEAR;
  } else {
    cca = CC2538_RF_CCA_BUSY;
  }

  /* If we were off, turn back off */
  if(was_off) {
    off();
  }

  return cca;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  LOG_INFO("On\n");

  if(!(rf_flags & RX_ACTIVE)) {
    CC2538_RF_CSP_ISFLUSHRX();
    CC2538_RF_CSP_ISRXON();

    rf_flags |= RX_ACTIVE;
  }

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  LOG_INFO("Off\n");

  /* Wait for ongoing TX to complete (e.g. this could be an outgoing ACK) */
  while(REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE);

  if(!(REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_FIFOP)) {
    CC2538_RF_CSP_ISFLUSHRX();
  }

  /* Don't turn off if we are off as this will trigger a Strobe Error */
  if(REG(RFCORE_XREG_RXENABLE) != 0) {
    CC2538_RF_CSP_ISRFOFF();
  }

  rf_flags &= ~RX_ACTIVE;

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  LOG_INFO("Init\r\n");

  LAN8720_Init();

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  uint8_t i;

  LOG_INFO("Prepare 0x%02x bytes\r\n", payload_len);

  LOG_INFO("data = ");
  int l = 0;
  uint8_t *buffer=(uint8_t *)ETH_GetCurrentTxBuffer(); //获取当前要发送的DMA描述符中的缓冲区地址
  memcpy((uint8_t *)buffer, (uint8_t *)payload, payload_len);
  LOG_INFO_("%02x", ((unsigned char *)(payload)));
  LOG_INFO_("\r\n");

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  LOG_INFO("Transmit\n");

  uint8_t res = ETH_Tx_Packet(transmit_len);
  if(res==ETH_ERROR) return RADIO_TX_ERR;
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
  prepare(payload, payload_len);
  return transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
read(void *buf, unsigned short bufsize)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  LOG_INFO("Receiving\n");

  /*
   * SFD high while transmitting and receiving.
   * TX_ACTIVE high only when transmitting
   *
   * FSMSTAT1 & (TX_ACTIVE | SFD) == SFD <=> receiving
   */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  LOG_INFO("Pending\n");

  return 0;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
    return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
    return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
const struct radio_driver cc2538_rf_driver = {
  init,
  prepare,
  transmit,
  send,
  read,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
  get_value,
  set_value,
  get_object,
  set_object
};
