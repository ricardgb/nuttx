/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pio_spi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * PIO-based SPI master + bus shim for the RP2350 (DRAFT)
 * ======================================================
 *
 * Why this exists
 * ---------------
 * The PL022 hardware SPI (rp23xx_spi.c) on the W5500-EVB-Pico2 is marginal
 * at the SPI rates the W5500 wants: the MISO (W5500 "SO") line is sampled at
 * a fixed point relative to SCK, and on this board that point lands too
 * close to the edge.  A PIO state machine lets us move the MISO sample point
 * by an arbitrary, *tunable* number of PIO clock cycles after the SCK rising
 * edge, which is exactly the knob a hardware engineer needs to get reliable
 * reads.
 *
 * This file implements struct spi_ops_s backed by ONE PIO state machine and
 * exposes rp23xx_pio_spibus_initialize(port), mirroring the PL022
 * rp23xx_spibus_initialize(port).  It is purely additive -- it does not
 * touch rp23xx_spi.c, the W5500 driver, or the board defconfig.
 *
 * ===========================================================================
 * HOW TO WIRE IT INTO THE W5500 PATH
 * ===========================================================================
 *   The board glue is boards/arm/rp23xx/common/src/rp23xx_w5500.c, function
 *   arm_netinitialize().  It currently does:
 *
 *       spi = rp23xx_spibus_initialize(CONFIG_RP23XX_W5500_SPI_CH);
 *
 *   To use this PIO SPI instead, change that single line to:
 *
 *       spi = rp23xx_pio_spibus_initialize(CONFIG_RP23XX_W5500_SPI_CH);
 *
 *   ...and add  #include "rp23xx_pio_spi.h"  near the existing
 *   #include "rp23xx_spi.h".  Nothing else in the board glue changes: CS is
 *   driven by this driver's select() callback using the SAME pin the PL022
 *   path uses (CONFIG_RP23XX_SPI0_CS_GPIO), and mode/frequency are pushed by
 *   the W5500 driver via SPI_SETMODE/SPI_SETFREQUENCY exactly as before.
 *
 *   Kconfig to set:
 *       CONFIG_RP23XX_PIO_SPI=y
 *       CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY=<n>   (start at the default, 2)
 *   Keep CONFIG_RP23XX_SPI0=y as well: the PIO SPI reuses the SPI0 *pin*
 *   Kconfig values (RX/CS/SCK/TX GPIO).  The PL022 hardware block is simply
 *   left unused; it costs only a few hundred bytes of unused .text.
 *
 *   Pin mapping on this board (all from the SPI0 Kconfig defaults):
 *       SCK  = CONFIG_RP23XX_SPI0_SCK_GPIO = GPIO18  (PIO side-set)
 *       MOSI = CONFIG_RP23XX_SPI0_TX_GPIO  = GPIO19  (PIO OUT)
 *       MISO = CONFIG_RP23XX_SPI0_RX_GPIO  = GPIO16  (PIO IN)
 *       CS   = CONFIG_RP23XX_SPI0_CS_GPIO  = GPIO17  (plain SIO GPIO, active
 *                                                     low, driven by select())
 *
 * ===========================================================================
 * HOW TO TUNE THE MISO SAMPLE DELAY ON HARDWARE
 * ===========================================================================
 *   The W5500 has a known read-only register, VERSIONR (common-block offset
 *   0x0039), that MUST read 0x04.  Use it as the golden read:
 *
 *     1. Bring up the link with CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY=0 and read
 *        VERSIONR repeatedly (e.g. from a small test, or just watch the W5500
 *        driver's probe).  Note whether it reads 0x04 reliably.
 *     2. Sweep the delay 0,1,2,...,15, rebuild, and for each value confirm
 *        VERSIONR == 0x04 over many reads (and that bulk RX of a known buffer
 *        is clean).  Each step adds one PIO clock (~6.7 ns at 150 MHz).
 *     3. On a scope / logic-analyser, trigger on SCK, watch MISO, and confirm
 *        the sample strobe (you can mirror it on a spare GPIO with a 'set'
 *        instruction if you want to see it) lands in the centre of the MISO
 *        data eye, comfortably after the W5500's t_SO output-valid time.
 *     4. Pick the delay in the middle of the range of values that give clean
 *        reads -- that is the most margin against temperature / unit spread.
 *
 *   Remember the RP2350 input synchroniser adds ~2 SCK-domain cycles of
 *   latency on MISO; that is part of what the delay has to absorb.  If you
 *   need the absolute minimum latency you can bypass it with
 *   rp23xx_pio_set_input_sync_bypass(pio, miso_gpio, true) -- left OFF here
 *   because the synchroniser is the safe default and the delay knob makes it
 *   unnecessary to bypass.
 *
 * ===========================================================================
 * DRAFT STATUS / TODO
 * ===========================================================================
 *   - Transfers are POLLED (FIFO push then blocking pull, one byte at a time).
 *     Correct and simple, but not fast.  TODO: use paired DMA channels
 *     (rp23xx_pio_get_dreq(pio, sm, true/false) gives the TX/RX DREQs, see
 *     rp23xx_ws2812.c update_pixels() for the DMA-to-PIO idiom) for bulk
 *     SNDBLOCK/RECVBLOCK once polled operation is proven on hardware.
 *   - The exact mode-0 edge alignment and the default sample delay are
 *     UNVERIFIED on silicon -- they are derived from the datasheet timing and
 *     the pico-examples spi.pio, and are exactly what the tuning procedure
 *     above is meant to dial in.
 *   - Only mode 0 / 8-bit is supported (that is all the W5500 needs); other
 *     modes/bit-widths are rejected with a warning rather than honoured.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "rp23xx_pio.h"
#include "rp23xx_pio_spi.h"
#include "rp23xx_gpio.h"

#ifdef CONFIG_RP23XX_PIO_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Tunable MISO sample delay (PIO clock cycles inserted after the SCK rising
 * edge, before the IN that captures MISO).  Encoded into the delay slot of
 * the 'nop side 1' instruction, so with one side-set bit the usable range is
 * 0..15.
 */

#ifndef CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY
#  define CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY 2
#endif

#define PIO_SPI_SAMPLE_DELAY  CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY

#if (PIO_SPI_SAMPLE_DELAY < 0) || (PIO_SPI_SAMPLE_DELAY > 15)
#  error "CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY must be 0..15 (PIO delay slot)"
#endif

/* Cycles the state machine spends per SPI bit (see the per-bit sequence in
 * rp23xx_pio_spi.pio):
 *   out pins,1 side 0 [1]            -> 2 cycles (SCK low, MOSI setup)
 *   nop        side 1 [SAMPLE_DELAY] -> 1 + SAMPLE_DELAY cycles (SCK high)
 *   in  pins,1 side 1 [0]            -> 1 cycle  (capture MISO)
 */

#define PIO_SPI_CYCLES_PER_BIT  (4 + PIO_SPI_SAMPLE_DELAY)

/* Word geometry: fixed 8-bit, MSB-first frames. */

#define PIO_SPI_NBITS           8

/* TX bytes are left-justified into OSR bit 31 (OUT shifts LEFT), RX bytes
 * land in ISR[7:0] (IN shifts LEFT, 8-bit push threshold).
 */

#define PIO_SPI_TX_SHIFT(b)     (((uint32_t)(uint8_t)(b)) << 24)
#define PIO_SPI_RX_MASK(w)      ((uint8_t)((w) & 0xff))

/* ---- Hand-assembled PIO program ----------------------------------------
 * See rp23xx_pio_spi.pio for the pioasm source these words come from.
 * The 16-bit PIO instruction layout (with .side_set 1) is:
 *
 *   [15:13] opcode  [12] side-set value  [11:8] delay  [7:5] dst/src
 *   [4:0] immediate/bit-count
 *
 * The OUT word below was cross-checked against rp23xx_ws2812.c
 * ('out x,1 side 0 [2]' == 0x6221), which validates this encoding scheme.
 */

#define PIO_SPI_INSTR_OUT \
  0x6101u   /* out pins, 1   side 0 [1]                             */

/* nop == 'mov y, y' (base 0xa042); OR in side-set bit (1<<12 = 0x1000) for
 * SCK high and the SAMPLE_DELAY into the delay slot (bits [11:8]).
 */

#define PIO_SPI_INSTR_NOP \
  (0xa042u | 0x1000u | ((PIO_SPI_SAMPLE_DELAY) << 8))

#define PIO_SPI_INSTR_IN \
  0x5001u   /* in  pins, 1   side 1 [0]                             */

#define PIO_SPI_WRAP_TARGET     0
#define PIO_SPI_WRAP            2

/* ---- Per-port pin selection --------------------------------------------
 * Reuse the existing SPIn pin Kconfig values when present, with fall-backs
 * so the file always compiles even if RP23XX_SPIn is not enabled.
 */

#ifdef CONFIG_RP23XX_SPI0_RX_GPIO
#  define PIO_SPI0_MISO_GPIO    CONFIG_RP23XX_SPI0_RX_GPIO
#else
#  define PIO_SPI0_MISO_GPIO    16
#endif
#ifdef CONFIG_RP23XX_SPI0_CS_GPIO
#  define PIO_SPI0_CS_GPIO      CONFIG_RP23XX_SPI0_CS_GPIO
#else
#  define PIO_SPI0_CS_GPIO      17
#endif
#ifdef CONFIG_RP23XX_SPI0_SCK_GPIO
#  define PIO_SPI0_SCK_GPIO     CONFIG_RP23XX_SPI0_SCK_GPIO
#else
#  define PIO_SPI0_SCK_GPIO     18
#endif
#ifdef CONFIG_RP23XX_SPI0_TX_GPIO
#  define PIO_SPI0_MOSI_GPIO    CONFIG_RP23XX_SPI0_TX_GPIO
#else
#  define PIO_SPI0_MOSI_GPIO    19
#endif

#ifdef CONFIG_RP23XX_SPI1_RX_GPIO
#  define PIO_SPI1_MISO_GPIO    CONFIG_RP23XX_SPI1_RX_GPIO
#else
#  define PIO_SPI1_MISO_GPIO    12
#endif
#ifdef CONFIG_RP23XX_SPI1_CS_GPIO
#  define PIO_SPI1_CS_GPIO      CONFIG_RP23XX_SPI1_CS_GPIO
#else
#  define PIO_SPI1_CS_GPIO      13
#endif
#ifdef CONFIG_RP23XX_SPI1_SCK_GPIO
#  define PIO_SPI1_SCK_GPIO     CONFIG_RP23XX_SPI1_SCK_GPIO
#else
#  define PIO_SPI1_SCK_GPIO     14
#endif
#ifdef CONFIG_RP23XX_SPI1_TX_GPIO
#  define PIO_SPI1_MOSI_GPIO    CONFIG_RP23XX_SPI1_TX_GPIO
#else
#  define PIO_SPI1_MOSI_GPIO    15
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_pio_spidev_s
{
  struct spi_dev_s spidev;        /* Externally visible part of the SPI IF */
  mutex_t          lock;          /* Bus exclusive-access mutex            */
  uint8_t          port;          /* Logical port number (0/1)             */
  uint8_t          sck_gpio;      /* SCK  GPIO (PIO side-set)              */
  uint8_t          mosi_gpio;     /* MOSI GPIO (PIO OUT)                   */
  uint8_t          miso_gpio;     /* MISO GPIO (PIO IN)                    */
  uint8_t          cs_gpio;       /* CS   GPIO (plain SIO, active low)     */
  uint32_t         pio;           /* PIO instance (0..2)                   */
  uint32_t         sm;            /* State machine index (0..3)            */
  uint32_t         offset;        /* Loaded program offset                 */
  uint32_t         frequency;     /* Requested SCK frequency               */
  uint32_t         actual;        /* Achieved SCK frequency                */
  uint8_t          mode;          /* Cached SPI mode                       */
  uint8_t          nbits;         /* Cached bits-per-word                  */
  bool             initialized;   /* Has the SM been brought up?           */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static void     spi_select(struct spi_dev_s *dev, uint32_t devid,
                           bool selected);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t  spi_status(struct spi_dev_s *dev, uint32_t devid);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);

static void     spi_do_exchange(struct spi_dev_s *dev,
                                const void *txbuffer, void *rxbuffer,
                                size_t nwords);

#ifdef CONFIG_SPI_EXCHANGE
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords);
#else
static void     spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                             size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The PIO program: see rp23xx_pio_spi.pio for the annotated source. */

static const uint16_t g_pio_spi_instructions[] =
{
  PIO_SPI_INSTR_OUT,    /* 0: out pins, 1  side 0 [1]              <- wrap   */
  PIO_SPI_INSTR_NOP,    /* 1: nop          side 1 [SAMPLE_DELAY]            */
  PIO_SPI_INSTR_IN,     /* 2: in  pins, 1  side 1 [0]              -> wrap   */
};

static const struct rp23xx_pio_program g_pio_spi_program =
{
  .instructions = g_pio_spi_instructions,
  .length       = 3,
  .origin       = -1,
};

static const struct spi_ops_s g_pio_spi_ops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = 0,                 /* Not supported */
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

static struct rp23xx_pio_spidev_s g_pio_spi0dev =
{
  .spidev      = { .ops = &g_pio_spi_ops },
  .lock        = NXMUTEX_INITIALIZER,
  .port        = 0,
  .sck_gpio    = PIO_SPI0_SCK_GPIO,
  .mosi_gpio   = PIO_SPI0_MOSI_GPIO,
  .miso_gpio   = PIO_SPI0_MISO_GPIO,
  .cs_gpio     = PIO_SPI0_CS_GPIO,
  .nbits       = PIO_SPI_NBITS,
  .mode        = SPIDEV_MODE0,
  .initialized = false,
};

static struct rp23xx_pio_spidev_s g_pio_spi1dev =
{
  .spidev      = { .ops = &g_pio_spi_ops },
  .lock        = NXMUTEX_INITIALIZER,
  .port        = 1,
  .sck_gpio    = PIO_SPI1_SCK_GPIO,
  .mosi_gpio   = PIO_SPI1_MOSI_GPIO,
  .miso_gpio   = PIO_SPI1_MISO_GPIO,
  .cs_gpio     = PIO_SPI1_CS_GPIO,
  .nbits       = PIO_SPI_NBITS,
  .mode        = SPIDEV_MODE0,
  .initialized = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   Take/release the bus mutex so a sequence of transfers is atomic.  Same
 *   contract as the PL022 spi_lock().
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }
  else
    {
      return nxmutex_unlock(&priv->lock);
    }
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Drive the chip-select line.  CS is a plain SIO GPIO (active low), the
 *   SAME pin the PL022 board glue uses (CONFIG_RP23XX_SPIn_CS_GPIO), so the
 *   wiring is unchanged when swapping drivers.
 *
 ****************************************************************************/

static void spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;

  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  /* Active-low CS: drive low to select. */

  rp23xx_gpio_put(priv->cs_gpio, !selected);
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Program the PIO clock divider to obtain the requested SCK frequency.
 *
 *   The state machine consumes PIO_SPI_CYCLES_PER_BIT PIO clocks per SPI
 *   bit, and the PIO clock is BOARD_SYS_FREQ / clkdiv.  Therefore:
 *
 *       SCK = BOARD_SYS_FREQ / (clkdiv * PIO_SPI_CYCLES_PER_BIT)
 *   =>  clkdiv = BOARD_SYS_FREQ / (SCK * PIO_SPI_CYCLES_PER_BIT)
 *
 *   clkdiv is a 16.8 fixed-point value (>= 1.0).  We compute it in 1/256ths
 *   to keep the fractional precision the hardware supports, clamp it to the
 *   representable range, then report back the frequency actually achieved.
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;
  uint64_t clkdiv256;
  uint32_t div_int;
  uint32_t div_frac;
  uint32_t actual;

  DEBUGASSERT(priv != NULL && frequency > 0);

  /* clkdiv in 1/256ths = (sysfreq << 8) / (freq * cycles_per_bit) */

  clkdiv256 = ((uint64_t)BOARD_SYS_FREQ << 8) /
              ((uint64_t)frequency * PIO_SPI_CYCLES_PER_BIT);

  /* Clamp to the hardware range: minimum divisor 1.0 (256), maximum the
   * 16.8 field (0xffffff).  A divisor of exactly 1.0 (clkdiv256 == 256)
   * encodes as div_int=1, div_frac=0; the hardware treats div_int==0 as the
   * full 65536, so never let it reach 0.
   */

  if (clkdiv256 < 256)
    {
      clkdiv256 = 256;                  /* SCK capped at sysclk/cycles_per_bit */
    }
  else if (clkdiv256 > 0xffffff)
    {
      clkdiv256 = 0xffffff;             /* Slowest representable clock */
    }

  div_int  = (uint32_t)(clkdiv256 >> 8);
  div_frac = (uint32_t)(clkdiv256 & 0xff);

  /* Apply to the live state machine (the divider is free-running; changing
   * it while enabled is safe -- see rp23xx_pio.h).
   */

  rp23xx_pio_sm_set_clkdiv_int_frac(priv->pio, priv->sm,
                                    (uint16_t)div_int, (uint8_t)div_frac);

  actual = (uint32_t)(((uint64_t)BOARD_SYS_FREQ << 8) /
                      (clkdiv256 * PIO_SPI_CYCLES_PER_BIT));

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %" PRId32 "->%" PRId32 " (clkdiv %" PRId32 ".%03"
          PRId32 ")\n",
          frequency, actual, div_int, (div_frac * 1000) / 256);

  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   The PIO program is hard-wired to SPI mode 0 (CPOL=0, CPHA=0).  We accept
 *   a mode-0 request and warn on anything else rather than silently lying.
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;

  if (mode != SPIDEV_MODE0)
    {
      spiwarn("PIO SPI only implements mode 0; ignoring request for mode %d\n",
              (int)mode);
    }

  priv->mode = SPIDEV_MODE0;
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   The PIO program shifts fixed 8-bit frames.  Accept 8, warn otherwise.
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;

  if (nbits != PIO_SPI_NBITS)
    {
      spiwarn("PIO SPI only implements 8-bit frames; ignoring %d\n", nbits);
    }

  priv->nbits = PIO_SPI_NBITS;
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   No status sources on this draft bus.
 *
 ****************************************************************************/

static uint8_t spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  UNUSED(dev);
  UNUSED(devid);
  return 0;
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange a single 8-bit word.  Push the (left-justified) TX byte into
 *   the SM's TX FIFO and block for the captured RX byte.  Because the SM
 *   does one full-duplex byte per TX-FIFO entry, the blocking pull naturally
 *   waits out the 8 SCK cycles.
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;
  uint32_t rxword;

  rp23xx_pio_sm_put_blocking(priv->pio, priv->sm, PIO_SPI_TX_SHIFT(wd));
  rxword = rp23xx_pio_sm_get_blocking(priv->pio, priv->sm);

  return PIO_SPI_RX_MASK(rxword);
}

/****************************************************************************
 * Name: spi_do_exchange
 *
 * Description:
 *   Full-duplex block transfer.  For each byte we push TX (or 0x00 don't-care
 *   on a pure read) and pull the captured RX byte.  This is a simple POLLED
 *   one-byte-deep loop -- correct, but a DMA pair would be far faster for
 *   the W5500's larger socket-buffer transfers (see the DRAFT/TODO header).
 *
 *   txbuffer == NULL  -> shift out 0x00 while sampling (read).
 *   rxbuffer == NULL  -> discard captured bytes        (write).
 *
 ****************************************************************************/

static void spi_do_exchange(struct spi_dev_s *dev,
                            const void *txbuffer, void *rxbuffer,
                            size_t nwords)
{
  struct rp23xx_pio_spidev_s *priv = (struct rp23xx_pio_spidev_s *)dev;
  const uint8_t *tx = (const uint8_t *)txbuffer;
  uint8_t       *rx = (uint8_t *)rxbuffer;
  uint32_t       rxword;
  uint8_t        txbyte;
  size_t         i;

  for (i = 0; i < nwords; i++)
    {
      txbyte = tx ? *tx++ : 0x00;

      rp23xx_pio_sm_put_blocking(priv->pio, priv->sm,
                                 PIO_SPI_TX_SHIFT(txbyte));
      rxword = rp23xx_pio_sm_get_blocking(priv->pio, priv->sm);

      if (rx)
        {
          *rx++ = PIO_SPI_RX_MASK(rxword);
        }
    }
}

#ifdef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: spi_exchange
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  spi_do_exchange(dev, txbuffer, rxbuffer, nwords);
}

#else

/****************************************************************************
 * Name: spi_sndblock
 ****************************************************************************/

static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  spi_do_exchange(dev, buffer, NULL, nwords);
}

/****************************************************************************
 * Name: spi_recvblock
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  spi_do_exchange(dev, NULL, buffer, nwords);
}

#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: spi_pio_bringup
 *
 * Description:
 *   Claim a PIO + state machine, load the program, mux the pins and start
 *   the SM.  Mirrors the claim/load idiom in rp23xx_ws2812.c.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure.
 *
 ****************************************************************************/

static int spi_pio_bringup(struct rp23xx_pio_spidev_s *priv)
{
  rp23xx_pio_sm_config config;
  uint32_t             pio;
  int                  sm = -1;

  /* Find a PIO block with a free state machine AND room for our program. */

  for (pio = 0; pio < RP23XX_PIO_NUM; pio++)
    {
      sm = rp23xx_pio_claim_unused_sm(pio, false);
      if (sm < 0)
        {
          continue;
        }

      if (rp23xx_pio_can_add_program(pio, &g_pio_spi_program))
        {
          priv->offset = rp23xx_pio_add_program(pio, &g_pio_spi_program);
          break;
        }

      /* No room here -- release the SM and try the next PIO. */

      rp23xx_pio_sm_unclaim(pio, sm);
      sm = -1;
    }

  if (pio >= RP23XX_PIO_NUM || sm < 0)
    {
      spierr("No free PIO/SM for PIO SPI\n");
      return -ENODEV;
    }

  priv->pio = pio;
  priv->sm  = (uint32_t)sm;

  /* ---- Pin muxing -------------------------------------------------------
   * SCK (side-set) and MOSI (OUT) are driven by the PIO; route their GPIO
   * function to this PIO block.  MISO (IN) is read by the PIO; routing its
   * function to the PIO keeps SIO from also driving it.  CS stays a plain
   * SIO output, driven by spi_select().
   */

  rp23xx_pio_gpio_init(pio, priv->sck_gpio);
  rp23xx_pio_gpio_init(pio, priv->mosi_gpio);
  rp23xx_pio_gpio_init(pio, priv->miso_gpio);

  /* Drive directions: SCK + MOSI are outputs, MISO is an input. */

  rp23xx_pio_sm_set_consecutive_pindirs(pio, sm, priv->sck_gpio,  1, true);
  rp23xx_pio_sm_set_consecutive_pindirs(pio, sm, priv->mosi_gpio, 1, true);
  rp23xx_pio_sm_set_consecutive_pindirs(pio, sm, priv->miso_gpio, 1, false);

  /* CS: plain SIO output, deasserted (high) to start. */

  rp23xx_gpio_init(priv->cs_gpio);
  rp23xx_gpio_setdir(priv->cs_gpio, true);
  rp23xx_gpio_put(priv->cs_gpio, true);

  /* ---- State-machine configuration -------------------------------------- */

  config = rp23xx_pio_get_default_sm_config();

  /* One mandatory side-set bit = SCK. */

  rp23xx_sm_config_set_sideset(&config, 1, false, false);
  rp23xx_sm_config_set_sideset_pins(&config, priv->sck_gpio);

  /* OUT -> MOSI (1 pin), IN base -> MISO. */

  rp23xx_sm_config_set_out_pins(&config, priv->mosi_gpio, 1);
  rp23xx_sm_config_set_in_pins(&config, priv->miso_gpio);

  /* MSB-first, 8-bit frames:
   *   OUT shift LEFT  (false), autopull, threshold 8 -> MOSI = OSR bit 31.
   *   IN  shift LEFT  (false), autopush, threshold 8 -> RX byte in ISR[7:0].
   */

  rp23xx_sm_config_set_out_shift(&config, false, true, PIO_SPI_NBITS);
  rp23xx_sm_config_set_in_shift(&config, false, true, PIO_SPI_NBITS);

  /* Wrap the 3-instruction bit loop. */

  rp23xx_sm_config_set_wrap(&config,
                            priv->offset + PIO_SPI_WRAP_TARGET,
                            priv->offset + PIO_SPI_WRAP);

  /* Load config and jump to the start of the program (SM left disabled). */

  rp23xx_pio_sm_init(pio, sm, priv->offset, &config);

  /* Default clock until the client calls setfrequency(); pick a safe, slow
   * 1 MHz so an early probe before setfrequency() still clocks sanely.
   */

  spi_setfrequency(&priv->spidev, 1000000);

  /* Go. */

  rp23xx_pio_sm_set_enabled(pio, sm, true);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pio_spibus_initialize
 *
 * Description:
 *   See rp23xx_pio_spi.h.  Drop-in counterpart of rp23xx_spibus_initialize().
 *
 ****************************************************************************/

struct spi_dev_s *rp23xx_pio_spibus_initialize(int port)
{
  struct rp23xx_pio_spidev_s *priv;

  switch (port)
    {
      case 0:
        priv = &g_pio_spi0dev;
        break;

      case 1:
        priv = &g_pio_spi1dev;
        break;

      default:
        spierr("Unsupported PIO SPI port %d\n", port);
        return NULL;
    }

  if (priv->initialized)
    {
      return &priv->spidev;
    }

  if (spi_pio_bringup(priv) != OK)
    {
      return NULL;
    }

  priv->initialized = true;
  spiinfo("PIO SPI%d up: pio=%" PRId32 " sm=%" PRId32
          " SCK=%d MOSI=%d MISO=%d CS=%d delay=%d\n",
          port, priv->pio, priv->sm,
          priv->sck_gpio, priv->mosi_gpio, priv->miso_gpio, priv->cs_gpio,
          PIO_SPI_SAMPLE_DELAY);

  return &priv->spidev;
}

#endif /* CONFIG_RP23XX_PIO_SPI */
