/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pio_spi.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_PIO_SPI_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_PIO_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_RP23XX_PIO_SPI

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rp23xx_pio_spibus_initialize
 *
 * Description:
 *   Initialise a PIO-backed SPI master that is a drop-in replacement for
 *   the PL022 hardware-SPI driver returned by rp23xx_spibus_initialize().
 *   The returned handle implements the same struct spi_ops_s contract, so
 *   it can be handed straight to w5500_initialize() (or any other SPI
 *   client) without further changes.
 *
 *   The bus is hard-wired to SPI mode 0 (CPOL=0, CPHA=0), MSB-first, 8-bit
 *   words.  The MISO sample point is tunable at build time through
 *   CONFIG_RP23XX_PIO_SPI_SAMPLE_DELAY -- see rp23xx_pio_spi.c for the
 *   wiring/tuning notes.
 *
 * Input Parameters:
 *   port - Logical port number.  Mirrors rp23xx_spibus_initialize():
 *          0 reuses the CONFIG_RP23XX_SPI0_* pins, 1 the SPI1 pins.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s *rp23xx_pio_spibus_initialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_RP23XX_PIO_SPI */
#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_PIO_SPI_H */
