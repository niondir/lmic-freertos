/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#ifndef _hal_hpp_
#define _hal_hpp_

#include <stdint.h>

//must be provided externally
//plus global cfg defines
//#define CFG_sx1272_radio
//#define CFG_eu868;

typedef struct {
	/*
	 * initialize hardware (IO, SPI, TIMER, IRQ).
	 */
	//void (*lmic_hal_init)(void);

	/*
	 * drive radio NSS pin (0=low, 1=high).
	 */
	void (*radio_spi_cs)(uint8_t val);

	/*
	 * drive radio RX/TX pins (0=rx, 1=tx).
	 */
	void (*radio_hf_switch_txrx)(uint8_t val);

	/*
	 * control radio RST pin (0=low, 1=high, 2=floating)
	 */
	void (*radio_reset)(uint8_t val);

	/*
	 * perform 8-bit SPI transaction with radio.
	 *   - write given byte 'outval'
	 *   - read byte and return value
	 */
	uint8_t (*radio_spi_write)(uint8_t outval);

	/*
	 * return 32-bit system time in ticks.
	 */
	//uint32_t (*lmic_hal_ticks)(void);

	/*
	 * busy-wait until specified timestamp (in ticks) is reached.
	 */
	//void (*lmic_hal_waitUntil)(uint32_t time);

	/*
	 * check and rewind timer for target time.
	 *   - return 1 if target time is close
	 *   - otherwise rewind timer for target time or full period and return 0
	 */
	//uint8_t (*lmic_hal_checkTimer)(uint32_t targettime);

	/*
	 * perform fatal failure action.
	 *   - called by assertions
	 *   - action could be HALT or reboot
	 */
	//void (*lmic_hal_failed)(void);
} lmicApi_t;

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void lmic_hal_init(lmicApi_t lmicApi);

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void lmic_hal_pin_nss (uint8_t val);

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void lmic_hal_pin_rxtx (uint8_t val);

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void lmic_hal_pin_rst (uint8_t val);

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
uint8_t lmic_hal_spi (uint8_t outval);

/*
 * disable all CPU interrupts.
 *   - might be invoked nested 
 *   - will be followed by matching call to hal_enableIRQs()
 */
void lmic_hal_disableIRQs (void);

/*
 * enable CPU interrupts.
 */
void lmic_hal_enableIRQs (void);

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void lmic_hal_sleep (void);

/*
 * return 32-bit system time in ticks.
 */
uint32_t lmic_hal_ticks (void);

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void lmic_hal_waitUntil (uint32_t time);

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
uint8_t lmic_hal_checkTimer (uint32_t targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void lmic_hal_failed(char* file, int linenum);
#endif // _hal_hpp_
