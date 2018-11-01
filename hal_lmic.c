#include "drv_lmic.h"
#include "lmic/oslmic.h"

#include "github.com/Lobaro/c-utils/logging.h"


// LMIC hal implementation based on freeRTOS

// Counts TIM9 overflows
volatile uint32_t tim9Overflows = 0;
volatile bool assertCalled = false;

static lmicApi_t api;

bool lmic_hal_asserCalled() {
	return assertCalled;
}

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void lmic_hal_init(lmicApi_t lmicApi) {
	lobaroASSERT(lmicApi.radio_hf_switch_txrx != NULL);
	lobaroASSERT(lmicApi.radio_spi_cs != NULL);
	lobaroASSERT(lmicApi.radio_spi_write != NULL);
	lobaroASSERT(lmicApi.radio_reset != NULL);
	api = lmicApi;


	// Configure TIM9 for systicks
	// with OSTICKS_PER_SEC = 32768
	// =======================================
#define USE_LSE_CLOCK

#ifdef USE_LSE_CLOCK
	PWR->CR |= PWR_CR_DBP; // disable write protect
	RCC->CSR |= RCC_CSR_LSEON; // switch on low-speed oscillator @32.768kHz
	while ((RCC->CSR & RCC_CSR_LSERDY) == 0)
		; // wait for it...
#endif

	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;     // enable clock to TIM9 peripheral
	RCC->APB2LPENR |= RCC_APB2LPENR_TIM9LPEN; // enable clock to TIM9 peripheral also in low power mode
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST;   // reset TIM9 interface
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST;  // reset TIM9 interface

#ifdef USE_LSE_CLOCK
	TIM9->SMCR = TIM_SMCR_ECE; // external clock enable (source clock mode 2) with no prescaler and no filter
#else
			TIM9->PSC = (640 - 1); // HSE_CLOCK_HWTIMER_PSC-1);  XXX: define HSE_CLOCK_HWTIMER_PSC somewhere
#endif

	NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
	NVIC->ISER[TIM9_IRQn >> 5] = 1 << (TIM9_IRQn & 0x1F);  // set enable IRQ

	// enable update (overflow) interrupt
	TIM9->DIER |= TIM_DIER_UIE;

	TIM9->CNT = 0;
	tim9Overflows = 0;

	// Enable timer counting
	TIM9->CR1 = TIM_CR1_CEN;
}



/*
 * drive radio NSS pin (0=low, 1=high).
 * SPI CS
 */
void lmic_hal_pin_nss(u1_t val) {
	api.radio_spi_cs(val);
}

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void lmic_hal_pin_rxtx(u1_t val) {
	api.radio_hf_switch_txrx(val);
}

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void lmic_hal_pin_rst(u1_t val) {
	api.radio_reset(val);
}

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
u1_t lmic_hal_spi(u1_t outval) {
	return hal_spi2_send(outval);
}

/* Masks off all bits but the VECTACTIVE bits in the ICSR register.
 *
 * Defined by FreeRTOS port and different per architecture
 */
#define portVECTACTIVE_MASK					( 0xFFUL )

// static UBaseType_t uxSavedInterruptStatus;

/*
 * disable all CPU interrupts.
 *   - might be invoked nested
 *   - will be followed by matching call to hal_enableIRQs()
 */
void lmic_hal_disableIRQs(void) {
	//taskENTER_CRITICAL();

	/*
	 if (( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) == 0) {
	 taskENTER_CRITICAL();
	 } else {
	 uxSavedInterruptStatus = taskEXIT_CRITICAL_FROM_ISR();
	 }*/
}

/*
 * enable CPU interrupts.
 */
void lmic_hal_enableIRQs(void) {
	//taskEXIT_CRITICAL();

	/*
	 if (( portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) == 0) {
	 taskEXIT_CRITICAL();
	 } else {
	 taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	 }*/
}

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void lmic_hal_sleep(void) {
	// TODO: Task can go to sleep until wakeup by SX1272 IRQ notification
}

/*
 * return 32-bit system time in ticks.
 */
uint32_t lmic_hal_ticks(void) {
	hal_disableIRQs();
	uint32_t t = tim9Overflows;
	uint16_t cnt = TIM9->CNT;
	if ((TIM9->SR & TIM_SR_UIF)) {
		// Overflow before we read CNT?
		// Include overflow in evaluation but
		// leave update of state to ISR once interrupts enabled again
		cnt = TIM9->CNT;
		t++;
	}
	hal_enableIRQs();
	return ((t << 16) | cnt);
	//return hal_rtc_32768Hz_Cnt();
}

void lmic_hal_increase_systicks(uint32_t ticks) {
	tim9Overflows += (ticks >> 16);
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static uint16_t deltaticks(uint32_t time) {
	uint32_t t = lmic_hal_ticks();
	int32_t d = time - t;
	if (d <= 0)
		return 0;    // in the past
	if ((d >> 16) != 0)
		return 0xFFFF; // far ahead
	return (uint16_t) d;
}

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void lmic_hal_waitUntil(uint32_t time) {
	void hal_rtc_waitUntil(uint32_t time) {
		while (deltaticks(time) != 0)
			; // busy wait until timestamp is reached
	}
}

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
u1_t lmic_hal_checkTimer(uint32_t targettime) {
	uint16_t dt;
	TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
	if ((dt = deltaticks(targettime)) < 5) { // event is now (a few ticks ahead)
		TIM9->DIER &= ~TIM_DIER_CC2IE; // disable IE
		return 1;
	} else { // rewind timer (fully or to exact time))
		TIM9->CCR2 = TIM9->CNT + dt;   // set comparator
		TIM9->DIER |= TIM_DIER_CC2IE;  // enable IE
		TIM9->CCER |= TIM_CCER_CC2E;   // enable capture/compare uint 2
		return 0;
	}
}

void TIM9_IRQHandler() {
	if (TIM9->SR & TIM_SR_UIF) { // overflow, ~ every 2 seconds
		tim9Overflows++;
	}
	if ((TIM9->SR & TIM_SR_CC2IF) && (TIM9->DIER & TIM_DIER_CC2IE)) { // compare expired
		// do nothing, only wake up cpu
	}
	TIM9->SR = 0; // clear IRQ flags

	drv_lmic_systick_irq_handler();
}

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void lmic_hal_failed(char* file, int linenum) {
	assertCalled = true;
	Log("LMIC ASSERT: %d:%s\n", (int) linenum, file);
	vAssertCalled(file, linenum);
	//configASSERT(false);
}

