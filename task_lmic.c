#include "drv_lmic.h"
#include "lmic/lmic.h"
#include "github.com/Lobaro/c-utils/logging.h"
#include "github.com/Lobaro/c-utils/parse.h"
#include <stdbool.h>

#define NOTIFY_SX_IRQ_0 (1 << 0)
#define NOTIFY_SX_IRQ_1 (1 << 1)
#define NOTIFY_SX_IRQ_2 (1 << 2)
#define NOTIFY_SYSTICK_IRQ (1 << 3)
#define NOTIFY_SEND (1 << 4)
#define NOTIFY_SLEEP (1 << 5)
#define NOTIFY_WAKE (1 << 6)

static TaskHandle_t Handle = NULL;
static SemaphoreHandle_t LmicRunningSemaphore = NULL; // Is the task, systick and scheduler running?
static SemaphoreHandle_t LmicBusySemaphore = NULL; // Is the lmic scheduler busy?
static SemaphoreHandle_t LmicSendingSemaphore = NULL;

typedef struct {
	uint8_t port;
	uint8_t data[MAX_LEN_FRAME];
	size_t len;
	bool confirmed;
} SendEvent_t;

static QueueHandle_t SendQueue = NULL;
static lmicCfg_t cfg;

void drv_lmic_setOTAA(bool otaa) {
	cfg.otaa = otaa;
}

void drv_lmic_setSpreadingFactor(uint8_t sf) {
	cfg.spreadingFactor = sf;
}

void drv_lmic_setAdr(bool adr) {
	cfg.adr = adr;
}

void drv_lmic_setTxPower(uint8_t txPower) {
	cfg.txPower = txPower;
}

void drv_lmic_setDevAddr(uint8_t* devAddr) {
	cfg.devAddr = ParseInt32BigEndian(devAddr);
}

void drv_lmic_setAppEUI(uint8_t* appEUI) {
	for (int i = 0; i < 8; i++) {
		cfg.appEUI[i] = appEUI[i];
	}
}

void drv_lmic_setDevEUI(uint8_t* devEUI) {
	for (int i = 0; i < 8; i++) {
		cfg.devEUI[i] = devEUI[i];
	}
}

// internally called DevKey, but thats not LoRaWAN spec conform
void drv_lmic_setAppKey(uint8_t* appKey) {
	for (int i = 0; i < 16; i++) {
		cfg.devKey[i] = appKey[i];
	}
}

void drv_lmic_setNetSessionKey(uint8_t* netSessionKey) {
	for (int i = 0; i < 16; i++) {
		cfg.netSessionKey[i] = netSessionKey[i];
	}
}

void drv_lmic_setAppSessionKey(uint8_t* appSessionKey) {
	for (int i = 0; i < 16; i++) {
		cfg.appSessionKey[i] = appSessionKey[i];
	}
}

//Keep in mind that in LMiC APPEUI and DEVEUI are LSBF, DEVKEY (AppKey) is MSBF. To make life easier
void os_getArtEui(uint8_t* buf) { // provide application router ID (8 bytes, LSBF)
	//Log("REQ: OTAA APP EUI\n");

	uint8_t* eui = cfg.appEUI;

	// Reversed!
	for (int i = 0; i < 8; i++) {
		buf[i] = eui[7 - i];
	}
}

void os_getDevEui(uint8_t* buf) { // provide device ID (8 bytes, LSBF)
	//Log("REQ: OTAA DEV EUI\n");

	uint8_t* eui = cfg.devEUI;

	// Reversed!
	for (int i = 0; i < 8; i++) {
		buf[i] = eui[7 - i];
	}
}

void os_getDevKey(uint8_t* buf) { // provide device key (16 bytes)
	//Log("REQ: OTAA APP/Dev key\n");

	uint8_t* eui = cfg.devKey;

	for (int i = 0; i < 16; i++) {
		buf[i] = eui[i];
	}
}

void getNetSessionKey(uint8_t* buf) { // provide device key (16 bytes)
	//Log("REQ: Network Session key\n");

	uint8_t* key = cfg.netSessionKey;

	for (int i = 0; i < 16; i++) {
		buf[i] = key[i];
	}
}

void getAppSessionKey(uint8_t* buf) { // provide device key (16 bytes)
	//Log("REQ: App Session key\n");

	uint8_t* key = cfg.appSessionKey;

	for (int i = 0; i < 16; i++) {
		buf[i] = key[i];
	}
}

uint32_t getDevAddr() {
	//Log("REQ: Device address\n");
	return cfg.devAddr;
}

#define LORA_OTAA 0
#define LORA_RF_DUTY_TESTMODE 0

static void LogNetworkInfo() {
	Log("netid = %d, Dev Addr: %08x\n", LMIC.netid, LMIC.devaddr);
	Log("Network Key: ");
	for (int i = 0; i < 16; i++) {
		Log("%02x", LMIC.nwkKey[i]);
	}
	Log("\n");

	Log("App Key: ");
	for (int i = 0; i < 16; i++) {
		Log("%02x", LMIC.artKey[i]);
	}
	Log("\n");
}

static void SetupLoraWAN() {
	LMIC_reset();

	// Activation by personalisation
	bool otaa = cfg.otaa;
	if (!otaa) {
		getNetSessionKey(LMIC.nwkKey);
		getAppSessionKey(LMIC.artKey);
		uint32_t devaddr = getDevAddr();
		LMIC_setSession(0, devaddr, NULL, NULL);
	}

	LMIC_setLinkCheckMode(0);
	LMIC_setAdrMode(1);

	signed char band;
#if LORA_RF_DUTY_TESTMODE == 1
	band = BAND_AUX;
#else
	band = BAND_CENTI;
#endif

	LMIC_setupBand(BAND_AUX, 14, 1); //10% duty rate
	LMIC_setupBand(BAND_CENTI, 14, 100); //1000 =  0.1%, 100 = 1%, 10 = 10%

	//must be configured after setSession
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);      // g-band
	LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7), band); //DR_SF7B), band);
	LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);
	LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);
	LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);      // g-band
	LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);      // g-band
	LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);      // g-band
	LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), band);      // g-band
	LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);      // g2-band

	//Initial SF and Power Setup
	s1_t txpower = (s1_t) cfg.txPower;
	if (txpower > 14 || txpower < 0) {
		Log("invalid START_POWER (must be 0...14dbm)! using 14dbm...\n");
		txpower = 14;
	}

	int cfgSF = cfg.spreadingFactor;
	if (cfgSF < 7) {
		cfgSF = 7;
	} else if (cfgSF > 12) {
		cfgSF = 12;
	}
	Log("Spreading Factor: %d, TxPower: %d dBm\n", cfgSF, txpower);
	switch (cfgSF) {
	case 7:
		LMIC_setDrTxpow(DR_SF7, txpower);
		break;
	case 8:
		LMIC_setDrTxpow(DR_SF8, txpower);
		break;
	case 9:
		LMIC_setDrTxpow(DR_SF9, txpower);
		break;
	case 10:
		LMIC_setDrTxpow(DR_SF10, txpower);
		break;
	case 11:
		default:
		LMIC_setDrTxpow(DR_SF11, txpower);
	}

	//    LMIC_disableChannel(0);
	//    LMIC_disableChannel(1);
	//    LMIC_disableChannel(2);
	LMIC_disableChannel(3);
	LMIC_disableChannel(4);
	LMIC_disableChannel(5);
	LMIC_disableChannel(6);
	LMIC_disableChannel(7);
	LMIC_disableChannel(8);

	LMIC_setAdrMode(cfg.adr);
	Log("LMIC ADR: %d\n", cfg.adr);

	if (otaa) {
		if (LMIC_startJoining()) {
			Log("OTAA Network join started!\n");
			xSemaphoreTake(LmicSendingSemaphore, 0);
		} else {
			Log("OTAA Network already joined!\n");
		}
	} else {
		Log("ABP join done\n");
		LogNetworkInfo();
	}
}

void drv_lmic_systick_irq_handler() {
	BaseType_t xHigherPriorityTaskWoken;
	xTaskNotifyFromISR(Handle, NOTIFY_SYSTICK_IRQ, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void drv_lmic_sx_irq_handler(uint8_t dio) {
	// Handle IRQ by lmic
	radio_irq_handler(dio);

	// Notify the task about the IRQ
	BaseType_t xHigherPriorityTaskWoken;
	uint32_t value = 0;
	if (dio == 0) {
		value = NOTIFY_SX_IRQ_0;
	} else if (dio == 1) {
		value = NOTIFY_SX_IRQ_1;
	} else if (dio == 2) {
		value = NOTIFY_SX_IRQ_2;
	}

	xTaskNotifyFromISR(Handle, value, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Send unconfirmed. Add a second confirmed method when needed.
BaseType_t drv_lmic_send(uint8_t port, uint8_t* data, size_t len, TickType_t ticksToWait) {
	configASSERT(len <= MAX_LEN_FRAME);

	SendEvent_t e;
	e.port = port;
	memcpy(e.data, data, len);
	e.len = len;
	e.confirmed = false;

	// If already sending just ignore and override current packet
	xSemaphoreTake(LmicSendingSemaphore, 0);

	BaseType_t ret = xQueueSend(SendQueue, &e, ticksToWait);
	xTaskNotify(Handle, NOTIFY_SEND, eSetBits);
	return ret;
}

BaseType_t drv_lmic_sendConfirmed(uint8_t port, uint8_t* data, size_t len, TickType_t ticksToWait) {
	configASSERT(len <= MAX_LEN_FRAME);

	SendEvent_t e;
	e.port = port;
	memcpy(e.data, data, len);
	e.len = len;
	e.confirmed = true;

	// If already sending just ignore and override current packet
	xSemaphoreTake(LmicSendingSemaphore, 0);

	BaseType_t ret = xQueueSend(SendQueue, &e, ticksToWait);
	xTaskNotify(Handle, NOTIFY_SEND, eSetBits);
	return ret;
}

void lmic_stop_systick() {
	TIM9->CR1 = TIM_CR1_UDIS;
}

void lmic_start_systick() {
	TIM9->CR1 = TIM_CR1_CEN;
}

void drv_lmic_sleep() {
	if (uxSemaphoreGetCount(LmicRunningSemaphore) > 0) {
		//xTaskNotify(Handle, NOTIFY_SLEEP, eSetValueWithOverwrite);
		Log("- lmic already sleeping\n");
		return;
	}

	xTaskNotify(Handle, NOTIFY_SLEEP, eSetValueWithOverwrite);
	configASSERT(xSemaphoreTake(LmicRunningSemaphore, 10000 / portTICK_PERIOD_MS));
	xSemaphoreGive(LmicRunningSemaphore);
	vTaskSuspend(Handle);
}

void drv_lmic_wakeup() {
	if (uxSemaphoreGetCount(LmicRunningSemaphore) == 0) {
		//xTaskNotify(Handle, NOTIFY_WAKE, eSetValueWithOverwrite);
		Log("+ lmic already running\n");
		return;
	}

	xTaskNotify(Handle, NOTIFY_WAKE, eSetValueWithOverwrite);
	vTaskResume(Handle);

	int i = 0;
	while (uxSemaphoreGetCount(LmicRunningSemaphore) > 1) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
		i++;
		configASSERT(i < 500);
	}
}

// Check if LMIC has work to do. e.g. for deciding to enter sleep mode
bool drv_lmic_IsBusy() {
	if (drv_lmic_IsSending() || uxSemaphoreGetCount(LmicSendingSemaphore) == 0) {
		return true;
	} else {
		return false;
	}
}

bool drv_lmic_IsSending() {
	if (uxSemaphoreGetCount(LmicSendingSemaphore) == 0) {
		return true;
	} else {
		return false;
	}
}

static Time_t rtc_now() {
	DateTime_t nowDate;
	hal_rtc_GetDateTime(&nowDate);
	return TimeFromDateTime(&nowDate);
}

void drv_lmic_start() {
	static bool started = false;
	lobaroASSERT(!started);
	started = true;

	SetupLoraWAN();
	vTaskResume(Handle);
}

int drv_lmic_TimeToNextJobMs() {
	osjob_t* nextJob = os_nextJob();

	if (nextJob != NULL) {
		return osticks2ms(nextJob->deadline - os_getTime());
	}

	return -1;
}

void LmicLoraWANTask(void* pvParameters) {
	static uint32_t notification;
	static SendEvent_t sendEvent;
	static Time_t sleepTime = 0;

	Log("LMIC LoRaWAN Task created. Not started yet!\n");
	vTaskSuspend(NULL);
// No need to sleep
//xTaskNotify(Handle, NOTIFY_SLEEP, eSetValueWithOverwrite);
	Log("LMIC LoRaWAN Task started.\n");

	for (;;) {

		osjob_t* nextJob = os_nextJob();
		TickType_t sleepTicks = portMAX_DELAY; // We get woken up by ostick timer IRQ anyway!

		if (nextJob != NULL) {
			ostime_t osTime = os_getTime();
			ostime_t nextJobTme = nextJob->deadline - osTime;
			sleepTicks = nextJobTme < 0 ? 0 : ((osticks2ms(nextJobTme) - 10) / portTICK_PERIOD_MS);
		}

		xTaskNotifyWait(0, ULONG_MAX, &notification, sleepTicks);

		if (notification & NOTIFY_SLEEP) {
			lmic_stop_systick();
			sleepTime = rtc_now();
			Log("- lmic sleeping\n");
			xSemaphoreGive(LmicRunningSemaphore);
		}
		if (notification & NOTIFY_WAKE) {
			lmic_start_systick();

			Time_t now = rtc_now();
			Time_t skipSeconds = now - sleepTime;
			if (skipSeconds > 1 * HOUR) {
				skipSeconds = 1 * HOUR;
			}
			// For testing only:
			//skipSeconds = 5 * MINUTE;

			Log("LMIC: Skipping %d seconds that we were sleeping\n", skipSeconds);
			lmic_hal_increase_systicks(sec2osticks(skipSeconds));

			// Throw away duty cycle for all bands
			/*for (u1_t bi = 0; bi < 4; bi++) {
			 LMIC.bands[bi].avail = os_getTime();
			 }*/

			Log("+ lmic running\n");
			xSemaphoreTake(LmicRunningSemaphore, 0);
		}

		// Task is sleeping -> ignore all further notifications
		if (uxSemaphoreGetCount(LmicRunningSemaphore) > 0) {
			continue;
		}

		if (xQueueReceive(SendQueue, &sendEvent, 0)) {
			Log("lmic: Sending queued packet @ %u\n", os_getTime());
			memcpy(LMIC.frame, sendEvent.data, sendEvent.len);
			LMIC_setTxData2(sendEvent.port, LMIC.frame, sendEvent.len, sendEvent.confirmed);
		}

		if (notification & NOTIFY_SX_IRQ_0) {
			Log("SX irq 0 @ %u\n", os_getTime());
			continue;
			//radio_irq_handler(0); // Handled in IRQ
		}
		if (notification & NOTIFY_SX_IRQ_1) {
			Log("SX irq 1 @ %u\n", os_getTime());
			continue;
			//radio_irq_handler(1); // Handled in IRQ
		}
		if (notification & NOTIFY_SX_IRQ_2) {
			Log("SX irq 2 @ %u\n", os_getTime());
			continue;
			//radio_irq_handler(2); // Handled in IRQ
		}

		if (notification & NOTIFY_SYSTICK_IRQ) {
			//Log("Systick @ %u\n", os_getTime());
		}

		nextJob = os_nextJob(); // Update next Job in case it changed by any irq
		if (nextJob != NULL) {
			xSemaphoreTake(LmicBusySemaphore, 0);
			Log("lmic NextJob %04x @ %u (in %d ms)\n", nextJob->func, nextJob->deadline, nextJob->deadline ? osticks2ms(nextJob->deadline - os_getTime()) : 0);
		} else {
			xSemaphoreGive(LmicBusySemaphore);
		}

		if (lmic_hal_asserCalled()) {
			Log("lmic ASSERT called!\n");
		}

		taskENTER_CRITICAL();
		os_runloop(false);
		taskEXIT_CRITICAL();

		//LMIC_sendAlive();
	}
}

void onLmicEvent(ev_t ev) {
	switch (ev) {
// network joined, session established
	case EV_JOINED:
		Log("Join Done.\n");
		LogNetworkInfo();
		xSemaphoreGive(LmicSendingSemaphore);
		break;
	case EV_JOINING:
		Log("OTAA join started\n");
		break;
	case EV_RESET:
		Log("stack reset!\n");
		break;
	case EV_TXCOMPLETE:
		Log("tx done (fc: %d)!\n", LMIC.seqnoUp - 1);
		xSemaphoreGive(LmicSendingSemaphore);
		if (LMIC.dataLen) { // data received in rx slot after tx
			//  debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
			Log("Rxed Data (unprocessed)!\n");
			for (int i = 0; i < LMIC.dataLen; i++) {
				Log("%x", (LMIC.frame + LMIC.dataBeg)[i]);
			}
			Log("\n");
		}
		break;
	case EV_RXCOMPLETE:
		Log("rx done!\n");
		break;
	default:
		Log("unhandeld net event: %d [%x]\n", ev, ev);
	}
}

static uint32_t lastTicks = 0;
void BenchmarkTimer_cb(TimerHandle_t xTimer) {
	uint32_t ticks = os_getTime();
	Log("Ticks per sec: ~%d\n", ticks - lastTicks);

	lastTicks = ticks;
}

void drv_lmic_init(lmicApi_t lmicApi, lmicCfg_t lmicCfg) {
	cfg = lmicCfg;

	LMIC.useLowPowerAntennaOutput = cfg.useLowPowerAntennaOutput;

	os_init(lmicApi);
	srand(radio_rand1() | ((u2_t) radio_rand1()) << 8 | ((u2_t) radio_rand1()) << 16 | ((u2_t) radio_rand1()) << 24);

	SendQueue = xQueueCreate(1, sizeof(SendEvent_t));
	configASSERT(SendQueue);

	LmicRunningSemaphore = xSemaphoreCreateBinary();
	configASSERT(LmicRunningSemaphore);

	LmicSendingSemaphore = xSemaphoreCreateBinary();
	configASSERT(LmicRunningSemaphore);
	xSemaphoreGive(LmicSendingSemaphore);

	LmicBusySemaphore = xSemaphoreCreateBinary();
	configASSERT(LmicBusySemaphore);

//TimerHandle_t benchTimer = xTimerCreate("bench", 1000 / portTICK_PERIOD_MS, true, NULL, BenchmarkTimer_cb);
//xTimerStart(benchTimer, 100);

	xTaskCreate(LmicLoraWANTask, "lmic loraWAN", 500, (void *) 0, 3, &Handle);
}
